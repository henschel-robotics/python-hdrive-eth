"""
Long-run **sinusoidal position demand** over TCP and **continuous tracking-error**
measurement from UDP.

**Master + CAN slaves 0 and 1** (telemetry ``slave_positions[0]``, ``[1]`` — same order as the
second and third arguments to ``send_can_pos``) follow the same sine::

    demand_deg(t) = amplitude_deg * sin(2 * pi * t / period_s)

Slave **positions** use the **``<canPos/>``** TCP ticket only (``send_can_pos``); no m7
``targetPosition`` writes.

Slaves are primed via ``<canConf/>`` and ``<canC2/>`` (**RXConfigTicketCANAdvanced**) for
per-slave speed / acc / dec.

Telemetry (**Binary CAN full**, 49×int32): actual angles are positions / 10 (degrees). Errors use
the shortest arc in degrees. **UDP inter-arrival times** (ms between packets) are also tracked.

**Ethernet master** torque and mode come from the same ``<canConf/>`` ticket as the slaves
(``RXConfigTicketCAN`` — first two ints are master **mNm** torque and mode). No separate
``<control/>`` / ``move_to`` is required for priming.

After priming, **0°** is commanded on master + slaves until UDP telemetry shows all within
``DEFAULT_HOMING_TOL_DEG``; only then does the timed sine run start (``t0`` is after homing).

The figure is **saved as PNG** by default (no GUI). Set ``DEFAULT_SHOW_PLOT`` to open a window.

On exit (normal end, Ctrl+C, or error) the run path calls :meth:`HDriveETH.stop_master_and_can_slaves`
for followers 1–2, then :meth:`HDriveETH.close`.

Edit the **Run defaults** block at the top of this file (IP, duration, sine parameters, etc.).
"""

from __future__ import annotations

import math
import signal
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

from hdrive_eth import HDriveETH, Mode
from hdrive_eth.protocol import TXTicket
from hdrive_eth.telemetry import BinaryCanFullTelemetryFrame, TelemetryPayload

# -----------------------------------------------------------------------------
# Run defaults — edit here (no CLI)
# -----------------------------------------------------------------------------
DEFAULT_IP = "192.168.2.102"
DEFAULT_TCP_PORT: Optional[int] = None
DEFAULT_UDP_PORT: Optional[int] = None
DEFAULT_DURATION_HOURS = 8.0
DEFAULT_AMPLITUDE_DEG = 300.0
DEFAULT_PERIOD_S = 10.0
DEFAULT_CONTROL_DT_MS = 50.0
# ``canC2`` / ``<control/>`` profile integers — firmware scales them to RPM / (RPM/s) in the web UI.
# Match “Demanded values” columns (e.g. accel 400 / decel 5000 RPM/s) by tuning these three.
DEFAULT_SPEED = 400
DEFAULT_TORQUE_MNM = 100  # ``<canConf/>`` master demanded torque (**mNm**)
DEFAULT_SLAVE_TARGET_TORQUE_MNM = 100  # ``<canConf/>`` slave torque (mNm), priming only here
DEFAULT_ACC = 400
DEFAULT_DECC = 5000
DEFAULT_PLOT_STRIDE = 200
DEFAULT_OUTPUT_PNG = "sinus_position_track_error.png"
DEFAULT_SHOW_PLOT = False
# After priming: command 0° until all axes report within this (shortest arc, degrees).
DEFAULT_HOMING_TOL_DEG = 0.5
DEFAULT_HOMING_TIMEOUT_S = 120.0


def prime_master_and_slaves_position(
    motor: HDriveETH,
    *,
    speed: int,
    torque: int,
    slave_target_torque_mnm: int,
    acc: int,
    decc: int,
) -> None:
    """Master + CAN slaves 1–2: position mode and torque limits (``<canConf/>``), then path (``<canC2/>``)."""
    mode = int(Mode.POSITION_CONTROL_NPP)
    slave_t = int(slave_target_torque_mnm)
    master_t = int(torque)
    motor.set_can_master_and_slave_configuration(
        mode,
        master_t,
        [mode, mode],
        [slave_t, slave_t],
    )

    dyn = (speed, acc, decc)
    motor.set_can_master_and_slave_target_profile(dyn, dyn, dyn)


def _axes_within_tol_deg(
    frame: BinaryCanFullTelemetryFrame,
    target_deg: float,
    tol_deg: float,
) -> bool:
    am = frame.master_position / 10.0
    a0 = frame.slave_positions[0] / 10.0
    a1 = frame.slave_positions[1] / 10.0
    return (
        abs(angle_error_deg(target_deg, am)) <= tol_deg
        and abs(angle_error_deg(target_deg, a0)) <= tol_deg
        and abs(angle_error_deg(target_deg, a1)) <= tol_deg
    )


def angle_error_deg(target_deg: float, actual_deg: float) -> float:
    """Shortest signed angle difference in degrees [-180, 180]."""
    d = actual_deg - target_deg
    return (d + 180.0) % 360.0 - 180.0


@dataclass
class Welford1D:
    n: int = 0
    mean: float = 0.0
    m2: float = 0.0

    def update(self, x: float) -> None:
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        d2 = x - self.mean
        self.m2 += d * d2

    @property
    def variance(self) -> float:
        return self.m2 / self.n if self.n > 1 else 0.0

    @property
    def std(self) -> float:
        return math.sqrt(self.variance) if self.n > 1 else 0.0


@dataclass
class SinusPositionTrack:
    ip: str
    duration_s: float
    amplitude_deg: float
    period_s: float
    control_dt_s: float
    speed: int
    torque: int
    slave_target_torque_mnm: int
    acc: int
    decc: int
    tcp_port: Optional[int] = None
    udp_port: Optional[int] = None
    plot_stride: int = 200
    output_png: Optional[str] = "sinus_position_track_error.png"
    show_plot: bool = False
    log_interval_s: float = 60.0

    _lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _cmd_deg: float = 0.0
    _w_m: Welford1D = field(default_factory=Welford1D, init=False)
    _w_s0: Welford1D = field(default_factory=Welford1D, init=False)
    _w_s1: Welford1D = field(default_factory=Welford1D, init=False)
    _w_udp_dt_ms: Welford1D = field(default_factory=Welford1D, init=False)
    _last_udp_mono: Optional[float] = field(default=None, init=False)
    _stop: threading.Event = field(default_factory=threading.Event, init=False)
    _t_trace: List[float] = field(default_factory=list, init=False)
    _e_m: List[float] = field(default_factory=list, init=False)
    _e_s0: List[float] = field(default_factory=list, init=False)
    _e_s1: List[float] = field(default_factory=list, init=False)
    _plot_n: int = 0

    def _on_telemetry(self, frame: TelemetryPayload) -> None:
        if not isinstance(frame, BinaryCanFullTelemetryFrame):
            return

        now = time.monotonic()
        if self._last_udp_mono is not None:
            dt_ms = (now - self._last_udp_mono) * 1000.0
            self._w_udp_dt_ms.update(dt_ms)
        self._last_udp_mono = now

        actual_m = frame.master_position / 10.0
        actual_s0 = frame.slave_positions[0] / 10.0
        actual_s1 = frame.slave_positions[1] / 10.0

        with self._lock:
            cmd = self._cmd_deg

        self._w_m.update(angle_error_deg(cmd, actual_m))
        self._w_s0.update(angle_error_deg(cmd, actual_s0))
        self._w_s1.update(angle_error_deg(cmd, actual_s1))

        self._plot_n += 1
        if self._plot_n % self.plot_stride == 0:
            self._t_trace.append(now)
            self._e_m.append(angle_error_deg(cmd, actual_m))
            self._e_s0.append(angle_error_deg(cmd, actual_s0))
            self._e_s1.append(angle_error_deg(cmd, actual_s1))

    def _wait_all_at_deg(
        self,
        motor: HDriveETH,
        target_deg: float,
        tol_deg: float,
        timeout_s: float,
        poll_s: float,
    ) -> None:
        """Command ``<canPos/>`` until UDP shows master + slaves within tolerance."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self._stop.is_set():

            motor.send_can_pos(target_deg, target_deg, target_deg)
            frame = motor.telemetry
            if isinstance(frame, BinaryCanFullTelemetryFrame) and _axes_within_tol_deg(
                frame, target_deg, tol_deg
            ):
                print(
                    f"  Homing done: all axes within ±{tol_deg:g}° of {target_deg:g}°.",
                    flush=True,
                )
                return
            time.sleep(poll_s)
        if self._stop.is_set():
            raise KeyboardInterrupt
        raise TimeoutError(
            f"Axes did not reach {target_deg:g}° within ±{tol_deg:g}° in {timeout_s:g} s."
        )

    def run(self) -> None:
        def on_int(_s=None, _f=None) -> None:
            print("\nStopping (Ctrl+C)…", flush=True)
            self._stop.set()

        signal.signal(signal.SIGINT, on_int)

        kwargs = {
            "telemetry_ticket": TXTicket.BINARY_CAN_FULL,
            "telemetry_format": "binary_can_full",
            "connect": False,
        }
        if self.tcp_port is not None:
            kwargs["tcp_port"] = self.tcp_port
        if self.udp_port is not None:
            kwargs["udp_port"] = self.udp_port

        motor = HDriveETH(self.ip, **kwargs)
        motor.on_telemetry(self._on_telemetry)
        motor.connect()

        print(
            f"Sinus position track (master + slaves 0 & 1) | {self.ip} | "
            f"duration {self.duration_s/3600:.3f} h\n"
            f"  demand: A={self.amplitude_deg:g}°  "
            f"T={self.period_s:g} s  dt={self.control_dt_s*1000:g} ms\n"
            f"  canConf master torque: {self.torque} mNm | "
            f"slave torque (canConf priming): {self.slave_target_torque_mnm} mNm | "
            f"speed={self.speed}  acc={self.acc}  decc={self.decc}\n"
            f"  Priming <canConf/> + <canC2/>, homing with <canPos/>, then sine via <canPos/> …",
            flush=True,
        )

        prime_master_and_slaves_position(
            motor,
            speed=self.speed,
            torque=self.torque,
            slave_target_torque_mnm=self.slave_target_torque_mnm,
            acc=self.acc,
            decc=self.decc,
        )

        print(
            f"  Homing: <canPos 0,0,0/> until master + slaves within ±{DEFAULT_HOMING_TOL_DEG:g}° …",
            flush=True,
        )
        self._wait_all_at_deg(
            motor,
            0.0,
            DEFAULT_HOMING_TOL_DEG,
            DEFAULT_HOMING_TIMEOUT_S,
            self.control_dt_s,
        )

        t0 = time.monotonic()
        t_end = t0 + self.duration_s
        omega = 2.0 * math.pi / self.period_s
        next_log = t0 + self.log_interval_s

        try:
            while time.monotonic() < t_end and not self._stop.is_set():
                now = time.monotonic()
                elapsed = now - t0
                demand = self.amplitude_deg * math.sin(omega * elapsed)

                motor.send_can_pos(demand, demand, demand)

                if now >= next_log:
                    print(
                        f"  [{elapsed/60:.1f} min]  n={self._w_m.n}  "
                        f"UDP Δt mean={self._w_udp_dt_ms.mean:.2f} ms σ={self._w_udp_dt_ms.std:.2f} ms\n"
                        f"       err°  master: μ={self._w_m.mean:.4f} σ={self._w_m.std:.4f} | "
                        f"slave0: μ={self._w_s0.mean:.4f} σ={self._w_s0.std:.4f} | "
                        f"slave1: μ={self._w_s1.mean:.4f} σ={self._w_s1.std:.4f}",
                        flush=True,
                    )
                    next_log = now + self.log_interval_s

                sleep_rem = self.control_dt_s - (time.monotonic() - now)
                if sleep_rem > 0:
                    time.sleep(sleep_rem)
        finally:
            try:
                motor.stop_master_and_can_slaves(1, 2)
            except Exception:
                pass
            motor.close()

        print(
            f"\nDone. n={self._w_m.n} UDP packets per axis.\n"
            f"  UDP interval (ms): mean={self._w_udp_dt_ms.mean:.4f}  "
            f"std={self._w_udp_dt_ms.std:.4f}  (n_dt={self._w_udp_dt_ms.n})\n"
            f"  Tracking error (deg, shortest arc):\n"
            f"    master: mean={self._w_m.mean:.6f}  std={self._w_m.std:.6f}\n"
            f"    slave0: mean={self._w_s0.mean:.6f}  std={self._w_s0.std:.6f}\n"
            f"    slave1: mean={self._w_s1.mean:.6f}  std={self._w_s1.std:.6f}",
            flush=True,
        )
        self._save_plot()

    def _save_plot(self) -> None:
        if not self.output_png and not self.show_plot:
            return
        try:
            import matplotlib

            if not self.show_plot:
                matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed; skip plot.", flush=True)
            return

        labels = ["master", "slave 0", "slave 1"]
        stds = [self._w_m.std, self._w_s0.std, self._w_s1.std]
        means = [self._w_m.mean, self._w_s0.mean, self._w_s1.mean]
        colors = ["#2ecc71", "#3498db", "#9b59b6"]

        fig, axes = plt.subplots(2, 1, figsize=(10, 8), height_ratios=[1, 1.2])

        ax0 = axes[0]
        bars = ax0.bar(labels, stds, color=colors)
        ax0.set_ylabel("Std. dev. (deg)")
        ax0.set_title(
            f"Sinus tracking error σ — n ≈ {self._w_m.n} samples per axis\n"
            f"UDP packet interval: μ={self._w_udp_dt_ms.mean:.2f} ms  "
            f"σ={self._w_udp_dt_ms.std:.2f} ms"
        )
        for b, sig, mu in zip(bars, stds, means):
            ax0.text(
                b.get_x() + b.get_width() / 2,
                b.get_height(),
                f"σ={sig:.4f}°\nμ={mu:.4f}°",
                ha="center",
                va="bottom",
                fontsize=9,
            )

        ax1 = axes[1]
        if len(self._t_trace) > 1:
            t0 = self._t_trace[0]
            t_h = [(x - t0) / 3600.0 for x in self._t_trace]
            ax1.plot(t_h, self._e_m, linewidth=0.7, color=colors[0], label="master", alpha=0.85)
            ax1.plot(t_h, self._e_s0, linewidth=0.7, color=colors[1], label="slave 0", alpha=0.85)
            ax1.plot(t_h, self._e_s1, linewidth=0.7, color=colors[2], label="slave 1", alpha=0.85)
            ax1.set_xlabel("Time (hours)")
            ax1.set_ylabel("Error (deg)")
            ax1.set_title(f"Downsampled error (every {self.plot_stride} UDP frames)")
            ax1.legend(loc="upper right")
            ax1.grid(True, alpha=0.3)
        else:
            ax1.text(0.5, 0.5, "Not enough telemetry for time series", ha="center", va="center")

        plt.tight_layout()
        if self.output_png:
            plt.savefig(self.output_png, dpi=150)
            print(f"Saved figure: {self.output_png}", flush=True)
        if self.show_plot:
            plt.show()
        else:
            plt.close(fig)


def main() -> int:
    test = SinusPositionTrack(
        ip=DEFAULT_IP,
        duration_s=DEFAULT_DURATION_HOURS * 3600.0,
        amplitude_deg=DEFAULT_AMPLITUDE_DEG,
        period_s=DEFAULT_PERIOD_S,
        control_dt_s=DEFAULT_CONTROL_DT_MS / 1000.0,
        speed=DEFAULT_SPEED,
        torque=DEFAULT_TORQUE_MNM,
        slave_target_torque_mnm=DEFAULT_SLAVE_TARGET_TORQUE_MNM,
        acc=DEFAULT_ACC,
        decc=DEFAULT_DECC,
        tcp_port=DEFAULT_TCP_PORT,
        udp_port=DEFAULT_UDP_PORT,
        plot_stride=DEFAULT_PLOT_STRIDE,
        output_png=DEFAULT_OUTPUT_PNG or None,
        show_plot=DEFAULT_SHOW_PLOT,
    )
    try:
        test.run()
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
