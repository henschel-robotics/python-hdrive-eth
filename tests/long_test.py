"""
Long-run **multiaxis demanded-position** test: each step, one random angle is applied to the
**master and both CAN slaves** (same value in ``<canPos/>`` for master and the two slave slots).

**Command path (TCP → firmware → CAN)** — priming uses ``<canConf/>`` and ``<canC2/>``; each step
:func:`send_demanded_positions_multiaxis` sends only ``<canPos/>`` (no object-dictionary slave
mirrors).

**Feedback path (UDP)** — Binary CAN full telemetry (49×int32): compare **demanded** angles
(stored when commands are sent) to **actual** positions from ``BinaryCanFullTelemetryFrame``
(master and ``slave_positions[0:2]``, degrees = int32 / 10).

Statistics: Welford tracking-error σ, UDP ticket count, inter-arrival Δt (ms).

Saves **motor_tracking_std.png** by default; pass ``--show`` to open the plot window.
"""

from __future__ import annotations

import argparse
import math
import random
import signal
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

from hdrive_eth import HDriveETH, Mode
from hdrive_eth.protocol import TXTicket
from hdrive_eth.telemetry import BinaryCanFullTelemetryFrame, TelemetryPayload

@dataclass(frozen=True)
class DemandedAxisDegrees:
    """Per-axis demanded position (degrees) for ``<canPos/>`` (master + two CAN slaves).

    In this test all three are **the same** each tick so slaves track the **same** setpoint
    as the master. Telemetry: ``slave_positions[0]`` → ``can_slave_1``, ``[1]`` → ``can_slave_2``.
    """

    master: float
    can_slave_1: float
    can_slave_2: float

    @classmethod
    def same_all_axes(cls, degrees: float) -> "DemandedAxisDegrees":
        """One commanded angle for master + CAN slaves 1 and 2 (identical to ``canPos`` row)."""
        return cls(master=degrees, can_slave_1=degrees, can_slave_2=degrees)


# -----------------------------------------------------------------------------
# Angle / statistics helpers
# -----------------------------------------------------------------------------


def shortest_arc_position_error_deg(demanded_deg: float, actual_deg: float) -> float:
    """Signed smallest angle difference in ``[-180°, 180°]``."""
    d = actual_deg - demanded_deg
    return (d + 180.0) % 360.0 - 180.0


@dataclass
class WelfordStream:
    """Numerically stable running mean and variance."""

    count: int = 0
    mean: float = 0.0
    m2: float = 0.0

    def update(self, value: float) -> None:
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        delta2 = value - self.mean
        self.m2 += delta * delta2

    @property
    def variance(self) -> float:
        return self.m2 / self.count if self.count > 1 else 0.0

    @property
    def std(self) -> float:
        return math.sqrt(self.variance) if self.count > 1 else 0.0


# -----------------------------------------------------------------------------
# Drive setup (once): CAN tickets + master ``move_to`` — **not** the periodic position stream
# -----------------------------------------------------------------------------


def configure_slaves_position_mode_via_can_tickets(
    motor: HDriveETH,
    *,
    master_torque_mnm: int,
    slave_target_torque_mnm: int,
    speed: int,
    acc: int,
    decc: int,
) -> None:
    """Prime slaves 1–2: ``<canConf/>`` (position mode + torque caps), zero ``<canPos/>``, ``<canC2/>``."""
    mode = int(Mode.POSITION_CONTROL)
    motor.set_can_master_and_slave_configuration(
        mode,
        int(master_torque_mnm),
        [mode, mode],
        [int(slave_target_torque_mnm), int(slave_target_torque_mnm)],
    )
    motor.send_can_pos(0.0, 0.0, 0.0)
    time.sleep(0.05)
    dyn = (speed, acc, decc)
    motor.set_can_master_and_slave_target_profile(dyn, dyn, dyn)


def prime_master_position_move_to_zero(
    motor: HDriveETH,
    *,
    speed: int,
    torque_limit_mnm: int,
    acc: int,
    decc: int,
) -> None:
    """Arm the **Ethernet master axis** in position mode at 0° (``<control/>``, not ``canPos``)."""
    motor.move_to(0, speed=speed, torque=torque_limit_mnm, acc=acc, decc=decc)


def prime_multiaxis_position_tracking(
    motor: HDriveETH,
    *,
    speed: int,
    torque_limit_mnm: int,
    slave_target_torque_mnm: int,
    acc: int,
    decc: int,
) -> None:
    """Full startup: ``<canConf/>`` / ``<canC2/>`` for slaves + master ``move_to`` for Ethernet axis."""
    configure_slaves_position_mode_via_can_tickets(
        motor,
        master_torque_mnm=torque_limit_mnm,
        slave_target_torque_mnm=slave_target_torque_mnm,
        speed=speed,
        acc=acc,
        decc=decc,
    )
    prime_master_position_move_to_zero(
        motor,
        speed=speed,
        torque_limit_mnm=torque_limit_mnm,
        acc=acc,
        decc=decc,
    )


# -----------------------------------------------------------------------------
# **CAN multiaxis demanded positions** — single place the hot loop sends motor commands
# -----------------------------------------------------------------------------


def send_demanded_positions_multiaxis(
    motor: HDriveETH,
    demanded: DemandedAxisDegrees,
) -> None:
    """Send ``<canPos/>`` for master + CAN slaves (torque caps were set at prime via ``<canConf/>``)."""
    motor.send_can_pos(demanded.master, demanded.can_slave_1, demanded.can_slave_2)


# -----------------------------------------------------------------------------
# Test run
# -----------------------------------------------------------------------------


def run_long_test(
    tcp_ip: str,
    tcp_port: int,
    udp_port: int,
    duration_s: float,
    command_interval_s: float,
    plot_stride: int,
    output_png: Optional[str],
    show_plot: bool,
    seed: Optional[int],
    *,
    speed: int,
    torque: int,
    acc: int,
    decc: int,
    prime_axes: bool,
    slave_target_torque_mnm: int,
) -> None:
    rng = random.Random(seed)
    lock = threading.Lock()
    stop_requested = threading.Event()
    udp_telemetry_seen = threading.Event()

    # Latest demanded positions (degrees) — written only from the TCP command thread.
    demanded_bundle = DemandedAxisDegrees.same_all_axes(0.0)

    err_stats_master = WelfordStream()
    err_stats_slave0 = WelfordStream()
    err_stats_slave1 = WelfordStream()
    udp_interarrival_ms = WelfordStream()
    udp_packets_total = 0
    last_udp_monotonic_s: Optional[float] = None

    downsample_times_mono: List[float] = []
    downsample_err_master: List[float] = []
    downsample_err_slave0: List[float] = []
    downsample_err_slave1: List[float] = []
    plot_sample_counter = 0

    def on_udp_binary_can_full(frame: TelemetryPayload) -> None:
        """UDP receiver thread: actual positions vs last **demanded** bundle."""
        nonlocal udp_packets_total, last_udp_monotonic_s, plot_sample_counter

        if not isinstance(frame, BinaryCanFullTelemetryFrame):
            return

        udp_telemetry_seen.set()
        now_mono = time.monotonic()
        udp_packets_total += 1
        if last_udp_monotonic_s is not None:
            udp_interarrival_ms.update((now_mono - last_udp_monotonic_s) * 1000.0)
        last_udp_monotonic_s = now_mono

        actual_master_deg = frame.master_position / 10.0
        actual_slave0_deg = frame.slave_positions[0] / 10.0
        actual_slave1_deg = frame.slave_positions[1] / 10.0

        with lock:
            d = demanded_bundle

        err_stats_master.update(shortest_arc_position_error_deg(d.master, actual_master_deg))
        err_stats_slave0.update(shortest_arc_position_error_deg(d.can_slave_1, actual_slave0_deg))
        err_stats_slave1.update(shortest_arc_position_error_deg(d.can_slave_2, actual_slave1_deg))

        plot_sample_counter += 1
        if plot_sample_counter % plot_stride == 0:
            downsample_times_mono.append(now_mono)
            downsample_err_master.append(
                shortest_arc_position_error_deg(d.master, actual_master_deg)
            )
            downsample_err_slave0.append(
                shortest_arc_position_error_deg(d.can_slave_1, actual_slave0_deg)
            )
            downsample_err_slave1.append(
                shortest_arc_position_error_deg(d.can_slave_2, actual_slave1_deg)
            )

    motor = HDriveETH(
        tcp_ip,
        tcp_port=tcp_port,
        udp_port=udp_port,
        connect=False,
        telemetry_ticket=TXTicket.BINARY_CAN_FULL,
        telemetry_format="binary_can_full",
    )
    motor.on_telemetry(on_udp_binary_can_full)
    motor.connect()

    if prime_axes:
        print(
            "Priming: <canConf/> + <canC2/> + master move_to(0°) …",
            flush=True,
        )
        prime_multiaxis_position_tracking(
            motor,
            speed=speed,
            torque_limit_mnm=torque,
            slave_target_torque_mnm=slave_target_torque_mnm,
            acc=acc,
            decc=decc,
        )
    else:
        print(
            "Skipping prime (--no-prime): no <canConf/> / move_to; motion unlikely.",
            flush=True,
        )

    def handle_sigint(_sig=None, _frame=None) -> None:
        print("\nStopping (interrupt)...")
        stop_requested.set()

    signal.signal(signal.SIGINT, handle_sigint)

    print(
        f"Running {duration_s / 3600:.2f} h | TCP {tcp_ip}:{tcp_port} | UDP {udp_port}\n"
        "  Control: periodic ``send_demanded_positions_multiaxis`` (TCP ``<canPos/>`` only) "
        "— independent of UDP.\n"
        "  Telemetry: needs 49×int32 Binary CAN full (m4s22=10) for stats.",
        flush=True,
    )

    last_console_log_mono = 0.0
    deadline_mono = time.monotonic() + duration_s

    try:
        while time.monotonic() < deadline_mono and not stop_requested.is_set():
            step_deg = rng.uniform(-360.0, 360.0)
            bundle = DemandedAxisDegrees.same_all_axes(step_deg)
            with lock:
                demanded_bundle = bundle

            # --- CAN / multiaxis motor commands (TCP only from this script) ---
            send_demanded_positions_multiaxis(motor, bundle)

            time.sleep(command_interval_s)

            now_mono = time.monotonic()
            if now_mono - last_console_log_mono >= 60.0:
                last_console_log_mono = now_mono
                remaining_s = deadline_mono - now_mono
                if remaining_s > 60:
                    udp_ok = "telemetry ok" if udp_telemetry_seen.is_set() else "no BinaryCAN-full UDP yet"
                    if udp_interarrival_ms.count > 0:
                        dt_txt = (
                            f"UDP pkts={udp_packets_total}  "
                            f"Δt μ={udp_interarrival_ms.mean:.2f} ms σ={udp_interarrival_ms.std:.2f} ms "
                            f"(n_Δ={udp_interarrival_ms.count})"
                        )
                    else:
                        dt_txt = f"UDP pkts={udp_packets_total} (need ≥2 pkts for Δt)"
                    print(
                        f"  ... {remaining_s / 3600:.2f} h left | "
                        f"samples master/slv0/slv1: "
                        f"{err_stats_master.count} / {err_stats_slave0.count} / {err_stats_slave1.count}  ({udp_ok})\n"
                        f"      {dt_txt}",
                        flush=True,
                    )
    finally:
        try:
            motor.stop_master_and_can_slaves(1, 2)
        except Exception:
            pass
        motor.close()

    if udp_interarrival_ms.count > 0:
        dt_footer = (
            f"mean = {udp_interarrival_ms.mean:.4f} ms, std = {udp_interarrival_ms.std:.4f} ms "
            f"(Δt samples: {udp_interarrival_ms.count})"
        )
    else:
        dt_footer = "n/a (fewer than 2 UDP packets)"

    print(
        "\n--- UDP telemetry (Binary CAN full) ---\n"
        f"  Packets received: {udp_packets_total}\n"
        f"  Inter-arrival time: {dt_footer}\n",
        flush=True,
    )

    render_summary_plots(
        err_stats_master,
        err_stats_slave0,
        err_stats_slave1,
        downsample_times_mono,
        downsample_err_master,
        downsample_err_slave0,
        downsample_err_slave1,
        udp_packets_total=udp_packets_total,
        udp_interarrival_ms=udp_interarrival_ms,
        output_png=output_png,
        show_plot=show_plot,
    )


def render_summary_plots(
    err_stats_master: WelfordStream,
    err_stats_slave0: WelfordStream,
    err_stats_slave1: WelfordStream,
    downsample_times_mono: List[float],
    downsample_err_master: List[float],
    downsample_err_slave0: List[float],
    downsample_err_slave1: List[float],
    *,
    udp_packets_total: int,
    udp_interarrival_ms: WelfordStream,
    output_png: Optional[str],
    show_plot: bool,
) -> None:
    try:
        import matplotlib

        if not show_plot:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover
        raise ImportError("Install matplotlib: pip install matplotlib") from exc

    axis_labels = ["master", "CAN slave 0", "CAN slave 1"]
    stds = [err_stats_master.std, err_stats_slave0.std, err_stats_slave1.std]
    counts = [err_stats_master.count, err_stats_slave0.count, err_stats_slave1.count]

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), height_ratios=[1, 1.2])

    ax0 = axes[0]
    colors = ["#2ecc71", "#3498db", "#9b59b6"]
    bars = ax0.bar(axis_labels, stds, color=colors)
    ax0.set_ylabel("Std. dev. of tracking error (deg)")
    if udp_interarrival_ms.count > 0:
        udp_title = (
            f"UDP pkts={udp_packets_total}  Δt μ={udp_interarrival_ms.mean:.2f} ms "
            f"σ={udp_interarrival_ms.std:.2f} ms"
        )
    else:
        udp_title = f"UDP pkts={udp_packets_total}  Δt n/a"
    ax0.set_title(
        "Demanded vs actual position — error σ (shortest arc)\n"
        f"n samples: {counts[0]} / {counts[1]} / {counts[2]}  |  {udp_title}"
    )
    for bar, sigma, n in zip(bars, stds, counts):
        ax0.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height(),
            f"{sigma:.4f}°\n(n={n})",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    ax1 = axes[1]
    if len(downsample_times_mono) > 1:
        t0 = downsample_times_mono[0]
        hours = [(t - t0) / 3600.0 for t in downsample_times_mono]
        ax1.plot(hours, downsample_err_master, alpha=0.7, label="master", linewidth=0.8, color=colors[0])
        ax1.plot(hours, downsample_err_slave0, alpha=0.7, label="CAN slave 0", linewidth=0.8, color=colors[1])
        ax1.plot(hours, downsample_err_slave1, alpha=0.7, label="CAN slave 1", linewidth=0.8, color=colors[2])
        ax1.set_xlabel("Time (hours)")
        ax1.set_ylabel("Position error (deg)")
        ax1.set_title("Downsampled tracking error vs time")
        ax1.legend(loc="upper right")
        ax1.grid(True, alpha=0.3)
    else:
        ax1.text(0.5, 0.5, "Not enough UDP samples for time plot", ha="center", va="center")

    plt.tight_layout()
    if output_png:
        plt.savefig(output_png, dpi=150)
        print(f"Saved figure: {output_png}")
    if show_plot:
        plt.show()
    else:
        plt.close(fig)

    print(
        "Std. dev. (deg): master={:.6f}, slave0={:.6f}, slave1={:.6f}".format(*stds)
    )


def main() -> None:
    p = argparse.ArgumentParser(description="Long-run multiaxis demanded-position tracking (hdrive_eth)")
    p.add_argument("--tcp-address", default="192.168.2.102", help="HDrive IP / TCP host")
    p.add_argument("--tcp-port", type=int, default=1000)
    p.add_argument("--udp-port", type=int, default=1001, help="Local UDP listen port (must match drive m4s17 if used)")
    p.add_argument(
        "--no-prime",
        action="store_true",
        help="Skip <canConf/>/<canC2/> + master move_to priming (debug only)",
    )
    p.add_argument("--speed", type=int, default=500, help="Priming: master move_to speed")
    p.add_argument("--torque", type=int, default=500, help="Priming: torque limit (mNm) for master move_to")
    p.add_argument(
        "--slave-target-torque-mnm",
        type=int,
        default=300,
        help="Priming: <canConf/> slave torque cap (mNm) for CAN slaves 1–2",
    )
    p.add_argument("--acc", type=int, default=5000, help="Priming: acceleration")
    p.add_argument("--decc", type=int, default=5000, help="Priming: deceleration")
    p.add_argument("--duration-hours", type=float, default=12.0)
    p.add_argument(
        "--command-interval",
        type=float,
        default=1.5,
        help="Seconds between new random demanded positions",
    )
    p.add_argument("--plot-stride", type=int, default=100, help="UDP frames per downsampled plot point")
    p.add_argument("--output-png", default="motor_tracking_std.png")
    p.add_argument(
        "--show",
        action="store_true",
        help="Open plot window after saving PNG",
    )
    p.add_argument("--seed", type=int, default=None)
    args = p.parse_args()

    run_long_test(
        tcp_ip=args.tcp_address,
        tcp_port=args.tcp_port,
        udp_port=args.udp_port,
        duration_s=args.duration_hours * 3600.0,
        command_interval_s=args.command_interval,
        plot_stride=args.plot_stride,
        output_png=args.output_png or None,
        show_plot=args.show,
        seed=args.seed,
        speed=args.speed,
        torque=args.torque,
        acc=args.acc,
        decc=args.decc,
        prime_axes=not args.no_prime,
        slave_target_torque_mnm=args.slave_target_torque_mnm,
    )


if __name__ == "__main__":
    main()
