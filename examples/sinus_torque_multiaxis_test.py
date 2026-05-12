"""
Long-running sinusoidal torque test: master + selected CAN slaves.

Torque setpoints follow  tau(t) = amplitude_mNm * sin(2*pi*t / period).

Master and slaves are driven via ``<canConf/>`` (``RXConfigTicketCAN``): demanded
torque (**mNm**) and mode (``Mode.TORQUE_CONTROL``).

Prerequisites:
  - HDrive ETH master at ``ip``, firmware >= 266
  - Slaves configured on CAN; IDs set in the master
  - Slaves in a profile suitable for torque/current mode

Usage:
  python sinus_torque_multiaxis_test.py --ip 192.168.122.102
  python sinus_torque_multiaxis_test.py --ip 192.168.122.102 --duration-hours 0.01

On normal completion (duration reached), the master is put in stop mode (mode 0)
and ``<canConf/>`` clears slave demands; Ctrl+C triggers the same via ``HDriveETH`` close.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Iterable, List, Optional, Tuple

from hdrive_eth import HDriveETH, Mode

# Absolute torque limit for this test (mNm)
MAX_TORQUE_MNM = 300


def _canconf_torque_mode_vectors(
    slave_slots: Tuple[int, ...], tau_mnm: int
) -> Tuple[List[int], List[int]]:
    """Eight-wide slave mode / slave torque lists for ``<canConf/>`` (only ``slave_slots`` set)."""
    sm = [0] * 8
    st = [0] * 8
    mode = int(Mode.TORQUE_CONTROL)
    tq = int(max(-32768, min(32767, tau_mnm)))
    for s in slave_slots:
        if 1 <= s <= 8:
            sm[s - 1] = mode
            st[s - 1] = tq
    return sm, st


class SinusTorqueMultiaxisTest:
    """Drive master + selected CAN slaves with a common sinusoidal torque (``<canConf/>`` only)."""

    def __init__(
        self,
        ip: str,
        *,
        duration_s: float,
        amplitude_mnm: float,
        sine_period_s: float,
        control_dt_s: float,
        slave_slots: Iterable[int] = (1, 2),
    ) -> None:
        self.ip = ip
        self.duration_s = duration_s
        self.amplitude_mnm = amplitude_mnm
        self.sine_period_s = sine_period_s
        self.control_dt_s = control_dt_s
        self.slave_slots = tuple(slave_slots)

        if amplitude_mnm <= 0:
            raise ValueError("amplitude_mNm must be positive")
        if sine_period_s <= 0:
            raise ValueError("sine_period_s must be positive")
        if control_dt_s <= 0:
            raise ValueError("control_dt_s must be positive")

    def _push_torques_canconf(self, motor: HDriveETH, tau_mnm: int) -> None:
        """One ``<canConf/>``: master + listed slaves in torque mode with the same ``tau_mnm``."""
        mode = int(Mode.TORQUE_CONTROL)
        sm, st = _canconf_torque_mode_vectors(self.slave_slots, tau_mnm)
        motor.set_can_master_and_slave_configuration(mode, tau_mnm, sm, st)

    def run(self, log_interval_s: float = 60.0) -> None:
        """Execute the test until duration expires or KeyboardInterrupt."""
        omega = 2.0 * math.pi / self.sine_period_s
        t0 = time.monotonic()
        next_log = t0 + log_interval_s

        amp = min(self.amplitude_mnm, MAX_TORQUE_MNM)

        with HDriveETH(self.ip) as motor:
            print(
                f"Sinus torque (canConf): A={amp:g} mNm (cap {MAX_TORQUE_MNM}), "
                f"T={self.sine_period_s:g} s, dt={self.control_dt_s:g} s, "
                f"duration={self.duration_s / 3600:.4g} h, slaves={self.slave_slots}",
                flush=True,
            )

            self._push_torques_canconf(motor, 0)

            while True:
                now = time.monotonic()
                elapsed = now - t0
                if elapsed >= self.duration_s:
                    break

                tau = amp * math.sin(omega * elapsed)
                tau_i = int(round(tau))
                tau_i = max(-MAX_TORQUE_MNM, min(MAX_TORQUE_MNM, tau_i))

                self._push_torques_canconf(motor, tau_i)

                if now >= next_log:
                    print(
                        f"[{elapsed / 60:.1f} min] tau_cmd≈{tau_i} mNm",
                        flush=True,
                    )
                    next_log = now + log_interval_s

                time.sleep(self.control_dt_s)

            motor.stop()
            self._push_torques_canconf(motor, 0)

        print("Test finished (duration reached).", flush=True)


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ip", default="192.168.2.102", help="Master HDrive IP")
    p.add_argument(
        "--duration-hours",
        type=float,
        default=4.0,
        help="Run time in hours (default: 4)",
    )
    p.add_argument(
        "--amplitude-mnm",
        type=float,
        default=300.0,
        help="Peak torque magnitude in mNm (default: 300)",
    )
    p.add_argument(
        "--sine-period-s",
        type=float,
        default=60.0,
        help="Sine period in seconds (default: 60)",
    )
    p.add_argument(
        "--control-dt-ms",
        type=float,
        default=20.0,
        help="Control loop period in ms (default: 20)",
    )
    p.add_argument(
        "--slaves",
        type=str,
        default="1,2",
        help="Comma-separated slave slots 1..8 (default: 1,2)",
    )
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    slots = tuple(int(x.strip()) for x in args.slaves.split(",") if x.strip())

    duration_s = args.duration_hours * 3600.0
    test = SinusTorqueMultiaxisTest(
        args.ip,
        duration_s=duration_s,
        amplitude_mnm=args.amplitude_mnm,
        sine_period_s=args.sine_period_s,
        control_dt_s=args.control_dt_ms / 1000.0,
        slave_slots=slots,
    )
    try:
        test.run()
    except KeyboardInterrupt:
        print("\nStopped by user.", file=sys.stderr, flush=True)
        return 130

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
