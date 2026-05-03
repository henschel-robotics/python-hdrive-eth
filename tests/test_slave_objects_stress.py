"""
Slave object stress (hardware)

For **slave slot 0** and **slave slot 1**: pick a random integer, write it to a slave OD cell via
``getData.cgi?slvobj=w_…``, read it back with ``slvobj=r_…``, and check it matches. Repeat many
times per slot. No TCP command socket is required (HTTP only).

Records round-trip latency (ms) per iteration and can save a figure with histograms, mean, and
±1 standard deviation markers.

Requires ``HDRIVE_IP``. Example::

    set HDRIVE_IP=192.168.122.102
    python tests/test_slave_objects_stress.py

Writes ``slave_objects_stress_latency.png`` by default (needs matplotlib). Pass ``--no-plot`` to skip.

Or with pytest (optional plot path)::

    set HDRIVE_SLVOBJ_PLOT=out.png
    pytest tests/test_slave_objects_stress.py -v
"""

from __future__ import annotations

import argparse
import math
import os
import random
import statistics
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pytest

from hdrive_eth.exceptions import CommandError
from hdrive_eth.motor import HDriveETH
from hdrive_eth.slvobj import SlaveOd

# Writable cell for round-trip tests (demanded torque, mNm scale — values kept moderate).
MAIN_KEY = SlaveOd.MAIN_DEMANDED_VALUES
SUB_KEY = SlaveOd.SUB_DEMANDED_TORQUE

DEFAULT_ITERATIONS = 1000
DEFAULT_SLVOBJ_HTTP_TIMEOUT = 15.0

# Always exercise the first two slaves on the chain.
STRESS_SLOTS = (0, 1)


def _slvobj_int(body: str) -> int:
    """Parse integer from HTTP body; firmware errors look like ``ERR 3``."""
    b = body.strip()
    if not b:
        raise ValueError("empty slvobj HTTP response")
    if b.upper().startswith("ERR"):
        raise CommandError(b)
    return int(b)


def _summarize_ms(samples: List[float]) -> Tuple[float, float, float, float]:
    """Return mean, sample std dev, min, max."""
    if not samples:
        return (float("nan"), float("nan"), float("nan"), float("nan"))
    mean_v = statistics.mean(samples)
    if len(samples) < 2:
        std_v = 0.0
    else:
        std_v = statistics.stdev(samples)
    return (mean_v, std_v, min(samples), max(samples))


def print_latency_summary(latencies_by_slot: Dict[int, List[float]]) -> None:
    print("\n--- Round-trip latency (write + read, ms) ---", flush=True)
    for slot in sorted(latencies_by_slot.keys()):
        samples = latencies_by_slot[slot]
        mean_v, std_v, mn, mx = _summarize_ms(samples)
        n = len(samples)
        print(
            f"  Slot {slot}: n={n}  mean={mean_v:.2f} ms  std={std_v:.2f} ms"
            f"  min={mn:.2f}  max={mx:.2f}",
            flush=True,
        )
    print(flush=True)


def plot_response_time_stats(
    latencies_by_slot: Dict[int, List[float]],
    output_path: Path,
    *,
    title_suffix: str = "",
) -> None:
    """Histogram per slot with mean line and mean ± 1σ markers."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    slots = sorted(latencies_by_slot.keys())
    if not slots:
        return

    ncols = len(slots)
    fig, axes = plt.subplots(
        1,
        ncols,
        figsize=(5.5 * ncols, 4.2),
        squeeze=False,
    )
    ax_row = axes[0]

    for ax, slot in zip(ax_row, slots):
        samples = latencies_by_slot[slot]
        mean_v, std_v, _, _ = _summarize_ms(samples)

        if samples:
            ax.hist(samples, bins=min(50, max(10, int(math.sqrt(len(samples))))), color="#4C72B0", edgecolor="white", alpha=0.9)
        ax.axvline(mean_v, color="#C44E52", linewidth=2.0, label=f"mean = {mean_v:.2f} ms")
        if len(samples) >= 2 and std_v > 0:
            ax.axvline(mean_v - std_v, color="#55A868", linestyle="--", linewidth=1.5, label=f"mean − σ = {mean_v - std_v:.2f} ms")
            ax.axvline(mean_v + std_v, color="#8172B2", linestyle="--", linewidth=1.5, label=f"mean + σ = {mean_v + std_v:.2f} ms")
            ax.axvspan(mean_v - std_v, mean_v + std_v, color="#CCCCCC", alpha=0.25, label=f"±1σ (σ = {std_v:.2f} ms)")

        ttl = f"Slave slot {slot}: round-trip time\nn = {len(samples)}"
        if title_suffix:
            ttl = f"{ttl}\n{title_suffix}"
        ax.set_title(ttl, fontsize=11)
        ax.set_xlabel("Latency (ms)")
        ax.set_ylabel("Count")
        ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"Latency plot saved: {output_path.resolve()}", flush=True)


def stress_slot(
    ip: str,
    slot: int,
    iterations: int,
    *,
    log_every: int = 200,
    http_timeout: float = DEFAULT_SLVOBJ_HTTP_TIMEOUT,
) -> Tuple[int, List[float]]:
    """Write random values and read them back over HTTP. Returns (failure_count, latencies_ms)."""
    label = f"slave slot {slot}"
    failures = 0
    latencies_ms: List[float] = []
    print(f"\n--- {label}: {iterations} write/read round trips ---", flush=True)

    try:
        for i in range(iterations):
            value = random.randint(-300, 300)
            t0 = time.perf_counter()
            try:
                HDriveETH.write_slvobj(ip, slot, MAIN_KEY, SUB_KEY, value, http_timeout)
                read_back = _slvobj_int(
                    HDriveETH.read_slvobj(ip, slot, MAIN_KEY, SUB_KEY, http_timeout)
                )
                elapsed_ms = (time.perf_counter() - t0) * 1000
                latencies_ms.append(elapsed_ms)
                if read_back != value:
                    failures += 1
                    print(
                        f"  [{i + 1}/{iterations}] mismatch: wrote {value}, read {read_back}",
                        flush=True,
                    )
                elif log_every and (i == 0 or (i + 1) % log_every == 0):
                    print(
                        f"  [{i + 1}/{iterations}] ok  value={value}  ({elapsed_ms:.0f} ms)",
                        flush=True,
                    )
            except Exception as exc:
                failures += 1
                elapsed_ms = (time.perf_counter() - t0) * 1000
                latencies_ms.append(elapsed_ms)
                print(
                    f"  [{i + 1}/{iterations}] error after {elapsed_ms:.0f} ms: {exc}",
                    flush=True,
                )
    finally:
        try:
            HDriveETH.write_slvobj(ip, slot, MAIN_KEY, SUB_KEY, 0, http_timeout)
            print(f"  {label}: demanded torque reset to 0.", flush=True)
        except Exception as exc:
            print(f"  {label}: could not reset torque: {exc}", flush=True)

    print(f"--- {label}: done ({failures} failures) ---\n", flush=True)
    return failures, latencies_ms


def run(
    ip: str,
    iterations: int,
    log_every: int,
    *,
    plot_path: Optional[Path] = None,
) -> Tuple[int, Dict[int, List[float]]]:
    slots_str = ", ".join(str(s) for s in STRESS_SLOTS)
    print(
        f"Slave object stress\n"
        f"  Drive IP:      {ip}\n"
        f"  Iterations:    {iterations} per slave\n"
        f"  Slave slots:   {slots_str} (fixed)\n"
        f"  OD cell:       main={MAIN_KEY} sub={SUB_KEY} (demanded torque)\n",
        flush=True,
    )

    latencies_by_slot: Dict[int, List[float]] = {}
    total_failures = 0
    for slot in STRESS_SLOTS:
        failures, lats = stress_slot(ip, slot, iterations, log_every=log_every)
        total_failures += failures
        latencies_by_slot[slot] = lats

    print_latency_summary(latencies_by_slot)

    if plot_path is not None:
        try:
            plot_response_time_stats(
                latencies_by_slot,
                plot_path,
                title_suffix=f"{iterations} iter/slot · {ip}",
            )
        except ImportError:
            print(
                "matplotlib is not installed; skipping plot. "
                "Install with: pip install matplotlib",
                flush=True,
            )

    if total_failures == 0:
        print("All round trips succeeded.", flush=True)
    else:
        print(f"Finished with {total_failures} failures.", flush=True)

    exit_code = 1 if total_failures else 0
    return exit_code, latencies_by_slot


@pytest.mark.integration
@pytest.mark.skipif(
    not os.environ.get("HDRIVE_IP"),
    reason="Set HDRIVE_IP to run slave object stress on hardware",
)
def test_slave_object_write_read_stress():
    ip = os.environ["HDRIVE_IP"]
    iterations = int(os.environ.get("HDRIVE_SLVOBJ_ITERATIONS", str(DEFAULT_ITERATIONS)))
    log_every = int(os.environ.get("HDRIVE_SLVOBJ_LOG_EVERY", "200"))
    plot_env = os.environ.get("HDRIVE_SLVOBJ_PLOT", "").strip()
    plot_path = Path(plot_env) if plot_env else None

    assert iterations >= 1
    exit_code, _ = run(ip, iterations, log_every, plot_path=plot_path)
    assert exit_code == 0


def main(argv: Optional[list[str]] = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    env_plot = os.environ.get("HDRIVE_SLVOBJ_PLOT", "").strip()
    default_plot_path = Path(env_plot) if env_plot else Path("slave_objects_stress_latency.png")

    p = argparse.ArgumentParser(
        prog="test_slave_objects_stress",
        description="Stress-test slave OD write/read on slots 0 and 1 via HTTP slvobj.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--ip",
        default=os.environ.get("HDRIVE_IP", "192.168.122.102"),
        help="Drive IP (or set HDRIVE_IP)",
    )
    p.add_argument(
        "--iterations",
        "-n",
        type=int,
        default=int(os.environ.get("HDRIVE_SLVOBJ_ITERATIONS", str(DEFAULT_ITERATIONS))),
        help="Write/read cycles per slave slot",
    )
    p.add_argument(
        "--log-every",
        type=int,
        default=int(os.environ.get("HDRIVE_SLVOBJ_LOG_EVERY", "200")),
        help="Print progress every N iterations (0 = only errors)",
    )
    p.add_argument(
        "--plot-path",
        type=Path,
        default=default_plot_path,
        help="PNG path for latency histogram (matplotlib required)",
    )
    p.add_argument(
        "--no-plot",
        action="store_true",
        help="Do not save the latency plot",
    )
    args = p.parse_args(argv)

    plot_path: Optional[Path] = None if args.no_plot else args.plot_path

    exit_code, _ = run(args.ip, args.iterations, args.log_every, plot_path=plot_path)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
