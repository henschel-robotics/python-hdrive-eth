"""
Master motor OD stress (hardware, **no CAN / slvobj**)

Exercises plain TCP ``objRead`` / ``objWrite`` against the **master** object dictionary only.
Uses **m4s34** (autosend flag): values must be **0** or **1**. Each iteration writes a random bit,
reads it back, and checks it matches.

Does **not** use ``HDriveETH.read_slvobj`` (CAN slave OD over HTTP).

Requires ``HDRIVE_IP``. Example::

    set HDRIVE_IP=192.168.122.102
    python tests/test_master_objects_stress.py

Pytest / plot (optional)::

    set HDRIVE_MASTER_OD_PLOT=master_od_latency.png
    pytest tests/test_master_objects_stress.py -v
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
from typing import List, Optional, Tuple

import pytest

from hdrive_eth import HDriveETH

# Master Ethernet comm settings row — same as ``tests/test_objects.py`` stress tooling.
MASTER_MAIN = 4
# Autosend enable (0 = off, 1 = on). Safe scalar round-trip on master OD only.
MASTER_SUB_AUTOSEND = 34

DEFAULT_ITERATIONS = 10000


def _summarize_ms(samples: List[float]) -> Tuple[float, float, float, float]:
    if not samples:
        return (float("nan"), float("nan"), float("nan"), float("nan"))
    mean_v = statistics.mean(samples)
    std_v = statistics.stdev(samples) if len(samples) >= 2 else 0.0
    return (mean_v, std_v, min(samples), max(samples))


def plot_master_latency(samples: List[float], output_path: Path, *, subtitle: str = "") -> None:
    """Histogram with mean and ±1σ (same style as slave stress plot)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mean_v, std_v, _, _ = _summarize_ms(samples)
    fig, ax = plt.subplots(figsize=(6.0, 4.2))
    if samples:
        bins = min(50, max(10, int(math.sqrt(len(samples)))))
        ax.hist(samples, bins=bins, color="#DD8452", edgecolor="white", alpha=0.9)
    ax.axvline(mean_v, color="#C44E52", linewidth=2.0, label=f"mean = {mean_v:.2f} ms")
    if len(samples) >= 2 and std_v > 0:
        ax.axvline(mean_v - std_v, color="#55A868", linestyle="--", linewidth=1.5)
        ax.axvline(mean_v + std_v, color="#8172B2", linestyle="--", linewidth=1.5)
        ax.axvspan(mean_v - std_v, mean_v + std_v, color="#CCCCCC", alpha=0.25, label=f"±1σ (σ = {std_v:.2f} ms)")
    ttl = f"Master OD round-trip (m{MASTER_MAIN}s{MASTER_SUB_AUTOSEND})\nn = {len(samples)}"
    if subtitle:
        ttl = f"{ttl}\n{subtitle}"
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


def stress_master_od_roundtrip(
    motor: HDriveETH,
    iterations: int,
    *,
    log_every: int = 200,
) -> Tuple[int, List[float]]:
    """
    Write random 0/1 to m4s34, read back. Returns (failure_count, latencies_ms).
    """
    failures = 0
    latencies_ms: List[float] = []
    label = f"m{MASTER_MAIN}s{MASTER_SUB_AUTOSEND} (autosend)"

    print(f"\n--- Master OD: {iterations} write/read round trips on {label} ---", flush=True)

    try:
        for i in range(iterations):
            value = random.randint(0, 1)
            t0 = time.perf_counter()
            try:
                motor.write_object(MASTER_MAIN, MASTER_SUB_AUTOSEND, value)
                read_back = motor.read_object(MASTER_MAIN, MASTER_SUB_AUTOSEND)
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
            motor.write_object(MASTER_MAIN, MASTER_SUB_AUTOSEND, 1)
            print("  Master: autosend (m4s34) restored to 1.", flush=True)
        except Exception as exc:
            print(f"  WARN: could not restore m4s34=1: {exc}", flush=True)

    print(f"--- Master OD: done ({failures} failures) ---\n", flush=True)
    return failures, latencies_ms


def run(
    ip: str,
    iterations: int,
    log_every: int,
    *,
    plot_path: Optional[Path] = None,
) -> Tuple[int, List[float]]:
    print(
        f"Master object stress (TCP OD only, no CAN mailbox)\n"
        f"  Drive IP:      {ip}\n"
        f"  Iterations:    {iterations}\n"
        f"  OD cell:       m{MASTER_MAIN}s{MASTER_SUB_AUTOSEND} (autosend 0/1)\n",
        flush=True,
    )

    with HDriveETH(ip) as motor:
        failures, latencies_ms = stress_master_od_roundtrip(
            motor, iterations, log_every=log_every
        )

    mean_v, std_v, mn, mx = _summarize_ms(latencies_ms)
    print(
        "\n--- Round-trip latency (write + read, ms) ---\n"
        f"  Master m{MASTER_MAIN}s{MASTER_SUB_AUTOSEND}: n={len(latencies_ms)}"
        f"  mean={mean_v:.2f}  std={std_v:.2f}  min={mn:.2f}  max={mx:.2f}\n",
        flush=True,
    )

    if plot_path is not None and latencies_ms:
        try:
            plot_master_latency(
                latencies_ms,
                plot_path,
                subtitle=f"{iterations} iterations · {ip}",
            )
        except ImportError:
            print(
                "matplotlib not installed; skipping plot. pip install matplotlib",
                flush=True,
            )

    if failures == 0:
        print("All master OD round trips succeeded.", flush=True)
    else:
        print(f"Finished with {failures} failures.", flush=True)

    return (1 if failures else 0), latencies_ms


@pytest.mark.integration
@pytest.mark.skipif(
    not os.environ.get("HDRIVE_IP"),
    reason="Set HDRIVE_IP to run master OD stress on hardware",
)
def test_master_object_write_read_stress():
    ip = os.environ["HDRIVE_IP"]
    iterations = int(os.environ.get("HDRIVE_MASTER_OD_ITERATIONS", str(DEFAULT_ITERATIONS)))
    log_every = int(os.environ.get("HDRIVE_MASTER_OD_LOG_EVERY", "200"))
    plot_env = os.environ.get("HDRIVE_MASTER_OD_PLOT", "").strip()
    plot_path = Path(plot_env) if plot_env else None

    assert iterations >= 1
    exit_code, _ = run(ip, iterations, log_every, plot_path=plot_path)
    assert exit_code == 0


def main(argv: Optional[list[str]] = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    env_plot = os.environ.get("HDRIVE_MASTER_OD_PLOT", "").strip()
    default_plot_path = Path(env_plot) if env_plot else Path("master_objects_stress_latency.png")

    p = argparse.ArgumentParser(
        prog="test_master_objects_stress",
        description="Stress-test master OD read/write over TCP (m4s34 autosend, no CAN).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--ip",
        default=os.environ.get("HDRIVE_IP", "192.168.2.102"),
        help="Drive IP (or set HDRIVE_IP)",
    )
    p.add_argument(
        "--iterations",
        "-n",
        type=int,
        default=int(os.environ.get("HDRIVE_MASTER_OD_ITERATIONS", str(DEFAULT_ITERATIONS))),
        help="Write/read cycles",
    )
    p.add_argument(
        "--log-every",
        type=int,
        default=int(os.environ.get("HDRIVE_MASTER_OD_LOG_EVERY", "200")),
        help="Print progress every N iterations (0 = errors only)",
    )
    p.add_argument(
        "--plot-path",
        type=Path,
        default=default_plot_path,
        help="PNG path for latency histogram",
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
