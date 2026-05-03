"""
Stress test for HDrive17-ETH TCP object read/write.

Hammers the embedded webserver with rapid read and write requests
to test stability. Reports throughput, latency, and errors.

Usage:
    python test_objects.py
    python test_objects.py --ip 192.168.122.102 --reads 500 --writes 200 --delay 0
"""

import argparse
import logging
import time

from hdrive_eth import HDriveETH

logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(message)s")


# Objects to cycle through during reads
READ_OBJECTS = [
    ("TCP port",          4, 16),
    ("UDP port",          4, 17),
    ("UDP communication", 4, 19),
    ("Ticket protocol",   4, 22),
    ("Autosend",          4, 34),
]


def run_read_stress(motor: HDriveETH, count: int, delay: float):
    """Blast read requests as fast as possible."""
    print(f"\n{'='*60}")
    print(f"  READ STRESS TEST — {count} requests, delay {delay*1000:.0f}ms")
    print(f"{'='*60}\n")

    ok = 0
    errors = 0
    times = []

    for i in range(count):
        name, idx, sub = READ_OBJECTS[i % len(READ_OBJECTS)]
        t0 = time.perf_counter()
        try:
            value = motor.read_object(index=idx, subindex=sub)
            dt = (time.perf_counter() - t0) * 1000
            times.append(dt)
            ok += 1
            if i < 10 or i % 100 == 0:
                print(f"  [{i+1:>5}/{count}]  m{idx}s{sub}  {name:.<25} "
                      f"{value:>8}   {dt:.1f}ms")
        except Exception as exc:
            dt = (time.perf_counter() - t0) * 1000
            errors += 1
            print(f"  [{i+1:>5}/{count}]  m{idx}s{sub}  ERROR ({dt:.1f}ms): {exc}")

        if delay > 0:
            time.sleep(delay)

    _print_stats("READ", ok, errors, times)


def run_write_stress(motor: HDriveETH, count: int, delay: float):
    """Blast write requests (toggle m4s34 between 0 and 1)."""
    print(f"\n{'='*60}")
    print(f"  WRITE STRESS TEST — {count} requests, delay {delay*1000:.0f}ms")
    print(f"{'='*60}\n")

    ok = 0
    errors = 0
    times = []

    for i in range(count):
        value = i % 2  # toggle 0 / 1
        t0 = time.perf_counter()
        try:
            motor.write_object(index=4, subindex=34, value=value)
            dt = (time.perf_counter() - t0) * 1000
            times.append(dt)
            ok += 1
            if i < 10 or i % 100 == 0:
                print(f"  [{i+1:>5}/{count}]  write m4s34 = {value}   {dt:.1f}ms")
        except Exception as exc:
            dt = (time.perf_counter() - t0) * 1000
            errors += 1
            print(f"  [{i+1:>5}/{count}]  write m4s34  ERROR ({dt:.1f}ms): {exc}")

        if delay > 0:
            time.sleep(delay)

    _print_stats("WRITE", ok, errors, times)


def run_readwrite_stress(motor: HDriveETH, count: int, delay: float):
    """Alternate read and write as fast as possible."""
    print(f"\n{'='*60}")
    print(f"  READ/WRITE MIX TEST — {count} rounds, delay {delay*1000:.0f}ms")
    print(f"{'='*60}\n")

    ok = 0
    errors = 0
    times = []

    for i in range(count):
        # Write
        t0 = time.perf_counter()
        try:
            motor.write_object(index=4, subindex=34, value=i % 2)
            dt_w = (time.perf_counter() - t0) * 1000
        except Exception as exc:
            dt_w = (time.perf_counter() - t0) * 1000
            errors += 1
            print(f"  [{i+1:>5}/{count}]  write ERROR ({dt_w:.1f}ms): {exc}")
            if delay > 0:
                time.sleep(delay)
            continue

        if delay > 0:
            time.sleep(delay)

        # Read back
        t0 = time.perf_counter()
        try:
            val = motor.read_object(index=4, subindex=34)
            dt_r = (time.perf_counter() - t0) * 1000
            times.append(dt_w + dt_r)
            ok += 1
            if i < 10 or i % 100 == 0:
                print(f"  [{i+1:>5}/{count}]  write+read m4s34 = {val}  "
                      f"w:{dt_w:.1f}ms  r:{dt_r:.1f}ms")
        except Exception as exc:
            dt_r = (time.perf_counter() - t0) * 1000
            errors += 1
            print(f"  [{i+1:>5}/{count}]  read ERROR ({dt_r:.1f}ms): {exc}")

        if delay > 0:
            time.sleep(delay)

    _print_stats("READ/WRITE MIX", ok, errors, times)


def _print_stats(label: str, ok: int, errors: int, times: list):
    total = ok + errors
    print(f"\n  --- {label} RESULTS ---")
    print(f"  Total:    {total}")
    print(f"  OK:       {ok}")
    print(f"  Errors:   {errors}  ({errors/total*100:.1f}%)" if total else "")
    if times:
        avg = sum(times) / len(times)
        mn = min(times)
        mx = max(times)
        total_time = sum(times) / 1000
        rps = ok / total_time if total_time > 0 else 0
        print(f"  Latency:  avg {avg:.1f}ms  min {mn:.1f}ms  max {mx:.1f}ms")
        print(f"  Throughput: {rps:.1f} req/s")
    print()


def main():
    parser = argparse.ArgumentParser(description="HDrive TCP object stress test")
    parser.add_argument("--ip", default="192.168.122.102", help="HDrive IP address")
    parser.add_argument("--reads", type=int, default=100, help="Number of read requests")
    parser.add_argument("--writes", type=int, default=100, help="Number of write requests")
    parser.add_argument("--mix", type=int, default=100, help="Number of read/write rounds")
    parser.add_argument("--delay", type=float, default=0.0,
                        help="Delay between requests in seconds (0 = no delay)")
    args = parser.parse_args()

    with HDriveETH(args.ip) as motor:
        print(f"\nTarget: {args.ip}")
        print(f"Delay:  {args.delay*1000:.0f}ms between requests")

        run_read_stress(motor, args.reads, args.delay)
        run_write_stress(motor, args.writes, args.delay)
        run_readwrite_stress(motor, args.mix, args.delay)

        # Restore autosend to enabled
        try:
            motor.write_object(index=4, subindex=34, value=1)
        except Exception:
            pass

        print("All tests complete.\n")


if __name__ == "__main__":
    main()
