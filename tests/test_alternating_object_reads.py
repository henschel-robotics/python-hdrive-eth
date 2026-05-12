"""
Alternating master/slave object reads (hardware)

Alternates one **master** TCP object read (``objRead``) with one **CAN slave** object read.
Slave reads use the unified ``read_slave_object(..., transport=...)`` API and default to
``transport="tcp"``. By default the slave reads cycle over **two slave slots** (``0`` and ``1``).

Requires ``HDRIVE_IP``. Example::

    set HDRIVE_IP=192.168.122.102
    python tests/test_alternating_object_reads.py

Pytest / custom objects::

    set HDRIVE_ALT_MASTER_OBJECTS=3:0,4:17
    set HDRIVE_ALT_SLAVE_OBJECTS=0:6
    set HDRIVE_ALT_SLAVE_TRANSPORT=http
    pytest tests/test_alternating_object_reads.py -v
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import pytest

from hdrive_eth import HDriveETH
from hdrive_eth.slvobj import SlaveOd

ObjectSpec = Tuple[int, int]

DEFAULT_MASTER_OBJECTS: Tuple[ObjectSpec, ...] = (
    (3, 0),   # firmware version
    (4, 17),  # UDP port
    (4, 22),  # TX ticket
)
DEFAULT_SLAVE_OBJECTS: Tuple[ObjectSpec, ...] = (
    (SlaveOd.MAIN_ACTUAL_MOTOR_DATA, SlaveOd.SUB_ACTUAL_VOLTAGE),
)
DEFAULT_ITERATIONS = 1000
DEFAULT_SLAVE_COUNT = 2
DEFAULT_SLAVE_TRANSPORT = "tcp"


def format_object_specs(specs: Sequence[ObjectSpec]) -> str:
    return ",".join(f"{index}:{subindex}" for index, subindex in specs)


def parse_object_specs(text: str) -> List[ObjectSpec]:
    specs: List[ObjectSpec] = []
    for part in text.split(","):
        item = part.strip()
        if not item:
            continue
        chunks = item.split(":")
        if len(chunks) != 2:
            raise argparse.ArgumentTypeError(
                f"Object spec {item!r} must look like 'index:subindex'"
            )
        specs.append((int(chunks[0]), int(chunks[1])))
    if not specs:
        raise argparse.ArgumentTypeError("At least one object spec is required")
    return specs


@dataclass(frozen=True)
class ObjectReadSample:
    source: str
    index: int
    subindex: int
    value: int
    elapsed_ms: float
    slot: Optional[int] = None


@dataclass
class AlternatingObjectReadTest:
    ip: str
    iterations: int = DEFAULT_ITERATIONS
    slave_count: int = DEFAULT_SLAVE_COUNT
    slave_transport: str = DEFAULT_SLAVE_TRANSPORT
    master_objects: Sequence[ObjectSpec] = field(
        default_factory=lambda: list(DEFAULT_MASTER_OBJECTS)
    )
    slave_objects: Sequence[ObjectSpec] = field(
        default_factory=lambda: list(DEFAULT_SLAVE_OBJECTS)
    )
    tcp_port: Optional[int] = None
    udp_port: Optional[int] = None
    log_every: int = 200

    _master_cursor: int = field(default=0, init=False)
    _slave_cursor: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        if self.iterations < 1:
            raise ValueError(f"iterations must be >= 1, got {self.iterations}")
        if self.slave_count < 1 or self.slave_count > 8:
            raise ValueError(f"slave_count must be in 1..8, got {self.slave_count}")
        if self.slave_transport not in {"tcp", "http"}:
            raise ValueError(
                f"slave_transport must be 'tcp' or 'http', got {self.slave_transport!r}"
            )
        if not self.master_objects:
            raise ValueError("master_objects must not be empty")
        if not self.slave_objects:
            raise ValueError("slave_objects must not be empty")

    @property
    def slave_slots(self) -> Tuple[int, ...]:
        return tuple(range(self.slave_count))

    def reset(self) -> None:
        self._master_cursor = 0
        self._slave_cursor = 0

    def read_master_once(self, motor: HDriveETH) -> ObjectReadSample:
        index, subindex = self.master_objects[self._master_cursor % len(self.master_objects)]
        self._master_cursor += 1
        t0 = time.perf_counter()
        value = motor.read_object(index, subindex)
        return ObjectReadSample(
            source="master",
            index=index,
            subindex=subindex,
            value=value,
            elapsed_ms=(time.perf_counter() - t0) * 1000.0,
        )

    def read_slave_once(self, motor: HDriveETH) -> ObjectReadSample:
        slot = self.slave_slots[self._slave_cursor % len(self.slave_slots)]
        index, subindex = self.slave_objects[self._slave_cursor % len(self.slave_objects)]
        self._slave_cursor += 1
        t0 = time.perf_counter()
        value = motor.read_slave_object(
            slot, index, subindex, transport=self.slave_transport
        )
        return ObjectReadSample(
            source="slave",
            slot=slot,
            index=index,
            subindex=subindex,
            value=value,
            elapsed_ms=(time.perf_counter() - t0) * 1000.0,
        )

    def run(
        self, motor: Optional[HDriveETH] = None
    ) -> Tuple[int, List[ObjectReadSample]]:
        own_motor = motor is None
        active_motor = motor
        failures = 0
        samples: List[ObjectReadSample] = []
        self.reset()

        print(
            f"Alternating object reads\n"
            f"  Drive IP:        {self.ip}\n"
            f"  Cycles:          {self.iterations}\n"
            f"  Master objects:  {format_object_specs(self.master_objects)}\n"
            f"  Slave objects:   {format_object_specs(self.slave_objects)}\n"
            f"  Slave transport: {self.slave_transport}\n"
            f"  Slave slots:     {', '.join(str(s) for s in self.slave_slots)}\n",
            flush=True,
        )

        if active_motor is None:
            active_motor = HDriveETH(
                self.ip,
                tcp_port=self.tcp_port,
                udp_port=self.udp_port,
            )

        try:
            for cycle in range(self.iterations):
                master_sample: Optional[ObjectReadSample] = None
                slave_sample: Optional[ObjectReadSample] = None

                try:
                    master_sample = self.read_master_once(active_motor)
                    samples.append(master_sample)
                except Exception as exc:
                    failures += 1
                    print(
                        f"  [{cycle + 1}/{self.iterations}] master read error: {exc}",
                        flush=True,
                    )

                try:
                    slave_sample = self.read_slave_once(active_motor)
                    samples.append(slave_sample)
                except Exception as exc:
                    failures += 1
                    print(
                        f"  [{cycle + 1}/{self.iterations}] slave read error: {exc}",
                        flush=True,
                    )

                if self.log_every and (cycle == 0 or (cycle + 1) % self.log_every == 0):
                    parts = [f"  [{cycle + 1}/{self.iterations}]"]
                    if master_sample is not None:
                        parts.append(
                            f"master m{master_sample.index}s{master_sample.subindex}="
                            f"{master_sample.value} ({master_sample.elapsed_ms:.0f} ms)"
                        )
                    if slave_sample is not None:
                        parts.append(
                            f"slave slot {slave_sample.slot} "
                            f"m{slave_sample.index}s{slave_sample.subindex}="
                            f"{slave_sample.value} ({slave_sample.elapsed_ms:.0f} ms)"
                        )
                    print("  | ".join(parts), flush=True)
        finally:
            if own_motor and active_motor is not None:
                active_motor.close()

        return (1 if failures else 0), samples


def _summarize_source_latencies(
    samples: Sequence[ObjectReadSample], source: str
) -> Tuple[int, float, float, float]:
    vals = [sample.elapsed_ms for sample in samples if sample.source == source]
    if not vals:
        return (0, float("nan"), float("nan"), float("nan"))
    return (len(vals), statistics.mean(vals), min(vals), max(vals))


def run(
    ip: str,
    iterations: int,
    *,
    slave_count: int = DEFAULT_SLAVE_COUNT,
    slave_transport: str = DEFAULT_SLAVE_TRANSPORT,
    master_objects: Optional[Sequence[ObjectSpec]] = None,
    slave_objects: Optional[Sequence[ObjectSpec]] = None,
    tcp_port: Optional[int] = None,
    udp_port: Optional[int] = None,
    log_every: int = 200,
) -> Tuple[int, List[ObjectReadSample]]:
    runner = AlternatingObjectReadTest(
        ip=ip,
        iterations=iterations,
        slave_count=slave_count,
        slave_transport=slave_transport,
        master_objects=master_objects or list(DEFAULT_MASTER_OBJECTS),
        slave_objects=slave_objects or list(DEFAULT_SLAVE_OBJECTS),
        tcp_port=tcp_port,
        udp_port=udp_port,
        log_every=log_every,
    )
    exit_code, samples = runner.run()

    m_n, m_mean, m_min, m_max = _summarize_source_latencies(samples, "master")
    s_n, s_mean, s_min, s_max = _summarize_source_latencies(samples, "slave")
    print(
        "\n--- Read latency summary (ms) ---\n"
        f"  Master reads: n={m_n}  mean={m_mean:.2f}  min={m_min:.2f}  max={m_max:.2f}\n"
        f"  Slave reads:  n={s_n}  mean={s_mean:.2f}  min={s_min:.2f}  max={s_max:.2f}\n",
        flush=True,
    )
    if exit_code == 0:
        print("All alternating reads succeeded.", flush=True)
    else:
        print("Alternating reads finished with failures.", flush=True)
    return exit_code, samples


class _FakeMotor:
    def __init__(self) -> None:
        self.calls: List[Tuple[str, int, int, Optional[int], Optional[str]]] = []

    def read_object(self, index: int, subindex: int) -> int:
        self.calls.append(("master", index, subindex, None, None))
        return 100 + len(self.calls)

    def read_slave_object(
        self, slot: int, index: int, subindex: int, transport: str = "tcp"
    ) -> int:
        self.calls.append(("slave", index, subindex, slot, transport))
        return 200 + len(self.calls)


def test_alternating_object_read_test_defaults_to_two_slaves():
    runner = AlternatingObjectReadTest(ip="127.0.0.1", iterations=3)
    assert runner.slave_count == 2
    assert runner.slave_transport == "tcp"
    assert runner.slave_slots == (0, 1)


def test_alternating_object_read_test_cycles_master_then_two_slave_slots():
    runner = AlternatingObjectReadTest(
        ip="127.0.0.1",
        iterations=3,
        master_objects=[(3, 0)],
        slave_objects=[(SlaveOd.MAIN_ACTUAL_MOTOR_DATA, SlaveOd.SUB_ACTUAL_VOLTAGE)],
        log_every=0,
    )
    motor = _FakeMotor()

    exit_code, samples = runner.run(motor)

    assert exit_code == 0
    assert [call[0] for call in motor.calls] == [
        "master",
        "slave",
        "master",
        "slave",
        "master",
        "slave",
    ]
    assert [call[3] for call in motor.calls if call[0] == "slave"] == [0, 1, 0]
    assert [call[4] for call in motor.calls if call[0] == "slave"] == ["tcp", "tcp", "tcp"]
    assert [sample.source for sample in samples] == [
        "master",
        "slave",
        "master",
        "slave",
        "master",
        "slave",
    ]


@pytest.mark.integration
@pytest.mark.skipif(
    not os.environ.get("HDRIVE_IP"),
    reason="Set HDRIVE_IP to run alternating master/slave reads on hardware",
)
def test_alternating_master_and_slave_reads():
    ip = os.environ["HDRIVE_IP"]
    iterations = int(os.environ.get("HDRIVE_ALT_ITERATIONS", str(DEFAULT_ITERATIONS)))
    slave_count = int(os.environ.get("HDRIVE_ALT_SLAVE_COUNT", str(DEFAULT_SLAVE_COUNT)))
    slave_transport = os.environ.get("HDRIVE_ALT_SLAVE_TRANSPORT", DEFAULT_SLAVE_TRANSPORT)
    log_every = int(os.environ.get("HDRIVE_ALT_LOG_EVERY", "200"))
    master_objects = parse_object_specs(
        os.environ.get(
            "HDRIVE_ALT_MASTER_OBJECTS",
            format_object_specs(DEFAULT_MASTER_OBJECTS),
        )
    )
    slave_objects = parse_object_specs(
        os.environ.get(
            "HDRIVE_ALT_SLAVE_OBJECTS",
            format_object_specs(DEFAULT_SLAVE_OBJECTS),
        )
    )

    exit_code, _ = run(
        ip,
        iterations,
        slave_count=slave_count,
        slave_transport=slave_transport,
        master_objects=master_objects,
        slave_objects=slave_objects,
        log_every=log_every,
    )
    assert exit_code == 0


def main(argv: Optional[List[str]] = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]

    p = argparse.ArgumentParser(
        prog="test_alternating_object_reads",
        description="Alternate master objRead and slave objReadCAN reads over TCP.",
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
        default=int(os.environ.get("HDRIVE_ALT_ITERATIONS", str(DEFAULT_ITERATIONS))),
        help="Alternating read cycles (one master read + one slave read per cycle)",
    )
    p.add_argument(
        "--slave-count",
        type=int,
        default=int(os.environ.get("HDRIVE_ALT_SLAVE_COUNT", str(DEFAULT_SLAVE_COUNT))),
        help="How many slave slots to cycle through",
    )
    p.add_argument(
        "--slave-transport",
        choices=("tcp", "http"),
        default=os.environ.get("HDRIVE_ALT_SLAVE_TRANSPORT", DEFAULT_SLAVE_TRANSPORT),
        help="Transport used for slave object reads",
    )
    p.add_argument(
        "--master-objects",
        type=parse_object_specs,
        default=parse_object_specs(
            os.environ.get(
                "HDRIVE_ALT_MASTER_OBJECTS",
                format_object_specs(DEFAULT_MASTER_OBJECTS),
            )
        ),
        help="Comma-separated master object specs: index:subindex,index:subindex",
    )
    p.add_argument(
        "--slave-objects",
        type=parse_object_specs,
        default=parse_object_specs(
            os.environ.get(
                "HDRIVE_ALT_SLAVE_OBJECTS",
                format_object_specs(DEFAULT_SLAVE_OBJECTS),
            )
        ),
        help="Comma-separated slave object specs: index:subindex,index:subindex",
    )
    p.add_argument(
        "--log-every",
        type=int,
        default=int(os.environ.get("HDRIVE_ALT_LOG_EVERY", "200")),
        help="Print progress every N cycles (0 = errors only)",
    )
    args = p.parse_args(argv)

    exit_code, _ = run(
        args.ip,
        args.iterations,
        slave_count=args.slave_count,
        slave_transport=args.slave_transport,
        master_objects=args.master_objects,
        slave_objects=args.slave_objects,
        log_every=args.log_every,
    )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
