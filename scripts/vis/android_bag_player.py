#!/usr/bin/env python3
"""Android ADAS bag session player (topic__name/*.bin protobuf bags).

Supports:
  - unpacked session directory (e.g. adas_bags/2026_07_18_09_45_15)
  - .zip / .tar.gz archives of that layout

Usage:
  python3 vis/android_bag_player.py /path/to/session --summary
  python3 vis/android_bag_player.py /path/to/session --topic vehicle/state
"""

from __future__ import annotations

import _path  # noqa: F401

import argparse
import heapq
import sys
import tarfile
import tempfile
import zipfile
from datetime import timedelta
from pathlib import Path
from typing import Any, Callable, Dict, Generator, List, Optional, Tuple, Union

from vis.bag_io import fix_topic, list_topics, load_topic_messages


class AndroidBagPlayer:
    """Read ADAS BagLogger sessions (vehicle/state, sensors/imu, …)."""

    def __init__(self, bag_path: Union[str, Path], quiet: bool = False):
        self.bag_path = Path(bag_path).resolve()
        self._tmpdir: Optional[tempfile.TemporaryDirectory] = None
        self._session_dir = self._resolve_session(self.bag_path)
        self._callbacks: Dict[str, Callable[[str, Any, int], None]] = {}
        self._enabled_topics: set[str] = set()
        self._cache: Dict[str, List[Tuple[int, Any, Any]]] = {}

        if not quiet:
            print(f"Loading bag session: {self._session_dir}")
            for topic in self.topics:
                n = len(self.get_topic_msgs(topic))
                print(f"  {topic}: {n} messages")

    def _resolve_session(self, path: Path) -> Path:
        if path.is_dir():
            if list_topics(path):
                return path
            # maybe archive extracted one level deep
            kids = [p for p in path.iterdir() if p.is_dir() and list_topics(p)]
            if len(kids) == 1:
                return kids[0]
            raise FileNotFoundError(f"No topic__*/*.bin under {path}")

        if not path.is_file():
            raise FileNotFoundError(path)

        self._tmpdir = tempfile.TemporaryDirectory(prefix="adas_bag_")
        extract_root = Path(self._tmpdir.name)
        if path.suffix == ".zip" or path.name.endswith(".zip"):
            with zipfile.ZipFile(path, "r") as zf:
                zf.extractall(extract_root)
        elif path.suffixes[-2:] == [".tar", ".gz"] or path.suffix == ".tgz":
            with tarfile.open(path, "r:*") as tf:
                tf.extractall(extract_root)
        else:
            raise ValueError(
                f"Unsupported archive: {path} (use session dir, .zip, .tar.gz)"
            )

        if list_topics(extract_root):
            return extract_root
        kids = [p for p in extract_root.iterdir() if p.is_dir() and list_topics(p)]
        if len(kids) == 1:
            return kids[0]
        raise FileNotFoundError(f"No ADAS topics found inside {path}")

    def close(self) -> None:
        if self._tmpdir is not None:
            self._tmpdir.cleanup()
            self._tmpdir = None

    def __enter__(self) -> "AndroidBagPlayer":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    @property
    def session_dir(self) -> Path:
        return self._session_dir

    @property
    def topics(self) -> List[str]:
        return list_topics(self._session_dir)

    def _msgs(self, topic: str) -> List[Tuple[int, Any, Any]]:
        if topic not in self._cache:
            self._cache[topic] = load_topic_messages(self._session_dir, topic)
        return self._cache[topic]

    def get_topic_msgs(self, topic_name: str) -> List[Tuple[int, Any]]:
        return [(ts, payload) for ts, payload, _ in self._msgs(topic_name)]

    def single_type_generator_with_ts(
        self,
        topic_name: str,
        start_time: Optional[int] = None,
        end_time: Optional[int] = None,
    ) -> Generator[Tuple[int, Any], None, None]:
        for ts, payload, _ in self._msgs(topic_name):
            if start_time is not None and ts < start_time:
                continue
            if end_time is not None and ts > end_time:
                break
            yield ts, payload

    def add_callback(
        self, topic_name: str, callback: Callable[[str, Any, int], None]
    ) -> None:
        self._callbacks[topic_name] = callback
        self._enabled_topics.add(topic_name)

    def play(
        self,
        start_time: Optional[int] = None,
        end_time: Optional[int] = None,
    ) -> None:
        if not self._enabled_topics:
            print("No callbacks registered. Use add_callback() first.")
            return

        gens: Dict[str, Generator[Tuple[int, Any], None, None]] = {}
        queue: List[Tuple[int, str, Any]] = []
        for topic in self._enabled_topics:
            gen = self.single_type_generator_with_ts(topic, start_time, end_time)
            try:
                ts, msg = next(gen)
                heapq.heappush(queue, (ts, topic, msg))
                gens[topic] = gen
            except StopIteration:
                continue

        while queue:
            ts, topic, msg = heapq.heappop(queue)
            cb = self._callbacks.get(topic)
            if cb:
                cb(topic, msg, ts)
            try:
                nts, nmsg = next(gens[topic])
                heapq.heappush(queue, (nts, topic, nmsg))
            except StopIteration:
                gens.pop(topic, None)

    def get_time_range(self) -> Tuple[int, int]:
        stamps: List[int] = []
        for topic in self.topics:
            for ts, _ in self.get_topic_msgs(topic):
                stamps.append(ts)
        if not stamps:
            return 0, 0
        return min(stamps), max(stamps)

    def get_duration(self) -> timedelta:
        start, end = self.get_time_range()
        return timedelta(milliseconds=max(0, end - start))

    def print_summary(self) -> None:
        print("\n=== Bag Session Summary ===")
        print(f"Path: {self._session_dir}")
        print(f"Topics: {len(self.topics)}")
        start, end = self.get_time_range()
        if start or end:
            print(f"Time range (ms): {start} … {end}")
            print(f"Duration: {self.get_duration()}")
        print("\n=== Topics ===")
        for topic in self.topics:
            print(
                f"  {topic}: {len(self.get_topic_msgs(topic))} msgs "
                f"({fix_topic(topic)}/)"
            )


def main() -> int:
    ap = argparse.ArgumentParser(description="Android ADAS bag player")
    ap.add_argument("bag", type=Path, help="Session dir or .zip/.tar.gz")
    ap.add_argument("--summary", action="store_true", help="Print summary")
    ap.add_argument("--topic", help="Dump messages for topic")
    ap.add_argument("--limit", type=int, default=20, help="Max messages to print")
    ap.add_argument("--extract-images", action="store_true", help="Dump camera JPEGs")
    args = ap.parse_args()

    with AndroidBagPlayer(args.bag) as player:
        if args.topic:
            if args.topic not in player.topics:
                print(f"Topic '{args.topic}' not found. Available: {player.topics}")
                return 1
            for i, (ts, msg) in enumerate(
                player.single_type_generator_with_ts(args.topic)
            ):
                print(f"{i}: t={ts} {type(msg).__name__}")
                print(f"  {msg}")
                if i + 1 >= args.limit:
                    break
            return 0

        if args.extract_images:
            topic = "sensors/camera/image"
            if topic not in player.topics:
                print("No sensors/camera/image in bag")
                return 1
            out = Path(args.bag).resolve()
            if out.is_file():
                out = out.parent / f"{out.stem}_images"
            else:
                out = out / "extracted_images"
            out.mkdir(parents=True, exist_ok=True)
            n = 0
            for ts, msg in player.single_type_generator_with_ts(topic):
                data = getattr(msg, "image_data", b"") or b""
                if not data:
                    continue
                (out / f"{ts}.jpg").write_bytes(data)
                n += 1
            print(f"Extracted {n} images → {out}")
            return 0

        player.print_summary()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
