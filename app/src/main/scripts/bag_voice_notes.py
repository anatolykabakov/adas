#!/usr/bin/env python3
"""Голосовые пометки водителя из звука заезда, со временем в шкале бага.

Звук лежит рядом с багом как `audio_<t>.m4a`, где t — момент старта записи по тому же монотонному
счётчику, в котором стоят метки всех сообщений:

    метка_в_баге = t_из_имени_файла + позиция_в_файле_мс

Ключевое слово (по умолчанию «запиши») отделяет пометку от разговора и радио.

    python3 bag_voice_notes.py adas_logs/2026_08_08_23_00_28
    python3 bag_voice_notes.py <bag> --model medium --all
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import List, Optional, Tuple

AUDIO_RE = re.compile(r"audio_(\d+)\.m4a$")

# «запиши», «записать», «запись» — распознаватель часто слышит соседнюю форму, и терять пометку
# из-за окончания было бы обиднее, чем поймать лишнюю фразу.
DEFAULT_KEYWORDS = ("запиш", "запис")


def find_audio(bag: Path) -> Tuple[Path, int]:
    hits = sorted(bag.glob("audio_*.m4a"))
    if not hits:
        raise SystemExit(f"нет звука в {bag}")
    if len(hits) > 1:
        print(f"# в каталоге {len(hits)} файлов звука, беру самый длинный")
        hits.sort(key=lambda p: p.stat().st_size, reverse=True)
    m = AUDIO_RE.search(hits[0].name)
    if not m:
        raise SystemExit(f"не разобрал метку времени из имени {hits[0].name}")
    return hits[0], int(m.group(1))


class _W:
    """Слово из кэша — тот же интерфейс, что у faster_whisper."""

    __slots__ = ("start", "end", "word")

    def __init__(self, start: float, end: float, word: str):
        self.start, self.end, self.word = start, end, word


class _S:
    __slots__ = ("start", "end", "text", "words")

    def __init__(self, start: float, end: float, text: str, words):
        self.start, self.end, self.text, self.words = start, end, text, words


def _cache_path(audio: Path, model_name: str, vad: bool) -> Path:
    return audio.with_suffix(f".{model_name}{'.vad' if vad else ''}.json")


def transcribe(
    path: Path,
    model_name: str,
    language: str,
    threads: int,
    vad: bool,
    vad_threshold: float,
):
    """Распознавание с кэшем рядом со звуком: medium без VAD считает 22-минутный заезд десятками
    минут, а меняется обычно разбор, а не распознавание."""
    cache = _cache_path(path, model_name, vad)
    if cache.exists():
        raw = json.loads(cache.read_text(encoding="utf-8"))
        segs = [
            _S(
                s["start"],
                s["end"],
                s["text"],
                [_W(w["start"], w["end"], w["word"]) for w in s["words"]],
            )
            for s in raw["segments"]
        ]
        print(f"# транскрипт из кэша {cache.name}")
        return segs, type("I", (), {"duration": raw["duration"]})()

    from faster_whisper import WhisperModel

    model = WhisperModel(
        model_name, device="cpu", compute_type="int8", cpu_threads=threads
    )
    segments, info = model.transcribe(
        str(path),
        language=language,
        # VAD по умолчанию выключен, и это следствие измерения, а не осторожность: на заезде
        # 2026_08_08_23_00_28 он оставил ДВА сегмента на 22 минуты и склеил разнесённые пометки в
        # одну. Речь в машине тише дорожного шума, детектор считает её тишиной. Без VAD дороже по
        # времени, зато ничего не теряется.
        vad_filter=vad,
        vad_parameters={"min_silence_duration_ms": 700, "threshold": vad_threshold}
        if vad
        else None,
        beam_size=5,
        condition_on_previous_text=False,
        # Пословные метки — то, ради чего всё: пометка должна попадать на свою секунду, а не на
        # начало сегмента, внутри которого она где-то есть.
        word_timestamps=True,
    )
    segs = list(segments)
    cache.write_text(
        json.dumps(
            {
                "duration": info.duration,
                "segments": [
                    {
                        "start": sg.start,
                        "end": sg.end,
                        "text": sg.text,
                        "words": [
                            {"start": w.start, "end": w.end, "word": w.word}
                            for w in (sg.words or [])
                        ],
                    }
                    for sg in segs
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return segs, info


def group_notes(
    segments, keywords: Tuple[str, ...], join_gap_s: float
) -> List[Tuple[float, float, str]]:
    """От каждого ключевого слова — речь до паузы больше join_gap_s или до следующего ключевого.

    Работа идёт по СЛОВАМ, а не по сегментам: сегмент распознавателя может тянуться десятки секунд и
    содержать несколько пометок, и тогда все они получили бы одно время — время начала сегмента.
    """
    words = []
    for seg in segments:
        for w in seg.words or []:
            words.append(w)
    if not words:
        return []

    starts = [
        i
        for i, w in enumerate(words)
        if any(k in w.word.strip().lower() for k in keywords)
    ]
    notes: List[Tuple[float, float, str]] = []
    for n, i in enumerate(starts):
        stop = starts[n + 1] if n + 1 < len(starts) else len(words)
        parts = [words[i].word.strip()]
        end = words[i].end
        j = i + 1
        while j < stop and (words[j].start - end) <= join_gap_s:
            parts.append(words[j].word.strip())
            end = words[j].end
            j += 1
        notes.append((words[i].start, end, " ".join(parts)))
    return notes


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", type=Path)
    ap.add_argument(
        "--model",
        default="small",
        help="small хватает для коротких фраз; medium точнее и втрое дольше",
    )
    ap.add_argument("--language", default="ru")
    ap.add_argument("--threads", type=int, default=8)
    ap.add_argument("--keyword", action="append", default=None, help="можно повторять")
    ap.add_argument(
        "--join-gap",
        type=float,
        default=2.0,
        help="пауза, до которой речь считается одной пометкой",
    )
    ap.add_argument(
        "--all", action="store_true", help="печатать весь транскрипт, а не только пометки"
    )
    ap.add_argument(
        "--vad",
        action="store_true",
        help="включить детектор речи — быстрее, но теряет тихую речь",
    )
    ap.add_argument("--vad-threshold", type=float, default=0.25)
    args = ap.parse_args()

    audio, t0_ms = find_audio(args.bag)
    keywords = tuple(k.lower() for k in (args.keyword or DEFAULT_KEYWORDS))

    print(f"# {audio.name}, старт записи t={t0_ms} мс (шкала бага)")
    segments, info = transcribe(
        audio, args.model, args.language, args.threads, args.vad, args.vad_threshold
    )
    print(
        f"# распознано сегментов: {len(segments)}, длительность {info.duration:.0f} с\n"
    )

    if args.all:
        for s in segments:
            print(f"[{t0_ms + int(s.start * 1000)}] {s.start:7.1f}s  {s.text.strip()}")
        print()

    notes = group_notes(segments, keywords, args.join_gap)
    if not notes:
        print("пометок с ключевым словом не найдено — попробуй --all или другую --model")
        return

    print(f"=== пометок: {len(notes)} ===\n")
    for n, (start, end, text) in enumerate(notes, 1):
        ts = t0_ms + int(start * 1000)
        mm, ss = divmod(int(start), 60)
        print(f"{n:2}. {mm:02d}:{ss:02d}  метка бага {ts}")
        print(f"    {text}\n")


if __name__ == "__main__":
    main()
