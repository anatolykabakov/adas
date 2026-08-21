#!/usr/bin/env python3
"""Widen an fp16 ONNX graph to fp32, losslessly.

Why this exists: openpilot ships supercombo in fp16, and **onnxruntime computes that graph wrongly on
ARM** — same file, same zero inputs, mean −7.6 / std 133.9 on the phone against −1.2455 / 3.2799 on a
desktop, with no error either way (`docs/THNEED.md`). So the fallback runner drives on an fp32 copy, and
this is the step that makes it. The GPU path does not need this: `thneed_from_onnx.py --half` converts
back to fp16 itself, where the arithmetic is done by our own kernels rather than by ORT.

Widening fp16 → fp32 loses nothing: every fp16 value is exactly representable in fp32. That is what makes
the result checkable — `--verify` compares against a reference file tensor by tensor and expects bit-for-bit
equality, not "close enough".

    cd scripts
    python3 -m tools.onnx_widen_fp32 ../models/supercombo_097_fp16.onnx ../models/supercombo_097_fp32.onnx
    python3 -m tools.onnx_widen_fp32 in.onnx out.onnx --verify ../app/src/main/assets/supercombo.onnx
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import numpy as np

import _path  # noqa: F401  (scripts/ on sys.path)

import onnx
from onnx import TensorProto, numpy_helper

FP16 = int(TensorProto.FLOAT16)
FP32 = int(TensorProto.FLOAT)


def widen(model: onnx.ModelProto) -> int:
    """fp16 → fp32 in place. Returns how many things were changed."""
    changed = 0

    # Weights.
    for init in model.graph.initializer:
        if int(init.data_type) != FP16:
            continue
        arr = numpy_helper.to_array(init).astype(np.float32)
        init.CopyFrom(numpy_helper.from_array(arr, init.name))
        changed += 1

    # Declared types of inputs, outputs and intermediate values.
    for group in (model.graph.input, model.graph.output, model.graph.value_info):
        for v in group:
            tt = v.type.tensor_type
            if int(tt.elem_type) == FP16:
                tt.elem_type = FP32
                changed += 1

    # Tensors and dtype arguments hidden inside node attributes: a Constant carrying fp16 data, or a Cast
    # asking for fp16. Missing these leaves a graph that mixes precisions and fails to load.
    for node in model.graph.node:
        for attr in node.attribute:
            if attr.type == onnx.AttributeProto.TENSOR and int(attr.t.data_type) == FP16:
                arr = numpy_helper.to_array(attr.t).astype(np.float32)
                attr.t.CopyFrom(numpy_helper.from_array(arr, attr.t.name))
                changed += 1
            elif attr.type == onnx.AttributeProto.TENSORS:
                for t in attr.tensors:
                    if int(t.data_type) == FP16:
                        arr = numpy_helper.to_array(t).astype(np.float32)
                        t.CopyFrom(numpy_helper.from_array(arr, t.name))
                        changed += 1
            elif node.op_type == "Cast" and attr.name == "to" and int(attr.i) == FP16:
                attr.i = FP32
                changed += 1

    return changed


def verify(produced: Path, reference: Path) -> bool:
    """Bit-for-bit comparison of every weight, plus the structure around them."""
    a = onnx.load(str(produced), load_external_data=False)
    b = onnx.load(str(reference), load_external_data=False)

    ok = True
    ops_a = [n.op_type for n in a.graph.node]
    ops_b = [n.op_type for n in b.graph.node]
    if ops_a != ops_b:
        print(f"  nodes differ: {len(ops_a)} vs {len(ops_b)}")
        ok = False

    wa = {i.name: i for i in a.graph.initializer}
    wb = {i.name: i for i in b.graph.initializer}
    if set(wa) != set(wb):
        print(
            f"  weight names differ: {len(wa)} vs {len(wb)}, "
            f"{len(set(wa) ^ set(wb))} not in both"
        )
        ok = False

    equal = 0
    for name in sorted(set(wa) & set(wb)):
        x = numpy_helper.to_array(wa[name])
        y = numpy_helper.to_array(wb[name])
        if x.shape == y.shape and np.array_equal(
            x.astype(np.float32), y.astype(np.float32)
        ):
            equal += 1
    print(f"  weights equal bit-for-bit: {equal} of {len(set(wa) & set(wb))}")
    if equal != len(set(wa) & set(wb)):
        ok = False

    md_a = {p.key: p.value for p in a.metadata_props}
    md_b = {p.key: p.value for p in b.metadata_props}
    for key in ("vision_model", "policy_model"):
        if md_a.get(key) != md_b.get(key):
            print(f"  {key} differs: {md_a.get(key)} vs {md_b.get(key)}")
            ok = False
        elif md_a.get(key):
            print(f"  {key} matches: {md_a[key]}")
    return ok


def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("src", type=Path, help="fp16 ONNX in")
    p.add_argument("dst", type=Path, help="fp32 ONNX out")
    p.add_argument(
        "--verify",
        type=Path,
        help="compare the result against this file, weight by weight",
    )
    a = p.parse_args(argv)

    if not a.src.is_file():
        raise SystemExit(f"not found: {a.src}")
    model = onnx.load(str(a.src))
    changed = widen(model)
    onnx.checker.check_model(model)
    a.dst.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(a.dst))
    print(
        f"{a.src.name} → {a.dst.name}: {changed} tensors and types widened, "
        f"{a.dst.stat().st_size / 1e6:.1f} MB"
    )

    if a.verify is not None:
        print(f"verify against {a.verify}:")
        if not verify(a.dst, a.verify):
            print("  DIFFERS — the produced file is not the same network")
            return 1
        print("  same network")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
