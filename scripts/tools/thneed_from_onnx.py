#!/usr/bin/env python3
"""ONNX to thneed in **sources**, so the model compiles on the phone itself.

Why. A `.thneed` is not a model but a recording of one run of the network on the GPU. The file in our
assets carries 105 **compiled** programs built for somebody else's Adreno: it works on our phone only
because the driver accepts them. On another GPU it may not be accepted at all.

When the file holds kernel **sources** instead of binaries, compilation happens at load time, on the
device that is going to run the model. Our loader has always been able to do that —
`cl_program_from_source` for a non-empty `programs` — it simply had nothing to feed it.

How it works. The model is run through tinygrad once, and along the way everything a thneed consists
of is captured: kernel sources, their launch order, grid sizes, buffer bindings and the contents of
the buffers that carry weights. The device doing the computing is whatever is at hand rather than an
Adreno — kernels are text and need not be born on the target GPU.

But the device needs `cl_khr_fp16`. Software pocl will not do: the model is fp16 and it refuses to
build kernels with `half`. Any desktop graphics card will, integrated Intel included.

    sudo apt install intel-opencl-icd
    pip install tinygrad onnx

    python3 tools/thneed_from_onnx.py ../app/src/main/assets/supercombo.onnx --half -o out.thneed
    python3 tools/thneed_check.py out.thneed --ref out.thneed.ref.npy

The second command is not to be skipped. A wrongly built thneed **does not crash**: it loads, runs and
returns plausible numbers — simply not the right ones. The check run executes the recorded file the
way the loader on the phone does and compares it against the reference written alongside.

Which ONNX to use. `supercombo.onnx` in the assets is 0.9.7 with 7 inputs and an output of 6504,
which is what the runner expects.

Details and measurements are in `docs/THNEED.md`.
"""

from __future__ import annotations

import argparse
import collections
import json
import re
import struct
import sys

ANSI = re.compile(r"\x1b\[[0-9;]*m")

# The input order our runner expects, and the sizes in bytes.
#
# The binding on the C++ side is **positional**, not by name: the JNI adds inputs in reverse order and
# `ThneedModel::execute` reverses the list before `copy_inputs`. So the file must declare exactly as
# many inputs in exactly this order, otherwise the i-th buffer goes to the wrong i-th one and
# `copy_inputs` copies somebody else's size — which is precisely how the 0.9.7 port crashed in memcpy
# on its first attempt.
#
# The 0.9.7 model has no `nav_*`: openpilot dropped the navigation inputs. Room for them is still
# needed or the positions shift, so they are declared as placeholders no kernel ever touches.
RUNNER_INPUT_ORDER = [
    ("features_buffer", 202752),
    ("nav_instructions", 600),
    ("nav_features", 1024),
    ("prev_desired_curv", 400),
    ("lateral_control_params", 8),
    ("traffic_convention", 8),
    ("desire", 3200),
    ("big_input_imgs", 1572864),
    ("input_imgs", 1572864),
]


# Row pitch alignment for images, in bytes.
#
# 512 is 64 pixels of RGBA half, exactly what both tested GPU families declare in
# `CL_DEVICE_IMAGE_PITCH_ALIGNMENT`. Adreno does not honour its own declaration and accepts 64 bytes —
# which is what stood here, taken from the stock flowpilot file. Mali does honour it: on a HONOR
# CRT-LX1 `clCreateImage` answered -39 for every image, the kernels computed over zeros, and the runner
# honestly refused to work. A value of 512 suits both, at the cost of padding on narrow images.
IMAGE_PITCH_ALIGN = 512


def image_geometry(buf) -> tuple[int, int, int, int] | None:
    """`(height, width, tight pitch, padded pitch)` for an image, or `None` for a plain buffer.

    tinygrad creates an image as a standalone OpenCL object, while the thneed format describes it as an
    image **over a buffer**, with an explicitly declared row pitch. So the geometry has to be
    reconstructed: RGBA, four elements per pixel, element size taken from the type (2 bytes for
    `imageh`).
    """
    image = getattr(getattr(buf, "options", None), "image", None)
    if image is None:
        return None
    height, width = int(image.shape[0]), int(image.shape[1])
    tight = width * 4 * int(image.itemsize)
    pitch = -(-tight // IMAGE_PITCH_ALIGN) * IMAGE_PITCH_ALIGN
    return height, width, tight, pitch


def prefer_wavefront(width: int) -> None:
    """Teach tinygrad to shape work groups for the target GPU's wavefront.

    tinygrad picks the group size with a heuristic whose candidate list is hardcoded:

        local_sz = next((x for x in ([32] * (axis == 0) + [16,8,4,3,2])
                         if k.full_shape[axis] % x == 0 and local_size * x <= 128), None)

    These numbers have nothing to do with the device, and on Adreno the outcome is poor: the most
    common group size comes out as 16 against a wavefront of 64 — three quarters of the lanes idle. In
    the stock file, recorded on an Adreno, the most common size is exactly 64.

    Fixing this later, in the recorded file, is **impossible**: the group size is baked into the kernel
    text itself. `E_1024_32_4_6` contains `alu0 = lidx0 + (gidx0 << 5)`, and the shift by 5 is the
    group of 32. Substitute the size at launch and the kernel computes over the wrong indices,
    silently and plausibly. Verified on the phone: the output mean moves from -1.5717 to -0.1659. So
    the place to intervene is here, before code generation.

    It is done by patching that very line in the heuristic's source. The trick is crude, but honester
    than the alternatives: rewriting the function wholesale means repeating its logic, and that logic
    changes from one tinygrad version to the next. If the line is not found, saying so out loud beats
    silently building as before.

    **Measured on an Adreno 640, and it did not help** — hence off by default. The argument about idle
    wavefront lanes sounds convincing but runs into occupancy: the larger the group, the fewer of them
    fit on a compute unit.

    | what was done | median |
    |---|---|
    | as tinygrad picks | **30.0 ms** |
    | 64 preferred, product ceiling unchanged (128) | 31.6 ms |
    | 64 preferred, ceiling raised to 256 | 36.3 ms |

    Left as a parameter: on a GPU with a different wavefront the answer may differ, and it takes one
    run to check. The output does not change either way — all three files gave the same mean of
    -1.5717.
    """
    import inspect
    import tinygrad.codegen.opt.heuristic as heuristic

    source = inspect.getsource(heuristic.hand_coded_optimizations)
    old = "([32] * (axis == 0) + [16,8,4,3,2])"
    new = f"([{width}, 32] * (axis == 0) + [16,8,4,3,2])"
    if old not in source:
        print(
            f"tinygrad's group-size heuristic has changed — kernels are shaped as they come, "
            f"ignoring the wavefront of {width}",
            file=sys.stderr,
        )
        return
    patched = source.replace(old, new)
    namespace = dict(vars(heuristic))
    exec(compile(patched, "<heuristic>", "exec"), namespace)  # noqa: S102
    heuristic.hand_coded_optimizations = namespace["hand_coded_optimizations"]

    # It is called by the name imported into `codegen.opt`, so that is where the patch must land, or
    # the change stays in a module nobody reads from.
    import tinygrad.codegen.opt as opt_module

    opt_module.hand_coded_optimizations = namespace["hand_coded_optimizations"]
    print(f"work groups shaped for a wavefront of {width}")


def to_half(path: str) -> str:
    """Convert the model to half precision and return the path to the copy.

    Why this is here. The assets hold the **fp32** variant, and not by oversight: onnxruntime computes
    an fp16 model wrongly on ARM — on zero inputs the phone gives a mean of -7.6 with a spread of 134
    instead of -1.25 and 3.3, identically on the CPU and on NNAPI, while the same model and the same
    zeros on a desktop ORT give the right answer. The fallback ONNX path therefore has to be fp32.

    The thneed, on the contrary, has to be fp16: half precision means half the weight bytes on the bus
    and nearly a twofold difference in time on Adreno. So the conversion happens here, along the way.

    Both the initialisers and the tensors inside node attributes are converted — the latter are easy to
    forget, and without them the model stays half in fp32 and will not build.
    """
    import tempfile

    import numpy as np
    import onnx
    from onnx import numpy_helper

    model = onnx.load(path)

    def cast(tensor) -> bool:
        if tensor.data_type != onnx.TensorProto.FLOAT:
            return False
        values = numpy_helper.to_array(tensor).astype(np.float16)
        tensor.CopyFrom(numpy_helper.from_array(values, tensor.name))
        return True

    converted = sum(int(cast(t)) for t in model.graph.initializer)
    inner = 0
    for node in model.graph.node:
        for attribute in node.attribute:
            if attribute.HasField("t"):
                inner += int(cast(attribute.t))
            for tensor in attribute.tensors:
                inner += int(cast(tensor))

    for value in list(model.graph.input) + list(model.graph.output):
        if value.type.tensor_type.elem_type == onnx.TensorProto.FLOAT:
            value.type.tensor_type.elem_type = onnx.TensorProto.FLOAT16
    for value in model.graph.value_info:
        if value.type.tensor_type.elem_type == onnx.TensorProto.FLOAT:
            value.type.tensor_type.elem_type = onnx.TensorProto.FLOAT16

    out = tempfile.NamedTemporaryFile(suffix=".fp16.onnx", delete=False).name
    onnx.save(model, out)
    print(
        f"converted to fp16: {converted} weights and {inner} tensors inside attributes -> {out}"
    )
    return out


def buffer_key(index: int) -> str:
    """An eight-byte buffer identifier, the way the format stores them.

    The loader reads `id` as raw `cl_mem` bytes and builds a mapping from "what was written" to "what
    was created on the device". The values are arbitrary and only uniqueness matters, so they are
    numbered consecutively from one: zero means NULL in this format.
    """
    return struct.pack("<Q", index + 1).decode("latin-1")


def main() -> int:
    global IMAGE_PITCH_ALIGN

    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("onnx", help="input model")
    ap.add_argument("-o", "--out", required=True, help="output .thneed file")
    ap.add_argument(
        "--device", default="GPU", help="tinygrad backend; OpenCL is required"
    )
    ap.add_argument(
        "--half",
        action="store_true",
        help="convert the model to fp16 before capturing; needed when the input is the fp32 variant",
    )
    ap.add_argument(
        "--no-images",
        action="store_true",
        help="compute through plain buffers instead of images; slower but easier to debug",
    )
    ap.add_argument(
        "--pitch-align",
        type=int,
        default=IMAGE_PITCH_ALIGN,
        help="image row pitch alignment in bytes; ask the target GPU for "
        "CL_DEVICE_IMAGE_PITCH_ALIGNMENT in pixels and multiply by 8",
    )
    ap.add_argument(
        "--wavefront",
        type=int,
        default=0,
        help="shape work groups for the target GPU wavefront; tested on Adreno 640 and it did "
        "not help, so off by default — see prefer_wavefront()",
    )
    args = ap.parse_args()
    IMAGE_PITCH_ALIGN = args.pitch_align

    import os

    if not args.no_images:
        # Convolutions and matrix products through `image2d_t`. The stock file is built the same way:
        # 300 of its 511 objects are images. On a mobile GPU this is not decoration: sampling goes
        # through the texture cache and an RGBA pixel is read by one instruction instead of four.
        os.environ.setdefault("IMAGE", "2")
        os.environ.setdefault("FLOAT16", "1")

    import numpy as np
    import onnx
    from tinygrad import Device, Tensor, dtypes
    import tinygrad.engine.realize as realize

    if args.wavefront:
        prefer_wavefront(args.wavefront)

    if "OpenCL" not in type(Device[args.device].renderer).__name__:
        print(
            f"device {args.device} does not render to OpenCL "
            f"({type(Device[args.device].renderer).__name__}); install any OpenCL ICD",
            file=sys.stderr,
        )
        return 2

    onnx_path = args.onnx
    if args.half:
        onnx_path = to_half(args.onnx)

    model = onnx.load(onnx_path)
    shapes = {
        i.name: [d.dim_value or 1 for d in i.type.tensor_type.shape.dim]
        for i in model.graph.input
    }
    half_inputs = {
        i.name
        for i in model.graph.input
        if i.type.tensor_type.elem_type == onnx.TensorProto.FLOAT16
    }
    print(f"model inputs: { {k: v for k, v in shapes.items()} }")
    if half_inputs:
        print(
            "model is fp16; cast at the boundary: the runner feeds and takes fp32, compute is fp16"
        )

    sources: dict[str, str] = {}
    launches: list[dict] = []
    buffers: dict[int, object] = {}

    original_init = realize.CompiledRunner.__init__
    original_call = realize.CompiledRunner.__call__

    def capture_init(self, p, precompiled=None):
        # tinygrad prints names with ANSI colouring; in the file that would be garbage.
        sources[ANSI.sub("", p.name)] = p.src
        return original_init(self, p, precompiled)

    def capture_call(self, rawbufs, var_vals=None, wait=False):
        p = self.p
        for b in rawbufs:
            buffers[id(b)] = b
        launches.append(
            {
                "name": ANSI.sub("", p.name),
                "global": list(p.global_size or [1, 1, 1]),
                "local": list(p.local_size or [1, 1, 1]),
                "bufs": [id(b) for b in rawbufs],
            }
        )
        return original_call(self, rawbufs, var_vals, wait)

    from tinygrad.frontend.onnx import OnnxRunner

    runner = OnnxRunner(onnx_path)

    # Materialise the inputs up front and remember their buffers: the loader needs to know where to
    # put the frame, and without that the file builds but there is nothing to feed it.
    #
    # **Before** the capture, deliberately. `.contiguous().realize()` is itself a copy kernel, and had
    # it landed in the recording the input would look like a buffer the graph computes on its own, with
    # all that follows: constant folding would take it for a constant and collapse the whole net
    # behind it.
    #
    # Always fp32, even when the net computes in fp16. The stock file is built the same way: its
    # inputs are exactly twice what an fp16 model needs and its output is 6504x4. The cast becomes an
    # ordinary kernel at the start and the end of the graph, and the Java-side runner never has to know
    # what precision the net computes in.
    input_tensors = {
        k: Tensor(np.zeros(v, dtype=np.float32)).contiguous().realize()
        for k, v in shapes.items()
    }
    input_buffers = {k: t.uop.buffer for k, t in input_tensors.items()}

    realize.CompiledRunner.__init__ = capture_init
    realize.CompiledRunner.__call__ = capture_call

    feed = {
        k: (t.cast(dtypes.half) if k in half_inputs else t)
        for k, t in input_tensors.items()
    }
    out = runner(feed)
    tensor = out if isinstance(out, Tensor) else list(out.values())[0]
    tensor = tensor.cast(dtypes.float).contiguous().realize()
    output_buffer = tensor.uop.buffer
    reference = tensor.numpy()

    realize.CompiledRunner.__init__ = original_init
    realize.CompiledRunner.__call__ = original_call

    print(
        f"captured: {len(sources)} kernels, {len(launches)} launches, {len(buffers)} buffers"
    )
    if not launches:
        print("not a single launch — the model was never evaluated", file=sys.stderr)
        return 1

    # The input and output buffers must reach the object table even if no captured kernel touched
    # them.
    for b in list(input_buffers.values()) + [output_buffer]:
        buffers.setdefault(id(b), b)
    input_ids = {id(b) for b in input_buffers.values()}

    # A buffer that comes from nowhere inside the graph is a weight or a constant. In tinygrad the
    # first launch argument is always the destination, so "from nowhere" means "was never first.
    #
    # By that test an input looks like a constant too, but it is not: its contents will come from the
    # camera. Exclude them explicitly, or 3.4 MB of zeros would travel into the file.
    produced = {launch["bufs"][0] for launch in launches}
    constants = {b for b in buffers if b not in produced and b not in input_ids}

    # Constant folding. `image_conv2d` moves the weights into images **every time**, and in the
    # recording that is 319 kernels out of 340 elementwise ones, over six million work items per frame.
    # Their result is the same from frame to frame because it depends only on the weights — so it is
    # enough to compute it here and put it into the file ready-made. The stock file has exactly 21
    # elementwise kernels: the ones touching frame data. We end up with the same number.
    #
    # Only what is written exactly once may be folded: tinygrad's allocator reuses buffers, and for
    # anything written twice the contents at the end of the run are no longer what that kernel
    # produced.
    written_once = collections.Counter(launch["bufs"][0] for launch in launches)
    kept: list[dict] = []
    for launch in launches:
        target = launch["bufs"][0]
        if (
            target not in input_ids
            and written_once[target] == 1
            and all(b in constants for b in launch["bufs"][1:])
        ):
            constants.add(target)
            continue
        kept.append(launch)
    folded = len(launches) - len(kept)
    launches = kept

    # After folding some buffers are not needed at all: they were intermediates only for weight prep.
    used = (
        {b for launch in launches for b in launch["bufs"]}
        | input_ids
        | {id(output_buffer)}
    )
    buffers = {k: v for k, v in buffers.items() if k in used}
    needs_load = {b: True for b in used if b in constants}
    if folded:
        print(f"weight-prep kernels folded: {folded}; launches left {len(launches)}")

    # An input no remaining kernel reads means a model computing past the frame. From outside that is
    # indistinguishable from healthy operation: the runner returns plausible numbers, only always the
    # same ones. Checked here because later there is nothing to check it with.
    read_by_kernels = {b for launch in launches for b in launch["bufs"][1:]}
    deaf = [name for name, buf in input_buffers.items() if id(buf) not in read_by_kernels]
    if deaf:
        print(
            f"inputs no kernel reads: {', '.join(deaf)} — "
            f"such a file would compute past the frame",
            file=sys.stderr,
        )
        return 1

    order = {buf_id: index for index, buf_id in enumerate(buffers)}
    next_index = len(order)
    blobs: list[bytes] = []
    objects = []
    images = 0
    for buf_id, buf in buffers.items():
        key = buffer_key(order[buf_id])
        load_it = bool(needs_load.get(buf_id, False))
        geometry = image_geometry(buf)

        if geometry is None:
            objects.append(
                {
                    "id": key,
                    "arg_type": "float*",
                    "size": int(getattr(buf, "nbytes", getattr(buf, "size", 0) * 4)),
                    "needs_load": load_it,
                }
            )
            if load_it:
                try:
                    blobs.append(bytes(buf.as_buffer()))
                except Exception as e:  # noqa: BLE001
                    print(
                        f"buffer {buf_id} cannot be read ({e}) — the file would be incomplete",
                        file=sys.stderr,
                    )
                    return 1
            continue

        # An image. The format describes it as a pair: first a plain buffer with the bytes, then the
        # image over it. That is how the loader reads it — it finds `buffer_id` among the objects
        # already created and builds `clCreateImage` on that backing, so the buffer must come first.
        height, width, tight, pitch = geometry
        backing = buffer_key(next_index)
        next_index += 1
        objects.append(
            {
                "id": backing,
                "arg_type": "<image buffer>",
                "size": height * pitch,
                "needs_load": load_it,
            }
        )
        if load_it:
            try:
                raw = bytes(buf.as_buffer())
            except Exception as e:  # noqa: BLE001
                print(
                    f"image {buf_id} cannot be read ({e}) — the file would be incomplete",
                    file=sys.stderr,
                )
                return 1
            # It always reads back tightly packed, but in the file it must lie at the declared row
            # pitch: the loader hands the blob to the driver as is, and without padding the rows would
            # run into one another.
            if pitch != tight:
                raw = b"".join(
                    raw[r * tight : (r + 1) * tight].ljust(pitch, b"\x00")
                    for r in range(height)
                )
            blobs.append(raw)
        objects.append(
            {
                "id": key,
                "arg_type": "image2d_t",
                "buffer_id": backing,
                "width": width,
                "height": height,
                "row_pitch": pitch,
                "size": height * pitch,
                "needs_load": False,
            }
        )
        images += 1

    if images:
        print(f"images: {images} of {len(buffers)} objects")

    kernels = []
    for launch in launches:
        bufs = launch["bufs"]
        kernels.append(
            {
                "name": launch["name"],
                "work_dim": 3,
                "global_work_size": [
                    g * l for g, l in zip(launch["global"], launch["local"])
                ],
                "local_work_size": launch["local"],
                "num_args": len(bufs),
                "args": [buffer_key(order[b]) for b in bufs],
                "args_size": [8] * len(bufs),
            }
        )

    # Inputs strictly in RUNNER_INPUT_ORDER. Whatever the model lacks is declared as a placeholder of
    # the right size: position matters more than contents, and a zero buffer is harmless.
    inputs_json = []
    for name, expected in RUNNER_INPUT_ORDER:
        buf = input_buffers.get(name)
        if buf is not None:
            size = int(getattr(buf, "nbytes", 0))
            if size != expected:
                print(
                    f"input {name}: the model gives {size} B, the runner expects {expected} B — positions will diverge",
                    file=sys.stderr,
                )
                return 1
            inputs_json.append(
                {"name": name, "size": size, "buffer_id": buffer_key(order[id(buf)])}
            )
        else:
            key = buffer_key(next_index)
            objects.append(
                {"id": key, "arg_type": "float*", "size": expected, "needs_load": False}
            )
            inputs_json.append({"name": name, "size": expected, "buffer_id": key})
            next_index += 1
            print(
                f"input {name}: absent from the model, declared as a {expected} B placeholder"
            )

    # Only the sources still launched: the phone would compile folded kernels for nothing, and
    # building 94 kernels on Adreno costs a hundred milliseconds at startup.
    live = {k["name"] for k in kernels}
    programs = {name: src for name, src in sources.items() if name in live}

    payload = {
        "kernels": kernels,
        "objects": objects,
        "programs": programs,
        "binaries": [],
        "inputs": inputs_json,
        "outputs": [
            {
                "size": int(getattr(output_buffer, "nbytes", 0)),
                "buffer_id": buffer_key(order[id(output_buffer)]),
            }
        ],
    }
    blob = json.dumps(payload).encode("utf-8")

    with open(args.out, "wb") as f:
        f.write(struct.pack("<i", len(blob)))
        f.write(blob)
        for b in blobs:
            f.write(b)

    total = 4 + len(blob) + sum(len(b) for b in blobs)
    print(
        f"written {args.out}: {len(programs)} kernels, {len(kernels)} launches, "
        f"header {len(blob)} B, {len(blobs)} blobs, {total/1e6:.1f} MB total"
    )
    # The reference is what the model computed here on zero inputs. With it `tools/thneed_check.py`
    # executes the recorded file and answers the question there is otherwise nobody to ask: does it
    # reproduce the model. On the phone that question cannot be asked — the model is recurrent there
    # and the output depends on how many frames have already passed.
    np.save(args.out + ".ref.npy", reference)
    print(f"output reference written: {args.out}.ref.npy")
    print(f"verify: python3 tools/thneed_check.py {args.out} --ref {args.out}.ref.npy")
    return 0


if __name__ == "__main__":
    sys.exit(main())
