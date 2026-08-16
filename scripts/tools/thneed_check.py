#!/usr/bin/env python3
"""Run a recorded thneed on the local OpenCL and compare the output against a reference.

Why a separate run. A file built by `thneed_from_onnx.py` could until now be checked only on the
phone, and the phone answers badly: the model is recurrent, the runner feeds its output back into its
own input, and the "output mean" depends on how many frames have run. Such a signature cannot tell
"the file was built wrong" from "the state is different" — which is exactly what is needed, because a
wrongly built file does not crash: it runs and returns plausible numbers.

Here the same thing is done from scratch. The file is read the way the loader on the phone reads it:
objects are created in declaration order, images are built over their buffers, kernels are compiled
from sources and launched in order. The inputs stay zero — exactly what the generator computed its
reference on — so a discrepancy can mean only one thing: the file does not reproduce the model.

    python3 tools/thneed_check.py supercombo.thneed --ref supercombo.thneed.ref.npy

The reference is written by the generator itself. Without `--ref` the run is still useful: it checks
that every object is created and every kernel builds and launches — that the file is accepted at all.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import struct
import sys

CL_MEM_READ_WRITE = 1 << 0
CL_MEM_COPY_HOST_PTR = 1 << 5
CL_MEM_OBJECT_IMAGE2D = 0x10F1
CL_RGBA = 0x10B5
CL_HALF_FLOAT = 0x10DD
CL_FLOAT = 0x10DE
CL_DEVICE_TYPE_ALL = 0xFFFFFFFF
CL_PROGRAM_BUILD_LOG = 0x1183


def check(status: int, what: str) -> None:
    if status != 0:
        raise RuntimeError(f"{what}: OpenCL code {status}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("thneed", help="file to check")
    ap.add_argument("--ref", help="reference output (.npy) written by the generator")
    ap.add_argument(
        "--tolerance",
        type=float,
        default=2e-2,
        help="tolerated absolute difference; not zero, because the summation order differs on "
        "another device and the model computes in fp16",
    )
    args = ap.parse_args()

    import numpy as np
    from tinygrad.helpers import to_char_p_p
    from tinygrad.runtime.autogen import opencl as cl

    with open(args.thneed, "rb") as f:
        header = struct.unpack("<i", f.read(4))[0]
        payload = json.loads(f.read(header).decode("latin-1"))
        blobs = f.read()

    platforms = (cl.cl_platform_id * 8)()
    count = ctypes.c_uint32()
    check(cl.clGetPlatformIDs(8, platforms, ctypes.byref(count)), "clGetPlatformIDs")
    devices = (cl.cl_device_id * 8)()
    ndev = ctypes.c_uint32()
    check(
        cl.clGetDeviceIDs(
            platforms[0], CL_DEVICE_TYPE_ALL, 8, devices, ctypes.byref(ndev)
        ),
        "clGetDeviceIDs",
    )
    device = devices[0]
    err = ctypes.c_int32()
    # The null callback has to be built explicitly: ctypes will not coerce None to a function type.
    context = cl.clCreateContext(
        None,
        1,
        (cl.cl_device_id * 1)(device),
        cl.clCreateContext.argtypes[3](),
        None,
        ctypes.byref(err),
    )
    check(err.value, "clCreateContext")
    queue = cl.clCreateCommandQueue(context, device, 0, ctypes.byref(err))
    check(err.value, "clCreateCommandQueue")

    # Objects strictly in declaration order: an image refers to its buffer by `buffer_id`, and the
    # blobs sit in the file back to back in the same order the loadable objects are declared.
    real: dict[bytes, object] = {}
    offset = 0
    images = 0
    for obj in payload["objects"]:
        size = int(obj["size"])
        key = obj["id"].encode("latin-1")
        backing = obj.get("buffer_id", "")
        if backing:
            mem = real[backing.encode("latin-1")]
        elif obj.get("needs_load"):
            host = (ctypes.c_char * size).from_buffer_copy(blobs[offset : offset + size])
            offset += size
            mem = cl.clCreateBuffer(
                context,
                CL_MEM_COPY_HOST_PTR | CL_MEM_READ_WRITE,
                size,
                host,
                ctypes.byref(err),
            )
            check(err.value, f"clCreateBuffer {size} B")
        else:
            host = (ctypes.c_char * size)()
            mem = cl.clCreateBuffer(
                context,
                CL_MEM_COPY_HOST_PTR | CL_MEM_READ_WRITE,
                size,
                host,
                ctypes.byref(err),
            )
            check(err.value, f"clCreateBuffer {size} B")

        if obj["arg_type"] in ("image2d_t", "image1d_t"):
            desc = cl.cl_image_desc()
            ctypes.memset(ctypes.byref(desc), 0, ctypes.sizeof(desc))
            desc.image_type = CL_MEM_OBJECT_IMAGE2D
            desc.image_width = int(obj["width"])
            desc.image_height = int(obj["height"])
            desc.image_row_pitch = int(obj["row_pitch"])
            desc._0.buffer = mem
            fmt = cl.cl_image_format()
            fmt.image_channel_order = CL_RGBA
            fmt.image_channel_data_type = (
                CL_FLOAT if obj.get("float32") else CL_HALF_FLOAT
            )
            mem = cl.clCreateImage(
                context,
                CL_MEM_READ_WRITE,
                ctypes.byref(fmt),
                ctypes.byref(desc),
                None,
                ctypes.byref(err),
            )
            check(
                err.value,
                f'clCreateImage {obj["width"]}x{obj["height"]} pitch {obj["row_pitch"]}',
            )
            images += 1
        real[key] = mem

    if offset != len(blobs):
        print(
            f"blobs not fully consumed: {offset} of {len(blobs)} B — the declared sizes do not match "
            f"the file",
            file=sys.stderr,
        )
        return 1

    kernels: dict[str, object] = {}
    for name, source in payload["programs"].items():
        src = source.encode()
        program = cl.clCreateProgramWithSource(
            context,
            1,
            to_char_p_p([src]),
            (ctypes.c_size_t * 1)(len(src)),
            ctypes.byref(err),
        )
        check(err.value, f"clCreateProgramWithSource {name}")
        status = cl.clBuildProgram(
            program,
            1,
            (cl.cl_device_id * 1)(device),
            None,
            cl.clBuildProgram.argtypes[4](),
            None,
        )
        if status != 0:
            length = ctypes.c_size_t()
            cl.clGetProgramBuildInfo(
                program, device, CL_PROGRAM_BUILD_LOG, 0, None, ctypes.byref(length)
            )
            log = ctypes.create_string_buffer(length.value)
            cl.clGetProgramBuildInfo(
                program, device, CL_PROGRAM_BUILD_LOG, length.value, log, None
            )
            print(
                f"kernel {name} failed to build:\n{log.value.decode(errors='replace')}",
                file=sys.stderr,
            )
            return 1
        kernels[name] = cl.clCreateKernel(program, name.encode(), ctypes.byref(err))
        check(err.value, f"clCreateKernel {name}")

    for launch in payload["kernels"]:
        kernel = kernels[launch["name"]]
        for i, (arg, arg_size) in enumerate(zip(launch["args"], launch["args_size"])):
            if arg_size == 8:
                mem = real[arg.encode("latin-1")]
                check(
                    cl.clSetKernelArg(kernel, i, 8, ctypes.byref(mem)),
                    f'clSetKernelArg {launch["name"]} #{i}',
                )
            else:
                check(
                    cl.clSetKernelArg(kernel, i, arg_size, None),
                    f'clSetKernelArg {launch["name"]} #{i}',
                )
        gws = (ctypes.c_size_t * 3)(*launch["global_work_size"])
        lws = (ctypes.c_size_t * 3)(*launch["local_work_size"])
        check(
            cl.clEnqueueNDRangeKernel(
                queue, kernel, int(launch["work_dim"]), None, gws, lws, 0, None, None
            ),
            f'clEnqueueNDRangeKernel {launch["name"]}',
        )
    check(cl.clFinish(queue), "clFinish")

    out = payload["outputs"][0]
    size = int(out["size"])
    host = (ctypes.c_char * size)()
    check(
        cl.clEnqueueReadBuffer(
            queue,
            real[out["buffer_id"].encode("latin-1")],
            1,
            0,
            size,
            host,
            0,
            None,
            None,
        ),
        "clEnqueueReadBuffer",
    )
    result = np.frombuffer(bytes(host), dtype=np.float32)

    print(
        f'{len(payload["programs"])} kernels, {len(payload["kernels"])} launches, '
        f'{len(payload["objects"])} objects ({images} of them images) — all created and executed'
    )
    print(
        f"output: {result.size} values, mean {result.mean():.5f}, spread {result.std():.5f}"
    )
    print(f"first: {np.array2string(result[:8], precision=5)}")

    if not args.ref:
        print(
            "no reference given — nothing to compare against; only execution was verified"
        )
        return 0

    reference = np.load(args.ref).astype(np.float32).ravel()
    if reference.size != result.size:
        print(
            f"output length {result.size} against the reference's {reference.size}",
            file=sys.stderr,
        )
        return 1
    delta = np.abs(result - reference)
    worst = int(delta.argmax())
    print(
        f"difference from the reference: largest {delta.max():.6f} (index {worst}: "
        f"{result[worst]:.5f} against {reference[worst]:.5f}), mean {delta.mean():.6f}"
    )
    if delta.max() > args.tolerance:
        print("the file does not reproduce the model", file=sys.stderr)
        return 1
    print("the file reproduces the model")
    return 0


if __name__ == "__main__":
    sys.exit(main())
