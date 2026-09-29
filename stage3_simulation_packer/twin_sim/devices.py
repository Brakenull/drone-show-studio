"""OpenCL device discovery and selection for the digital twin.

Device specs (the `--device` option, `DigitalTwin(device=...)`):

* ``None`` / ``"auto"`` -- the first GPU, else the first device of any kind
* ``"gpu"`` / ``"cpu"`` -- the first device of that type (error if none)
* ``"opencl:P:D"``      -- device D of platform P, as listed by `opencl_devices()`

A CPU device needs a CPU OpenCL runtime (Intel CPU Runtime for OpenCL
Applications, or PoCL); GPU drivers from Intel, NVIDIA and AMD ship their own.
"""

from __future__ import annotations

from dataclasses import dataclass

import pyopencl as cl


class DeviceNotFoundError(RuntimeError):
    pass


@dataclass(frozen=True)
class DeviceInfo:
    id: str            # "opencl:P:D"
    name: str
    kind: str          # "gpu", "cpu" or "other"
    compute_units: int
    memory_mib: int

    @property
    def label(self) -> str:
        return f"{self.name} ({self.id})"


def _kind(device: cl.Device) -> str:
    if device.type & cl.device_type.GPU:
        return "gpu"
    if device.type & cl.device_type.CPU:
        return "cpu"
    return "other"


def _all() -> list[tuple[DeviceInfo, cl.Device]]:
    try:
        platforms = cl.get_platforms()
    except cl.Error:            # no ICD loader / no platform installed
        return []
    out = []
    for pi, platform in enumerate(platforms):
        try:
            devices = platform.get_devices()
        except cl.Error:
            continue
        for di, dev in enumerate(devices):
            out.append((DeviceInfo(f"opencl:{pi}:{di}", dev.name.strip(), _kind(dev), dev.max_compute_units,
                                   dev.global_mem_size // 2**20), dev))
    return out


def opencl_devices() -> list[DeviceInfo]:
    """Every OpenCL device, GPUs first (the order `"auto"` picks from)."""
    return sorted((info for info, _ in _all()), key=lambda i: i.kind != "gpu")


def select_device(spec: str | None = None) -> tuple[DeviceInfo, cl.Device]:
    found = _all()
    if not found:
        raise DeviceNotFoundError("no OpenCL device found: install a GPU driver or a CPU OpenCL runtime")
    if spec in (None, "", "auto"):
        return next(((i, d) for i, d in found if i.kind == "gpu"), found[0])
    if spec in ("gpu", "cpu"):
        match = next(((i, d) for i, d in found if i.kind == spec), None)
        if match is None:
            hint = (" (install Intel's CPU Runtime for OpenCL Applications or PoCL)" if spec == "cpu" else "")
            raise DeviceNotFoundError(f"no OpenCL {spec.upper()} device found{hint}; available: "
                                      + ", ".join(i.label for i, _ in found))
        return match
    match = next(((i, d) for i, d in found if i.id == spec), None)
    if match is None:
        raise DeviceNotFoundError(f"unknown device {spec!r}; available: " + ", ".join(i.label for i, _ in found))
    return match
