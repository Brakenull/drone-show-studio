"""LED color channel extraction (spec section: Color & Light Channel).

`linear_to_srgb8` is pure and unit-testable. The Blender-facing readers below
it need `bpy` and import it locally so this module still loads outside
Blender.
"""

from __future__ import annotations

from typing import Sequence, Tuple

BLACK_RGB8 = (0, 0, 0)


def _linear_to_srgb_component(c: float) -> float:
    c = max(0.0, min(1.0, c))
    if c <= 0.0031308:
        return c * 12.92
    return 1.055 * (c ** (1.0 / 2.4)) - 0.055


def linear_to_srgb8(rgb: Sequence[float]) -> Tuple[int, int, int]:
    """Convert a linear-light RGB triple (0-1 floats, Blender's native space)
    into an 8-bit sRGB triple suitable for the LED color channel."""
    return tuple(
        int(round(_linear_to_srgb_component(float(c)) * 255.0)) for c in rgb[:3]
    )  # type: ignore[return-value]


# --------------------------------------------------------------------------
# bpy-dependent readers (only usable inside Blender)
# --------------------------------------------------------------------------

def get_vertex_color(obj, loop_index: int, layer_name: str | None = None) -> Tuple[int, int, int]:
    """Read a per-loop vertex color attribute and convert it to sRGB8."""
    mesh = obj.data
    color_attrs = getattr(mesh, "color_attributes", None)
    if not color_attrs or len(color_attrs) == 0:
        return BLACK_RGB8

    layer = color_attrs.get(layer_name) if layer_name else color_attrs.active_color
    if layer is None:
        layer = color_attrs[0]

    data = layer.data
    if loop_index >= len(data):
        return BLACK_RGB8

    color = data[loop_index].color
    return linear_to_srgb8(tuple(color))


def get_material_base_color(obj, slot: int = 0) -> Tuple[int, int, int]:
    """Read the Principled BSDF 'Base Color' of a material slot and convert to sRGB8."""
    if not obj.material_slots or slot >= len(obj.material_slots):
        return BLACK_RGB8

    material = obj.material_slots[slot].material
    if material is None or not material.use_nodes:
        if material is not None:
            return linear_to_srgb8(tuple(material.diffuse_color)[:3])
        return BLACK_RGB8

    for node in material.node_tree.nodes:
        if node.type == "BSDF_PRINCIPLED":
            base_color_input = node.inputs.get("Base Color")
            if base_color_input is not None:
                return linear_to_srgb8(tuple(base_color_input.default_value)[:3])

    return BLACK_RGB8
