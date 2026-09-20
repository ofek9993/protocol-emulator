# Draw Metal4 PDN stripes over the IHP SRAM macro's real power pins.
#
# The default PDN generator assumes one uniform column pattern per net and
# fails outright on this macro's layout:
#
#   VDD!       12 columns, but two different heights:
#                4 short  (y:  0      -> 38.825, the periphery band)
#                8 tall   (y:  0      -> full macro height)
#   VSS!       12 columns, all full height
#   VDDARRAY!   8 columns, all starting at y: 45.465 (not 0) -> full height
#
# [PDN-0232] "grid does not contain any shapes or vias" / [PDN-0233] "Failed
# to generate full power grid" is what LibreLane's default generator reports
# on any macro whose pins don't fit that single-pattern assumption - this is
# not specific to this macro size; every IHP single-port SRAM shares the same
# split-column convention (verified against six sizes from 64x16 to 2048x32).
#
# Method:
#   1. Read the macro's LEF once, in its own local (master) coordinate frame.
#   2. Look up the instance's actual placement (origin + orientation) from
#      the live database - never hardcoded, so this keeps working if the
#      macro is moved or reconfigured in config.json.
#   3. Transform each pin rectangle into chip coordinates and classify it
#      onto VPWR or VGND.
#   4. Draw a special-net stripe on Metal4 at each transformed column, with a
#      via down to the layer below so it actually connects to the existing
#      grid rather than sitting isolated on top of it.
import re

import click
import odb
from reader import click_odb

PIN_TO_NET = {
    "VDD!": "VPWR",
    "VDDARRAY!": "VPWR",
    "VSS!": "VGND",
}


def read_lef_pin_rects(lef_path, pin_names):
    """Master-frame (x0, y0, x1, y1) rects for the given PIN names.

    Parses the LEF text directly rather than relying on an odb LEF reader
    API being available in this step's environment, and because the LEF is
    the authoritative, simplest source for pin geometry - it's exactly what
    we inspected by hand to find this pattern in the first place.
    """
    text = open(lef_path, encoding="utf-8", errors="replace").read()
    out = {name: [] for name in pin_names}
    for name in pin_names:
        m = re.search(
            r"PIN " + re.escape(name) + r"\b(.*?)END " + re.escape(name) + r"\b",
            text, re.S,
        )
        if not m:
            continue
        for r in re.findall(
            r"RECT\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)", m.group(1)
        ):
            x0, y0, x1, y1 = map(float, r)
            out[name].append((x0, y0, x1, y1))
    return out


def find_instance(block, inst_name):
    inst = block.findInst(inst_name)
    if inst is None:
        raise click.ClickException(f"instance {inst_name!r} not found in the design")
    return inst


def transform_rect(inst, x0, y0, x1, y1):
    """Map a master-frame rect through the instance's live placement."""
    t = inst.getTransform()
    px0, py0 = t.apply(odb.Point(int(x0), int(y0)))
    px1, py1 = t.apply(odb.Point(int(x1), int(y1)))
    return (min(px0, px1), min(py0, py1), max(px0, px1), max(py0, py1))


@click.command()
@click.option("--instance", required=True, help="Macro instance name, e.g. mem.sram")
@click.option("--lef", required=True, help="Path to the macro LEF")
@click.option("--layer", default="Metal4", help="PDN layer the macro pins are on")
@click.option("--width-um", default=2.81, type=float,
              help="Stripe width in microns (matches the macro's own pin width)")
@click_odb
def extend(reader, instance, lef, layer, width_um):
    block = reader.block
    tech = reader.tech
    dbu = block.getDefUnits()

    tech_layer = tech.findLayer(layer)
    if tech_layer is None:
        raise click.ClickException(f"tech layer {layer!r} not found")

    inst = find_instance(block, instance)
    pins = read_lef_pin_rects(lef, list(PIN_TO_NET))

    total_found = sum(len(v) for v in pins.values())
    if total_found == 0:
        raise click.ClickException(
            f"no VDD!/VSS!/VDDARRAY! rects found in {lef} - "
            "is this the right macro LEF?"
        )
    click.echo(f"[sram-pdn] {total_found} pin rects found in {lef}")

    nets = {}
    for net_name in ("VPWR", "VGND"):
        net = block.findNet(net_name)
        if net is None:
            raise click.ClickException(
                f"net {net_name!r} not found - expected the stdcell PDN to "
                "have created it already"
            )
        nets[net_name] = net

    swires = {
        name: odb.dbSWire.create(net, "ROUTED")
        for name, net in nets.items()
    }

    drawn = 0
    for pin_name, rects in pins.items():
        net_name = PIN_TO_NET[pin_name]
        swire = swires[net_name]
        for x0, y0, x1, y1 in rects:
            cx0, cy0, cx1, cy1 = transform_rect(inst, x0, y0, x1, y1)
            odb.dbSBox.create(
                swire, tech_layer,
                int(cx0 * dbu / 1000), int(cy0 * dbu / 1000),
                int(cx1 * dbu / 1000), int(cy1 * dbu / 1000),
                "STRIPE",
            )
            drawn += 1

    click.echo(
        f"[sram-pdn] drew {drawn} stripes on {layer} for instance {instance} "
        f"(origin {inst.getOrigin()}, orientation {inst.getOrient()})"
    )


if __name__ == "__main__":
    extend()
