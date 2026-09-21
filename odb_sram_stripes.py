# Draw Metal4 PDN stripes over the IHP SRAM macro's real power columns.
#
# LibreLane's default pdn_cfg.tcl ends with a generic
#   define_pdn_grid -macro -default ...
# that grids every macro with one uniform pattern. This macro does not fit
# that pattern, so it produced PDN-0232/0233. src/pdn_cfg.tcl drops that
# block; this step fills the gap it leaves.
#
# The macro's power face, read out of its LEF (32 rects, verified):
#
#   VDD!       12 columns - 4 short (y 0 -> 38.825, the periphery band)
#                           8 tall  (y 0 -> 336.46)
#   VSS!       12 columns - all tall
#   VDDARRAY!   8 columns - all starting at y 45.465, not 0
#
# The short VDD! columns and the VDDARRAY! columns share x positions: VDD!
# feeds the periphery at the bottom, VDDARRAY! takes over above the break at
# y 38.825..45.465 and feeds the bit-cell array. Both are power, so one
# full-height stripe per column covers both.
#
# Method: parse the LEF in the macro's own (master) coordinates, look up
# where the instance is actually placed from the live database, translate,
# then draw one stripe per column. Placement is never hardcoded, so moving
# the macro in config.json needs no change here.
import re
from collections import defaultdict

import click
import odb
from reader import click_odb

# Macro-side pin name -> the design's PDN net.
PIN_TO_NET = {
    "VDD!": "VPWR",
    "VDDARRAY!": "VPWR",
    "VSS!": "VGND",
}


def read_lef_pin_rects(lef_path, pin_names):
    """Master-frame rects, in microns, for each named LEF PIN.

    The LEF text is parsed directly: it is the authoritative source for this
    geometry and needs no tool state. Note the pin names end in '!', which
    is not a word character - a \\b anchor after them can never match, so
    the block is delimited on whole lines instead.
    """
    text = open(lef_path, encoding="utf-8", errors="replace").read()
    out = {}
    for name in pin_names:
        m = re.search(
            r"^\s*PIN\s+" + re.escape(name) + r"\s*$(.*?)^\s*END\s+"
            + re.escape(name) + r"\s*$",
            text, re.S | re.M,
        )
        if not m:
            out[name] = []
            continue
        out[name] = [
            tuple(map(float, r))
            for r in re.findall(
                r"RECT\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*;",
                m.group(1),
            )
        ]
    return out


def instance_placement(block, inst_name):
    inst = block.findInst(inst_name)
    if inst is None:
        raise click.ClickException(
            f"instance {inst_name!r} not found - it must match a "
            "MACROS.<macro>.instances key in config.json"
        )
    origin = inst.getOrigin()
    orient = str(inst.getOrient())
    return inst, (int(origin[0]), int(origin[1])), orient


def make_to_chip(origin, orient, dbu):
    """Micron master coordinates -> database-unit chip coordinates.

    Only the unrotated orientations are handled, because that is what this
    design uses and an unverified rotation would silently place stripes in
    the wrong spot rather than fail.
    """
    if orient not in ("R0", "N"):
        raise click.ClickException(
            f"instance orientation {orient!r} is not supported by this step; "
            "only R0/N has been verified. Add and test the transform before "
            "using a rotated or mirrored placement."
        )
    ox, oy = origin

    def to_chip(x_um, y_um):
        return ox + int(round(x_um * dbu)), oy + int(round(y_um * dbu))

    return to_chip


@click.command()
@click.option("--instance", required=True, help="Macro instance, e.g. mem.sram")
@click.option("--lef", required=True, help="Path to the macro LEF")
@click.option("--layer", default="Metal4", help="Layer the macro's power pins are on")
@click.option("--extend-um", default=5.0, type=float,
              help="How far to extend each stripe past the macro, in microns, "
                   "so it reaches the standard-cell rails outside the footprint")
@click_odb
def extend(reader, instance, lef, layer, extend_um):
    block = reader.block
    dbu = block.getDefUnits()

    tech_layer = reader.tech.findLayer(layer)
    if tech_layer is None:
        raise click.ClickException(f"tech layer {layer!r} not found")

    inst, origin, orient = instance_placement(block, instance)
    to_chip = make_to_chip(origin, orient, dbu)

    pins = read_lef_pin_rects(lef, list(PIN_TO_NET))
    found = {k: len(v) for k, v in pins.items()}
    if not sum(found.values()):
        raise click.ClickException(
            f"no VDD!/VSS!/VDDARRAY! rects found in {lef} - wrong LEF?"
        )
    click.echo(f"[sram-pdn] LEF rects: {found}")
    click.echo(f"[sram-pdn] {instance} at {origin} dbu, orientation {orient}, "
               f"{dbu} dbu/um")

    # Collapse the rects into one stripe per column per net: a column's short
    # VDD! and the VDDARRAY! above it are both power and share an x position.
    columns = defaultdict(lambda: [None, None])  # (net, x0, x1) -> [ymin, ymax]
    for pin_name, rects in pins.items():
        net_name = PIN_TO_NET[pin_name]
        for x0, y0, x1, y1 in rects:
            key = (net_name, round(x0, 3), round(x1, 3))
            span = columns[key]
            span[0] = y0 if span[0] is None else min(span[0], y0)
            span[1] = y1 if span[1] is None else max(span[1], y1)

    nets = {}
    for net_name in sorted({n for n, _, _ in columns}):
        net = block.findNet(net_name)
        if net is None:
            raise click.ClickException(
                f"net {net_name!r} not found - the standard-cell PDN should "
                "have created it already"
            )
        nets[net_name] = odb.dbSWire.create(net, "ROUTED")

    drawn = defaultdict(int)
    for (net_name, x0, x1), (ymin, ymax) in sorted(columns.items()):
        cx0, cy0 = to_chip(x0, ymin - extend_um)
        cx1, cy1 = to_chip(x1, ymax + extend_um)
        odb.dbSBox.create(
            nets[net_name], tech_layer,
            min(cx0, cx1), min(cy0, cy1), max(cx0, cx1), max(cy0, cy1),
            "STRIPE",
        )
        drawn[net_name] += 1

    click.echo(
        "[sram-pdn] drew " +
        ", ".join(f"{n} {c} stripes" for n, c in sorted(drawn.items())) +
        f" on {layer}, each extended {extend_um}um past the macro"
    )


if __name__ == "__main__":
    extend()
