# Route the tile's Metal4 PDN through the IHP SRAM's legal supply columns.
#
# Why the obvious approach does not work
# --------------------------------------
# The macro declares only three Metal4 PINs (VDD!, VDDARRAY!, VSS!), and it
# is tempting to simply draw a stripe on each declared pin rectangle. That
# fails three ways at once, all of which this project hit:
#
#   * the pins are not where metal is *allowed* - the Metal4 OBS blocks most
#     of the face, and a stripe crossing an obstruction bar is an illegal
#     overlap;
#   * newly drawn stripes are electrically floating, because nothing
#     connects them down to the standard-cell rails;
#   * VDD! stops at y 38.825 while VDDARRAY! starts at 45.465, so a stripe
#     spanning both crosses the obstruction bar sitting in that break.
#
# What actually works
# -------------------
# The Metal4 OBS leaves a regular lattice of narrow corridors - for this
# macro, 24 of them, each exactly 3.33um wide, on a ~5.62um pitch. Those
# corridors are the legal supply columns. Every corridor of this macro
# contains exactly one declared pin, so each column's polarity is known
# directly (verified against the PDK LEF; larger IHP SRAMs have more
# corridors than pins and need polarity inferred by alternation instead).
#
# So rather than adding stripes, this step *relocates* them: the tile
# stripes that already cross the macro footprint are removed and redrawn on
# the nearest legal column, full core height, picking up Metal1<->Metal4 via
# stacks where they cross standard-cell rails outside the macro. The via
# masters are not constructed here - pdngen already built them for the
# standard-cell grid, so they are looked up by name and reused.
#
# Not every column needs feeding: the macro distributes internally, so a
# couple of VPWR/VGND pairs is enough.
#
# Method modelled on the approach used in WilliamZhang20/protocol-emulator
# and ihp-um-janestreet-prism; implementation written for this project.
import re

import click

try:  # so the geometry helpers can be unit-tested without OpenROAD present
    import odb
    from reader import click_odb
except ImportError:  # pragma: no cover
    odb = None
    click_odb = None

PIN_TO_NET = {"VDD!": "VPWR", "VDDARRAY!": "VPWR", "VSS!": "VGND"}
OTHER_NET = {"VPWR": "VGND", "VGND": "VPWR"}
RAIL_VIA_PREFIXES = ("via1_2_2100_440", "via2_3_2100_440", "via3_4_2100_440")

TALL_OBS_UM = 50.0      # an OBS rect taller than this defines a column edge
MAX_CORRIDOR_UM = 4.0   # wider than this is a region break, not a supply slot


# --------------------------------------------------------------- LEF parsing
def _pin_rects(text, name):
    m = re.search(
        r"^\s*PIN\s+" + re.escape(name) + r"\s*$(.*?)^\s*END\s+"
        + re.escape(name) + r"\s*$",
        text, re.S | re.M,
    )
    if not m:
        return []
    return [
        tuple(map(float, r))
        for r in re.findall(
            r"RECT\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*;",
            m.group(1),
        )
    ]


def legal_columns(lef_path, layer="Metal4"):
    """Legal supply columns in the macro's own frame, in microns.

    Returns [(x0, x1, net)] - the Metal4 OBS gaps, each tagged with the net
    of the declared pin inside it. Pin names end in '!', which is not a word
    character, so pin blocks are matched on whole lines rather than with a
    \\b anchor (a \\b after '!' can never match).
    """
    text = open(lef_path, encoding="utf-8", errors="replace").read()

    polarity = {}
    for pin_name, net in PIN_TO_NET.items():
        for x0, _y0, x1, _y1 in _pin_rects(text, pin_name):
            polarity[(round(x0, 3), round(x1, 3))] = net
    if not polarity:
        raise click.ClickException(
            f"{lef_path}: no VDD!/VDDARRAY!/VSS! pins found - wrong LEF?"
        )

    obs = re.search(r"LAYER %s SPACING.*?(?=\n\s*END)" % layer, text, re.S)
    if obs is None:
        raise click.ClickException(f"{lef_path}: no {layer} OBS block")
    bars = sorted({
        (float(x0), float(x1))
        for x0, y0, x1, y1 in re.findall(
            r"RECT\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)", obs.group(0))
        if float(y1) - float(y0) > TALL_OBS_UM
    })

    columns = []
    for (_a0, a1), (b0, _b1) in zip(bars, bars[1:]):
        if b0 - a1 <= 0 or b0 - a1 > MAX_CORRIDOR_UM:
            continue
        inside = [
            net for (p0, p1), net in polarity.items()
            if p0 >= a1 - 0.01 and p1 <= b0 + 0.01
        ]
        if len(inside) == 1:
            columns.append((round(a1, 3), round(b0, 3), inside[0]))
    if not columns:
        raise click.ClickException(
            f"{lef_path}: no legal {layer} corridors found between OBS bars"
        )
    return columns


# ------------------------------------------------------------- odb helpers
def _is_sram(master):
    return (master.findMTerm("VDD!") is not None
            and master.findMTerm("VSS!") is not None)


def _vertical(boxes, layer_name):
    out = []
    for b in boxes:
        tl = b.getTechLayer()
        if tl is not None and tl.getName() == layer_name \
                and (b.yMax() - b.yMin()) > (b.xMax() - b.xMin()):
            out.append(b)
    return out


def _centre(x0, x1):
    return (x0 + x1) // 2


def allocate(columns_dbu, crossing_centres, min_pairs, pair_gap):
    """Pick which legal columns to drive, per net.

    Each tile stripe crossing the macro claims the nearest free column of
    its own net; then each net is topped up until at least min_pairs
    VPWR/VGND pairs sit close enough together to behave like a supply pair.
    """
    chosen = {"VPWR": [], "VGND": []}

    for net in ("VPWR", "VGND"):
        free = [c for c in columns_dbu if c[2] == net]
        for tx in sorted(crossing_centres.get(net, [])):
            avail = [c for c in free if c not in chosen[net]]
            if not avail:
                break
            chosen[net].append(
                min(avail, key=lambda c: abs(_centre(c[0], c[1]) - tx)))

    def pair_count():
        n = 0
        for c in chosen["VPWR"]:
            cx = _centre(c[0], c[1])
            if any(abs(_centre(o[0], o[1]) - cx) <= pair_gap
                   for o in chosen["VGND"]):
                n += 1
        return n

    # Top up so each region of the macro is actually fed from both rails.
    guard = 0
    while pair_count() < min_pairs and guard < 64:
        guard += 1
        for net in ("VPWR", "VGND"):
            avail = [c for c in columns_dbu
                     if c[2] == net and c not in chosen[net]]
            if not avail:
                continue
            anchors = chosen[OTHER_NET[net]] or chosen[net]
            if anchors:
                ax = _centre(anchors[0][0], anchors[0][1])
                avail.sort(key=lambda c: abs(_centre(c[0], c[1]) - ax))
            chosen[net].append(avail[0])
    return chosen


# ------------------------------------------------------------------- main
def build(reader, lef, layer, min_pairs, pair_gap_um):
    block = reader.block
    dbu = block.getDefUnits()
    m4 = reader.tech.findLayer(layer)
    if m4 is None:
        raise click.ClickException(f"tech layer {layer!r} not found")
    core = block.getCoreArea()
    ylo, yhi = core.yMin(), core.yMax()
    pair_gap = int(pair_gap_um * dbu)

    rail_vias = []
    for prefix in RAIL_VIA_PREFIXES:
        hit = [v for v in block.getVias() if v.getName().startswith(prefix)]
        if hit:
            rail_vias.append(hit[0])
    if not rail_vias:
        raise click.ClickException(
            "no pdngen rail via masters found (expected names starting "
            + ", ".join(RAIL_VIA_PREFIXES)
            + ") - the standard-cell grid should have created them"
        )
    click.echo(f"[sram-pdn] reusing {len(rail_vias)} pdngen via masters: "
               + ", ".join(v.getName() for v in rail_vias))

    srams = [i for i in block.getInsts()
             if i.getMaster().isBlock() and _is_sram(i.getMaster())]
    if not srams:
        raise click.ClickException("no SRAM macro instance found in the design")

    master_cols = legal_columns(lef, layer)
    click.echo(f"[sram-pdn] {len(master_cols)} legal {layer} columns in the LEF "
               f"({sum(1 for c in master_cols if c[2]=='VPWR')} VPWR / "
               f"{sum(1 for c in master_cols if c[2]=='VGND')} VGND)")

    nets = {}
    for net_name in ("VPWR", "VGND"):
        net = block.findNet(net_name)
        if net is None:
            raise click.ClickException(f"net {net_name!r} not found")
        swires = list(net.getSWires())
        if not swires:
            raise click.ClickException(f"net {net_name!r} has no special wires")
        nets[net_name] = (net, swires[0])

    for inst in srams:
        ib = inst.getBBox()
        ox = ib.xMin()
        cols_dbu = [
            (ox + int(round(x0 * dbu)), ox + int(round(x1 * dbu)), net)
            for x0, x1, net in master_cols
        ]

        crossing_centres = {}
        crossing_boxes = {}
        for net_name, (_net, swire) in nets.items():
            boxes = [
                b for b in _vertical(swire.getWires(), layer)
                if b.xMax() > ib.xMin() and b.xMin() < ib.xMax()
            ]
            crossing_boxes[net_name] = boxes
            crossing_centres[net_name] = sorted(
                {_centre(b.xMin(), b.xMax()) for b in boxes})

        chosen = allocate(cols_dbu, crossing_centres, min_pairs, pair_gap)

        for net_name, (_net, swire) in nets.items():
            rails = [
                b for b in swire.getWires()
                if b.getTechLayer() is not None
                and b.getTechLayer().getName() == "Metal1"
            ]
            removed = 0
            for b in crossing_boxes[net_name]:
                odb.dbSBox_destroy(b)
                removed += 1

            placed = vias = 0
            for x0, x1, _net in chosen[net_name]:
                odb.dbSBox_create(swire, m4, x0, ylo, x1, yhi, "STRIPE")
                placed += 1
                cx = _centre(x0, x1)
                for r in rails:
                    # only outside the macro: no cell rows run underneath it
                    if r.xMin() <= cx <= r.xMax() and (
                            r.yMax() <= ib.yMin() or r.yMin() >= ib.yMax()):
                        ry = _centre(r.yMin(), r.yMax())
                        for via in rail_vias:
                            odb.dbSBox_create(swire, via, cx, ry, "STRIPE")
                        vias += 1
            click.echo(
                f"[sram-pdn] {inst.getName()} {net_name}: removed {removed} "
                f"crossing tile stripe(s), placed {placed} column stripe(s) "
                f"at x=" + ", ".join(
                    f"{_centre(c[0], c[1])/dbu:.2f}" for c in chosen[net_name])
                + f" um, {vias} rail via stack(s)"
            )


def _cli():
    """Built lazily so the geometry helpers stay importable without OpenROAD."""
    @click.command()
    @click.option("--lef", required=True, help="Path to the SRAM macro LEF")
    @click.option("--layer", default="Metal4", help="Vertical PDN layer")
    @click.option("--min-pairs", default=2, type=int,
                  help="Minimum VPWR/VGND column pairs to drive per macro; "
                       "the macro's internal mesh carries the rest")
    @click.option("--pair-gap-um", default=6.5, type=float,
                  help="Max centre spacing for two columns to count as a pair")
    @click_odb
    def run(reader, lef, layer, min_pairs, pair_gap_um):
        build(reader, lef, layer, min_pairs, pair_gap_um)

    return run


if __name__ == "__main__":
    _cli()()
