"""Pin labels on the bridge's silkscreen for the two modules, so a fitted module can be checked
against the board pin by pin: the WeAct board in its own style (two label lines per column between
the header rows, the upper line naming the upper row) and the LSM6DSV16X breakout with its own pin
names (the SPI half of AD0/MISO, SDA/MOSI, SCL/SCLK), written vertically beside each pin. The labels lie under the modules once they are fitted.

Used by gen_bridge_pcb.py; run directly to add them to an existing board (VM, repo root):
    python3 scripts/kicad/module_labels.py hardware/bridge/bridge.kicad_pcb
"""
import re, sys
import pcbnew

SIZE, THICK = 0.8, 0.12
# the breakout's own names; for its dual-purpose pins the SPI half, as the bridge uses it
IMU_NAMES = {"1": "VCC", "2": "GND", "3": "MISO", "4": "MOSI", "5": "SCLK", "6": "CS",
             "7": "INT1", "8": "SDX", "9": "SCX", "10": "INT2"}

def short(name):
    """WeAct's own silkscreen spelling: B13 for PB13, B8 for PB8/BOOT0, V+ for VREF+, RST for NRST."""
    name = name.split("/")[0]
    if re.fullmatch(r"P[A-F]\d+", name):
        return name[1:]
    return {"VREF+": "V+", "NRST": "RST"}.get(name, name)

def add_text(board, text, pos, rot=0):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text); t.SetPosition(pos); t.SetLayer(pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(SIZE), pcbnew.FromMM(SIZE)))
    t.SetTextThickness(pcbnew.FromMM(THICK)); t.SetTextAngleDegrees(rot)
    board.Add(t)
    return t

def label_modules(board, weact="U2", imu="U3"):
    mm = pcbnew.FromMM
    # WeAct: pads grouped by header (above or below the module centre), then by column
    fp = board.FindFootprintByReference(weact)
    cy = fp.GetPosition().y
    for top in (True, False):
        cols = {}
        for p in fp.Pads():
            if (p.GetPosition().y < cy) == top:
                cols.setdefault(p.GetPosition().x, []).append(p)
        for x, ps in cols.items():
            upper, lower = sorted(ps, key=lambda p: p.GetPosition().y)
            if top:      # labels below the header's lower (inner) row
                y1 = lower.GetPosition().y + mm(1.75)
            else:        # labels above the header's upper (inner) row
                y1 = upper.GetPosition().y - mm(2.75)
            add_text(board, short(upper.GetPinFunction()), pcbnew.VECTOR2I(x, y1))
            add_text(board, short(lower.GetPinFunction()), pcbnew.VECTOR2I(x, y1 + mm(1.0)))
    # IMU breakout: vertical labels running from each pin towards the board's middle row
    fp = board.FindFootprintByReference(imu)
    ys = sorted({p.GetPosition().y for p in fp.Pads()})
    mid = (ys[0] + ys[-1]) // 2
    row1 = fp.FindPadByNumber("1").GetPosition().y
    for p in fp.Pads():
        name = IMU_NAMES[p.GetNumber()]
        half = mm(0.45 * SIZE * len(name) + 0.3)       # about half the text length, plus a gap
        y = p.GetPosition().y
        start = mm(2.2) if y == row1 else mm(1.1)      # past the footprint's own "1" on pin 1's row
        y += (start + half) if y < mid else -(start + half)
        add_text(board, name, pcbnew.VECTOR2I(p.GetPosition().x, y), 90)

if __name__ == "__main__":
    b = pcbnew.LoadBoard(sys.argv[1])
    label_modules(b)
    pcbnew.SaveBoard(sys.argv[1], b)
    print("labels added")
