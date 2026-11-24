"""Turns oled_render PBM frames into OLED-looking PNGs and a contact sheet.

Usage: python3 make_png.py <pbm_dir> <out_dir> <captions.tsv>

Each 128x128 frame is drawn at 4x with a 1 px gap between pixels, white
on near-black like the SH1107 module; frames rendered at the idle contrast
(0x04) are drawn dimmer so the dimming shows. Writes <name>.png per frame
and contact_sheet.png.
"""
import re
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

SCALE = 4
PANEL = 128
BG = (8, 10, 14)
OFF = (22, 25, 31)
ON = (236, 242, 255)
DIM = (120, 124, 132)
BEZEL = 10


def read_pbm(path):
    """Returns (pixel rows, contrast) of a plain PBM written by oled_render."""
    text = path.read_text()
    m = re.search(r"# contrast (\d+)", text)
    contrast = int(m.group(1)) if m else 127
    body = [ln for ln in text.splitlines()[1:] if not ln.startswith("#")]
    rows = [ln for ln in body[1:] if ln]
    return rows, contrast


def render(rows, contrast):
    """Draws one frame as the module would look."""
    size = PANEL * SCALE + 2 * BEZEL
    img = Image.new("RGB", (size, size), BG)
    d = ImageDraw.Draw(img)
    on = DIM if contrast <= 8 else ON
    for y, row in enumerate(rows):
        for x, c in enumerate(row):
            x0 = BEZEL + x * SCALE
            y0 = BEZEL + y * SCALE
            d.rectangle([x0, y0, x0 + SCALE - 2, y0 + SCALE - 2],
                        fill=on if c == "1" else OFF)
    return img


def font(px):
    """A legible caption font, falling back to PIL's default."""
    for name in ("/System/Library/Fonts/Helvetica.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except OSError:
            continue
    return ImageFont.load_default()


def main():
    pbm_dir, out_dir, captions = (Path(a) for a in sys.argv[1:4])
    out_dir.mkdir(parents=True, exist_ok=True)
    items = []
    for line in captions.read_text().splitlines():
        name, caption = line.split("\t", 1)
        img = render(*read_pbm(pbm_dir / f"{name}.pbm"))
        img.save(out_dir / f"{name}.png")
        items.append((name, caption, img))

    cols = 4
    tile = items[0][2].width
    cap_h = 46
    pad = 18
    rows = (len(items) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * (tile + pad) + pad,
                              rows * (tile + cap_h + pad) + pad), (245, 246, 248))
    d = ImageDraw.Draw(sheet)
    f_name, f_cap = font(15), font(17)
    for i, (name, caption, img) in enumerate(items):
        x = pad + (i % cols) * (tile + pad)
        y = pad + (i // cols) * (tile + cap_h + pad)
        sheet.paste(img, (x, y))
        d.text((x, y + tile + 4), caption, fill=(20, 24, 32), font=f_cap)
        d.text((x, y + tile + 25), name, fill=(110, 116, 128), font=f_name)
    sheet.save(out_dir / "contact_sheet.png")


if __name__ == "__main__":
    main()
