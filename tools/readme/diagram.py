"""Draws the README diagram docs/img/pipeline.svg and its dark version, in the style of a LaTeX paper figure.

uv run --no-project --with fonttools --with brotli python tools/readme/diagram.py

The Computer Modern font (CMU Serif, SIL OFL) is downloaded once, cut down to the letters in use, and put inside each SVG.
"""
import base64
import io
import re
import urllib.request
from pathlib import Path

from fontTools import subset
from fontTools.ttLib import TTFont

ROOT = Path(__file__).resolve().parents[2]
CACHE = Path.home() / ".cache" / "sobang-fonts"
FONT_URL = "https://cdn.jsdelivr.net/npm/computer-modern@0.1.3/fonts/{}.woff2"
FS = 17
BW, BH, GAP, X0 = 136, 56, 72, 20
X = [X0 + i * (BW + GAP) for i in range(5)]
Y = 56
W, H = X[4] + BW + X0, Y + 120 + BH + 50


def font_face(text):
    css = ""
    for name, style in (("cmu-serif-500-roman", "normal"), ("cmu-serif-500-italic", "italic")):
        path = CACHE / f"{name}.woff2"
        if not path.exists():
            CACHE.mkdir(parents=True, exist_ok=True)
            urllib.request.urlretrieve(FONT_URL.format(name), path)
        font = TTFont(path)
        sub = subset.Subsetter(subset.Options())
        sub.populate(text=text)
        sub.subset(font)
        font.flavor = "woff2"
        buf = io.BytesIO()
        font.save(buf)
        data = base64.b64encode(buf.getvalue()).decode()
        css += f'@font-face{{font-family:CMU;font-style:{style};src:url(data:font/woff2;base64,{data}) format("woff2")}}'
    return css


def draw(ink, bg):
    def box(x, y, t):
        return (f'<rect x="{x}" y="{y}" width="{BW}" height="{BH}" fill="{bg}" stroke="{ink}"/>'
                f'<text x="{x + BW / 2}" y="{y + BH / 2 + 6}" text-anchor="middle" font-size="{FS}">{t}</text>')

    def arrow(p, label, lx, ly, anchor="middle"):
        return (f'<polyline points="{" ".join(f"{x},{y}" for x, y in p)}" fill="none" stroke="{ink}" marker-end="url(#a)"/>'
                f'<text x="{lx}" y="{ly}" text-anchor="{anchor}" font-size="{FS}" font-style="italic">{label}</text>')

    def group(x, y, w, h, caption):
        return (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="none" stroke="{ink}" stroke-width="0.8" stroke-dasharray="4 3"/>'
                f'<text x="{x}" y="{y - 8}" font-size="15" font-style="italic">{caption}</text>')

    wz = "v, ω<tspan font-size='12' dy='4'>z</tspan>"
    b = "".join(box(X[i], Y, t) for i, t in enumerate(["Camera", "Fire detector", "Aiming", "Walking policy", "Legs"]))
    for i, label in enumerate(["I", "b", wz, "q*"]):
        a, c = X[i] + BW, X[i + 1]
        b += arrow([(a, Y + BH / 2), (c, Y + BH / 2)], label, (a + c) / 2, Y + BH / 2 - 9)
    px = X[3] + BW / 2
    b += box(X[3], Y + 120, "IMU")
    b += arrow([(px, Y + 120), (px, Y + BH)], "ω, g", px + 10, Y + BH + 38, "start")
    b += group(X[0] - 8, Y - 20, X[2] + BW - X[0] + 16, BH + 40, "perception 15 Hz, aiming 10 Hz")
    b += group(X[3] - 8, Y - 20, BW + 16, BH + 160, "control 25 Hz")
    text = "".join(re.findall(r">([^<>]*)<", b)) + "0123456789"
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
            f'font-family="CMU, \'Latin Modern Roman\', Georgia, serif" fill="{ink}" stroke-width="1">'
            f'<title>How Sobang-I works</title><defs><style>{font_face(text)}</style>'
            f'<marker id="a" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="9" markerHeight="9" markerUnits="userSpaceOnUse" orient="auto">'
            f'<path d="M0 0.8L10 5L0 9.2z" fill="{ink}"/></marker></defs>'
            f'<rect width="{W}" height="{H}" fill="{bg}"/>{b}</svg>\n')


if __name__ == "__main__":
    (ROOT / "docs/img/pipeline.svg").write_text(draw("#000000", "#ffffff"))
    (ROOT / "docs/img/pipeline-dark.svg").write_text(draw("#e6edf3", "#0d1117"))
