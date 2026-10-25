"""Build true-size vector print files from the downloaded bwip-js barcodes."""
from pathlib import Path
import json
import string
from PIL import Image
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.graphics.barcode.code39 import Standard39

ROOT = Path(__file__).resolve().parent
LETTERS = string.ascii_uppercase
WIDTH_MM = 141
HEIGHT_MM = 50
QUIET_MM = 30


def read_runs(letter):
    image = Image.open(ROOT / "downloaded_png" / f"{letter}.png").convert("RGB")
    runs = []
    for x in range(image.width):
        black = image.getpixel((x, image.height // 2))[0] < 128
        if runs and runs[-1][0] == black:
            runs[-1][1] += 1
        else:
            runs.append([black, 1])
    assert len(runs) == 29 and runs[0][0] and runs[-1][0], letter
    unit = min(width for _, width in runs)
    widths = [width * 3 / unit for _, width in runs]
    # An independent encoder confirms start/data/stop, no checksum, 3:1 ratio.
    reference = Standard39(letter, checksum=0, stop=1, ratio=3,
                           barWidth=3, quiet=0)
    reference._calculate()
    expected = [9 if item.isupper() else 3 for item in reference.decomposed]
    assert reference.encoded == f"*{letter}*"
    assert widths == expected and sum(widths) == WIDTH_MM, letter
    return [(black, width) for (black, _), width in zip(runs, widths)]


def draw_card(pdf, letter, top):
    """top is the label baseline in millimetres measured from page top."""
    page_width, page_height = A4
    x = (page_width - WIDTH_MM * mm) / 2
    y = page_height - (top + 56) * mm
    pdf.setFont("Helvetica-Bold", 12)
    pdf.drawString(x, page_height - top * mm, f"{letter}")
    pdf.setFont("Helvetica", 8)
    pdf.drawRightString(x + WIDTH_MM * mm, page_height - top * mm,
                        f"Code 39  *{letter}*  |  bars: 141 x 50 mm")
    for black, width in RUNS[letter]:
        if black:
            pdf.rect(x, y, width * mm, HEIGHT_MM * mm, stroke=0, fill=1)
        x += width * mm


def page_heading(pdf, subtitle):
    _, height = A4
    pdf.setTitle("Robot test barcodes A-Z - 141 mm - print at 100 percent")
    pdf.setAuthor("Robot barcode test pack")
    pdf.setFont("Helvetica-Bold", 12)
    pdf.drawCentredString(105 * mm, height - 12 * mm,
                         "ROBOT TEST BARCODES | A-Z | 141 mm")
    pdf.setFont("Helvetica", 8)
    pdf.drawCentredString(105 * mm, height - 18 * mm,
                         "A4 - Actual size / 100% - Disable Fit / Shrink")
    pdf.drawCentredString(105 * mm, height - 23 * mm, subtitle)


def page_footer(pdf, number, total):
    _, height = A4
    # Keep this ruler on the scrap below the barcode cards.
    y = height - 284 * mm
    pdf.setLineWidth(0.4)
    pdf.line(55 * mm, y, 155 * mm, y)
    for x in (55, 155):
        pdf.line(x * mm, y - mm, x * mm, y + mm)
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(105 * mm, height - 289 * mm,
                         f"Ruler must measure 100 mm | Page {number}/{total}")


RUNS = {letter: read_runs(letter) for letter in LETTERS}
pdf_dir = ROOT / "individual_pdf"
svg_dir = ROOT / "individual_svg"
pdf_dir.mkdir(exist_ok=True)
svg_dir.mkdir(exist_ok=True)

combined = canvas.Canvas(str(ROOT / "PRINT_ALL_A-Z_141mm_A4.pdf"), pagesize=A4)
for page in range(7):
    group = LETTERS[page * 4:page * 4 + 4]
    page_heading(combined, "Keep the white area beside each barcode clear; cut between rows.")
    for row, letter in enumerate(group):
        draw_card(combined, letter, 27 + row * 64)
    page_footer(combined, page + 1, 7)
    combined.showPage()
combined.save()

for letter in LETTERS:
    pdf = canvas.Canvas(str(pdf_dir / f"{letter}_141mm_A4.pdf"), pagesize=A4)
    page_heading(pdf, "Single-letter test sheet; preserve at least 30 mm of white on each side.")
    draw_card(pdf, letter, 110)
    page_footer(pdf, 1, 1)
    pdf.showPage()
    pdf.save()
    rects = []
    x = QUIET_MM
    for black, width in RUNS[letter]:
        if black:
            rects.append(f'<rect x="{x:g}" y="0" width="{width:g}" height="50"/>')
        x += width
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="201mm" height="50mm" '
           f'viewBox="0 0 201 50"><title>Code 39 {letter}, 141 mm bars</title>'
           '<rect width="201" height="50" fill="white"/>'
           '<g fill="black">' + ''.join(rects) + '</g></svg>\n')
    (svg_dir / f"{letter}_141mm.svg").write_text(svg)

manifest = {
    "type": "Code 39", "data": list(LETTERS), "checksum": False,
    "start_stop": "*", "bar_span_mm": WIDTH_MM, "bar_height_mm": HEIGHT_MM,
    "narrow_mm": 3, "wide_mm": 9, "quiet_margin_mm_each_side": QUIET_MM,
    "source": "https://github.com/metafloor/bwip-js/wiki/Online-Barcode-API",
    "downloads": {
        letter: ("https://bwipjs-api.metafloor.com/?bcid=code39&text=" + letter
                 + "&scale=4&height=20&ratio=3&backgroundcolor=FFFFFF")
        for letter in LETTERS
    },
    "print_notes": "A4, Actual size / 100%; never Fit or Shrink. PDFs and SVGs set physical size; source PNGs do not.",
}
(ROOT / "source_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print("Created 27 vector PDFs and 26 SVGs from 26 verified downloaded PNGs.")
