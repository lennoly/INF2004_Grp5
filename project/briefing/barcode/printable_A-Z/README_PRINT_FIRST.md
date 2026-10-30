# Print the robot test barcodes

Open **PRINT_ALL_A-Z_141mm_A4.pdf** to print all 26 letters on seven A4 pages.
Each complete barcode is **141 mm from the first black bar to the last black
bar**, including the Code 39 start and stop symbols. Each encodes one letter:
`*A*`, `*B*`, through `*Z*`. There is no extra checksum character.

## Printer settings

1. Paper: **A4**.
2. Scale: **Actual size / 100%**. Disable Fit, Shrink, and multiple pages per sheet.
3. Use black ink on plain, matte white paper.
4. Measure the ruler at the bottom: its end marks must be **100 mm apart**.
5. Also check the barcode itself measures **141 mm** across the outside edges
   of the first and last black bars.
6. Cut horizontally between rows if needed. Keep the full sheet width so the
   white margins to the left and right remain clear. Remove the page heading
   and bottom ruler from any pieces placed in the robot's scanning path.

The PDFs contain sharp vector rectangles rather than enlarged screenshots.
Use these PDFs for printing; raw downloaded PNGs do not specify the required
physical print size. Keep the label text outside the sensor's scanning path.

## Dimensions

- Complete barcode bar span: **141 mm**.
- Narrow element / inter-character gap: **3 mm**.
- Wide element: **9 mm** (3:1 wide-to-narrow ratio, matching the supplied sample).
- Bar height: **50 mm**, selected for testing because the supplied specification
  specifies the 141 mm span but does not specify bar height.
- Preserve at least **30 mm of blank white on both sides** of the barcode.
  The A4 PDFs provide 34.5 mm on each side; the SVGs provide 30 mm.

The supplied `Barcode & Line Specification.pdf` also specifies an **18 mm**
line width and a **20 mm white gap** between the bottom of the barcode bars
and the nearest edge of the line. The printable files here contain barcodes
only; arrange the separate line on your test surface using those dimensions.

## Folder contents

- `PRINT_ALL_A-Z_141mm_A4.pdf`: the recommended seven-page print pack.
- `individual_pdf/`: 26 single-letter A4 PDFs, for reprinting selected letters.
- `individual_svg/`: 26 vector files with physical dimensions. Overall SVG
  width is 201 mm including the two white margins; the bars span 141 mm.
- `downloaded_png/`: the 26 original Code 39 images downloaded from bwip-js.
- `source_manifest.json`: exact source URLs and dimensions.
- `verification.json`: automated pattern and PDF geometry verification results.
- `build_print_pack.py`: rebuilds the PDF/SVG files from the downloaded images
  using Python, Pillow, and ReportLab.

Downloads: https://github.com/metafloor/bwip-js/wiki/Online-Barcode-API

Standard reference: https://www.keyence.com/ss/products/auto_id/codereader/basic/code39.jsp

All downloaded patterns were checked against an independent ReportLab Code 39
encoder. All A-Z patterns were checked again after rendering the final PDF,
and bar dimensions were checked in all 27 PDFs. Actual print size still depends
on your printer settings: measure the first printed sheet before using it.

For navigation testing, configure your four instructor-selected letters over
MQTT using `map=<forward>,<uturn>,<left>,<right>` before starting the robot.
