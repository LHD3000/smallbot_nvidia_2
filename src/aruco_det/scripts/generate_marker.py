#!/usr/bin/python3
"""Export a physically sized ArUco PNG, SVG and PDF, including the white margin."""
import argparse
import json
import math
from pathlib import Path
import cv2
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


def draw_marker(dictionary, marker_id, pixels):
    function = getattr(cv2.aruco, "generateImageMarker", None)
    if function is None:
        function = cv2.aruco.drawMarker
    return function(dictionary, marker_id, pixels)


def write_a4_pdf(path, cells, marker_id, dictionary_id, total_mm, black_mm, margin_mm):
    """Center the physical marker on A4; put crop marks outside its white margin."""
    pdf = canvas.Canvas(str(path), pagesize=A4)
    pdf.setTitle('ArUco ID {} - A4 - total {} x {} mm'.format(marker_id, total_mm, total_mm))
    left = (A4[0] / mm - total_mm) / 2
    bottom = (A4[1] / mm - total_mm) / 2
    cell_mm = black_mm / cells.shape[0]
    pdf.setFillColorRGB(0, 0, 0)
    for row in range(cells.shape[0]):
        for col in range(cells.shape[1]):
            if cells[row, col] == 0:
                pdf.rect((left + margin_mm + col * cell_mm) * mm,
                         (bottom + total_mm - margin_mm - (row + 1) * cell_mm) * mm,
                         cell_mm * mm, cell_mm * mm, fill=1, stroke=0)
    pdf.setLineWidth(0.3)
    for x, dx in ((left, -1), (left + total_mm, 1)):
        for y, dy in ((bottom, -1), (bottom + total_mm, 1)):
            pdf.line((x + dx) * mm, y * mm, (x + 5 * dx) * mm, y * mm)
            pdf.line(x * mm, (y + dy) * mm, x * mm, (y + 5 * dy) * mm)
    pdf.setFont('Helvetica', 12)
    pdf.drawCentredString(A4[0] / 2, (bottom + total_mm + 15) * mm,
                         'ArUco dictionary {} / ID {}'.format(dictionary_id, marker_id))
    pdf.setFont('Helvetica', 10)
    for offset, text in ((15, 'Crop size: {0:g} x {0:g} mm (including white margin)'.format(total_mm)),
                         (21, 'Black marker: {0:g} x {0:g} mm; margin: {1:g} mm per side'.format(black_mm, margin_mm)),
                         (29, 'Print on A4 at Actual Size / 100%. Do not fit to page.')):
        pdf.drawCentredString(A4[0] / 2, (bottom - offset) * mm, text)
    pdf.showPage()
    pdf.save()


def generate(marker_id, dictionary_id, pixels, margin, total_mm, output, a4=False):
    if not 0 <= dictionary_id <= 20 or pixels < 32 or margin < 1:
        raise ValueError("Invalid dictionary, pixels or margin")
    if not math.isfinite(total_mm) or total_mm <= 0:
        raise ValueError("Total size in millimetres must be finite and positive")
    if a4 and total_mm > 190:
        raise ValueError("A4 layout needs total size <= 190 mm to leave room for crop marks")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if not 0 <= marker_id < dictionary.bytesList.shape[0]:
        raise ValueError("ID is outside the selected dictionary")
    path = Path(output).expanduser().resolve()
    if path.suffix.lower() != '.png':
        raise ValueError("--output must end with .png; .pdf/.svg/.json are also exported")
    path.parent.mkdir(parents=True, exist_ok=True)

    total_pixels = pixels + 2 * margin
    black_mm = total_mm * pixels / total_pixels
    margin_mm = total_mm * margin / total_pixels
    raster = draw_marker(dictionary, marker_id, pixels)
    raster = cv2.copyMakeBorder(raster, margin, margin, margin, margin,
                               cv2.BORDER_CONSTANT, value=255)
    dpi = total_pixels * 25.4 / total_mm
    Image.fromarray(raster).save(str(path), dpi=(dpi, dpi))

    # Draw native black/white modules as vectors; no lossy image compression.
    module_count = dictionary.markerSize + 2  # One black border module per edge.
    cells = draw_marker(dictionary, marker_id, module_count)
    cell_pixels = pixels / module_count
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{0:g}mm" height="{0:g}mm" '
        'viewBox="0 0 {1} {1}" shape-rendering="crispEdges">'.format(total_mm, total_pixels),
        '<title>ArUco dictionary {} ID {}; total {} mm, black edge {} mm</title>'.format(
            dictionary_id, marker_id, total_mm, black_mm),
        '<rect width="100%" height="100%" fill="white"/>', '<g fill="black">']
    pdf = canvas.Canvas(str(path.with_suffix('.pdf')), pagesize=(total_mm * mm, total_mm * mm))
    pdf.setTitle('ArUco ID {} - total {} x {} mm'.format(marker_id, total_mm, total_mm))
    pdf.setFillColorRGB(1, 1, 1)
    pdf.rect(0, 0, total_mm * mm, total_mm * mm, fill=1, stroke=0)
    pdf.setFillColorRGB(0, 0, 0)
    cell_mm = black_mm / module_count
    for row in range(module_count):
        for col in range(module_count):
            if cells[row, col] != 0:
                continue
            svg.append('<rect x="{:.10g}" y="{:.10g}" width="{:.10g}" height="{:.10g}"/>'.format(
                margin + col * cell_pixels, margin + row * cell_pixels, cell_pixels, cell_pixels))
            pdf.rect((margin_mm + col * cell_mm) * mm,
                     (total_mm - margin_mm - (row + 1) * cell_mm) * mm,
                     cell_mm * mm, cell_mm * mm, fill=1, stroke=0)
    svg.extend(['</g>', '</svg>'])
    path.with_suffix('.svg').write_text('\n'.join(svg) + '\n')
    pdf.showPage()
    pdf.save()
    if a4:
        write_a4_pdf(path.with_name(path.stem + '_A4.pdf'), cells, marker_id, dictionary_id,
                     total_mm, black_mm, margin_mm)
    metadata = {
        'dictionary_id': dictionary_id, 'marker_id': marker_id,
        'total_width_mm': total_mm, 'total_height_mm': total_mm,
        'black_marker_width_mm': black_mm, 'black_marker_height_mm': black_mm,
        'white_margin_each_side_mm': margin_mm, 'aruco_length_m': black_mm / 1000,
        'png_width_px': total_pixels, 'png_height_px': total_pixels, 'png_dpi': dpi,
        'print_instruction': 'Print PDF at Actual Size / 100%, not Fit to Page. Measure after printing.'}
    if a4:
        metadata['a4_pdf'] = path.stem + '_A4.pdf'
        metadata['a4_page_width_mm'] = 210.0
        metadata['a4_page_height_mm'] = 297.0
        metadata['a4_marker_left_mm'] = (210.0 - total_mm) / 2
        metadata['a4_marker_bottom_mm'] = (297.0 - total_mm) / 2
    path.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--dictionary-id", type=int, default=10)
    parser.add_argument("--pixels", type=int, default=800)
    parser.add_argument("--margin", type=int, default=100)
    parser.add_argument("--total-mm", type=float, default=100.0,
                        help="Total width AND height including white margin, in mm (default: 100)")
    parser.add_argument("--output", help="PNG path; PDF, SVG and dimension JSON use the same stem")
    parser.add_argument("--a4", action="store_true", help="Also export a centered A4 PDF with crop marks")
    args = parser.parse_args()
    output = args.output or 'aruco_{}_total_{:g}mm.png'.format(args.id, args.total_mm)
    try:
        data = generate(args.id, args.dictionary_id, args.pixels, args.margin, args.total_mm, output, a4=args.a4)
    except ValueError as error:
        parser.error(str(error))
    print('Saved PNG/PDF/SVG/JSON: {}'.format(Path(output).resolve()))
    if args.a4:
        print('A4 print PDF: {}'.format(Path(output).resolve().with_name(data['a4_pdf'])))
    print('Total: {total_width_mm:g} x {total_height_mm:g} mm; black marker: '
          '{black_marker_width_mm:g} mm; margin per side: {white_margin_each_side_mm:g} mm; '
          'aruco_length:={aruco_length_m:g}'.format(**data))
    print('Print the PDF at Actual Size / 100%. Do not use Fit to Page.')


if __name__ == "__main__":
    main()
