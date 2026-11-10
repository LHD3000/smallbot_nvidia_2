#!/usr/bin/python3
"""Check printed dimensions, identities and physical scale against launch defaults."""
import importlib.util
import json
from pathlib import Path
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

import cv2
import numpy as np
from PIL import Image

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('generate_marker', PACKAGE / 'scripts/generate_marker.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class MarkerAssetsTest(unittest.TestCase):
    def test_total_size_and_all_identities(self):
        with tempfile.TemporaryDirectory() as folder:
            for marker_id in (0, 18, 19):
                with self.subTest(marker_id=marker_id):
                    path = Path(folder) / 'aruco_{}.png'.format(marker_id)
                    data = generator.generate(marker_id, 10, 800, 100, 100.0, path)
                    self.assertEqual(json.loads(path.with_suffix('.json').read_text()), data)
                    with Image.open(path) as image:
                        self.assertEqual(image.size, (1000, 1000))
                        for pixels, dpi in zip(image.size, image.info['dpi']):
                            self.assertAlmostEqual(pixels / dpi * 25.4, 100, places=3)
                        raster = np.array(image)
                    rows, cols = np.where(raster == 0)
                    self.assertEqual((rows.min(), rows.max(), cols.min(), cols.max()), (100, 899, 100, 899))
                    _, ids, _ = cv2.aruco.detectMarkers(raster, cv2.aruco.getPredefinedDictionary(10))
                    self.assertEqual(ids.flatten().tolist(), [marker_id])
                    svg = ET.parse(path.with_suffix('.svg')).getroot()
                    self.assertEqual((svg.attrib['width'], svg.attrib['height']), ('100mm', '100mm'))
                    pdf = path.with_suffix('.pdf').read_bytes()
                    box = re.search(rb'/MediaBox\s*\[\s*0\s+0\s+([\d.]+)\s+([\d.]+)\s*\]', pdf)
                    self.assertIsNotNone(box)
                    for points in box.groups():
                        self.assertAlmostEqual(float(points) * 25.4 / 72, 100, places=3)
                    config = yaml.safe_load((PACKAGE / 'config/aruco.yaml').read_text())
                    defaults = config['aruco_det']['ros__parameters']
                    self.assertAlmostEqual(defaults['aruco_length'], data['aruco_length_m'])
                    self.assertEqual(defaults['dictionary_id'], data['dictionary_id'])


    def test_print_scale_recovers_known_distance(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'aruco.png'
            data = generator.generate(0, 10, 800, 100, 100.0, path)
            # At fx=800 and z=0.8 m, the whole 100 mm print occupies 100 pixels.
            printed = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            projected = cv2.resize(printed, (100, 100), interpolation=cv2.INTER_NEAREST)
            scene = np.full((480, 640), 255, dtype=np.uint8)
            scene[190:290, 270:370] = projected
            factory = getattr(cv2.aruco, 'DetectorParameters_create', None)
            params = factory() if factory else cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            corners, ids, _ = cv2.aruco.detectMarkers(scene, cv2.aruco.getPredefinedDictionary(10), parameters=params)
            self.assertEqual(ids.flatten().tolist(), [0])
            k = np.array([[800., 0, 320], [0, 800., 240], [0, 0, 1]])
            _, position, _ = cv2.aruco.estimatePoseSingleMarkers(corners, data['aruco_length_m'], k, np.zeros(5))
            _, wrong_position, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.10, k, np.zeros(5))
            self.assertAlmostEqual(position[0, 0, 2], 0.8, delta=0.02)
            self.assertAlmostEqual(wrong_position[0, 0, 2] / position[0, 0, 2], 1.25, places=5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
