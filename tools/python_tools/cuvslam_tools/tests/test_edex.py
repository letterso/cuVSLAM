import os
from pathlib import Path
import tempfile
import unittest

from cuvslam_tools.common.edex import EDEXMetadata, DistortionModel


DATA_DIR = Path(os.path.abspath(__file__)).parent / "data"


class TestEdex(unittest.TestCase):
    def test_read_edex(self):
        edex = EDEXMetadata.read(DATA_DIR / "edex")
        self.assertEqual(edex.header.version, "0.9")
        self.assertEqual(edex.header.frame_start, 0)
        self.assertEqual(edex.header.frame_end, 1600)
        self.assertEqual(len(edex.header.cameras), 2)
        self.assertEqual(
            edex.header.cameras[0].intrinsics.distortion_model,
            DistortionModel.POLYNOMIAL,
        )
        self.assertEqual(
            edex.header.cameras[0].intrinsics.distortion_params.shape, (8,)
        )
        self.assertEqual(edex.header.cameras[0].intrinsics.focal.shape, (2,))
        self.assertEqual(edex.header.cameras[0].intrinsics.principal.shape, (2,))
        self.assertEqual(edex.header.cameras[0].intrinsics.resolution.shape, (2,))
        self.assertEqual(edex.header.cameras[0].transform.shape, (3, 4))

    def test_read_edex_copy_transform(self):
        # Read the intrinsics-only EDEX file
        edex_intrinsics = EDEXMetadata.read(DATA_DIR / "edex_intrinsics")
        self.assertIsNone(edex_intrinsics.header.cameras[0].transform)

        # Written output should be the same as the input
        with tempfile.NamedTemporaryFile(
            mode="w+", delete=True, encoding="utf-8"
        ) as temp_file:
            edex_intrinsics.write(Path(temp_file.name))
            edex_intrinsics_2 = EDEXMetadata.read(Path(temp_file.name))
            self.assertEqual(edex_intrinsics, edex_intrinsics_2)

        # Read the original EDEX file
        edex = EDEXMetadata.read(DATA_DIR / "edex")

        edex_intrinsics.header.cameras[0].transform = edex.header.cameras[0].transform
        edex_intrinsics.header.cameras[1].transform = edex.header.cameras[1].transform

        # Written output should be the same as the input with the transform added
        with tempfile.NamedTemporaryFile(
            mode="w+", delete=True, encoding="utf-8"
        ) as temp_file:
            edex_intrinsics.write(Path(temp_file.name))
            edex_intrinsics_2 = EDEXMetadata.read(Path(temp_file.name))
            self.assertEqual(edex, edex_intrinsics_2)

    def test_read_edex_bad(self):
        with self.assertRaises(Exception):
            EDEXMetadata.read(DATA_DIR / "edex_bad")


if __name__ == "__main__":
    unittest.main()
