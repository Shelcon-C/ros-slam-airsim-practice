#!/usr/bin/env python3
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from slam_practice.dataset import DatasetFormatError, load_euroc_camera_index


class EurocDatasetTest(unittest.TestCase):
    def make_dataset(self, rows):
        temporary = tempfile.TemporaryDirectory()
        root = Path(temporary.name)
        camera = root / "mav0" / "cam0"
        data = camera / "data"
        data.mkdir(parents=True)
        (camera / "data.csv").write_text(
            "#timestamp [ns],filename\n" + "\n".join(rows) + "\n",
            encoding="utf-8",
        )
        return temporary, root, data

    def test_loads_sorted_frames_and_converts_nanoseconds(self):
        temporary, root, data = self.make_dataset(
            ["1403636579763555584,first.png", "1403636579813555456,second.png"]
        )
        self.addCleanup(temporary.cleanup)
        (data / "first.png").touch()
        (data / "second.png").touch()

        frames = load_euroc_camera_index(root)

        self.assertEqual([frame.filename for frame in frames], ["first.png", "second.png"])
        self.assertAlmostEqual(frames[0].timestamp_sec, 1403636579.7635555, places=6)
        self.assertLess(frames[0].timestamp_ns, frames[1].timestamp_ns)

    def test_rejects_missing_image(self):
        temporary, root, _ = self.make_dataset(["1,missing.png"])
        self.addCleanup(temporary.cleanup)

        with self.assertRaisesRegex(FileNotFoundError, "missing.png"):
            load_euroc_camera_index(root)

    def test_rejects_non_monotonic_timestamps(self):
        temporary, root, data = self.make_dataset(["2,a.png", "1,b.png"])
        self.addCleanup(temporary.cleanup)
        (data / "a.png").touch()
        (data / "b.png").touch()

        with self.assertRaisesRegex(DatasetFormatError, "strictly increasing"):
            load_euroc_camera_index(root)


if __name__ == "__main__":
    unittest.main()
