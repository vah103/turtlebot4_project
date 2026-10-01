import tempfile
import unittest
from pathlib import Path

from mx029_hospital_collection import local_path


class MX029PathTests(unittest.TestCase):
    def test_local_path_accepts_child(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            self.assertEqual(local_path(run, "decisions.csv"), run / "decisions.csv")

    def test_local_path_rejects_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                local_path(Path(directory), "../outside")


if __name__ == "__main__":
    unittest.main()
