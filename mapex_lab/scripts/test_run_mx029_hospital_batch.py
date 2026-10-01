import csv
import tempfile
import unittest
from pathlib import Path

from mapex_lab.scripts.run_mx029_hospital_batch import read_decision_progress


class MX029BatchTests(unittest.TestCase):
    def test_decision_progress_missing_file(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(
                read_decision_progress(Path(directory) / "decisions.csv"),
                (0, None),
            )

    def test_decision_progress_returns_last_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "decisions.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["decision_id", "map_generation"])
                writer.writeheader()
                writer.writerow({"decision_id": 1, "map_generation": 10})
                writer.writerow({"decision_id": 2, "map_generation": 12})
            self.assertEqual(read_decision_progress(path), (2, "12"))


if __name__ == "__main__":
    unittest.main()
