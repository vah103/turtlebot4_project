"""ROS-free checks for prediction recording and checkpoint resolution."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'

def module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

class RecordingTest(unittest.TestCase):
    def test_distinct_prediction_artifacts(self):
        tree = ast.parse((SCRIPTS / 'mapex_run.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MapExRun')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_save_prediction_maps')
        namespace = {'np': np}
        exec(compile(ast.Module(body=[method], type_ignores=[]), 'recorder', 'exec'), namespace)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'predictions').mkdir()
            info = SimpleNamespace(height=2, width=2, resolution=0.1,
                origin=SimpleNamespace(position=SimpleNamespace(x=1., y=2.)))
            recorder = SimpleNamespace(save_predictions=True,
                last_ensemble_predictions=np.full((3, 2, 2), .2),
                last_mean_map=np.full((2, 2), .2), last_variance_map=np.zeros((2, 2)),
                map_msg=SimpleNamespace(info=info), run=root, environment='new_room',
                ensemble=SimpleNamespace())
            paths = namespace['_save_prediction_maps'](recorder, 1)
            self.assertEqual(len(paths), 5)
            with np.load(root / paths[3]) as data:
                np.testing.assert_allclose(data['data'], .2)
            self.assertFalse(any('alltrain' in name for name in paths))

    def test_offline_manifest_and_original_data_preserved(self):
        offline = module('predict_alltrain_offline')
        evaluator = module('evaluate_mapex_run')
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            np.savez(root/'raw.npz', data=np.array([[0, -1], [100, -1]]),
                     resolution=.1, origin_x=1., origin_y=2.)
            original = 'decision_id,time_s,raw_map,mean_map\n1,0,raw.npz,mean.npz\n'
            (root/'decisions.csv').write_text(original)
            checkpoint = root/'best.ckpt'
            checkpoint.write_bytes(b'test checkpoint')
            seen = []
            def predict(observed):
                seen.append(observed.copy())
                return np.pad(observed, 1, constant_values=.9)
            offline.generate(root, checkpoint, predict)
            np.testing.assert_array_equal(seen[0], [[0., .5], [1., .5]])
            self.assertEqual((root/'decisions.csv').read_text(), original)
            rows = evaluator._read_csv(root/'decisions.csv')
            attached = evaluator.attach_offline_predictions(root, rows)
            self.assertNotIn('alltrain_map', rows[0])
            with np.load(root/attached[0]['alltrain_map']) as data:
                self.assertEqual(int(data['pad_top']), 1)
                self.assertEqual(float(data['origin_x']), 1.)
            with self.assertRaises(FileExistsError):
                offline.generate(root, checkpoint, predict)
            (root/'decisions.csv').write_text(original + '\n')
            with self.assertRaisesRegex(ValueError, 'different decisions'):
                evaluator.attach_offline_predictions(root, rows)

    def test_checkpoint_resolution_and_missing_checkpoint(self):
        worker = module('mapex_lama_worker')
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            checkpoint = root / 'alltrain' / 'models' / 'custom.ckpt'
            checkpoint.parent.mkdir(parents=True)
            checkpoint.touch()
            spec = worker.resolve_models(temp, ['alltrain/models/custom.ckpt'] * 3)[0]
            self.assertEqual(spec, (str(root / 'alltrain'), 'custom.ckpt'))
            with self.assertRaisesRegex(RuntimeError, 'not found'):
                worker.resolve_models(temp, ['missing'] * 3)

    def test_evaluation_csv_preserves_source_and_secondary_metrics(self):
        evaluator = module('evaluate_mapex_run')
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'eval.csv'
            evaluator._write_csv(path, [{'prediction_source': 'alltrain_map',
                'prediction_map': 'alltrain.npz', 'mean_map': 'mean.npz',
                'occupied_iou': .9, 'ensemble_mean_occupied_iou': .3}])
            row = evaluator._read_csv(path)[0]
            self.assertEqual(row['prediction_source'], 'alltrain_map')
            self.assertEqual(row['prediction_map'], 'alltrain.npz')
            self.assertEqual(row['ensemble_mean_occupied_iou'], '0.3')

if __name__ == '__main__':
    unittest.main()
