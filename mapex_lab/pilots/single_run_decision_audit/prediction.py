"""One resident model, CPU float32; original transforms/aggregation."""
import ctypes
import gc
import sys
import time
from pathlib import Path
import numpy as np
from .contract import SOURCE, WEIGHTS, MODELS, SHAPE_MODEL, CONFIG, array_sha

def source_modules():
    for path in [SOURCE, SOURCE+'/scripts', SOURCE+'/lama']:
        if path not in sys.path:
            sys.path.insert(0, path)
    from scripts import sim_utils, simple_mask_utils
    import lama_pred_utils
    return sim_utils, simple_mask_utils, lama_pred_utils

def configure_torch():
    import torch
    torch.set_num_threads(CONFIG['threads'])
    # Set only before the first torch parallel operation in this process.
    if torch.get_num_interop_threads() != CONFIG['interop']:
        torch.set_num_interop_threads(CONFIG['interop'])
    return torch

def trim():
    gc.collect()
    try:
        ctypes.CDLL(None).malloc_trim(0)
    except (AttributeError, OSError):
        pass

class Predictor:
    def __init__(self, guard, repeat_reference=False):
        self.guard = guard
        self.repeat_reference = repeat_reference
        self.sim, self.smu, self.lpu = source_modules()
        self.torch = configure_torch()
        self.transform = self.lpu.get_lama_transform('default_map_eval', (512, 512))

    def inputs(self, obs):
        batch, mask = self.lpu.convert_obsimg_to_model_input(
            np.stack([obs, obs, obs], axis=2), self.transform, 'cpu')
        assert tuple(batch['image'].shape) == (1, 3)+SHAPE_MODEL
        assert batch['image'].dtype == self.torch.float32
        # Same center padding and mask as the source. Added cells stay known/free.
        np.testing.assert_array_equal(batch['image'][0, 0].numpy()[6:1209, :1263], obs)
        np.testing.assert_array_equal(mask[0, 6:1209, :1263], (obs == 0.5).astype(np.float32))
        return batch, mask

    def predict(self, obs):
        torch = self.torch
        with self.guard.phase('preprocessing'):
            batch, mask = self.inputs(obs)
        outputs, timings, identity = {}, [], []
        for model_id, relative, _, _ in MODELS:
            begin = time.monotonic()
            with self.guard.phase('load_'+model_id):
                model = self.lpu.load_lama_model(str(Path(WEIGHTS)/relative), device='cpu')
                assert not model.training
                assert all(p.dtype == torch.float32 for p in model.parameters())
            loaded = time.monotonic()
            # Source models mutate the dictionary, not the input tensors.
            inp = {'image': batch['image'].clone(), 'mask': batch['mask'].clone()}
            with self.guard.phase('inference_'+model_id), torch.no_grad():
                pred = model(inp)
                rgb = pred['inpainted'][0].permute(1, 2, 0).detach().cpu().numpy().copy()
                np.testing.assert_array_equal(inp['image'], batch['image'])
                assert np.isfinite(rgb).all() and tuple(rgb.shape[:2]) == SHAPE_MODEL
            inferred = time.monotonic()
            if self.repeat_reference:
                with self.guard.phase('repeat_reference_'+model_id), torch.no_grad():
                    reference = model({'image': batch['image'].clone(), 'mask': batch['mask'].clone()})
                    expected = reference['inpainted'][0].permute(1, 2, 0).cpu().numpy()
                    np.testing.assert_array_equal(rgb, expected)
                    identity.append({'model': model_id, 'bit_identical_repeat': True,
                                     'output_sha256': array_sha(rgb)})
                    del reference, expected
            # Original G -> uint8 viz BGR -> >128 occupancy is retained.
            if model_id == 'G':
                import cv2
                viz = cv2.cvtColor(np.clip(rgb*255, 0, 255).astype('uint8'), cv2.COLOR_RGB2BGR)
                pred_map = np.zeros(SHAPE_MODEL)
                pred_map[viz[:, :, 0] > 128] = 1
                outputs['alltrain_rgb'] = rgb
                outputs['alltrain_viz'] = viz
                outputs['pred_maputils'] = pred_map
            else:
                outputs[model_id] = np.ascontiguousarray(rgb[:, :, 0])
            timings.append({'model': model_id, 'load_s': loaded-begin, 'inference_s': inferred-loaded})
            del pred, inp, model, rgb
            trim()
        with self.guard.phase('aggregation'):
            stack = torch.stack([torch.from_numpy(outputs[key]) for key in ['G1', 'G2', 'G3']])
            outputs['variance'] = torch.var(stack, dim=0).numpy()
            outputs['mean'] = np.mean(stack.numpy(), axis=0)
            outputs['unknown'] = mask[0].astype(bool)
            outputs['padded_obs'] = batch['image'][0, 0].numpy().copy()
            np.testing.assert_array_equal(outputs['padded_obs'][~outputs['unknown']],
                                          outputs['mean'][~outputs['unknown']])
        return outputs, {'models': timings, 'repeat_reference': identity,
                         'preprocessing_image_sha256': array_sha(batch['image'].numpy()),
                         'mask_sha256': array_sha(mask), 'torch_threads': torch.get_num_threads(),
                         'torch_interop': torch.get_num_interop_threads()}

def fixture_predict(obs):
    """Deterministic observation-only model stand-in, ONLY for technical parity."""
    import torch
    import albumentations as A
    import cv2
    padded = A.PadIfNeeded(min_height=None, min_width=None,
              pad_height_divisor=16, pad_width_divisor=16,
              border_mode=cv2.BORDER_CONSTANT, value=0)(image=obs)['image']
    unknown = padded == 0.5
    maps = [padded.copy() for _ in range(3)]
    for i, a in enumerate(maps):
        a[unknown] = [0.2, 0.45, 0.7][i]
    stack = torch.stack([torch.from_numpy(a) for a in maps])
    rgb = np.repeat(maps[1][:, :, None], 3, axis=2)
    viz = np.clip(rgb*255, 0, 255).astype('uint8')
    return {'G1': maps[0], 'G2': maps[1], 'G3': maps[2],
            'mean': np.mean(stack.numpy(), axis=0), 'variance': torch.var(stack, dim=0).numpy(),
            'alltrain_rgb': rgb, 'alltrain_viz': viz,
            'pred_maputils': (viz[:, :, 0] > 128).astype(np.float64),
            'unknown': unknown, 'padded_obs': padded}, {'fixture_only': True}
