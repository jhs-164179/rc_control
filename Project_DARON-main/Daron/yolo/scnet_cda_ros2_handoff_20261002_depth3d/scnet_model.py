"""Paper-inspired model/trainer; loss adaptation is explicitly NOT author code.

Loss plumbing derives from Ultralytics v8.3.12 (AGPL-3.0, see upstream/LICENSE).
Keep this importable module beside scnet.py to reload training checkpoints.
"""
import re
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PIL import Image
import torch
from ultralytics import YOLO
from ultralytics.models.yolo.detect import DetectionTrainer
from ultralytics.nn.tasks import DetectionModel
from ultralytics.utils.loss import v8DetectionLoss
from ultralytics.utils.tal import make_anchors
from ultralytics.utils import LOGGER


def size_weights(target_boxes, foreground, reference_side, gain):
    """Smoothly upweight positives with sqrt(area) < reference_side, at most 1+gain."""
    wh = (target_boxes[..., 2:] - target_boxes[..., :2]).clamp(min=0)
    diameter = wh.prod(-1).sqrt()
    extra = (1 - diameter / reference_side).clamp(0, 1)
    return 1 + gain * extra * foreground.to(extra.dtype)


class SmallObjectLoss(v8DetectionLoss):
    """Optional size-dependent box/cls/DFL weights; stock targets/assigner unchanged.

    No separate objectness branch is introduced. A YOLO11 confidence score comes
    from the class branch. This is an ablatable local interpretation of §2.2.2.
    """

    def __init__(self, model):
        super().__init__(model)
        options = model.yaml.get('scnet', {})
        self.gain = float(options.get('small_gain', 0))
        self.small_ratio = float(options.get('small_ratio', .05))
        self.progress = 1.0
        if self.gain < 0 or not 0 < self.small_ratio <= 1:
            raise ValueError('small_gain >= 0 and 0 < small_ratio <= 1 required')

    def __call__(self, preds, batch):
        # Zero gain is exactly the stock implementation, also used for baseline.
        gain = self.gain * self.progress
        if gain == 0:
            return super().__call__(preds, batch)
        loss = torch.zeros(3, device=self.device)
        feats = preds[1] if isinstance(preds, tuple) else preds
        distribution, scores = torch.cat(
            [x.view(feats[0].shape[0], self.no, -1) for x in feats], 2
        ).split((self.reg_max * 4, self.nc), 1)
        scores = scores.permute(0, 2, 1).contiguous()
        distribution = distribution.permute(0, 2, 1).contiguous()
        dtype, batch_size = scores.dtype, scores.shape[0]
        image_size = torch.tensor(feats[0].shape[2:], device=self.device, dtype=dtype) * self.stride[0]
        points, strides = make_anchors(feats, self.stride, .5)
        targets = torch.cat((batch['batch_idx'].view(-1, 1), batch['cls'].view(-1, 1), batch['bboxes']), 1)
        targets = self.preprocess(targets.to(self.device), batch_size, image_size[[1, 0, 1, 0]])
        labels, boxes = targets.split((1, 4), 2)
        valid = boxes.sum(2, keepdim=True).gt_(0)
        predicted_boxes = self.bbox_decode(points, distribution)
        _, target_boxes, target_scores, foreground, _ = self.assigner(
            scores.detach().sigmoid(), (predicted_boxes.detach() * strides).type(boxes.dtype),
            points * strides, labels, boxes, valid,
        )
        normalizer = max(target_scores.sum(), 1)
        weights = size_weights(target_boxes, foreground, image_size.max() * self.small_ratio, gain)
        # Weight the BCE, never its target probabilities (which must remain in [0,1]).
        loss[1] = (self.bce(scores, target_scores.to(dtype)) * weights.unsqueeze(-1)).sum() / normalizer
        if foreground.any():
            loss[0], loss[2] = self.bbox_loss(
                distribution, predicted_boxes, points, target_boxes / strides,
                target_scores * weights.unsqueeze(-1), normalizer, foreground,
            )
        loss[0] *= self.hyp.box
        loss[1] *= self.hyp.cls
        loss[2] *= self.hyp.dfl
        return loss.sum() * batch_size, loss.detach()


class SCNetModel(DetectionModel):
    def init_criterion(self):
        return SmallObjectLoss(self)


def transfer_coco(model, weight_path, variant):
    """Map equivalent layers by function, not shifted P2 graph layer numbers.

    New P2/P3 fusion blocks start randomly. Only matching tensor shapes transfer.
    COCO final class predictors never transfer to the crater class.
    """
    source = YOLO(str(weight_path)).model.float().state_dict()
    destination = model.state_dict()
    layer_map = {i: i for i in range(17)}
    layer_map.update({i: i + 6 for i in range(17, 24)} if variant == 'p2' else {i: i for i in range(17, 24)})
    transferred, pairs = {}, []
    for old_key, value in source.items():
        match = re.match(r'model\.(\d+)\.(.*)', old_key)
        if not match:
            continue
        old_index, suffix = int(match[1]), match[2]
        if old_index not in layer_map:
            continue
        if old_index == 23:
            branch = re.match(r'(cv[23])\.(\d+)\.(.*)', suffix)
            if branch:
                # cv3.<branch>.2 is the final 80-class Conv2d.
                if branch[1] == 'cv3' and branch[3].startswith('2.'):
                    continue
                branch_index = int(branch[2]) + (1 if variant == 'p2' else 0)
                suffix = f'{branch[1]}.{branch_index}.{branch[3]}'
        new_key = f'model.{layer_map[old_index]}.{suffix}'
        if new_key in destination and destination[new_key].shape == value.shape:
            transferred[new_key] = value
            pairs.append((old_key, new_key))
    model.load_state_dict(transferred, strict=False)
    return {
        'source': str(weight_path), 'source_tensors': len(source),
        'destination_tensors': len(destination), 'transferred_tensors': len(transferred),
        'transferred_elements': sum(x.numel() for x in transferred.values()),
        'mapping': pairs, 'class_predictors_transferred': False,
    }


class SCNetTrainer(DetectionTrainer):
    def _setup_train(self, world_size):
        # The author checkout omits ultralytics/assets/bus.jpg required by the
        # stock check_amp. Compare THIS detector on a local training image instead.
        # Scope the hook to setup, then restore the upstream function immediately.
        with patch('ultralytics.engine.trainer.check_amp', self.check_local_amp):
            return super()._setup_train(world_size)

    @staticmethod
    def check_local_amp(model):
        device = next(model.parameters()).device
        if device.type != 'cuda':
            return False
        root = Path(model.yaml['scnet']['weights']).parent.parent
        source = sorted((root/'dataset/images/train').glob('*.png'))[0]
        with Image.open(source) as im:
            array = np.array(im.convert('RGB').resize((256,256)), copy=True)
        tensor = torch.from_numpy(array).permute(2,0,1).unsqueeze(0).to(device).float()/255
        was_training = model.training
        try:
            model.eval()
            with torch.inference_mode():
                full = model(tensor)[0].float()
                with torch.autocast('cuda'):
                    mixed = model(tensor)[0].float()
            passed = (torch.isfinite(full).all() and torch.isfinite(mixed).all()
                and torch.allclose(full[:,:4], mixed[:,:4], rtol=.05, atol=.5)
                and torch.allclose(full[:,4:], mixed[:,4:], rtol=.05, atol=.001))
            LOGGER.info(f'Local AMP check (actual detector, train image): {"passed" if passed else "failed; using FP32"}')
            return bool(passed)
        finally:
            model.train(was_training)

    def get_model(self, cfg=None, weights=None, verbose=True):
        model = SCNetModel(cfg, nc=self.data['nc'], verbose=verbose)
        if weights is not None:
            model.load(weights)
        else:
            options = model.yaml['scnet']
            model.transfer_report = transfer_coco(model, options['weights'], options['variant'])
        return model

    def preprocess_batch(self, batch):
        batch = super().preprocess_batch(batch)
        if not hasattr(self.model, 'criterion'):
            self.model.criterion = self.model.init_criterion()
        warmup = self.model.yaml.get('scnet', {}).get('loss_warmup_epochs', 5)
        self.model.criterion.progress = min(1., (self.epoch + 1) / max(warmup, 1))
        return batch
