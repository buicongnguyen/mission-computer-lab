"""Synthetic camera and a deliberately simple, untrained ONNX segmentation graph."""

from pathlib import Path
import time
import numpy as np
import onnx
from onnx import TensorProto, helper
import onnxruntime as ort


def create_model(path):
    graph = helper.make_graph(
        [
            helper.make_node('ReduceMean', ['image'], ['brightness'], axes=[1], keepdims=1),
            helper.make_node('Greater', ['brightness', 'threshold'], ['mask']),
        ],
        'synthetic_brightness_detector',
        [helper.make_tensor_value_info('image', TensorProto.FLOAT, [1, 3, 48, 64])],
        [helper.make_tensor_value_info('mask', TensorProto.BOOL, [1, 1, 48, 64])],
        [helper.make_tensor('threshold', TensorProto.FLOAT, [], [0.7])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)], producer_name='mission-computer-lab')
    model.ir_version = 8
    onnx.checker.check_model(model)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(path))


def camera_frame(t, present=True):
    frame = np.full((1, 3, 48, 64), 0.15, dtype=np.float32)
    frame[:, 2, :, :] = 0.35
    if present:
        x = int(27 + 17 * np.sin(t * 0.35))
        y = int(21 + 7 * np.cos(t * 0.22))
        frame[:, :, y : y + 4, x : x + 9] = 0.95
        frame[:, :, y - 2 : y + 6, x + 3 : x + 6] = 0.95
    return frame


class Detector:
    def __init__(self, model):
        # Prefer verified bytes from security.boot_gate over a path that could change after checking.
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            model if isinstance(model, bytes) else str(model), sess_options=options, providers=['CPUExecutionProvider']
        )

    def infer(self, frame):
        start = time.perf_counter_ns()
        mask = self.session.run(['mask'], {'image': frame})[0][0, 0]
        elapsed = (time.perf_counter_ns() - start) / 1e6
        ys, xs = np.where(mask)
        bbox = [int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)] if len(xs) else None
        return {'bbox': bbox, 'pixels': int(len(xs)), 'label': 'synthetic aerial object', 'inference_ms': elapsed}
