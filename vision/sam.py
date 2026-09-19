"""MobileSAM through ONNX Runtime on the CPU: click-to-mask on one image.

Model files (44 MB, Apache 2.0) come from https://huggingface.co/vietanhdev/segment-anything-onnx-models,
file mobile_sam_20230629.zip, unpacked into vision/models/. The encoder runs once per image (about a second), and
each prompt point then costs a few milliseconds.
"""
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

MODELS = Path(__file__).parent / "models"
INPUT_H, INPUT_W = 684, 1024  # the size this export was built for


class Sam:
    def __init__(self):
        options = ort.SessionOptions()
        options.log_severity_level = 3
        self.encoder = ort.InferenceSession(str(MODELS / "mobile_sam.encoder.onnx"), options, providers=["CPUExecutionProvider"])
        self.decoder = ort.InferenceSession(str(MODELS / "sam_vit_h_4b8939.decoder.onnx"), options, providers=["CPUExecutionProvider"])

    def set_image(self, bgr):
        self.shape = bgr.shape[:2]
        self.scale = min(INPUT_W / bgr.shape[1], INPUT_H / bgr.shape[0])
        matrix = np.array([[self.scale, 0, 0], [0, self.scale, 0]], np.float32)
        fitted = cv2.warpAffine(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), matrix, (INPUT_W, INPUT_H), flags=cv2.INTER_LINEAR)
        self.embedding = self.encoder.run(None, {"input_image": fitted.astype(np.float32)})[0]

    def mask_at(self, points, labels=None):
        """points: [(x, y), ...] in image pixels, labels 1 = on the object, 0 = not on it. Returns (mask, score)."""
        points = np.asarray(points, np.float32).reshape(-1, 2) * self.scale
        labels = np.ones(len(points), np.float32) if labels is None else np.asarray(labels, np.float32)
        # SAM expects one padding point with label -1 when no box is given.
        coords = np.concatenate([points, [[0, 0]]])[None].astype(np.float32)
        point_labels = np.concatenate([labels, [-1]])[None].astype(np.float32)
        masks, scores, _ = self.decoder.run(None, {
            "image_embeddings": self.embedding, "point_coords": coords, "point_labels": point_labels,
            "mask_input": np.zeros((1, 1, 256, 256), np.float32), "has_mask_input": np.zeros(1, np.float32),
            "orig_im_size": np.array([INPUT_H, INPUT_W], np.float32),
        })
        best = int(np.argmax(scores[0]))
        mask = (masks[0, best] > 0).astype(np.uint8)
        h, w = int(round(self.shape[0] * self.scale)), int(round(self.shape[1] * self.scale))
        return cv2.resize(mask[:h, :w], self.shape[::-1], interpolation=cv2.INTER_NEAREST), float(scores[0, best])
