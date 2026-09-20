"""MobileSAM through ONNX Runtime on the CPU: click-to-mask on one image.

Model files (44 MB, Apache 2.0) come from https://huggingface.co/vietanhdev/segment-anything-onnx-models,
file mobile_sam_20230629.zip, unpacked into vision/models/. The encoder runs once per image (about 0.36 s), and only
when a point is asked about that image. Each prompt point then costs a few milliseconds.
"""
import os
from concurrent.futures import ThreadPoolExecutor
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
        # The decoder is small. One thread per call and many calls at once is much faster than the reverse
        # (ONNX Runtime releases Python's lock while it runs, so plain threads work).
        single = ort.SessionOptions()
        single.log_severity_level = 3
        single.intra_op_num_threads = 1
        self.decoder = ort.InferenceSession(str(MODELS / "sam_vit_h_4b8939.decoder.onnx"), single, providers=["CPUExecutionProvider"])
        self.workers = max(2, (os.cpu_count() or 4) - 2)
        self.pool = ThreadPoolExecutor(max_workers=self.workers)

    def set_image(self, bgr):
        """The encoder is most of a repeated scan (0.36 s of 0.5 s), and a scan of an unchanged arena asks about no
        point at all. The image is therefore only encoded when the first point is asked."""
        self.shape = bgr.shape[:2]
        self.scale = min(INPUT_W / bgr.shape[1], INPUT_H / bgr.shape[0])
        self.image, self.embedding = bgr, None

    def encode(self):
        if self.embedding is None:
            matrix = np.array([[self.scale, 0, 0], [0, self.scale, 0]], np.float32)
            fitted = cv2.warpAffine(cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB), matrix, (INPUT_W, INPUT_H), flags=cv2.INTER_LINEAR)
            self.embedding = self.encoder.run(None, {"input_image": fitted.astype(np.float32)})[0]

    def mask_at(self, points, labels=None):
        """points: [(x, y), ...] in image pixels, labels 1 = on the object, 0 = not on it. Returns (mask, score)."""
        self.encode()
        points = np.asarray(points, np.float32).reshape(-1, 2) * self.scale
        labels = np.ones(len(points), np.float32) if labels is None else np.asarray(labels, np.float32)
        # SAM expects one padding point with label -1 when no box is given.
        coords = np.concatenate([points, [[0, 0]]])[None].astype(np.float32)
        point_labels = np.concatenate([labels, [-1]])[None].astype(np.float32)
        # Only the small 256 x 256 mask is requested. The model's full-size output is that same mask enlarged, and
        # asking for it made every call about 25 ms. Enlarging here, only to the size needed, takes about 1 ms.
        scores, low = self.decoder.run(["iou_predictions", "low_res_masks"], {
            "image_embeddings": self.embedding, "point_coords": coords, "point_labels": point_labels,
            "mask_input": np.zeros((1, 1, 256, 256), np.float32), "has_mask_input": np.zeros(1, np.float32),
            "orig_im_size": np.array([INPUT_H, INPUT_W], np.float32),
        })
        best = int(np.argmax(scores[0]))
        # The 256 x 256 mask covers the 1024 x 1024 padded input, so the image occupies its top-left part.
        h, w = self.shape[0] * self.scale / 4, self.shape[1] * self.scale / 4
        logits = cv2.resize(low[0, best][: int(np.ceil(h)), : int(np.ceil(w))], None, fx=self.shape[1] / w, fy=self.shape[0] / h, interpolation=cv2.INTER_LINEAR)
        mask = np.zeros(self.shape, np.uint8)
        hh, ww = min(self.shape[0], logits.shape[0]), min(self.shape[1], logits.shape[1])
        mask[:hh, :ww] = logits[:hh, :ww] > 0
        return mask, float(scores[0, best])

    def masks_at(self, points):
        """One mask per point, computed in parallel. Returns [(mask, score), ...] in the order of the points."""
        self.encode()  # before the threads start, so that they do not each encode the image
        return list(self.pool.map(lambda p: self.mask_at([p]), points))
