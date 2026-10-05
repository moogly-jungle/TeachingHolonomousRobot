"""Image of the camera: color with the objects recognized by YOLO, depth map and infrared, side by side,
as in tools/camera_snapshot.py."""
import base64
import io

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

MIN_DEPTH = 0.2  # m: the camera does not measure closer


def decode(text):
    """An image sent by the probe (PNG or JPEG in base64), as a numpy array."""
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(text))))


def depth_of(frames):
    """The depth map in meters; 0 where there is no measurement."""
    return decode(frames["depth"]) * frames["depth_scale"]


def depth_summary(frames):
    """The share of the pixels with a depth measurement, and the distance at the center of the image (m)."""
    depth = depth_of(frames)
    height, width = depth.shape
    return float(np.mean(depth > 0)), float(depth[height // 2, width // 2])


def draw(frames, title, path):
    """Saves the three images side by side in `path`, with the objects recognized by YOLO, and returns the figure."""
    color, depth, infrared = decode(frames["color"]), depth_of(frames), decode(frames["infrared"])

    fig, (left, middle, right) = plt.subplots(1, 3, figsize=(18, 5), constrained_layout=True)
    left.imshow(color)
    detections = frames.get("detections")
    left.set_title("color, with the objects recognized by YOLO" if isinstance(detections, list) else "color")
    for detection in detections if isinstance(detections, list) else []:
        x1, y1, x2, y2 = detection["box"]
        left.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, color="lime", lw=2))
        left.text(x1, max(y1 - 6, 12), f"{detection['name']} {detection['confidence']:.0%}", fontsize=9,
                  color="black", backgroundcolor="lime")
    colors = plt.get_cmap("turbo").copy()
    colors.set_bad("black")
    measured = depth[depth > 0]
    far = float(np.percentile(measured, 95)) if measured.size else 4.0  # the color scale follows the scene
    shown = middle.imshow(np.ma.masked_equal(depth, 0), cmap=colors, vmin=MIN_DEPTH, vmax=far)
    middle.set_title("depth (black: no measurement)")
    fig.colorbar(shown, ax=middle, label="distance (m)", shrink=0.9)
    right.imshow(infrared, cmap="gray")
    right.set_title("infrared, left sensor (the dots: the projector)")
    for ax in (left, middle, right):
        ax.axis("off")
    fig.suptitle(title)
    fig.savefig(path, dpi=90)
    return fig
