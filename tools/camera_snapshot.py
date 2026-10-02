#!/usr/bin/env python3
"""Photo couleur et carte de profondeur de la RealSense, côte à côte, dans une image.

La profondeur est alignée sur l'image couleur : chaque pixel de la carte correspond au même
point de la photo. Les pixels sans mesure de profondeur sont en noir. La distance au centre de
l'image est indiquée par une croix.

Usage : python3 tools/camera_snapshot.py [--image camera.png] [--max-depth 4]
"""
import argparse

import matplotlib

matplotlib.use("Agg")  # pas besoin d'écran
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pyrealsense2 as rs  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--image", default="camera.png")
    parser.add_argument("--max-depth", type=float, default=4.0, help="distance (m) en haut de l'échelle de couleurs")
    parser.add_argument("--rotate", type=int, choices=(0, 180), default=0,
                        help="rotation de l'image, en degrés : 180 si la caméra est montée tête en bas")
    args = parser.parse_args()

    devices = rs.context().query_devices()
    if len(devices) == 0:
        raise SystemExit("Aucune RealSense détectée")
    device = devices[0]
    usb = device.get_info(rs.camera_info.usb_type_descriptor)
    print(f"{device.get_info(rs.camera_info.name)} | n° de série {device.get_info(rs.camera_info.serial_number)}"
          f" | firmware {device.get_info(rs.camera_info.firmware_version)} | USB {usb}")

    # En USB 2, la bande passante limite les résolutions et les cadences
    color_mode, depth_mode = ((1280, 720, 30), (848, 480, 30)) if usb.startswith("3") else ((640, 480, 15), (640, 480, 15))
    config = rs.config()
    config.enable_stream(rs.stream.color, *color_mode[:2], rs.format.rgb8, color_mode[2])
    config.enable_stream(rs.stream.depth, *depth_mode[:2], rs.format.z16, depth_mode[2])
    pipeline = rs.pipeline()
    profile = pipeline.start(config)
    try:
        scale = profile.get_device().first_depth_sensor().get_depth_scale()  # mètres par unité
        align = rs.align(rs.stream.color)
        for _ in range(30):  # laisse l'exposition automatique se régler
            pipeline.wait_for_frames()
        frames = align.process(pipeline.wait_for_frames())
        color = np.asanyarray(frames.get_color_frame().get_data()).copy()
        depth = np.asanyarray(frames.get_depth_frame().get_data()).astype(float) * scale
    finally:
        pipeline.stop()
    if args.rotate == 180:
        color, depth = color[::-1, ::-1], depth[::-1, ::-1]

    height, width = depth.shape
    center = depth[height // 2, width // 2]
    valid = depth > 0
    print(f"couleur {color_mode[0]}x{color_mode[1]}, profondeur {depth_mode[0]}x{depth_mode[1]} alignée sur la couleur"
          f" | {100 * valid.mean():.0f} % des pixels mesurés | distance au centre : "
          + (f"{center:.2f} m" if center > 0 else "pas de mesure"))
    if valid.any():
        p5, p50, p95 = np.percentile(depth[valid], [5, 50, 95])
        print(f"profondeur des pixels mesurés : 5 % sous {p5:.2f} m, médiane {p50:.2f} m, 95 % sous {p95:.2f} m")

    shown = np.ma.masked_where(~valid, depth)
    cmap = plt.get_cmap("turbo").copy()
    cmap.set_bad("black")
    fig, (left, right) = plt.subplots(1, 2, figsize=(14, 4.6), constrained_layout=True)
    left.imshow(color)
    left.set_title("Caméra couleur")
    image = right.imshow(shown, cmap=cmap, vmin=0.2, vmax=args.max_depth)
    right.set_title("Profondeur (noir : pas de mesure)")
    for ax in (left, right):
        ax.plot(width / 2, height / 2, "w+", markersize=18, markeredgewidth=2)
        ax.axis("off")
    right.annotate(f"{center:.2f} m" if center > 0 else "?", (width / 2, height / 2), xytext=(12, -12),
                   textcoords="offset points", color="white", fontsize=12, fontweight="bold")
    fig.colorbar(image, ax=right, label="distance (m)", shrink=0.9)
    fig.savefig(args.image, dpi=100)
    print("image enregistrée :", args.image)


if __name__ == "__main__":
    main()
