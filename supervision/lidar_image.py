"""Image of a lidar scan and its segments, drawn as in the dashboard notebook (tableau_de_bord.ipynb)."""
import matplotlib.pyplot as plt
import numpy as np


def draw(scan, title, path):
    """Saves the image of a scan in `path` and returns the figure: points in grey, segments colored by confidence."""
    xy = np.array(scan["points"])
    fig, ax = plt.subplots(figsize=(7, 7))
    # Top view, the 0° of the lidar at the top: a point (x, y) of the lidar is drawn at (-y, x)
    ax.plot(-xy[:, 1], xy[:, 0], ".", color="0.6", ms=3)
    colors = plt.get_cmap("viridis")
    for segment in scan["segments"]:
        (x1, y1), (x2, y2) = segment["start"], segment["end"]
        ax.plot([-y1, -y2], [x1, x2], color=colors(segment["confidence"]), lw=4)
    ax.plot(0, 0, "r^", ms=10)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("m")
    ax.set_ylabel("m")
    ax.set_title(title)
    fig.colorbar(plt.cm.ScalarMappable(cmap=colors, norm=plt.Normalize(0, 1)), ax=ax, shrink=0.75,
                 label="confidence")
    fig.savefig(path, dpi=110, bbox_inches="tight")
    return fig
