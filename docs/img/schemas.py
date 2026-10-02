#!/usr/bin/env python3
"""Dessine les schémas de la documentation des moteurs : docs/img/*.png.

Usage : python3 docs/img/schemas.py   (il faut matplotlib)

Repère du robot : x vers l'avant (côté caméra), y vers la gauche, rotation ω positive dans le sens
inverse des aiguilles d'une montre, vu de dessus. Sur les schémas, l'avant est en haut.
"""
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Arc, Circle, FancyArrowPatch, FancyBboxPatch, Polygon  # noqa: E402

HERE = Path(__file__).resolve().parent
INK, MUTED, BLUE, RED, GREEN = "#1A1C2B", "#5C6076", "#2A3F9D", "#C1443C", "#2E7D4F"
CHASSIS, WHEEL, ROLLER = "#ECE9E1", "#3A3D4F", "#C9C6BD"


def P(x, y):
    """Coordonnées du robot (x vers l'avant, y vers la gauche) -> coordonnées du dessin."""
    return (-y, x)


def arrow(ax, start, end, color=INK, lw=2.2, ls="-", size=16, z=4):
    ax.add_patch(FancyArrowPatch(P(*start), P(*end), arrowstyle="-|>", mutation_scale=size, color=color,
                                 lw=lw, linestyle=ls, zorder=z, shrinkA=0, shrinkB=0))


def text(ax, xy, s, color=INK, size=14, **kw):
    kw.setdefault("ha", "center")
    kw.setdefault("va", "center")
    ax.text(*P(*xy), s, color=color, fontsize=size, zorder=6, **kw)


def rotation(ax, center, radius, color=BLUE, s=r"$\omega$", size=15, label_angle=45):
    """Flèche courbe dans le sens inverse des aiguilles d'une montre, vu de dessus."""
    cx, cy = P(*center)
    start, end = -50.0, 215.0
    ax.add_patch(Arc((cx, cy), 2 * radius, 2 * radius, theta1=start, theta2=end - 8, color=color, lw=2.2, zorder=4))
    a0, a1 = math.radians(end - 12), math.radians(end)
    tail = (cx + radius * math.cos(a0), cy + radius * math.sin(a0))
    tip = (cx + radius * math.cos(a1), cy + radius * math.sin(a1))
    ax.add_patch(FancyArrowPatch(tail, tip, arrowstyle="-|>", mutation_scale=16, color=color, lw=2.2, zorder=4,
                                 shrinkA=0, shrinkB=0))
    la = math.radians(label_angle)
    ax.text(cx + radius * 1.45 * math.cos(la), cy + radius * 1.45 * math.sin(la), s, color=color, fontsize=size,
            ha="center", va="center", zorder=6)


def axes(ax, origin=(0, 0), length=0.12, size=14):
    arrow(ax, origin, (origin[0] + length, origin[1]), color=INK, lw=1.8, size=13)
    arrow(ax, origin, (origin[0], origin[1] + length), color=INK, lw=1.8, size=13)
    text(ax, (origin[0] + length + 0.025, origin[1]), "$x$", size=size)
    text(ax, (origin[0], origin[1] + length + 0.03), "$y$", size=size)


def setup(width, height):
    fig, ax = plt.subplots(figsize=(width, height))
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def save(fig, name):
    fig.savefig(HERE / name, dpi=110, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("écrit :", HERE / name)


def robot_frame():
    """Repère du robot, vitesse du robot (v, ω) et vitesse d'un point P."""
    fig, ax = setup(6.4, 6.2)
    lx, ly = 0.26, 0.21
    ax.add_patch(FancyBboxPatch(P(-lx, ly), 2 * ly, 2 * lx, boxstyle="round,pad=0,rounding_size=0.03",
                                fc=CHASSIS, ec=MUTED, lw=1.5, zorder=1))
    text(ax, (lx + 0.035, ly - 0.06), "avant (caméra)", color=MUTED, size=10.5)
    axes(ax, length=0.13)
    text(ax, (-0.035, -0.035), "$O$", size=13)
    rotation(ax, (0, 0), 0.07, label_angle=158)
    v = (0.14, 0.07)
    arrow(ax, (0, 0), v, color=GREEN, lw=2.6)
    text(ax, (v[0] + 0.03, v[1] + 0.06), r"$\vec v=(v_x,\,v_y)$", color=GREEN, size=13)
    # Point P (par exemple une roue) et sa vitesse : v_P = v + ω × OP
    p = (0.12, -0.14)
    omega = 0.7
    ax.plot(*P(*p), "o", color=RED, ms=7, zorder=6)
    arrow(ax, (0, 0), p, color=RED, lw=1.2, ls="--", size=10)
    text(ax, (p[0] - 0.035, p[1] - 0.06), "$P\\,(x_P,\\,y_P)$", color=RED, size=13)
    rot = (-omega * p[1], omega * p[0])
    arrow(ax, p, (p[0] + v[0], p[1] + v[1]), color=GREEN, lw=1.4, size=11)
    arrow(ax, (p[0] + v[0], p[1] + v[1]), (p[0] + v[0] + rot[0], p[1] + v[1] + rot[1]), color=BLUE, lw=1.4, size=11)
    vp = (p[0] + v[0] + rot[0], p[1] + v[1] + rot[1])
    arrow(ax, p, vp, color=RED, lw=2.8)
    text(ax, (vp[0] + 0.035, vp[1] - 0.01), r"$\vec v_P$", color=RED, size=14)
    text(ax, (p[0] + v[0] + rot[0] / 2 + 0.02, p[1] + v[1] + rot[1] / 2 - 0.05), r"$\omega\,\vec k\wedge\overrightarrow{OP}$",
         color=BLUE, size=11)
    text(ax, (-lx - 0.1, 0), r"$\vec v_P = (\,v_x-\omega\,y_P\,,\;\; v_y+\omega\,x_P\,)$", color=INK, size=15)
    ax.set_xlim(-0.36, 0.36)
    ax.set_ylim(-0.42, 0.48)
    save(fig, "repere_robot.png")


def wheel_rect(ax, center, length, width, angle_deg, rollers=None, roller_angle=0.0):
    """Roue vue de dessus : rectangle `length` x `width` (en coordonnées du dessin), tourné de `angle_deg`."""
    cx, cy = center
    a = math.radians(angle_deg)
    u = (math.cos(a), math.sin(a))  # sens de la longueur (plan de la roue)
    w = (-math.sin(a), math.cos(a))  # sens de la largeur (axe de la roue)
    corners = [(cx + su * length / 2 * u[0] + sw * width / 2 * w[0], cy + su * length / 2 * u[1] + sw * width / 2 * w[1])
               for su, sw in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    ax.add_patch(Polygon(corners, closed=True, fc=WHEEL, ec=WHEEL, lw=1, zorder=3))
    if rollers:
        r = math.radians(angle_deg + roller_angle)
        d = (math.cos(r), math.sin(r))
        half = width / 2 / max(abs(math.sin(math.radians(roller_angle))), 0.5)
        for k in range(rollers):
            t = (k + 0.5) / rollers - 0.5
            mx, my = cx + t * length * 0.9 * u[0], cy + t * length * 0.9 * u[1]
            ax.plot([mx - half * d[0] * 0.8, mx + half * d[0] * 0.8], [my - half * d[1] * 0.8, my + half * d[1] * 0.8],
                    color=ROLLER, lw=2.4, solid_capstyle="round", zorder=4)


def mecanum_wheel(ax, x, y, top_roller, length=0.12, width=0.036, rollers=5):
    """Roue mecanum vue de dessus, à la position robot (x, y), qui roule selon x.

    `top_roller` donne l'orientation des galets du dessus dans le dessin : "\\" ou "/".
    """
    cx, cy = P(x, y)
    ax.add_patch(FancyBboxPatch((cx - width / 2, cy - length / 2), width, length,
                                boxstyle="round,pad=0,rounding_size=0.008", fc=WHEEL, ec=WHEEL, zorder=3))
    sx = 1 if top_roller == "/" else -1
    half = width * 0.62
    for k in range(rollers):
        my = cy + ((k + 0.5) / rollers - 0.5) * length * 0.86
        ax.plot([cx - half, cx + half], [my - sx * half, my + sx * half], color=ROLLER, lw=2.6,
                solid_capstyle="round", zorder=4)


def mecanum():
    """Base mecanum vue de dessus, galets du dessus en X."""
    fig, ax = setup(7.2, 7.4)
    lx, ly = 0.2, 0.17
    ax.add_patch(FancyBboxPatch(P(-lx - 0.05, ly - 0.035), 2 * (ly - 0.035), 2 * lx + 0.1,
                                boxstyle="round,pad=0,rounding_size=0.025", fc=CHASSIS, ec=MUTED, lw=1.5, zorder=1))
    text(ax, (lx + 0.17, 0), "avant (caméra)", color=MUTED, size=11)
    wheels = {
        # nom : position (x, y) et galets du dessus vus de dessus ; ensemble, ils dessinent un X
        "avant gauche": ((lx, ly), "\\"),
        "avant droite": ((lx, -ly), "/"),
        "arrière gauche": ((-lx, ly), "/"),
        "arrière droite": ((-lx, -ly), "\\"),
    }
    for name, ((x, y), top_roller) in wheels.items():
        mecanum_wheel(ax, x, y, top_roller)
        text(ax, (x + (0.1 if x > 0 else -0.1), y), name, size=12)
    # Repère et dimensions : les flèches partent du centre O
    arrow(ax, (0, 0), (lx, 0), color=RED, lw=1.6, size=12)
    arrow(ax, (0, 0), (0, ly), color=RED, lw=1.6, size=12)
    text(ax, (lx / 2, -0.03), "$l_x$", color=RED, size=14)
    text(ax, (0.03, ly / 2), "$l_y$", color=RED, size=14)
    text(ax, (lx + 0.025, -0.02), "$x$", size=13)
    text(ax, (0.02, ly + 0.035), "$y$", size=13)
    text(ax, (-0.08, 0.0), "$O$", size=12)
    rotation(ax, (0, 0), 0.05)
    # Roue avant gauche : sens d'avance, et axe du galet au contact du sol (symétrique de celui du dessus)
    x, y = wheels["avant gauche"][0]
    arrow(ax, (x - 0.05, y + 0.06), (x + 0.05, y + 0.06), color=GREEN, lw=1.8, size=12)
    text(ax, (x + 0.075, y + 0.06), "avance", color=GREEN, size=10.5)
    s = 0.07
    arrow(ax, (x - s / 2, y + s / 2), (x + s / 2, y - s / 2), color=BLUE, lw=2, ls="--", size=13, z=5)
    text(ax, (x - 0.13, y + 0.205), r"$\vec u$ : axe du galet" + "\nau contact du sol\n(symétrique de celui\ndu dessus)",
         color=BLUE, size=10.5, ha="left")
    ax.set_xlim(-0.42, 0.37)
    ax.set_ylim(-0.31, 0.45)
    save(fig, "mecanum.png")


def omni3():
    """Base à 3 roues holonomes, vue de dessus."""
    fig, ax = setup(6.8, 6.8)
    radius = 0.2
    ax.add_patch(Circle((0, 0), radius + 0.035, fc=CHASSIS, ec=MUTED, lw=1.5, zorder=1))
    text(ax, (radius + 0.09, 0), "avant", color=MUTED, size=11)
    axes(ax, length=0.11)
    rotation(ax, (0, 0), 0.05)
    for k, theta_deg in enumerate((60, 180, 300), start=1):
        th = math.radians(theta_deg)
        x, y = radius * math.cos(th), radius * math.sin(th)
        # La roue est tangente au cercle : elle roule selon d = (-sin θ, cos θ)
        d = (-math.sin(th), math.cos(th))
        angle_plot = math.degrees(math.atan2(*reversed(P(*d))))
        wheel_rect(ax, P(x, y), 0.1, 0.032, angle_plot, rollers=5, roller_angle=90)
        arrow(ax, (x + 0.06 * math.cos(th), y + 0.06 * math.sin(th)),
              (x + 0.06 * math.cos(th) + 0.1 * d[0], y + 0.06 * math.sin(th) + 0.1 * d[1]), color=GREEN, lw=2, size=13)
        text(ax, (x + 0.06 * math.cos(th) + 0.13 * d[0] + 0.03 * math.cos(th),
                  y + 0.06 * math.sin(th) + 0.13 * d[1] + 0.03 * math.sin(th)), f"$\\vec d_{k}$", color=GREEN, size=14)
        text(ax, (1.42 * x, 1.42 * y), f"roue {k}", size=12)
    # Angle θ1 et rayon R
    ax.add_patch(Arc((0, 0), 0.16, 0.16, theta1=90, theta2=90 + 60, color=RED, lw=1.6, zorder=4))
    text(ax, (0.085 * math.cos(math.radians(30)), 0.085 * math.sin(math.radians(30))), r"$\theta_1$", color=RED, size=13)
    th = math.radians(300)
    arrow(ax, (0, 0), (radius * math.cos(th), radius * math.sin(th)), color=RED, lw=1.3, ls="--", size=10)
    text(ax, (0.5 * radius * math.cos(th) + 0.02, 0.5 * radius * math.sin(th) - 0.04), "$R$", color=RED, size=14)
    ax.set_xlim(-0.36, 0.36)
    ax.set_ylim(-0.34, 0.36)
    save(fig, "holonome3.png")


if __name__ == "__main__":
    robot_frame()
    mecanum()
    omni3()
