#!/usr/bin/env python3
"""Dessine les schémas de la documentation des moteurs : docs/img/*.png.

Usage : python3 docs/img/schemas.py   (il faut matplotlib)

Repère du robot : x vers la droite, y vers l'avant, rotation ω positive dans le sens inverse des
aiguilles d'une montre, vu de dessus. Sur les schémas, l'avant est en haut : les coordonnées du
robot sont aussi celles du dessin.
"""
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Arc, Circle, Ellipse, FancyArrowPatch, FancyBboxPatch  # noqa: E402

HERE = Path(__file__).resolve().parent
INK, MUTED, BLUE, RED, GREEN, GHOST = "#1A1C2B", "#5C6076", "#2A3F9D", "#C1443C", "#2E7D4F", "#9EA2B3"
CHASSIS, WHEEL, ROLLER = "#ECE9E1", "#3A3D4F", "#C9C6BD"


def arrow(ax, start, end, color=INK, lw=2.2, ls="-", size=16, z=4, style="-|>"):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=size, color=color, lw=lw,
                                 linestyle=ls, zorder=z, shrinkA=0, shrinkB=0))


def text(ax, xy, s, color=INK, size=14, **kw):
    kw.setdefault("ha", "center")
    kw.setdefault("va", "center")
    ax.text(*xy, s, color=color, fontsize=size, zorder=6, **kw)


def rotation(ax, center, radius, color=BLUE, s=r"$\omega$", size=15, label_angle=45):
    """Flèche courbe dans le sens inverse des aiguilles d'une montre, vu de dessus."""
    cx, cy = center
    start, end = -50.0, 215.0
    ax.add_patch(Arc((cx, cy), 2 * radius, 2 * radius, theta1=start, theta2=end - 8, color=color, lw=2.2, zorder=4))
    a0, a1 = np.radians(end - 12), np.radians(end)
    tail = (cx + radius * np.cos(a0), cy + radius * np.sin(a0))
    tip = (cx + radius * np.cos(a1), cy + radius * np.sin(a1))
    ax.add_patch(FancyArrowPatch(tail, tip, arrowstyle="-|>", mutation_scale=16, color=color, lw=2.2, zorder=4,
                                 shrinkA=0, shrinkB=0))
    la = np.radians(label_angle)
    ax.text(cx + radius * 1.45 * np.cos(la), cy + radius * 1.45 * np.sin(la), s, color=color, fontsize=size,
            ha="center", va="center", zorder=6)


def axes(ax, origin=(0, 0), length=0.12, size=14):
    ox, oy = origin
    arrow(ax, origin, (ox + length, oy), color=INK, lw=1.8, size=13)
    arrow(ax, origin, (ox, oy + length), color=INK, lw=1.8, size=13)
    text(ax, (ox + length + 0.025, oy), "$x$", size=size)
    text(ax, (ox, oy + length + 0.03), "$y$", size=size)


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
    fig, ax = setup(6.2, 6.6)
    hw, hl = 0.21, 0.26  # demi-largeur (selon x), demi-longueur (selon y)
    ax.add_patch(FancyBboxPatch((-hw, -hl), 2 * hw, 2 * hl, boxstyle="round,pad=0,rounding_size=0.03",
                                fc=CHASSIS, ec=MUTED, lw=1.5, zorder=1))
    text(ax, (-0.12, hl - 0.04), "avant", color=MUTED, size=11)
    axes(ax, length=0.13)
    text(ax, (-0.014, -0.034), "$O$", size=13)
    rotation(ax, (0, 0), 0.07, label_angle=235)
    v = (0.06, 0.13)
    arrow(ax, (0, 0), v, color=GREEN, lw=2.6)
    text(ax, (v[0] - 0.13, v[1] + 0.005), r"$\vec v=(v_x,\,v_y)$", color=GREEN, size=13)
    # Point P (par exemple une roue) et sa vitesse : v_P = v + ω k ∧ OP
    p = (0.14, -0.12)
    omega = 0.75
    ax.plot(*p, "o", color=RED, ms=7, zorder=6)
    arrow(ax, (0, 0), p, color=RED, lw=1.2, ls="--", size=10)
    text(ax, (p[0] - 0.01, p[1] - 0.045), r"$P\,(x_P,\,y_P)$", color=RED, size=13)
    rot = (-omega * p[1], omega * p[0])
    tip_v = (p[0] + v[0], p[1] + v[1])
    arrow(ax, p, tip_v, color=GREEN, lw=1.4, size=11)
    vp = (tip_v[0] + rot[0], tip_v[1] + rot[1])
    arrow(ax, tip_v, vp, color=BLUE, lw=1.4, size=11)
    arrow(ax, p, vp, color=RED, lw=2.8)
    text(ax, (vp[0] + 0.045, vp[1] + 0.01), r"$\vec v_P$", color=RED, size=14)
    text(ax, (tip_v[0] + 0.12, tip_v[1] + 0.03), r"$\omega\,\vec k\wedge\overrightarrow{OP}$", color=BLUE, size=11)
    text(ax, (0.02, -hl - 0.07), r"$\vec v_P = (\,v_x-\omega\,y_P\,,\;\; v_y+\omega\,x_P\,)$", size=15)
    ax.set_xlim(-0.3, 0.42)
    ax.set_ylim(-0.37, 0.33)
    save(fig, "repere_robot.png")


def wheel_box(ax, x, y, length=0.12, width=0.036, ghost=False):
    """Roue vue de dessus, qui roule selon y : pleine, ou en transparence."""
    if ghost:
        ax.add_patch(FancyBboxPatch((x - width / 2, y - length / 2), width, length, boxstyle="round,pad=0,rounding_size=0.008",
                                    fc="#F4F4F7", ec=GHOST, lw=1.4, ls="--", zorder=2))
    else:
        ax.add_patch(FancyBboxPatch((x - width / 2, y - length / 2), width, length, boxstyle="round,pad=0,rounding_size=0.008",
                                    fc=WHEEL, ec=WHEEL, zorder=3))


def roller(ax, x, y, slant, length=0.056, thickness=0.017, fc=ROLLER, ec="#8E8B83", lw=1.0, z=4):
    """Un galet vu de dessus : un tonneau allongé selon son axe, incliné en « \\ » ou « / »."""
    ax.add_patch(Ellipse((x, y), length, thickness, angle=45 if slant == "/" else -45, fc=fc, ec=ec, lw=lw, zorder=z))


def mecanum():
    """Base mecanum : galets du dessus (en X), puis galet au contact du sol, croisé avec eux : ce qui pousse et ce qui glisse."""
    fig, (top, ground) = plt.subplots(1, 2, figsize=(13, 9.55))
    lx, ly = 0.16, 0.2  # demi-écarts des roues : gauche-droite (selon x) et avant-arrière (selon y)
    wheels = {"avant gauche": (-lx, ly), "avant droite": (lx, ly), "arrière gauche": (-lx, -ly), "arrière droite": (lx, -ly)}
    slant_top = {"avant gauche": "\\", "avant droite": "/", "arrière gauche": "/", "arrière droite": "\\"}
    for ax, title in ((top, "Ce qu'on voit, de dessus :\nles galets du dessus forment un X"),
                      (ground, "Ce qui touche le sol : le galet du dessous,\ncroisé avec ceux du dessus")):
        ax.set_aspect("equal")
        ax.axis("off")
        ax.add_patch(FancyBboxPatch((-lx + 0.06, -ly - 0.05), 2 * lx - 0.12, 2 * ly + 0.1,
                                    boxstyle="round,pad=0,rounding_size=0.025", fc=CHASSIS, ec=MUTED, lw=1.5, zorder=1))
        text(ax, (0, ly + 0.025), "avant", color=MUTED, size=11)
        text(ax, (0, ly + 0.28), title, size=13, weight="bold")  # assez haut pour l'exemple de la roue avant gauche
        ax.set_xlim(-0.37, 0.37)
        ax.set_ylim(-0.56, 0.53)

    # À gauche : ce qu'on voit sur le robot
    for name, (x, y) in wheels.items():
        wheel_box(top, x, y)
        for k in range(4):
            roller(top, x, y + (k - 1.5) * 0.029, slant_top[name])
        text(top, (x * 1.65, y + (0.085 if y > 0 else -0.085)), name, size=11.5)
    x, y = wheels["avant gauche"]
    arrow(top, (x - 0.07, y - 0.05), (x - 0.07, y + 0.05), color=GREEN, lw=1.8, size=12)
    text(top, (x - 0.08, y - 0.075), "la roue\navance\nselon $y$", color=GREEN, size=10, ha="right", va="top")
    text(top, (0, -ly - 0.15), "Pour vérifier le montage de votre robot :\nvus de dessus, les galets du dessus doivent former un X.",
         color=MUTED, size=10.5)

    # À droite : ce qui se passe au sol ; les galets du dessus restent en gris clair, pour comparaison
    faint = dict(fc="#F1F0EC", ec="#CFCDC6", lw=0.8, z=3)
    for name, (x, y) in wheels.items():
        wheel_box(ground, x, y, ghost=True)
        for k in range(4):
            roller(ground, x, y + (k - 1.5) * 0.029, slant_top[name], **faint)
        slant = "/" if slant_top[name] == "\\" else "\\"  # galet du dessous : incliné dans l'autre sens
        roller(ground, x, y, slant, length=0.075, thickness=0.024, fc="#DCE2F5", ec=BLUE, lw=1.4, z=4)
        sx = 1 if slant == "/" else -1
        u = (sx / np.sqrt(2), 1 / np.sqrt(2))  # vecteur unitaire de l'axe du galet, orienté vers l'avant
        b = 0.045
        ground.plot([x - b * u[1], x + b * u[1]], [y + b * u[0], y - b * u[0]], color=GHOST, lw=1.6, ls=(0, (3, 2)), zorder=5)
        # Le vecteur u_i part du centre de la roue, au-dessus du point de contact avec le sol
        a = 0.068
        tip = (x + a * u[0], y + a * u[1])
        ground.plot(x, y, "o", color=BLUE, ms=4.5, zorder=6)
        arrow(ground, (x, y), tip, color=BLUE, lw=2.6, size=15, z=5)
        if y > 0:  # roues avant : la flèche pointe vers le coin du châssis, l'étiquette va au-dessus
            label = (tip[0], tip[1] + 0.026)
        else:  # roues arrière : la flèche pointe vers l'extérieur, l'étiquette la prolonge
            label = (tip[0] + 0.02 * u[0], tip[1] + 0.02 * u[1] + 0.012)
        text(ground, label, r"$\vec u_i$", color=BLUE, size=14)
    axes(ground, length=0.1, size=13)
    arrow(ground, (0, 0), (-lx, 0), color=RED, lw=1.4, size=11)
    arrow(ground, (0, 0), (0, -ly), color=RED, lw=1.4, size=11)
    text(ground, (-lx / 2, 0.025), "$l_x$", color=RED, size=13)
    text(ground, (0.028, -ly / 2 - 0.02), "$l_y$", color=RED, size=13)
    text(ground, (0.022, -0.028), "$O$", size=12)
    rotation(ground, (0, 0), 0.045, label_angle=225)
    # Exemple sur la roue avant gauche, pour une vitesse v_Pi quelconque du centre de la roue : r ω_i y a la
    # même composante selon u_i que v_Pi, et leur différence, perpendiculaire à u_i, est absorbée par la
    # rotation du galet. Ici v_Pi va vers le haut, près du nord-ouest : presque perpendiculaire à u_i, il
    # ne fait tourner la roue que lentement.
    c = np.array(wheels["avant gauche"])
    u = np.array([1.0, 1.0]) / np.sqrt(2)
    v_p = 0.2 * np.array([np.cos(np.radians(115)), np.sin(np.radians(115))])
    roll = np.array([0.0, (v_p @ u) / u[1]])  # r ω_i y, avec r ω_i = (v_Pi · u_i) / (y · u_i)
    arrow(ground, c, c + v_p, color=RED, lw=2.2, size=13, z=6)
    arrow(ground, c, c + roll, color=GREEN, lw=2.2, size=13, z=6)
    d = (v_p - roll) / np.linalg.norm(v_p - roll)
    end = c + v_p - 0.012 * d  # s'arrête juste avant la pointe de v_Pi
    ground.plot([c[0] + roll[0], end[0] - 0.012 * d[0]], [c[1] + roll[1], end[1] - 0.012 * d[1]], color=MUTED, lw=1.3,
                ls=(0, (3, 2)), zorder=5)
    arrow(ground, end - 0.014 * d, end, color=MUTED, lw=1.3, size=11, z=5)
    text(ground, c + v_p + (-0.022, 0.022), r"$\vec v_{P_i}$", color=RED, size=13)
    text(ground, c + roll + (0.035, 0.022), r"$r\,\omega_i\,\vec y$", color=GREEN, size=13)
    text(ground, c + (v_p + roll) / 2 + (0.05, 0.03), r"$\vec v_{P_i} - r\,\omega_i\,\vec y$", color=MUTED, size=11)
    text(ground, (-0.355, 0.29), "exemple sur\ncette roue", color=MUTED, size=10, ha="left")
    # Légende
    y0 = -ly - 0.12
    roller(ground, -0.297, y0, "/", length=0.05, thickness=0.018, fc="#DCE2F5", ec=BLUE, lw=1.4, z=4)
    text(ground, (-0.26, y0), r"galet du dessous, au contact du sol, d'axe $\vec u_i$ :"
         "\nla roue ne peut pousser que dans cet axe (frottement)", color=BLUE, size=10.5, ha="left")
    roller(ground, -0.297, y0 - 0.09, "\\", length=0.05, thickness=0.018, **faint)
    text(ground, (-0.26, y0 - 0.09), "galets du dessus, pour comparaison : quand la roue\n"
         "fait un demi-tour, un galet du dessus passe dessous,\net son inclinaison s'inverse, d'où le croisement",
         color=MUTED, size=10.5, ha="left")
    ground.plot([-0.32, -0.275], [y0 - 0.19, y0 - 0.19], color=GHOST, lw=1.6, ls=(0, (3, 2)), zorder=5)
    text(ground, (-0.26, y0 - 0.19), "perpendiculairement à $\\vec u_i$, le galet roule :\nla roue glisse librement",
         color=MUTED, size=10.5, ha="left")
    fig.subplots_adjust(wspace=0.04)
    save(fig, "mecanum.png")


def omni_wheel(ax, center, d, length=0.1, width=0.034):
    """Roue holonome vue de dessus, qui roule selon d (dessinée comme une roue double du commerce).

    Deux rangées décalées de galets : l'axe de chaque galet suit la jante, donc vus de dessus ce
    sont des fuseaux allongés selon d. La roue pousse selon d et glisse selon son axe.
    """
    cx, cy = center
    n = (-d[1], d[0])
    corners = [(cx + su * length / 2 * d[0] + sn * width / 2 * n[0], cy + su * length / 2 * d[1] + sn * width / 2 * n[1])
               for su, sn in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    ax.add_patch(plt.Polygon(corners, closed=True, fc=WHEEL, ec=WHEEL, lw=1, zorder=3))
    along = np.degrees(np.arctan2(d[1], d[0]))
    step = 0.23 * length  # pas entre deux galets d'une rangée ; la seconde rangée est décalée d'un demi-pas
    for offset, shift in ((-width / 4, -1.75), (width / 4, -1.25)):
        for k in range(4):
            t = (k + shift) * step
            ax.add_patch(Ellipse((cx + t * d[0] + offset * n[0], cy + t * d[1] + offset * n[1]),
                                 0.2 * length, 0.25 * width, angle=along, fc=ROLLER, ec="#8E8B83", lw=0.8, zorder=4))


def omni_side_view(fig, rect):
    """Encart : une roue holonome double vue de côté ; l'axe de chaque galet suit la jante.

    Deux rangées de 5 galets en tonneau, décalées : ceux de la rangée de devant sont en clair, ceux de
    derrière, plus sombres, apparaissent entre eux.
    """
    ax = fig.add_axes(rect)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(Circle((0, 0), 0.78, fc=WHEEL, ec=WHEEL, zorder=2))
    for k in range(10):
        a = 2 * np.pi * k / 10 + np.pi / 2
        c, t = (np.cos(a), np.sin(a)), (-np.sin(a), np.cos(a))
        front = k % 2 == 0
        ax.add_patch(Ellipse(c, 0.66, 0.38, angle=np.degrees(a) + 90, fc=ROLLER if front else "#A9A69D",
                             ec="#8E8B83" if front else "#7E7B74", lw=0.9, zorder=4 if front else 3))
        if front:  # l'axe du galet, le long de la jante
            ax.plot([c[0] - 0.2 * t[0], c[0] + 0.2 * t[0]], [c[1] - 0.2 * t[1], c[1] + 0.2 * t[1]], color="#6E6B64",
                    lw=1.2, zorder=5)
    ax.add_patch(Circle((0, 0), 0.14, fc=CHASSIS, ec=MUTED, lw=1, zorder=4))
    ax.text(0, -1.7, "roue holonome vue de côté :\nchaque galet tourne autour\nd'un axe qui suit la jante",
            ha="center", va="center", fontsize=9.5, color=MUTED)
    ax.set_xlim(-1.45, 1.45)
    ax.set_ylim(-2.15, 1.4)


def omni3():
    """Base à 3 roues holonomes, vue de dessus."""
    fig, ax = setup(6.8, 7.0)
    radius = 0.2
    ax.add_patch(Circle((0, 0), radius + 0.035, fc=CHASSIS, ec=MUTED, lw=1.5, zorder=1))
    text(ax, (0, radius + 0.06), "avant", color=MUTED, size=11)
    axes(ax, length=0.11)
    rotation(ax, (0, 0), 0.045, label_angle=225)
    for k, theta_deg in enumerate((30, 150, 270), start=1):
        th = np.radians(theta_deg)
        x, y = radius * np.cos(th), radius * np.sin(th)
        d = (-np.sin(th), np.cos(th))  # tangente au cercle : la roue roule (et pousse) selon d
        omni_wheel(ax, (x, y), d)
        ox, oy = x + 0.06 * np.cos(th), y + 0.06 * np.sin(th)
        arrow(ax, (ox, oy), (ox + 0.1 * d[0], oy + 0.1 * d[1]), color=GREEN, lw=2, size=13)
        text(ax, (ox + 0.13 * d[0] + 0.03 * np.cos(th), oy + 0.13 * d[1] + 0.03 * np.sin(th)), f"$\\vec d_{k}$",
             color=GREEN, size=14)
        if k != 3:
            text(ax, (1.5 * x - 0.03 * d[0], 1.5 * y - 0.03 * d[1]), f"roue {k}", size=12)
        else:  # à gauche de la roue : la droite est prise par l'exemple de vitesse
            text(ax, (-0.065, -radius + 0.035), "roue 3", size=12, ha="right")
    # La roue 3 glisse librement le long de son axe (perpendiculairement à d)
    ax.plot([0, 0], [-radius - 0.055, -radius + 0.055], color=GHOST, lw=1.6, ls=(0, (3, 2)), zorder=5)
    text(ax, (-0.035, -radius - 0.075), "glisse", color=MUTED, size=10, ha="right")
    text(ax, (0.0, -radius - 0.155), "la roue 3 pousse selon $\\vec d_3$\net glisse selon son axe", color=MUTED, size=10)
    # Exemple sur la roue 3, pour une vitesse v_P3 quelconque du centre de la roue : seule sa projection sur d_3,
    # r ω_3 d_3, fait tourner la roue ; le reste, le long de l'axe de la roue, est absorbé par les galets.
    c = np.array([0.0, -radius])
    d3 = np.array([1.0, 0.0])
    v_p = 0.15 * np.array([np.cos(np.radians(50)), np.sin(np.radians(50))])
    g = c + (v_p @ d3) * d3  # pointe de r ω_3 d_3, avec r ω_3 = d_3 · v_P3
    ax.plot(*c, "o", color=INK, ms=4, zorder=7)
    arrow(ax, c, c + v_p, color=RED, lw=2.2, size=13, z=6)
    arrow(ax, c, g, color=GREEN, lw=2.2, size=13, z=6)
    s = 0.016  # angle droit : r ω_3 d_3 est la projection de v_P3 sur d_3
    ax.plot([g[0] - s, g[0] - s, g[0]], [g[1], g[1] + s, g[1] + s], color=MUTED, lw=1, zorder=5)
    end = c + v_p - (0, 0.012)  # s'arrête juste avant la pointe de v_P3
    ax.plot([g[0], end[0]], [g[1], end[1] - 0.012], color=MUTED, lw=1.3, ls=(0, (3, 2)), zorder=5)
    arrow(ax, end - (0, 0.014), end, color=MUTED, lw=1.3, size=11, z=5)
    text(ax, c + v_p + (-0.025, 0.018), r"$\vec v_{P_3}$", color=RED, size=13)
    text(ax, g + (0.028, -0.01), r"$r\,\omega_3\,\vec d_3$", color=GREEN, size=13, ha="left")
    text(ax, (g[0] + 0.02, (g[1] + c[1] + v_p[1]) / 2 + 0.015), r"$\vec v_{P_3} - r\,\omega_3\,\vec d_3$", color=MUTED, size=11,
         rotation=90)  # le long de son vecteur
    # Angle θ1 et rayon R
    ax.add_patch(Arc((0, 0), 0.18, 0.18, theta1=0, theta2=30, color=RED, lw=1.6, zorder=4))
    text(ax, (0.115 * np.cos(np.radians(15)), 0.115 * np.sin(np.radians(15))), r"$\theta_1$", color=RED, size=13)
    th = np.radians(150)
    arrow(ax, (0, 0), (radius * np.cos(th), radius * np.sin(th)), color=RED, lw=1.3, ls="--", size=10)
    text(ax, (0.5 * radius * np.cos(th) - 0.005, 0.5 * radius * np.sin(th) + 0.03), "$R$", color=RED, size=14)
    ax.set_xlim(-0.37, 0.37)
    ax.set_ylim(-0.42, 0.34)
    omni_side_view(fig, [0.0, 0.02, 0.24, 0.27])
    save(fig, "holonome3.png")


if __name__ == "__main__":
    robot_frame()
    mecanum()
    omni3()
