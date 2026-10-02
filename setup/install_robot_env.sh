#!/bin/bash
# Installe l'environnement Python partagé du robot dans /opt/robot.
#
# - Python 3.12, installé par uv : pyrealsense2 n'a pas de version ARM pour Python 3.13,
#   celui de Raspberry Pi OS Trixie.
# - Un environnement virtuel /opt/robot/venv avec pypot, pyrealsense2, pyserial, matplotlib,
#   ipython, JupyterLab et ipywidgets. Il est en lecture seule pour les étudiants, mais chacun
#   peut y ajouter des paquets pour lui seul avec « pip install --user ». Pas de PyLidar3, qui
#   est bogué : le lidar se lit avec notre pilote, holorobot.lidar.
# - La bibliothèque holorobot : le dépôt est copié dans /opt/robot/src et installé dans
#   l'environnement, pour qu'elle s'importe de n'importe où. Relancer le script la met à jour.
# - /opt/robot/venv/bin en tête du PATH de tous les comptes.
# - Les règles udev d'Intel, pour utiliser la RealSense sans être root.
#
# Usage, sur la Pi, depuis une copie du dépôt : sudo setup/install_robot_env.sh
# Le script peut être relancé : il ne refait que ce qui manque.
set -euo pipefail

PREFIX=/opt/robot
PYTHON_VERSION=3.12
UV_VERSION=0.12.21
UV_SHA256=030b69227b40af8c1981b7301793dc66e71ed3c796ea8688209dd268bd91ec51
PACKAGES=(pypot pyrealsense2 pyserial matplotlib ipython jupyterlab ipywidgets)
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$HERE")

[ "$(id -u)" = 0 ] || { echo "À lancer avec sudo" >&2; exit 1; }
[ "$(uname -m)" = aarch64 ] || { echo "Prévu pour une Raspberry Pi en 64 bits" >&2; exit 1; }

echo "=== uv $UV_VERSION"
if [ "$(/usr/local/bin/uv --version 2>/dev/null | cut -d' ' -f2)" != "$UV_VERSION" ]; then
    tmp=$(mktemp -d)
    curl -fsSL -o "$tmp/uv.tar.gz" \
        "https://github.com/astral-sh/uv/releases/download/$UV_VERSION/uv-aarch64-unknown-linux-gnu.tar.gz"
    echo "$UV_SHA256  $tmp/uv.tar.gz" | sha256sum -c --quiet -
    tar -xzf "$tmp/uv.tar.gz" -C "$tmp"
    install -m 755 "$tmp"/uv-aarch64-unknown-linux-gnu/{uv,uvx} /usr/local/bin/
    rm -rf "$tmp"
fi

echo "=== Python $PYTHON_VERSION et environnement $PREFIX/venv"
# Tout reste sous $PREFIX, hors des dossiers personnels, pour que chaque compte y ait accès
export UV_PYTHON_INSTALL_DIR=$PREFIX/python UV_PYTHON_BIN_DIR=$PREFIX/python/bin
export UV_CACHE_DIR=/var/cache/uv UV_PYTHON_PREFERENCE=only-managed
/usr/local/bin/uv python install "$PYTHON_VERSION"
if [ ! -x "$PREFIX/venv/bin/python" ]; then
    # --system-site-packages laisse actif le « pip install --user » de chaque étudiant
    /usr/local/bin/uv venv --python "$PYTHON_VERSION" --system-site-packages --seed "$PREFIX/venv"
fi
/usr/local/bin/uv pip install --python "$PREFIX/venv/bin/python" "${PACKAGES[@]}"

echo "=== Bibliothèque holorobot"
# Copie appartenant à root : les étudiants l'utilisent sans pouvoir la modifier
mkdir -p "$PREFIX/src"
rsync -a --delete --exclude .git --exclude __pycache__ "$REPO/" "$PREFIX/src/TeachingHolonomousRobot/"
/usr/local/bin/uv pip install --python "$PREFIX/venv/bin/python" -e "$PREFIX/src/TeachingHolonomousRobot"
chmod -R a+rX "$PREFIX"

echo "=== PATH de tous les comptes"
cat > /etc/profile.d/robot-env.sh <<'EOF'
# Environnement Python du robot (pypot, pyrealsense2…), installé par setup/install_robot_env.sh
case ":$PATH:" in
    *":/opt/robot/venv/bin:"*) ;;
    *) PATH="/opt/robot/venv/bin:$PATH" ;;
esac
export PATH
EOF
if ! grep -q robot-env.sh /etc/bash.bashrc; then
    printf '\n# Environnement Python du robot, aussi dans les terminaux qui ne sont pas des connexions\n[ -r /etc/profile.d/robot-env.sh ] && . /etc/profile.d/robot-env.sh\n' >> /etc/bash.bashrc
fi

echo "=== Règles udev de la RealSense (librealsense v2.58.4)"
install -m 644 "$HERE/99-realsense-libusb.rules" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb

echo "=== Vérification"
"$PREFIX/venv/bin/python" - <<'EOF'
import sys
from importlib.metadata import version

import holorobot.lidar, ipywidgets, jupyterlab, matplotlib, numpy, pyrealsense2, pypot.dynamixel, serial  # noqa: F401

print("Python", sys.version.split()[0], "|", " | ".join(
    f"{p} {version(p)}" for p in ("holorobot", "pypot", "pyrealsense2", "pyserial", "numpy", "matplotlib", "jupyterlab")))
EOF
