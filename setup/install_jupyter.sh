#!/bin/bash
# Met JupyterLab en service pour le compte etudiant : on programme le robot depuis un navigateur.
#
# - JupyterLab tourne en permanence sous le compte etudiant (service systemd « jupyterlab »), avec
#   le Python et les bibliothèques de /opt/robot/venv (voir install_robot_env.sh).
# - Il écoute sur le port 8888 : http://<nom-du-robot>.local:8888 depuis un ordinateur du même
#   réseau. L'accès est protégé par un mot de passe, stocké seulement sous forme hachée.
# - Les carnets du dépôt (notebooks/) sont copiés dans ~etudiant/notebooks s'ils n'y sont pas déjà :
#   un carnet modifié par les étudiants n'est jamais écrasé.
#
# Usage, sur la Pi, après install_robot_env.sh : sudo setup/install_jupyter.sh
# Le mot de passe est demandé au clavier, ou lu sur l'entrée standard.
set -euo pipefail

USER_NAME=etudiant
PORT=8888
VENV=/opt/robot/venv
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$HERE")

[ "$(id -u)" = 0 ] || { echo "À lancer avec sudo" >&2; exit 1; }
[ -x "$VENV/bin/jupyter-lab" ] || { echo "JupyterLab absent : lancer d'abord setup/install_robot_env.sh" >&2; exit 1; }
HOME_DIR=$(getent passwd "$USER_NAME" | cut -d: -f6)

if [ -t 0 ]; then
    read -r -s -p "Mot de passe de JupyterLab pour $USER_NAME : " PASSWORD
    echo
else
    read -r PASSWORD
fi
[ -n "$PASSWORD" ] || { echo "Mot de passe vide" >&2; exit 1; }

echo "=== Configuration"
HASH=$(PW="$PASSWORD" "$VENV/bin/python" -c 'import os; from jupyter_server.auth import passwd; print(passwd(os.environ["PW"]))')
unset PASSWORD
mkdir -p /etc/jupyter
cat > /etc/jupyter/robot_server_config.py <<EOF
# Configuration de JupyterLab du robot, écrite par setup/install_jupyter.sh
c.ServerApp.ip = "0.0.0.0"
c.ServerApp.port = $PORT
c.ServerApp.open_browser = False
c.ServerApp.allow_remote_access = True
c.ServerApp.root_dir = "$HOME_DIR"
c.PasswordIdentityProvider.hashed_password = "$HASH"
EOF
chmod 644 /etc/jupyter/robot_server_config.py

echo "=== Service jupyterlab"
cat > /etc/systemd/system/jupyterlab.service <<EOF
[Unit]
Description=JupyterLab du robot (compte $USER_NAME, port $PORT)
After=network-online.target
Wants=network-online.target

[Service]
User=$USER_NAME
WorkingDirectory=$HOME_DIR
Environment=PATH=$VENV/bin:/usr/local/bin:/usr/bin:/bin
ExecStart=$VENV/bin/jupyter-lab --config=/etc/jupyter/robot_server_config.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --quiet jupyterlab.service
systemctl restart jupyterlab.service

echo "=== Carnets"
install -d -m 755 -o "$USER_NAME" -g "$USER_NAME" "$HOME_DIR/notebooks"
for notebook in "$REPO"/notebooks/*.ipynb; do
    target="$HOME_DIR/notebooks/$(basename "$notebook")"
    if [ -e "$target" ]; then
        echo "déjà présent, laissé tel quel : $target"
    else
        install -m 644 -o "$USER_NAME" -g "$USER_NAME" "$notebook" "$target"
        echo "copié : $target"
    fi
done

echo "=== Vérification"
for _ in $(seq 1 30); do
    curl -fs -o /dev/null "http://127.0.0.1:$PORT/login" && break
    sleep 1
done
if systemctl is-active --quiet jupyterlab && curl -fs -o /dev/null "http://127.0.0.1:$PORT/login"; then
    echo "JupyterLab répond : http://$(hostname).local:$PORT"
else
    echo "JupyterLab ne répond pas" >&2
    journalctl -u jupyterlab -n 20 --no-pager >&2
    exit 1
fi
