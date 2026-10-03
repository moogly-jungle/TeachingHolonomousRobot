#!/bin/bash
# Prépare le port série du GPIO pour le bus Dynamixel.
#
# - Le vrai UART de la Pi (PL011 : /dev/ttyAMA0, lien /dev/serial0) passe sur les GPIO 14/15,
#   et le Bluetooth se contente du mini-UART, ce qui suffit pour une manette.
# - La console de démarrage est retirée du port série : sinon le noyau écrirait sur le bus moteurs.
#
# Usage, sur la Pi, depuis une copie du dépôt : sudo setup/configure_serial.sh
# puis redémarrer si le script le demande. Il peut être relancé sans risque.
set -euo pipefail

CONFIG=/boot/firmware/config.txt
CMDLINE=/boot/firmware/cmdline.txt

[ "$(id -u)" = 0 ] || { echo "À lancer avec sudo" >&2; exit 1; }

changed=0
if grep -q 'console=serial0' "$CMDLINE"; then
    raspi-config nonint do_serial_cons 1
    changed=1
fi
if ! grep -q '^enable_uart=1' "$CONFIG"; then
    raspi-config nonint do_serial_hw 0
    changed=1
fi
if ! grep -q '^dtoverlay=miniuart-bt' "$CONFIG"; then
    # « [all] » : la ligne vaut pour toutes les cartes, même si le fichier finit par une section [cm4]…
    printf '\n[all]\n# Bus Dynamixel sur les GPIO 14/15 : le vrai UART (PL011) va aux broches,\n# le Bluetooth passe sur le mini-UART (suffisant pour une manette).\ndtoverlay=miniuart-bt\n' >> "$CONFIG"
    changed=1
fi

if [ "$changed" = 1 ]; then
    echo "Configuration modifiée : redémarrer la Pi (sudo reboot) pour l'appliquer."
elif [ "$(readlink /dev/serial0)" = ttyAMA0 ]; then
    echo "Port série prêt : /dev/serial0 -> ttyAMA0, sur les GPIO 14/15."
else
    echo "Configuration en place, mais /dev/serial0 ne pointe pas encore vers ttyAMA0 : redémarrer la Pi."
fi
