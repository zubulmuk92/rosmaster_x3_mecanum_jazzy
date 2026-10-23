#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copie dans le dépôt local les fichiers du fabricant (Yahboom) que ce dépôt ne peut pas redistribuer (pas de
# licence) ; git les ignore :
#   - pièces 3D du X3 -> x3_description/meshes/yahboom/ (affichage seulement ; le corps sans écran est obtenu en
#     retirant tous les triangles au-dessus de 0,13 m, c'est-à-dire l'écran et son support)
#   - Rosmaster_Lib (bibliothèque série de la carte STM32) -> pip install pour l'utilisateur (nécessaire à x3_base
#     carte:=serie)
#
#   bash deploy/recuperer_yahboom.sh <dossier yahboomcar_description> [<dossier py_install de Rosmaster_Lib>]
#
# Les deux sont fournis avec le logiciel Yahboom du ROSMASTER X3 (image de la carte SD du robot, ou dossiers
# « yahboomcar_ros2_ws » et « py_install » des téléchargements Yahboom). Recompiler ensuite x3_description.
set -euo pipefail
DESC=${1:?usage : recuperer_yahboom.sh <dossier yahboomcar_description> [<dossier py_install>]}
LIB=${2:-}
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$HERE/x3_description/meshes/yahboom"

for f in front_left_wheel_X3 front_right_wheel_X3 back_left_wheel_X3 back_right_wheel_X3 laser_link camera_link; do
  cp "$DESC/meshes/$f.STL" "$OUT/"
done
python3 - "$DESC/meshes/base_link_X3.STL" "$OUT/base_link_X3_sans_ecran.STL" <<'PY'
import struct, sys
import numpy as np
data = open(sys.argv[1], 'rb').read()
n = struct.unpack('<I', data[80:84])[0]
tri = np.frombuffer(data[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]), count=n)
keep = tri[tri['v'].reshape(-1, 3, 3)[:, :, 2].max(1) <= 0.13]
open(sys.argv[2], 'wb').write(b'X3 base_link sans ecran'.ljust(80, b' ') + struct.pack('<I', len(keep)) + keep.tobytes())
print('corps : %d -> %d triangles (écran retiré)' % (n, len(keep)))
PY
echo "pièces 3D copiées dans $OUT"

if [ -n "$LIB" ]; then
  pip3 install --user --break-system-packages "$LIB"
  echo "Rosmaster_Lib installée depuis $LIB"
fi
