#!/usr/bin/env bash
# Creates the final deliverable package:
#   1. Entregable_1_Codigo:   working folder with the implemented source code
#                             (ship it as .tar, or .zip if they cannot handle tar)
#   2. Entregable_2_Documentacion: REPORT_WS3.md with the answers to the
#                             workshop questions 2.1-2.5
#   3. Entregable_3_Videos:   trajectory + mapping videos (Rviz/Gazebo real time)
set -euo pipefail

ROOT="${HOME}/workshop_03"
NAME="WS3_grupo14_RASC"
OUT="${HOME}/${NAME}.tar"
STAGE="/tmp/opencode/${NAME}_stage"

rm -f "${OUT}"
rm -rf "${STAGE}"
mkdir -p "${STAGE}/Entregable_1_Codigo/src" \
         "${STAGE}/Entregable_2_Documentacion" \
         "${STAGE}/Entregable_3_Videos"

cp -r "${ROOT}/src/omni_workshop" "${ROOT}/src/worlds" "${ROOT}/src/omni_teleop_gui" \
      "${STAGE}/Entregable_1_Codigo/src/"
cp -f "${ROOT}/src/README.md" "${ROOT}/src/REQUIREMENTS.md" "${ROOT}/src/dependencies.repos" \
      "${STAGE}/Entregable_1_Codigo/"

find "${STAGE}/Entregable_1_Codigo" -type d -name "__pycache__" -prune -exec rm -rf {} +

cp -f "${ROOT}/src/docs/REPORT_WS3.md" "${STAGE}/Entregable_2_Documentacion/"

cp -f /home/ricardo/Downloads/WS3G14TrayectoriaRASC.mp4 "${STAGE}/Entregable_3_Videos/"
cp -f "${ROOT}/src/entregable 2/mapeo_coral.mp4" "${STAGE}/Entregable_3_Videos/"

tar -C "${STAGE}" -cf "${OUT}" \
    Entregable_1_Codigo \
    Entregable_2_Documentacion \
    Entregable_3_Videos

rm -rf "${STAGE}"

echo "Paquete creado: ${OUT}"
echo "Tamaño: $(du -h "${OUT}" | cut -f1)"
echo
echo "Contenido del paquete:"
tar -tf "${OUT}" | grep -E '^Entregable' | sort -u
