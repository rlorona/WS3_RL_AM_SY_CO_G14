#!/usr/bin/env bash
# Creates the deliverable tarball without build/install/log and without the
# Clearpath repositories (they are re-downloaded with dependencies.repos).
set -euo pipefail

ROOT="${HOME}/workshop_03"
NAME="WS3_grupo14_RASC"
OUT="${HOME}/${NAME}.tar"

cd "${ROOT}"
rm -f "${OUT}"

tar \
  --exclude='./build' \
  --exclude='./install' \
  --exclude='./log' \
  --exclude='./src/clearpath_ws' \
  --exclude='./GUION_VIDEO.md' \
  --exclude='__pycache__' \
  --exclude='*.pyc' \
  --exclude='*.tar.gz' \
  --exclude='./WS3_grupo14_RASC.tar' \
  -cf "${OUT}" .

echo "Paquete creado: ${OUT}"
echo "Tamaño: $(du -h "${OUT}" | cut -f1)"
echo
echo "Contenido principal:"
tar -tf "${OUT}" | grep -E 'docs/|maps/|omni_workshop/omni_workshop/|scripts/' | head -40
