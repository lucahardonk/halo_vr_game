#!/usr/bin/env bash
# Crea (o aggiorna) il venv unico dei test preliminari in test_preliminari/.venv
# e installa le dipendenze di tutti i test. Si può rilanciare quando vuoi.
set -euo pipefail

cd "$(dirname "$0")"
VENV_DIR=".venv"

if [ ! -x "$VENV_DIR/bin/python" ]; then
    echo ">> Creo il venv in $(pwd)/$VENV_DIR"
    python3 -m venv "$VENV_DIR"
fi

"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install -r requirements.txt

echo
echo ">> Fatto. Per attivarlo:"
echo "   source $(pwd)/$VENV_DIR/bin/activate"
