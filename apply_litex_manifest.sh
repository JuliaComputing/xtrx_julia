#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Force the user to explicitly install litex somewhere, instead of just into the current directory
LITEX_CHECKOUT_PATH="${1}"
if [[ -z "${1}" ]]; then
    echo "Usage: apply_litex_manifest.sh /usr/local/litex" >&2
    exit 1
fi

# Ensure that litex is installed (we just install latest master, we don't care, we'll override later)
curl -fLso "${LITEX_CHECKOUT_PATH}/litex_setup.py" https://raw.githubusercontent.com/enjoy-digital/litex/master/litex_setup.py
(cd "${LITEX_CHECKOUT_PATH}"; python3 litex_setup.py --update)

# For each line in the manifest, reset the concordant git repo to that sha
while IFS="," read -r repo sha; do
    git -C "${LITEX_CHECKOUT_PATH}/${repo}" reset --hard "${sha}" || true
done < "${SCRIPT_DIR}/LitexManifest"

# Install the updated versions
(cd "${LITEX_CHECKOUT_PATH}"; python3 litex_setup.py --install)
