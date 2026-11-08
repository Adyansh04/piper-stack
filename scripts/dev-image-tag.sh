#!/usr/bin/env bash
# Prints the dev image's tag: a hash of everything it is built from (docker/dev/). CI publishes
# the image under this tag and manage.sh pulls it, so a machine builds the image only when its
# inputs changed.
#
#   ./scripts/dev-image-tag.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
readonly ROOT

find "${ROOT}/docker/dev" -type f -print0 | sort -z | xargs -0 sha256sum \
    | sed "s#${ROOT}/##" | sha256sum | cut -c1-16
