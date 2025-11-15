#!/bin/sh

SCRIPT_PATH="$(dirname "$(realpath "$0")")"
SCRIPT_NAME="$(basename "$(realpath "$0")")"
SCRIPT_BASE="$(basename -s .sh "$SCRIPT_NAME")"
OS_NAME="$(uname -s)"
AS_PATH="/usr/bin"

unset ELECTRON_RUN_AS_NODE

exec "$AS_PATH/advantagescope" "--no-sandbox"