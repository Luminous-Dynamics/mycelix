#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
base64 -d < "$HERE/digest.b64" > "$TMP/digest.bin"
base64 -d < "$HERE/signature.b64" > "$TMP/signature.bin"
openssl pkeyutl -verify -pubin -inkey "$HERE/verification-key.pem" -rawin \
  -in "$TMP/digest.bin" -sigfile "$TMP/signature.bin"
