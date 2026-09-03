#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
MANIFEST="$ROOT_DIR/crates/mycelix-nix-cache/Cargo.toml"
TMP=$(mktemp -d)
PORT=18087
CACHE_URL="http://127.0.0.1:${PORT}"
CACHE_STORE="${CACHE_URL}?trusted=false"
SERVER_PID=""

cleanup() {
  if [[ -n "$SERVER_PID" ]]; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  rm -rf "$TMP"
}
trap cleanup EXIT

# Deliberately avoid nixpkgs. This is a zero-input, input-addressed derivation
# whose builder uses only /bin/sh and the shell builtin printf. Sandboxing is
# disabled for this one fixture so /bin/sh is available without introducing a
# store dependency on a package set.
cat > "$TMP/fixture.nix" <<'NIX'
builtins.derivation {
  name = "cf07-stock-nix-qualification";
  system = builtins.currentSystem;
  builder = "/bin/sh";
  args = [
    "-c"
    ''
      printf 'cf07-stock-nix-qualified\n' > "$out"
    ''
  ];
}
NIX

STORE_PATH=$(nix-build --option sandbox false --no-out-link "$TMP/fixture.nix")
[[ "$STORE_PATH" == /nix/store/*-cf07-stock-nix-qualification ]]

# The negative-key half of this qualification is meaningful only for an
# input-addressed store object. Ask Nix itself for JSON v3 store-object facts
# instead of inferring the addressing mode from the fixture expression.
nix path-info --json --json-format 3 "$STORE_PATH" > "$TMP/path-info.json"
python3 - "$TMP/path-info.json" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as handle:
    payload = json.load(handle)

if isinstance(payload, list):
    if len(payload) != 1:
        raise SystemExit(f"expected one path-info record, found {len(payload)}")
    info = payload[0]
elif isinstance(payload, dict) and "path" in payload:
    info = payload
elif isinstance(payload, dict) and len(payload) == 1:
    info = next(iter(payload.values()))
else:
    raise SystemExit("unrecognized nix path-info JSON shape")

if not isinstance(info, dict):
    raise SystemExit("nix path-info record is not an object")
if info.get("ca") is not None:
    raise SystemExit(f"qualification fixture is content-addressed: ca={info.get('ca')!r}")
references = info.get("references")
if references != []:
    raise SystemExit(f"qualification fixture unexpectedly has references: {references!r}")
PY

# Keep the legacy query as a second independent reference check. It also makes
# failures easier to diagnose if Nix changes an experimental JSON representation.
REFERENCES=$(nix-store --query --references "$STORE_PATH")
if [[ -n "$REFERENCES" ]]; then
  printf 'qualification fixture unexpectedly has runtime references:\n%s\n' "$REFERENCES" >&2
  exit 1
fi

nix key generate-secret --key-name cf07-qualification-1 > "$TMP/secret-key"
PUBLIC_KEY=$(nix key convert-secret-to-public < "$TMP/secret-key")
nix store sign --key-file "$TMP/secret-key" "$STORE_PATH"

# Nix 2.35 JSON format 3 represents signatures structurally. Recover the exact
# Nix-generated Sig line from that machine-readable representation instead of
# scraping human-readable output.
nix path-info --json --json-format 3 --sigs "$STORE_PATH" > "$TMP/signed-path-info.json"
SIGNATURE=$(python3 - "$TMP/signed-path-info.json" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as handle:
    payload = json.load(handle)

if isinstance(payload, list):
    if len(payload) != 1:
        raise SystemExit(f"expected one signed path-info record, found {len(payload)}")
    info = payload[0]
elif isinstance(payload, dict) and "path" in payload:
    info = payload
elif isinstance(payload, dict) and len(payload) == 1:
    info = next(iter(payload.values()))
else:
    raise SystemExit("unrecognized signed nix path-info JSON shape")

signatures = info.get("signatures") if isinstance(info, dict) else None
if not isinstance(signatures, list):
    raise SystemExit("nix path-info did not return a signatures array")
matches = [
    f"{item.get('keyName')}:{item.get('sig')}"
    for item in signatures
    if isinstance(item, dict) and item.get("keyName") == "cf07-qualification-1" and item.get("sig")
]
if len(matches) != 1:
    raise SystemExit(f"expected exactly one qualification signature, found {len(matches)}")
print(matches[0])
PY
)

nix store verify --trusted-public-keys "$PUBLIC_KEY" "$STORE_PATH"
nix store dump-path "$STORE_PATH" > "$TMP/fixture.nar"

cargo run --quiet --manifest-path "$MANIFEST" --example qualification_server -- \
  "$TMP/cas" "$PORT" "$STORE_PATH" "$TMP/fixture.nar" "$SIGNATURE" \
  >"$TMP/server.log" 2>&1 &
SERVER_PID=$!

for _ in $(seq 1 120); do
  if curl --fail --silent --show-error "$CACHE_URL/nix-cache-info" > /dev/null; then
    break
  fi
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    cat "$TMP/server.log" >&2
    exit 1
  fi
  sleep 1
done
curl --fail --silent --show-error "$CACHE_URL/nix-cache-info" > /dev/null

STORE_BASENAME=${STORE_PATH##*/}
STORE_HASH=${STORE_BASENAME%%-*}
curl --fail --silent --show-error "$CACHE_URL/${STORE_HASH}.narinfo" > "$TMP/served.narinfo"
grep -F "StorePath: $STORE_PATH" "$TMP/served.narinfo"
grep -F "Compression: none" "$TMP/served.narinfo"
grep -F "Sig: $SIGNATURE" "$TMP/served.narinfo"

# A different public key must not authorize the signed input-addressed object.
# The HTTP source store is explicitly untrusted rather than relying only on the
# HTTP store's default.
nix key generate-secret --key-name cf07-wrong-1 > "$TMP/wrong-secret-key"
WRONG_PUBLIC_KEY=$(nix key convert-secret-to-public < "$TMP/wrong-secret-key")
NEGATIVE_STORE="$TMP/negative-store"
if NIX_CONFIG="experimental-features = nix-command
trusted-public-keys = $WRONG_PUBLIC_KEY
require-sigs = true" \
  nix copy --from "$CACHE_STORE" --to "$NEGATIVE_STORE" "$STORE_PATH"; then
  echo 'stock Nix accepted CF-07 content without a trusted matching signature' >&2
  exit 1
fi
if [[ -e "$NEGATIVE_STORE$STORE_PATH" ]]; then
  echo 'failed negative substitution left the store object installed' >&2
  exit 1
fi

# The exact same explicitly-untrusted HTTP cache succeeds once Nix is given the
# matching public key. CF-07 itself never becomes a trusted store.
POSITIVE_STORE="$TMP/positive-store"
NIX_CONFIG="experimental-features = nix-command
trusted-public-keys = $PUBLIC_KEY
require-sigs = true" \
  nix copy --from "$CACHE_STORE" --to "$POSITIVE_STORE" "$STORE_PATH"

nix store verify \
  --store "$POSITIVE_STORE" \
  --trusted-public-keys "$PUBLIC_KEY" \
  "$STORE_PATH"

test -e "$POSITIVE_STORE$STORE_PATH"
test "$(cat "$POSITIVE_STORE$STORE_PATH")" = 'cf07-stock-nix-qualified'

echo 'PASS: stock Nix rejected the wrong key and restored the signed object through explicitly-untrusted CF-07 with the trusted key.'
