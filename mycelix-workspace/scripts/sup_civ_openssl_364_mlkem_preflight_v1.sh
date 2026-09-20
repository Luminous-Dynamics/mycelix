#!/usr/bin/env bash
set -euo pipefail

OPENSSL_VERSION='3.6.4'
OPENSSL_TAG='openssl-3.6.4'
TARBALL="openssl-${OPENSSL_VERSION}.tar.gz"
SOURCE_URL="https://github.com/openssl/openssl/releases/download/${OPENSSL_TAG}/${TARBALL}"
SOURCE_SHA256='9bffaa1ad1e07b354c21bd3324ec02fa15579f45a7d0494b3e74bc449b7333ef'

ROOT="${RUNNER_TEMP:-$(mktemp -d)}/sup-civ-000d1c2a"
SRC="$ROOT/src"
PREFIX="$ROOT/prefix"
WORK="$ROOT/work"
mkdir -p "$ROOT" "$WORK"

cleanup() {
  rm -rf "$ROOT"
}
trap cleanup EXIT

printf 'C2A: downloading pinned OpenSSL %s\n' "$OPENSSL_VERSION"
curl --fail --location --silent --show-error \
  --proto '=https' --tlsv1.2 \
  "$SOURCE_URL" -o "$ROOT/$TARBALL"
printf '%s  %s\n' "$SOURCE_SHA256" "$ROOT/$TARBALL" | sha256sum --check --strict

tar -xzf "$ROOT/$TARBALL" -C "$ROOT"
mv "$ROOT/openssl-$OPENSSL_VERSION" "$SRC"

printf 'C2A: building isolated OpenSSL toolchain\n'
(
  cd "$SRC"
  ./Configure \
    --prefix="$PREFIX" \
    --openssldir="$PREFIX/ssl" \
    no-shared \
    no-tests
  make -j2
  make install_sw
)

OPENSSL="$PREFIX/bin/openssl"
export OPENSSL_MODULES="$PREFIX/lib64/ossl-modules"
if [[ ! -d "$OPENSSL_MODULES" ]]; then
  export OPENSSL_MODULES="$PREFIX/lib/ossl-modules"
fi

version="$($OPENSSL version)"
printf 'C2A: version=%s\n' "$version"
grep -Eq '^OpenSSL 3\.6\.4([[:space:]]|$)' <<<"$version"

printf 'C2A: checking required primitive/provider surfaces\n'
"$OPENSSL" list -kem-algorithms | grep -q 'ML-KEM-768'
"$OPENSSL" list -kdf-algorithms | grep -qi 'HKDF'
"$OPENSSL" list -cipher-algorithms | grep -Eqi 'AES-256-WRAP|id-aes256-wrap'

cms_help="$($OPENSSL cms -help 2>&1 || true)"
grep -q -- '-recip_kdf' <<<"$cms_help"
grep -q -- '-recip_ukm' <<<"$cms_help"

SEED="$(python3 - <<'PY'
print(''.join(f'{i:02x}' for i in range(64)))
PY
)"
IKME="$(python3 - <<'PY'
print(''.join(f'{i:02x}' for i in range(32)))
PY
)"

[[ ${#SEED} -eq 128 ]]
[[ ${#IKME} -eq 64 ]]

printf 'C2A: generating deterministic test-only ML-KEM-768 keys\n'
"$OPENSSL" genpkey \
  -algorithm ML-KEM-768 \
  -pkeyopt "hexseed:$SEED" \
  -out "$WORK/key1.pem"
"$OPENSSL" genpkey \
  -algorithm ML-KEM-768 \
  -pkeyopt "hexseed:$SEED" \
  -out "$WORK/key2.pem"

"$OPENSSL" pkey -in "$WORK/key1.pem" -pubout -outform DER -out "$WORK/pub1.der"
"$OPENSSL" pkey -in "$WORK/key2.pem" -pubout -outform DER -out "$WORK/pub2.der"
cmp "$WORK/pub1.der" "$WORK/pub2.der"
"$OPENSSL" pkey -in "$WORK/key1.pem" -pubout -out "$WORK/pub.pem"

printf 'C2A: deterministic test-only encapsulation\n'
"$OPENSSL" pkeyutl \
  -encap \
  -inkey "$WORK/pub.pem" \
  -pubin \
  -pkeyopt "hexikme:$IKME" \
  -out "$WORK/kemct1.bin" \
  -secret "$WORK/ss1.bin"
"$OPENSSL" pkeyutl \
  -encap \
  -inkey "$WORK/pub.pem" \
  -pubin \
  -pkeyopt "hexikme:$IKME" \
  -out "$WORK/kemct2.bin" \
  -secret "$WORK/ss2.bin"

cmp "$WORK/kemct1.bin" "$WORK/kemct2.bin"
cmp "$WORK/ss1.bin" "$WORK/ss2.bin"

printf 'C2A: decapsulation round trip\n'
"$OPENSSL" pkeyutl \
  -decap \
  -inkey "$WORK/key1.pem" \
  -in "$WORK/kemct1.bin" \
  -secret "$WORK/ss-dec.bin"
cmp "$WORK/ss1.bin" "$WORK/ss-dec.bin"

kemct_len="$(wc -c < "$WORK/kemct1.bin" | tr -d '[:space:]')"
secret_len="$(wc -c < "$WORK/ss1.bin" | tr -d '[:space:]')"
[[ "$kemct_len" == '1088' ]]
[[ "$secret_len" == '32' ]]

printf 'C2A: public-key-sha256=%s\n' "$(sha256sum "$WORK/pub1.der" | awk '{print $1}')"
printf 'C2A: kemct-sha256=%s\n' "$(sha256sum "$WORK/kemct1.bin" | awk '{print $1}')"
printf 'C2A: shared-secret-sha256=%s\n' "$(sha256sum "$WORK/ss1.bin" | awk '{print $1}')"
printf 'C2A PASS: pinned OpenSSL 3.6.4 ML-KEM/CMS tooling preflight completed\n'
