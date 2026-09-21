#!/usr/bin/env bash
set -euo pipefail

MANIFEST="${1:-mycelix-workspace/docs/civic-resilience/sup_civ_000d1c2b2b1_openssl_cms_interop.json}"
OPENSSL_VERSION='3.6.4'
OPENSSL_TAG='openssl-3.6.4'
TARBALL="openssl-${OPENSSL_VERSION}.tar.gz"
SOURCE_URL="https://github.com/openssl/openssl/releases/download/${OPENSSL_TAG}/${TARBALL}"
SOURCE_SHA256='9bffaa1ad1e07b354c21bd3324ec02fa15579f45a7d0494b3e74bc449b7333ef'
RECIP_DER='mycelix-workspace/docs/civic-resilience/vectors/sup_civ_000d1c2b2b0_recipient.der'
PLAINTEXT='mycelix-workspace/docs/civic-resilience/vectors/sup_civ_000d1c2b2b1_plaintext.bin'
SPKI_SHA256='c23e23dd3d485a9256cda09358a4a286e00b373db10761eadf99f710649ca31c'
RECIP_CERT_SHA256='6639c042c6e263aab522652b744f4620ce22d4749156cfc21ed657146fb3804b'
PLAINTEXT_SHA256='d54dc56071885aacd704e4e15ff0325649ade754a98d8a206fd40348d8c7d0c8'
SEED='000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f'
WRONG_SEED='404142434445464748494a4b4c4d4e4f505152535455565758595a5b5c5d5e5f606162636465666768696a6b6c6d6e6f707172737475767778797a7b7c7d7e7f'
UKM='6d7963656c69782f70726f7465637465642d656e76656c6f70652f726563697069656e742d777261702f7631000001f101f111f21111111111111111111111111111111111111111111111111111111111111111110000000000000001000000000000000131313131313131313131313131313131313131313131313131313131313131314141414141414141414141414141414141414141414141414141414141414141'
ROOT="${RUNNER_TEMP:-$(mktemp -d)}/sup-civ-000d1c2b2b1"
SRC="$ROOT/src"
PREFIX="$ROOT/prefix"
WORK="$ROOT/work"
mkdir -p "$ROOT" "$WORK"

cleanup() { rm -rf "$ROOT"; }
trap cleanup EXIT

python3 - "$MANIFEST" <<'PY'
import json, pathlib, sys
m=json.loads(pathlib.Path(sys.argv[1]).read_text())
assert m['schema']=='sup-civ-000d1c2b2b1-openssl-cms-interop-v1'
assert m['structural_parent']=='c3599c9c73e3cec5da6c1352227981859563f61c'
assert m['openssl']['version']=='3.6.4'
assert m['openssl']['source_sha256']=='9bffaa1ad1e07b354c21bd3324ec02fa15579f45a7d0494b3e74bc449b7333ef'
assert m['recipient']['spki_sha256']=='c23e23dd3d485a9256cda09358a4a286e00b373db10761eadf99f710649ca31c'
assert m['cms_profile']['ukm_length']==165
assert m['cms_profile']['ukm_sha256']=='4a6bbfc2baa973f7bf098f730fb1554d677a0c814ba178269571f1044cf8b53d'
assert m['plaintext']['length']==52
assert m['plaintext']['sha256']=='d54dc56071885aacd704e4e15ff0325649ade754a98d8a206fd40348d8c7d0c8'
PY

test "$(sha256sum "$RECIP_DER" | awk '{print $1}')" = "$RECIP_CERT_SHA256"
test "$(sha256sum "$PLAINTEXT" | awk '{print $1}')" = "$PLAINTEXT_SHA256"
test "$(wc -c < "$PLAINTEXT" | tr -d '[:space:]')" = '52'

printf 'B2B1: downloading pinned OpenSSL %s\n' "$OPENSSL_VERSION"
curl --fail --location --silent --show-error --proto '=https' --tlsv1.2 "$SOURCE_URL" -o "$ROOT/$TARBALL"
printf '%s  %s\n' "$SOURCE_SHA256" "$ROOT/$TARBALL" | sha256sum --check --strict

tar -xzf "$ROOT/$TARBALL" -C "$ROOT"
mv "$ROOT/openssl-$OPENSSL_VERSION" "$SRC"
(
  cd "$SRC"
  ./Configure --prefix="$PREFIX" --openssldir="$PREFIX/ssl" no-shared no-tests
  make -j2
  make install_sw
)

OPENSSL="$PREFIX/bin/openssl"
export OPENSSL_MODULES="$PREFIX/lib64/ossl-modules"
if [[ ! -d "$OPENSSL_MODULES" ]]; then
  export OPENSSL_MODULES="$PREFIX/lib/ossl-modules"
fi

version="$($OPENSSL version)"
grep -Eq '^OpenSSL 3\.6\.4([[:space:]]|$)' <<<"$version"
cms_help="$($OPENSSL cms -help 2>&1 || true)"
grep -q -- '-recip_kdf' <<<"$cms_help"
grep -q -- '-recip_ukm' <<<"$cms_help"

printf 'B2B1: regenerate deterministic test-only recipient private key\n'
"$OPENSSL" genpkey -algorithm ML-KEM-768 -pkeyopt "hexseed:$SEED" -out "$WORK/recipient-key.pem"
"$OPENSSL" pkey -in "$WORK/recipient-key.pem" -pubout -outform DER -out "$WORK/generated-spki.der"
test "$(sha256sum "$WORK/generated-spki.der" | awk '{print $1}')" = "$SPKI_SHA256"

"$OPENSSL" x509 -inform DER -in "$RECIP_DER" -out "$WORK/recipient.pem"
"$OPENSSL" x509 -in "$WORK/recipient.pem" -pubkey -noout > "$WORK/cert-pub.pem"
"$OPENSSL" pkey -pubin -in "$WORK/cert-pub.pem" -outform DER -out "$WORK/cert-spki.der"
test "$(sha256sum "$WORK/cert-spki.der" | awk '{print $1}')" = "$SPKI_SHA256"
cmp "$WORK/generated-spki.der" "$WORK/cert-spki.der"

printf 'B2B1: encrypt using CMS KEMRecipientInfo profile\n'
"$OPENSSL" cms -encrypt -binary -keyid \
  -in "$PLAINTEXT" \
  -out "$WORK/cms.der" -outform DER \
  -aes-256-gcm -aes256-wrap \
  -recip "$WORK/recipient.pem" \
  -recip_kdf HKDF-SHA256 \
  -recip_ukm "$UKM"

test -s "$WORK/cms.der"

python3 - "$WORK/cms.der" "$MANIFEST" <<'PY'
import json, pathlib, sys
cms=pathlib.Path(sys.argv[1]).read_bytes()
m=json.loads(pathlib.Path(sys.argv[2]).read_text())
needles={
  'ML-KEM-768 AlgorithmIdentifier': bytes.fromhex(m['required_der_needles']['mlkem768_algorithm_identifier_hex']),
  'HKDF-SHA256 AlgorithmIdentifier': bytes.fromhex(m['required_der_needles']['hkdf_sha256_algorithm_identifier_hex']),
  'AES-256-WRAP AlgorithmIdentifier': bytes.fromhex(m['required_der_needles']['aes256_wrap_algorithm_identifier_hex']),
  'recipient SKI': bytes.fromhex(m['required_der_needles']['recipient_ski_hex']),
  'exact D1B UKM': bytes.fromhex(m['cms_profile']['ukm_hex']),
}
for name, needle in needles.items():
    count=cms.count(needle)
    if count < 1:
        raise AssertionError(f'{name} absent from CMS DER')
ukm=needles['exact D1B UKM']
if cms.count(ukm) != 1:
    raise AssertionError(f'exact UKM must occur once, got {cms.count(ukm)}')
PY

printf 'B2B1: decrypt with matching certificate/private key\n'
"$OPENSSL" cms -decrypt -binary -inform DER \
  -in "$WORK/cms.der" \
  -recip "$WORK/recipient.pem" \
  -inkey "$WORK/recipient-key.pem" \
  -out "$WORK/recovered.bin"
cmp "$PLAINTEXT" "$WORK/recovered.bin"

printf 'B2B1: wrong deterministic ML-KEM private key must fail\n'
"$OPENSSL" genpkey -algorithm ML-KEM-768 -pkeyopt "hexseed:$WRONG_SEED" -out "$WORK/wrong-key.pem"
if "$OPENSSL" cms -decrypt -binary -inform DER \
    -in "$WORK/cms.der" -recip "$WORK/recipient.pem" -inkey "$WORK/wrong-key.pem" \
    -out "$WORK/wrong-recovered.bin" >/dev/null 2>"$WORK/wrong-key.err"; then
  echo 'wrong ML-KEM private key unexpectedly decrypted CMS' >&2
  exit 1
fi

printf 'B2B1: post-encryption UKM mutation must fail\n'
python3 - "$WORK/cms.der" "$WORK/cms-mutated-ukm.der" "$UKM" <<'PY'
import pathlib, sys
src=pathlib.Path(sys.argv[1]).read_bytes()
ukm=bytes.fromhex(sys.argv[3])
if src.count(ukm) != 1:
    raise AssertionError('expected exactly one UKM occurrence before mutation')
pos=src.index(ukm)
mut=bytearray(src)
mut[pos + len(ukm)//2] ^= 0x01
pathlib.Path(sys.argv[2]).write_bytes(mut)
assert bytes(mut).count(ukm) == 0
PY
if "$OPENSSL" cms -decrypt -binary -inform DER \
    -in "$WORK/cms-mutated-ukm.der" -recip "$WORK/recipient.pem" -inkey "$WORK/recipient-key.pem" \
    -out "$WORK/mutated-recovered.bin" >/dev/null 2>"$WORK/mutated-ukm.err"; then
  echo 'UKM-mutated CMS unexpectedly decrypted' >&2
  exit 1
fi

printf 'openssl-version=%s\n' "$version"
printf 'cms-sha256=%s\n' "$(sha256sum "$WORK/cms.der" | awk '{print $1}')"
printf 'plaintext-sha256=%s\n' "$PLAINTEXT_SHA256"
printf 'SUP-CIV-000D1C2B2B1 PASS: pinned OpenSSL CMS ML-KEM interoperability and negative cases completed\n'
