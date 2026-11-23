#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'
export LC_ALL=C
API_BASE="https://data.neonscience.org/api/v0"
PRODUCT="" SITE="" START_MONTH="" END_MONTH="" RELEASE="" PACKAGE="basic" OUTPUT_DIR=""

usage() {
  cat <<'USAGE'
Usage:
  NEON_API_TOKEN=<token> bash research/terra-preta/tools/acquire-neon-data.sh \
    --product DP1.00094.001 --site SRER --start 2021-01 --end 2021-12 \
    --release RELEASE-2026 --output /path/to/immutable/raw

NEON_API_TOKEN must be supplied through the environment, never an argument or URL.
Required: --product CODE --site SITE --start YYYY-MM --end YYYY-MM
          --release TAG --output DIR
Optional: --package basic|expanded (default: basic)
USAGE
}
valid_month(){ [[ "$1" =~ ^[0-9]{4}-(0[1-9]|1[0-2])$ ]]; }
valid_code(){ [[ "$1" =~ ^DP[0-9]\.[0-9]{5}\.[0-9]{3}$ ]]; }
valid_site(){ [[ "$1" =~ ^[A-Z0-9]{4}$ ]]; }
valid_label(){ [[ "$1" =~ ^[A-Za-z0-9._-]+$ ]]; }

while (($#)); do
  case "$1" in
    --product) PRODUCT="${2:?missing --product value}"; shift 2 ;;
    --site) SITE="${2:?missing --site value}"; shift 2 ;;
    --start) START_MONTH="${2:?missing --start value}"; shift 2 ;;
    --end) END_MONTH="${2:?missing --end value}"; shift 2 ;;
    --release) RELEASE="${2:?missing --release value}"; shift 2 ;;
    --output) OUTPUT_DIR="${2:?missing --output value}"; shift 2 ;;
    --package) PACKAGE="${2:?missing --package value}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

: "${NEON_API_TOKEN:?NEON_API_TOKEN must be set in the environment}"
[[ "$NEON_API_TOKEN" != *$'\n'* && "$NEON_API_TOKEN" != *$'\r'* ]] || { echo "NEON_API_TOKEN must be single-line" >&2; exit 2; }
valid_code "$PRODUCT" || { echo "invalid product code" >&2; exit 2; }
valid_site "$SITE" || { echo "site must be four uppercase alphanumeric characters" >&2; exit 2; }
valid_month "$START_MONTH" && valid_month "$END_MONTH" || { echo "months must use YYYY-MM" >&2; exit 2; }
if [[ "$START_MONTH" > "$END_MONTH" ]]; then echo "start month is after end month" >&2; exit 2; fi
valid_label "$RELEASE" || { echo "invalid release tag" >&2; exit 2; }
[[ "$PACKAGE" == "basic" || "$PACKAGE" == "expanded" ]] || { echo "invalid package type" >&2; exit 2; }
[[ -n "$OUTPUT_DIR" ]] || { echo "--output is required" >&2; exit 2; }

for c in curl jq sha256sum md5sum stat mktemp base64 awk; do
  command -v "$c" >/dev/null 2>&1 || { echo "missing command: $c" >&2; exit 2; }
done

mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd -P)"
METADATA_DIR="$OUTPUT_DIR/metadata"
DOWNLOAD_DIR="$OUTPUT_DIR/files"
mkdir -p "$METADATA_DIR" "$DOWNLOAD_DIR"
chmod 700 "$OUTPUT_DIR" "$METADATA_DIR" "$DOWNLOAD_DIR"

QUERY_PATH="$METADATA_DIR/query_${PRODUCT}_${SITE}_${START_MONTH}_${END_MONTH}_${RELEASE}.json"
REQUEST_PATH="$(mktemp)"
TOKEN_HEADER="$(mktemp)"
trap 'rm -f "$REQUEST_PATH" "$TOKEN_HEADER"' EXIT
chmod 600 "$REQUEST_PATH" "$TOKEN_HEADER"
[[ ! -e "$QUERY_PATH" ]] || { echo "query response exists; refusing overwrite" >&2; exit 2; }

jq -n --arg p "$PRODUCT" --arg s "$SITE" --arg a "$START_MONTH" --arg b "$END_MONTH" \
  --arg r "$RELEASE" --arg pkg "$PACKAGE" \
  '{productCode:$p,siteCodes:[$s],startDateMonth:$a,endDateMonth:$b,release:$r,package:$pkg,includeProvisional:false}' > "$REQUEST_PATH"

# Token is stored in a protected header file, not argv or a query parameter.
printf 'X-API-Token: %s\n' "$NEON_API_TOKEN" > "$TOKEN_HEADER"
unset NEON_API_TOKEN
curl --header "@$TOKEN_HEADER" --fail --silent --show-error \
  --retry 4 --retry-delay 2 --retry-connrefused --connect-timeout 30 --max-time 600 \
  -H 'Content-Type: application/json' -X POST --data-binary "@$REQUEST_PATH" \
  "$API_BASE/data/query" > "$QUERY_PATH.tmp"
jq -e '.data.releases | type == "array"' "$QUERY_PATH.tmp" >/dev/null || {
  echo "invalid NEON data-query response; preserving response" >&2
  mv "$QUERY_PATH.tmp" "$QUERY_PATH"
  exit 1
}
mv "$QUERY_PATH.tmp" "$QUERY_PATH"
chmod 600 "$QUERY_PATH"
COUNT="$(jq '[.data.releases[]?.packages[]?.files[]?] | length' "$QUERY_PATH")"
((COUNT > 0)) || { echo "query returned no files" >&2; exit 1; }

MANIFEST="$OUTPUT_DIR/acquisition-manifest.tsv"
HEADER=$'product_code\trelease\tsite_code\tmonth\tpackage\tfilename\tbytes\tpublisher_md5\tlocal_md5\tlocal_sha256'
if [[ ! -e "$MANIFEST" ]]; then
  printf '%s\n' "$HEADER" > "$MANIFEST"
  chmod 600 "$MANIFEST"
else
  IFS= read -r existing < "$MANIFEST" || true
  [[ "$existing" == "$HEADER" ]] || { echo "manifest header mismatch" >&2; exit 2; }
fi

DOWNLOADED=0
while IFS= read -r encoded; do
  record="$(printf '%s' "$encoded" | base64 --decode)"
  filename="$(jq -r '.name // empty' <<< "$record")"
  bytes="$(jq -r '.size // empty' <<< "$record")"
  publisher_md5="$(jq -r '.md5 // empty' <<< "$record")"
  url="$(jq -r '.url // empty' <<< "$record")"
  [[ -n "$filename" && -n "$bytes" && -n "$publisher_md5" && -n "$url" ]] || { echo "incomplete file metadata" >&2; exit 1; }
  [[ "$filename" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "unsafe filename: $filename" >&2; exit 1; }
  [[ "$bytes" =~ ^[0-9]+$ ]] || { echo "invalid publisher size: $filename" >&2; exit 1; }
  [[ "$publisher_md5" =~ ^[A-Fa-f0-9]{32}$ ]] || { echo "invalid publisher MD5: $filename" >&2; exit 1; }
  [[ "$url" == https://* ]] || { echo "non-HTTPS URL: $filename" >&2; exit 1; }

  target="$DOWNLOAD_DIR/$filename"
  partial="$DOWNLOAD_DIR/.$filename.part"
  [[ ! -e "$target" && ! -e "$partial" ]] || { echo "file exists; refusing overwrite: $target" >&2; exit 2; }

  # The signed storage URL is used without the NEON API credential.
  curl --fail --silent --show-error --location --retry 4 --retry-delay 2 \
    --retry-connrefused --connect-timeout 30 --max-time 1800 "$url" -o "$partial"

  actual_bytes="$(stat -c '%s' "$partial")"
  actual_md5="$(md5sum "$partial" | awk '{print tolower($1)}')"
  publisher_md5="$(printf '%s' "$publisher_md5" | tr '[:upper:]' '[:lower:]')"
  [[ "$actual_bytes" == "$bytes" ]] || { rm -f "$partial"; echo "size mismatch: $filename" >&2; exit 1; }
  [[ "$actual_md5" == "$publisher_md5" ]] || { rm -f "$partial"; echo "MD5 mismatch: $filename" >&2; exit 1; }
  actual_sha256="$(sha256sum "$partial" | awk '{print $1}')"
  mv "$partial" "$target"
  chmod 444 "$target"

  site_month="$(jq -r --arg f "$filename" '[.data.releases[]?.packages[]? | select(any(.files[]?; .name == $f)) | [.siteCode,.month,.packageType] | @tsv] | first // empty' "$QUERY_PATH")"
  IFS=$'\t' read -r file_site file_month file_package <<< "$site_month"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$PRODUCT" "$RELEASE" "${file_site:-$SITE}" "${file_month:-unknown}" "${file_package:-$PACKAGE}" \
    "$filename" "$actual_bytes" "$publisher_md5" "$actual_md5" "$actual_sha256" >> "$MANIFEST"
  DOWNLOADED=$((DOWNLOADED + 1))
  printf 'verified %s (%s bytes)\n' "$filename" "$actual_bytes"
done < <(jq -r '.data.releases[]?.packages[]?.files[]? | @base64' "$QUERY_PATH")

printf 'NEON acquisition complete: %s files verified; query=%s; manifest=%s\n' "$DOWNLOADED" "$QUERY_PATH" "$MANIFEST"
