#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'
export LC_ALL=C
API_BASE="https://data.neonscience.org/api/v0"
PRODUCT="" SITES_FILE="" START_MONTH="" END_MONTH="" RELEASE="" PACKAGE="basic" OUTPUT_DIR=""
usage() {
  cat <<'USAGE'
Usage:
  NEON_API_TOKEN=<token> bash research/terra-preta/tools/inventory-neon-data.sh \
    --product DP1.00094.001 --sites-file sites.txt --start 2019-01 --end 2025-06 \
    --release RELEASE-2026 --output /path/to/metadata-inventory
Metadata only: this tool does not download product files.
USAGE
}
valid_month() { [[ "$1" =~ ^[0-9]{4}-(0[1-9]|1[0-2])$ ]]; }
valid_code() { [[ "$1" =~ ^DP[0-9]\.[0-9]{5}\.[0-9]{3}$ ]]; }
valid_site() { [[ "$1" =~ ^[A-Z0-9]{4}$ ]]; }
valid_label() { [[ "$1" =~ ^[A-Za-z0-9._-]+$ ]]; }
while (($#)); do
  case "$1" in
    --product) PRODUCT="${2:?missing --product value}"; shift 2 ;;
    --sites-file) SITES_FILE="${2:?missing --sites-file value}"; shift 2 ;;
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
valid_month "$START_MONTH" && valid_month "$END_MONTH" || { echo "months must use YYYY-MM" >&2; exit 2; }
[[ "$START_MONTH" > "$END_MONTH" || "$START_MONTH" == "$END_MONTH" ]] || { echo "start month is after end month" >&2; exit 2; }
valid_label "$RELEASE" || { echo "invalid release tag" >&2; exit 2; }
[[ "$PACKAGE" == "basic" || "$PACKAGE" == "expanded" ]] || { echo "package must be basic or expanded" >&2; exit 2; }
[[ -f "$SITES_FILE" ]] || { echo "--sites-file must name an existing file" >&2; exit 2; }
[[ -n "$OUTPUT_DIR" ]] || { echo "--output is required" >&2; exit 2; }
for command_name in curl jq sha256sum mktemp; do
  command -v "$command_name" >/dev/null 2>&1 || { echo "missing command: $command_name" >&2; exit 2; }
done
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd -P)"
METADATA_DIR="$OUTPUT_DIR/metadata"
mkdir -p "$METADATA_DIR"
chmod 700 "$OUTPUT_DIR" "$METADATA_DIR"
INVENTORY="$OUTPUT_DIR/file-inventory.tsv"
QUERY_MANIFEST="$OUTPUT_DIR/query-manifest.tsv"
[[ ! -e "$INVENTORY" && ! -e "$QUERY_MANIFEST" ]] || { echo "inventory exists; refusing overwrite" >&2; exit 2; }
printf '%s\n' $'product_code\trelease\tsite_code\tmonth\tpackage\tfilename\tbytes\tpublisher_md5\tpublisher_crc32c\tquery_sha256' > "$INVENTORY"
printf '%s\n' $'product_code\trelease\tsite_code\tstart_month\tend_month\tpackage\tfile_count\tquery_sha256\tresponse_file' > "$QUERY_MANIFEST"
chmod 600 "$INVENTORY" "$QUERY_MANIFEST"
REQUEST_PATH="$(mktemp)"
TOKEN_HEADER="$(mktemp)"
trap 'rm -f "$REQUEST_PATH" "$TOKEN_HEADER"' EXIT
chmod 600 "$REQUEST_PATH" "$TOKEN_HEADER"
printf 'X-API-Token: %s\n' "$NEON_API_TOKEN" > "$TOKEN_HEADER"
unset NEON_API_TOKEN
declare -A SEEN_SITES=()
SITE_COUNT=0
TOTAL_FILES=0
while IFS= read -r site || [[ -n "$site" ]]; do
  site="${site%%#*}"
  site="${site//[[:space:]]/}"
  [[ -z "$site" ]] && continue
  valid_site "$site" || { echo "invalid site code: $site" >&2; exit 2; }
  [[ -z "${SEEN_SITES[$site]+x}" ]] || { echo "duplicate site: $site" >&2; exit 2; }
  SEEN_SITES["$site"]=1
  SITE_COUNT=$((SITE_COUNT + 1))
  response_name="query_${PRODUCT}_${site}_${START_MONTH}_${END_MONTH}_${RELEASE}.json"
  response_path="$METADATA_DIR/$response_name"
  response_tmp="$response_path.tmp"
  [[ ! -e "$response_path" && ! -e "$response_tmp" ]] || { echo "query response exists: $response_path" >&2; exit 2; }
  jq -n --arg p "$PRODUCT" --arg s "$site" --arg a "$START_MONTH" --arg b "$END_MONTH" --arg r "$RELEASE" --arg pkg "$PACKAGE" \
    '{productCode:$p,siteCodes:[$s],startDateMonth:$a,endDateMonth:$b,release:$r,package:$pkg,includeProvisional:false}' > "$REQUEST_PATH"
  curl --header "@$TOKEN_HEADER" --fail --silent --show-error --retry 4 --retry-delay 2 --retry-connrefused --connect-timeout 30 --max-time 600 \
    -H 'Content-Type: application/json' -X POST --data-binary "@$REQUEST_PATH" "$API_BASE/data/query" > "$response_tmp"
  jq -e '.data.releases | type == "array"' "$response_tmp" >/dev/null || {
    echo "invalid NEON query response for $site; response retained" >&2
    mv "$response_tmp" "$response_path"
    exit 1
  }
  mv "$response_tmp" "$response_path"
  chmod 600 "$response_path"
  query_sha256="$(sha256sum "$response_path" | awk '{print $1}')"
  rows="$(jq -r '.data.releases[]? as $release | $release.packages[]? as $package | $package.files[]? | [($package.siteCode // ""),($package.month // ""),($package.packageType // ""),(.name // ""),((.size // "") | tostring),(.md5 // ""),(.crc32c // "")] | @tsv' "$response_path")"
  file_count=0
  if [[ -n "$rows" ]]; then
    while IFS=$'\t' read -r file_site file_month file_package filename bytes publisher_md5 publisher_crc32c; do
      [[ "$file_site" == "$site" ]] || { echo "unexpected site in response: $file_site" >&2; exit 1; }
      [[ "$file_month" =~ ^[0-9]{4}-(0[1-9]|1[0-2])$ ]] || { echo "invalid month for $site / $filename" >&2; exit 1; }
      [[ "$file_package" == "basic" || "$file_package" == "expanded" ]] || { echo "invalid package for $site / $filename" >&2; exit 1; }
      [[ "$filename" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "unsafe or empty filename for $site" >&2; exit 1; }
      [[ "$bytes" =~ ^[0-9]+$ ]] || { echo "invalid file size for $site / $filename" >&2; exit 1; }
      [[ "$publisher_md5" =~ ^[A-Fa-f0-9]{32}$ ]] || { echo "invalid publisher MD5 for $site / $filename" >&2; exit 1; }
      [[ -z "$publisher_crc32c" || "$publisher_crc32c" =~ ^[A-Fa-f0-9]+$ ]] || { echo "invalid publisher CRC32C for $site / $filename" >&2; exit 1; }
      printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$PRODUCT" "$RELEASE" "$file_site" "$file_month" "$file_package" "$filename" "$bytes" "${publisher_md5,,}" "${publisher_crc32c,,}" "$query_sha256" >> "$INVENTORY"
      file_count=$((file_count + 1))
    done <<< "$rows"
  fi
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$PRODUCT" "$RELEASE" "$site" "$START_MONTH" "$END_MONTH" "$PACKAGE" "$file_count" "$query_sha256" "$response_name" >> "$QUERY_MANIFEST"
  TOTAL_FILES=$((TOTAL_FILES + file_count))
  printf 'inventoried %s: %s files; query SHA-256 %s\n' "$site" "$file_count" "$query_sha256"
done < "$SITES_FILE"
((SITE_COUNT > 0)) || { echo "sites file contained no site codes" >&2; exit 2; }
printf 'NEON metadata inventory complete: %s sites; %s file records; no product data downloaded\n' "$SITE_COUNT" "$TOTAL_FILES"
printf 'file inventory: %s\nquery manifest: %s\n' "$INVENTORY" "$QUERY_MANIFEST"
