#!/usr/bin/env bash
# Acquire immutable public research source artifacts into a local raw-data directory.
# Raw datasets are deliberately not committed to Git.
set -Eeuo pipefail

RAW_DIR="${1:-data/raw/terra-preta}"
FIGSHARE_API_BASE="${FIGSHARE_API_BASE:-https://api.figshare.com/v2}"
ZENODO_API_BASE="${ZENODO_API_BASE:-https://zenodo.org/api/records}"
mkdir -p "$RAW_DIR"

for tool in curl jq md5sum sha256sum stat; do
  command -v "$tool" >/dev/null 2>&1 || {
    printf 'missing required tool: %s\n' "$tool" >&2
    exit 2
  }
done

download() {
  local url="$1" destination="$2"
  curl --fail --show-error --location --retry 4 --retry-all-errors \
    --connect-timeout 30 --max-time 0 \
    --output "$destination" "$url"
  test -s "$destination"
}

verify_file() {
  local file="$1" expected_size="$2" expected_md5="$3" expected_sha256="$4" label="$5"
  local actual_size actual_md5 actual_sha
  actual_size="$(stat -c '%s' "$file")"
  actual_md5="$(md5sum "$file" | awk '{print $1}')"
  actual_sha="$(sha256sum "$file" | awk '{print $1}')"

  if [[ -n "$expected_size" && "$actual_size" != "$expected_size" ]]; then
    printf 'size mismatch for %s: expected %s, got %s\n' "$label" "$expected_size" "$actual_size" >&2
    return 1
  fi
  if [[ -n "$expected_md5" && "${actual_md5,,}" != "${expected_md5,,}" ]]; then
    printf 'publisher MD5 mismatch for %s: expected %s, got %s\n' "$label" "$expected_md5" "$actual_md5" >&2
    return 1
  fi
  if [[ -n "$expected_sha256" && "${actual_sha,,}" != "${expected_sha256,,}" ]]; then
    printf 'publisher SHA-256 mismatch for %s: expected %s, got %s\n' "$label" "$expected_sha256" "$actual_sha" >&2
    return 1
  fi

  printf '%s\t%s\t%s\t%s\t%s\n' \
    "$label" "$actual_size" "$actual_md5" "$actual_sha" "$file" \
    >> "$RAW_DIR/acquisition-manifest.tsv"
  printf 'verified %-38s size=%s md5=%s sha256=%s\n' \
    "$label" "$actual_size" "$actual_md5" "$actual_sha"
}

fetch_figshare_file() {
  local article_id="$1" filename="$2" label="$3"
  local metadata="$RAW_DIR/.figshare-${article_id}-files.json"
  local info url expected_md5 expected_size destination

  if [[ ! -s "$metadata" ]]; then
    download "$FIGSHARE_API_BASE/articles/${article_id}/files" "$metadata"
  fi
  info="$(jq -ce --arg name "$filename" '
    [.[] | select(.name == $name)] |
    if length == 1 then .[0]
    else error("expected exactly one matching file in Figshare item")
    end
  ' "$metadata")"
  url="$(jq -r '.download_url // empty' <<<"$info")"
  expected_md5="$(jq -r '.computed_md5 // .supplied_md5 // .md5 // empty' <<<"$info")"
  expected_size="$(jq -r '.size // empty' <<<"$info")"
  destination="$RAW_DIR/$filename"

  [[ -n "$url" ]] || {
    printf 'Figshare metadata has no download_url for %s (item %s)\n' "$filename" "$article_id" >&2
    return 1
  }
  download "$url" "$destination"
  verify_file "$destination" "$expected_size" "$expected_md5" "" "$label"
}

fetch_zenodo_file() {
  local record_id="$1" filename="$2" label="$3"
  local metadata="$RAW_DIR/.zenodo-${record_id}.json"
  local info url checksum expected_md5 expected_sha256 expected_size destination

  if [[ ! -s "$metadata" ]]; then
    download "$ZENODO_API_BASE/${record_id}" "$metadata"
  fi
  info="$(jq -ce --arg name "$filename" '
    [.files[] | select((.key // .filename) == $name)] |
    if length == 1 then .[0]
    else error("expected exactly one matching file in Zenodo record")
    end
  ' "$metadata")"
  url="$(jq -r '.links.download // .links.self // empty' <<<"$info")"
  checksum="$(jq -r '.checksum // empty' <<<"$info")"
  expected_md5=""
  expected_sha256=""
  case "$checksum" in
    md5:*) expected_md5="${checksum#md5:}" ;;
    sha256:*) expected_sha256="${checksum#sha256:}" ;;
    "") printf 'warning: no publisher checksum for %s; local SHA-256 will still be recorded\n' "$filename" >&2 ;;
    *) printf 'warning: unsupported publisher checksum for %s: %s\n' "$filename" "$checksum" >&2 ;;
  esac
  expected_size="$(jq -r '.size // empty' <<<"$info")"
  destination="$RAW_DIR/$filename"

  [[ -n "$url" ]] || {
    printf 'Zenodo metadata has no file download link for %s (record %s)\n' "$filename" "$record_id" >&2
    return 1
  }
  download "$url" "$destination"
  verify_file "$destination" "$expected_size" "$expected_md5" "$expected_sha256" "$label"
}

: > "$RAW_DIR/acquisition-manifest.tsv"
printf 'source_artifact\tsize_bytes\tpublisher_md5\tlocal_sha256\tlocal_path\n' \
  > "$RAW_DIR/acquisition-manifest.tsv"

printf '\n== BiocharDS V1.0: CSV and full workbook ==\n'
fetch_figshare_file 24781764 BiocharDS_V1.0.csv biochards_csv
fetch_figshare_file 24781737 BiocharDS_V1.0.xlsx biochards_full_workbook

printf '\n== Soils4Africa: field context and dictionary ==\n'
fetch_zenodo_file 17699297 README.md soils4africa_readme
fetch_zenodo_file 17699297 s4a_field_data-tableschema.csv soils4africa_tableschema
fetch_zenodo_file 17699297 s4a_field_data.csv soils4africa_field_csv

printf '\n== Western Highveld: South African soil/spectra reference ==\n'
fetch_zenodo_file 17510653 NWU_NIR_MIR_Dataset.csv western_highveld_nir_mir

printf '\nAcquisition complete. Preserve this directory as immutable raw source data.\n'
printf 'Manifest: %s/acquisition-manifest.tsv\n' "$RAW_DIR"
printf 'Do not treat download success as dataset validation, permission to redistribute, or evidence of biochar efficacy.\n'
