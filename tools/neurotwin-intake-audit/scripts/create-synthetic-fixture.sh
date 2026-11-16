#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: create-synthetic-fixture.sh TARGET_DIR [normal|small]" >&2
  exit 2
fi

target="$1"
mode="${2:-normal}"
case "$mode" in
  normal) count=21 ;;
  small) count=3 ;;
  *) echo "mode must be normal or small" >&2; exit 2 ;;
esac

rm -rf "$target"
mkdir -p "$target"
printf '{}\n' > "$target/dataset_description.json"
printf 'participant_id\tage\n' > "$target/participants.tsv"

for i in $(seq 1 "$count"); do
  label=$(printf 's%03d' "$i")
  mkdir -p "$target/sub-$label/anat"
  printf 'SYNTHETIC_ONLY_NOT_NIFTI\n' > "$target/sub-$label/anat/sub-${label}_T1w.placeholder"
  printf 'sub-%s\t30\n' "$label" >> "$target/participants.tsv"
done
