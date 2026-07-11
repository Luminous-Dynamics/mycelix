#!/usr/bin/env bash
# Amber A/B/C experiment: does a demurrage-exempt SAP class shift SAP inequality,
# or just shelter the already-protected? Compares final SAP Gini + total commons
# compost across seeds for baseline / Amber-on-low-balance / Amber-on-high-balance.
set -euo pipefail
BIN=$(find "${CARGO_TARGET_DIR:-./target}" -name macro-economy-sim -type f 2>/dev/null | head -1)
[ -n "$BIN" ] || { echo "binary not found; run cargo build --release first" >&2; exit 1; }
SEEDS=(42 123 789 1337 2024)
AGENTS=500
DAYS=365
FRAC=0.15
CAP=20000   # SAP sheltered per Amber holder (fully exempts a child/elder/project fund)

# sap_gini is CSV column 4 (day,total_sap,sap_velocity,sap_gini,...)
run_scenario() {
  local name="$1"; shift
  local gini_sum=0 compost_sum=0 n=0
  for s in "${SEEDS[@]}"; do
    out=$("$BIN" --agents "$AGENTS" --days "$DAYS" --seed "$s" "$@" 2>/tmp/amber_err.$$)
    gini=$(echo "$out" | tail -1 | cut -d, -f4)
    compost=$(grep -oE 'local=[0-9.]+, regional=[0-9.]+, global=[0-9.]+' /tmp/amber_err.$$ \
      | tr -d 'a-z=,' | awk '{print $1+$2+$3}')
    gini_sum=$(awk -v a="$gini_sum" -v b="$gini" 'BEGIN{print a+b}')
    compost_sum=$(awk -v a="$compost_sum" -v b="$compost" 'BEGIN{print a+b}')
    n=$((n+1))
  done
  awk -v g="$gini_sum" -v c="$compost_sum" -v n="$n" -v name="$name" \
    'BEGIN{printf "%-22s  SAP_Gini=%.4f   total_compost=%.0f  (mean of %d seeds)\n", name, g/n, c/n, n}'
  rm -f /tmp/amber_err.$$
}

echo "=== Amber experiment: frac=$FRAC, cap=$CAP SAP, $AGENTS agents, $DAYS days ==="
run_scenario "baseline (no Amber)"
run_scenario "Amber -> low balances"  --amber-frac "$FRAC" --amber-cap "$CAP" --amber-target low
run_scenario "Amber -> high balances" --amber-frac "$FRAC" --amber-cap "$CAP" --amber-target high
