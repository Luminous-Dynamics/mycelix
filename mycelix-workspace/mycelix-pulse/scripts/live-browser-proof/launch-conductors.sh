#!/usr/bin/env bash
# Launches two independent, throwaway Holochain sandbox conductors for the
# Pulse working-alpha live-browser proof — one per simulated user (Alice,
# Bob). Each gets its own admin/app port pair and its own fresh agent
# keystore, so they are two genuinely distinct identities, not two apps on
# one conductor. Uses --in-process-lair to avoid depending on a separate
# lair-keystore binary.
#
# Ports: matches the project's documented "disposable conductor on
# 4444/4445" convention (CLAUDE.md) for Alice; Bob gets the adjacent
# 4446/4447 pair, both inside the ad-hoc dev/test range. Temporary and torn
# down by stop-conductors.sh — not a persistent service allocation.
#
# Requires: hc + holochain 0.6.1 on PATH (or HC_PATH/HOLOCHAIN_PATH env),
# and a freshly packed holochain/dna-alpha/mycelix_pulse_alpha.happ (run
# `just build-happ` first).
#
# Networking: both conductors are pointed at LOCAL bootstrap + relay
# servers instead of the real internet-facing dev-test-bootstrap2.holochain.org.
# This removes the external-network round-trip from peer discovery/gossip
# entirely, which is both faster and immune to outside network flakiness —
# but it takes TWO separate local servers, not one:
#   - kitsune2-bootstrap-srv for agent-info bootstrap (works fine for this —
#     bootstrap is a simple protocol-agnostic HTTP GET/POST).
#   - iroh-relay (from the iroh-relay-holochain crate, the exact fork/version
#     this Holochain build's `hc sandbox generate network quic` transport
#     depends on) for the actual P2P relay/NAT-traversal. kitsune2-bootstrap-srv
#     ALSO bundles a relay component (SBD), but that speaks the older
#     tx5/WebRTC signal protocol, not iroh's — pointing the QUIC transport's
#     relay_url at it produces "400 Bad Request" on iroh's HTTPS netcheck
#     probe and the websocket relay dial never completes, so peer discovery
#     succeeds (agent info shows up) but no P2P connection or gossip ever
#     happens. Root-caused 2026-07-18 via RUST_LOG=debug on both conductors.
# Requires kitsune2-bootstrap-srv AND iroh-relay on PATH (or
# KITSUNE2_BOOTSTRAP_SRV_PATH / IROH_RELAY_PATH env) — install with:
#   cargo install kitsune2-bootstrap-srv
#   cargo install iroh-relay-holochain --version 0.95.1 --features server --bin iroh-relay --locked
# (match the iroh-relay-holochain version to whatever `iroh-holochain` this
# Holochain build actually depends on — check its Cargo.lock if upgrading.)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PULSE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
HAPP_PATH="${HAPP_PATH:-$PULSE_DIR/holochain/mycelix_pulse_alpha.happ}"
SANDBOX_ROOT="${SANDBOX_ROOT:-/tmp/mycelix-pulse-live-proof}"
HC_BIN="${HC_PATH:-hc}"
BOOTSTRAP_BIN="${KITSUNE2_BOOTSTRAP_SRV_PATH:-kitsune2-bootstrap-srv}"
BOOTSTRAP_PORT="${BOOTSTRAP_PORT:-9600}"
BOOTSTRAP_URL="http://127.0.0.1:${BOOTSTRAP_PORT}"
RELAY_BIN="${IROH_RELAY_PATH:-iroh-relay}"
RELAY_PORT="${RELAY_PORT:-3340}"
RELAY_URL="http://127.0.0.1:${RELAY_PORT}"

if [[ ! -f "$HAPP_PATH" ]]; then
  echo "hApp bundle not found at $HAPP_PATH — run 'just build-happ' first." >&2
  exit 1
fi

mkdir -p "$SANDBOX_ROOT"
rm -rf "$SANDBOX_ROOT"/alice "$SANDBOX_ROOT"/bob
mkdir -p "$SANDBOX_ROOT"/alice "$SANDBOX_ROOT"/bob

echo "[launch-conductors] starting local bootstrap server on :${BOOTSTRAP_PORT}"
"$BOOTSTRAP_BIN" --listen "127.0.0.1:${BOOTSTRAP_PORT}" \
  > "$SANDBOX_ROOT"/bootstrap.log 2>&1 &
echo $! > "$SANDBOX_ROOT"/bootstrap.pid

echo "[launch-conductors] starting local iroh-relay server on :${RELAY_PORT}"
"$RELAY_BIN" --dev \
  > "$SANDBOX_ROOT"/relay.log 2>&1 &
echo $! > "$SANDBOX_ROOT"/relay.pid

for svc in "bootstrap:$BOOTSTRAP_PORT" "relay:$RELAY_PORT"; do
  name="${svc%%:*}"
  port="${svc##*:}"
  up=0
  for _ in $(seq 1 30); do
    if (exec 3<>/dev/tcp/127.0.0.1/"$port") 2>/dev/null; then
      exec 3>&- 3<&-
      up=1
      break
    fi
    sleep 1
  done
  if [[ "$up" -ne 1 ]]; then
    echo "[launch-conductors] FAILED: local $name server never came up." >&2
    cat "$SANDBOX_ROOT"/"$name".log >&2 2>/dev/null || true
    exit 1
  fi
done
echo "[launch-conductors] local bootstrap is up on ${BOOTSTRAP_URL}, relay is up on ${RELAY_URL}"

echo "[launch-conductors] Alice: admin=4444 app=4445"
"$HC_BIN" sandbox --piped -f=4444 generate --in-process-lair -a mycelix_mail -r=4445 \
  --root "$SANDBOX_ROOT"/alice -d alice "$HAPP_PATH" \
  network -b "$BOOTSTRAP_URL" quic "$RELAY_URL" \
  < /dev/null > "$SANDBOX_ROOT"/alice.log 2>&1 &
echo $! > "$SANDBOX_ROOT"/alice.pid

echo "[launch-conductors] Bob: admin=4446 app=4447"
"$HC_BIN" sandbox --piped -f=4446 generate --in-process-lair -a mycelix_mail -r=4447 \
  --root "$SANDBOX_ROOT"/bob -d bob "$HAPP_PATH" \
  network -b "$BOOTSTRAP_URL" quic "$RELAY_URL" \
  < /dev/null > "$SANDBOX_ROOT"/bob.log 2>&1 &
echo $! > "$SANDBOX_ROOT"/bob.pid

echo "[launch-conductors] waiting for both admin websockets..."
for port in 4444 4446; do
  up=0
  for _ in $(seq 1 120); do
    if (exec 3<>/dev/tcp/127.0.0.1/"$port") 2>/dev/null; then
      exec 3>&- 3<&-
      echo "[launch-conductors] admin port $port is up"
      up=1
      break
    fi
    sleep 1
  done
  if [[ "$up" -ne 1 ]]; then
    echo "[launch-conductors] FAILED: admin port $port never came up after 120s." >&2
    echo "[launch-conductors] alice.log / bob.log:" >&2
    tail -n 40 "$SANDBOX_ROOT"/alice.log "$SANDBOX_ROOT"/bob.log >&2 2>/dev/null || true
    exit 1
  fi
done

echo "[launch-conductors] Alice app ws: ws://localhost:4445"
echo "[launch-conductors] Bob   app ws: ws://localhost:4447"
echo "[launch-conductors] logs: $SANDBOX_ROOT/{alice,bob}.log"
echo "[launch-conductors] stop with: ./stop-conductors.sh"
