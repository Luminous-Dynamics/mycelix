#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-}"
ARTIFACT_ROOT="${2:-$ROOT/.promotion/$PROFILE}"
case "$PROFILE" in base|settlement|arbitration) ;; *) echo "usage: $0 {base|settlement|arbitration} [artifact-root]" >&2; exit 2;; esac

: "${HOLOCHAIN_BIN:=holochain}"
"$ROOT/scripts/check-promotion-environment.sh"
MANIFEST="$ARTIFACT_ROOT/artifact-manifest.json"
[[ -f "$MANIFEST" ]] || { echo "artifact manifest missing: $MANIFEST" >&2; exit 2; }
HAPP="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["artifacts"]["happ"]["path"])' "$MANIFEST")"
DNA="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["artifacts"]["marketplace_dna"]["path"])' "$MANIFEST")"
FINANCE="$(python3 -c 'import json,sys; v=json.load(open(sys.argv[1]))["artifacts"]["finance_dna"]; print(v["path"] if v else "")' "$MANIFEST")"

WORK="$(mktemp -d "${TMPDIR:-/tmp}/mycelix-promotion.XXXXXX")"
PID=""
cleanup() {
  set +e
  [[ -n "$PID" ]] && kill "$PID" 2>/dev/null
  [[ -n "$PID" ]] && wait "$PID" 2>/dev/null
  rm -f "$WORK/actors.env"
  if [[ "${KEEP_PROMOTION_WORKSPACE:-0}" != 1 ]]; then rm -rf "$WORK"; else echo "kept $WORK" >&2; fi
}
trap cleanup EXIT INT TERM

read -r ADMIN_PORT SELLER_PORT BUYER_PORT ARBITRATOR_PORT < <(python3 - <<'PY'
import socket
ports=[]
for _ in range(4):
 s=socket.socket(); s.bind(('127.0.0.1',0)); ports.append(s.getsockname()[1]); s.close()
print(*ports)
PY
)
python3 "$ROOT/scripts/generate-disposable-conductor-config.py" \
  --data-root "$WORK/conductor-data" --admin-port "$ADMIN_PORT" --output "$WORK/conductor.yaml" >/dev/null
"$HOLOCHAIN_BIN" -c "$WORK/conductor.yaml" >"$WORK/conductor.log" 2>&1 & PID=$!
python3 - "$ADMIN_PORT" "$PID" <<'PY'
import os,socket,sys,time
port=int(sys.argv[1]); pid=int(sys.argv[2]); deadline=time.time()+60
while time.time()<deadline:
 try:
  os.kill(pid,0)
 except OSError:
  raise SystemExit('conductor exited before admin interface became ready')
 try:
  with socket.create_connection(('127.0.0.1',port),timeout=.5): break
 except OSError: time.sleep(.25)
else: raise SystemExit('timed out waiting for conductor admin interface')
PY

python3 - "$WORK/setup.json" "$HAPP" "$WORK/actors.env" "$PROFILE" "$ADMIN_PORT" "$SELLER_PORT" "$BUYER_PORT" "$ARBITRATOR_PORT" <<'PY'
import json,sys
out,happ,env,profile,admin,seller,buyer,arb=sys.argv[1:]
actors=[{"prefix":"SELLER","installed_app_id":"mycelix-seller","app_port":int(seller)},
        {"prefix":"BUYER","installed_app_id":"mycelix-buyer","app_port":int(buyer)}]
if profile=='arbitration': actors.append({"prefix":"ARBITRATOR","installed_app_id":"mycelix-arbitrator","app_port":int(arb)})
json.dump({"admin_url":f"ws://127.0.0.1:{admin}","happ_path":happ,"allowed_origin":"mycelix-promotion://local","output_env":env,"actors":actors},open(out,'w'),indent=2)
PY

npm --prefix "$ROOT/frontend-leptos/bridge" ci --ignore-scripts
MARKETPLACE_DISPOSABLE_CONFIG="$WORK/setup.json" npm --prefix "$ROOT/frontend-leptos/bridge" run run:disposable-setup
set -a; source "$WORK/actors.env"; set +a

if [[ "$PROFILE" == settlement ]]; then
  [[ -n "$FINANCE" ]] || { echo "settlement artifact manifest has no Finance DNA" >&2; exit 2; }
  : "${MARKETPLACE_SETTLEMENT_BOOTSTRAP:?settlement requires an executable MARKETPLACE_SETTLEMENT_BOOTSTRAP funding hook}"
  [[ -x "$MARKETPLACE_SETTLEMENT_BOOTSTRAP" ]] || { echo "funding hook is not executable" >&2; exit 2; }
  "$MARKETPLACE_SETTLEMENT_BOOTSTRAP" "$WORK/actors.env" "$MANIFEST"
fi

EVIDENCE="$ARTIFACT_ROOT/evidence-$PROFILE"
rm -rf "$EVIDENCE"; mkdir -p "$EVIDENCE"
export MARKETPLACE_HAPP_PATH="$HAPP" MARKETPLACE_DNA_PATH="$DNA" MARKETPLACE_EVIDENCE_DIR="$EVIDENCE"
if [[ -n "$FINANCE" ]]; then export FINANCE_DNA_PATH="$FINANCE"; fi
"$ROOT/scripts/run-live-promotion.sh" "$PROFILE"
kill "$PID"
wait "$PID" 2>/dev/null || true
PID=""
cp "$WORK/conductor.log" "$EVIDENCE/conductor.log"
sha256sum "$WORK/conductor.yaml" > "$EVIDENCE/conductor-config.sha256"
printf '%s\n' 'single_conductor_multi_agent' > "$EVIDENCE/topology.txt"
if [[ "$PROFILE" == settlement ]]; then
  sha256sum "$MARKETPLACE_SETTLEMENT_BOOTSTRAP" > "$EVIDENCE/settlement-bootstrap.sha256"
fi
sha256sum "$EVIDENCE"/* > "$EVIDENCE/ALL_SHA256SUMS"
echo "$EVIDENCE"
