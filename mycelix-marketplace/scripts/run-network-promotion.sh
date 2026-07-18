#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_ROOT="${1:-$ROOT/.promotion/network}"
: "${HOLOCHAIN_BIN:=holochain}"
: "${MARKETPLACE_NETWORK_SERVICES:?set MARKETPLACE_NETWORK_SERVICES to the controlled local bootstrap/signaling hook}"
: "${MARKETPLACE_NETWORK_CONTROL:?set MARKETPLACE_NETWORK_CONTROL to the controlled partition/heal hook}"
[[ -x "$MARKETPLACE_NETWORK_SERVICES" ]] || { echo "network services hook is not executable" >&2; exit 2; }
[[ -x "$MARKETPLACE_NETWORK_CONTROL" ]] || { echo "network control hook is not executable" >&2; exit 2; }

"$ROOT/scripts/check-promotion-environment.sh"
MANIFEST="$ARTIFACT_ROOT/artifact-manifest.json"
[[ -f "$MANIFEST" ]] || { echo "artifact manifest missing: $MANIFEST" >&2; exit 2; }
PROFILE="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["profile"])' "$MANIFEST")"
[[ "$PROFILE" == network ]] || { echo "network promotion requires a network artifact profile" >&2; exit 2; }
HAPP="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["artifacts"]["happ"]["path"])' "$MANIFEST")"
DNA="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["artifacts"]["marketplace_dna"]["path"])' "$MANIFEST")"

WORK="$(mktemp -d "${TMPDIR:-/tmp}/mycelix-network-promotion.XXXXXX")"
SELLER_PID=""; BUYER_PID=""; SERVICES_STARTED=0
cleanup() {
  set +e
  for pid in "$SELLER_PID" "$BUYER_PID"; do
    [[ -n "$pid" ]] && kill "$pid" 2>/dev/null
  done
  for pid in "$SELLER_PID" "$BUYER_PID"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null
  done
  if [[ "$SERVICES_STARTED" == 1 ]]; then
    "$MARKETPLACE_NETWORK_SERVICES" stop "$WORK" "$WORK/network-services.json" >/dev/null 2>&1 || true
  fi
  rm -f "$WORK/actors.env"
  if [[ "${KEEP_PROMOTION_WORKSPACE:-0}" != 1 ]]; then rm -rf "$WORK"; else echo "kept $WORK" >&2; fi
}
trap cleanup EXIT INT TERM

"$MARKETPLACE_NETWORK_SERVICES" start "$WORK" "$WORK/network-services.json"
SERVICES_STARTED=1
[[ -f "$WORK/network-services.json" ]] || { echo "network services hook did not create a receipt" >&2; exit 2; }
python3 "$ROOT/scripts/validate-network-services-receipt.py" \
  "$WORK/network-services.json" --implementation "$MARKETPLACE_NETWORK_SERVICES" >/dev/null
read -r BOOTSTRAP_URL SIGNAL_URL < <(python3 - "$WORK/network-services.json" <<'PY'
import json,sys
v=json.load(open(sys.argv[1])); print(v['bootstrap_url'],v['signal_url'])
PY
)

read -r SELLER_ADMIN SELLER_APP BUYER_ADMIN BUYER_APP < <(python3 - <<'PY'
import socket
ports=[]
for _ in range(4):
 s=socket.socket(); s.bind(('127.0.0.1',0)); ports.append(s.getsockname()[1]); s.close()
print(*ports)
PY
)
python3 "$ROOT/scripts/generate-networked-conductor-config.py" \
  --data-root "$WORK/seller-data" --admin-port "$SELLER_ADMIN" \
  --bootstrap-url "$BOOTSTRAP_URL" --signal-url "$SIGNAL_URL" \
  --output "$WORK/seller-conductor.yaml" >/dev/null
python3 "$ROOT/scripts/generate-networked-conductor-config.py" \
  --data-root "$WORK/buyer-data" --admin-port "$BUYER_ADMIN" \
  --bootstrap-url "$BOOTSTRAP_URL" --signal-url "$SIGNAL_URL" \
  --output "$WORK/buyer-conductor.yaml" >/dev/null

"$HOLOCHAIN_BIN" -c "$WORK/seller-conductor.yaml" >"$WORK/seller-conductor.log" 2>&1 & SELLER_PID=$!
"$HOLOCHAIN_BIN" -c "$WORK/buyer-conductor.yaml" >"$WORK/buyer-conductor.log" 2>&1 & BUYER_PID=$!
python3 - "$SELLER_ADMIN" "$SELLER_PID" "$BUYER_ADMIN" "$BUYER_PID" <<'PY'
import os,socket,sys,time
pairs=[(int(sys.argv[1]),int(sys.argv[2]),'seller'),(int(sys.argv[3]),int(sys.argv[4]),'buyer')]
for port,pid,label in pairs:
 deadline=time.time()+60
 while time.time()<deadline:
  try: os.kill(pid,0)
  except OSError: raise SystemExit(f'{label} conductor exited before admin interface became ready')
  try:
   with socket.create_connection(('127.0.0.1',port),timeout=.5): break
  except OSError: time.sleep(.25)
 else: raise SystemExit(f'timed out waiting for {label} conductor admin interface')
PY

python3 - "$WORK/setup.json" "$HAPP" "$WORK/actors.env" "$SELLER_ADMIN" "$SELLER_APP" "$BUYER_ADMIN" "$BUYER_APP" <<'PY'
import json,sys
out,happ,env,sadmin,sapp,badmin,bapp=sys.argv[1:]
json.dump({
 'happ_path':happ,
 'allowed_origin':'mycelix-network-promotion://local',
 'output_env':env,
 'actors':[
  {'prefix':'SELLER','admin_url':f'ws://127.0.0.1:{sadmin}','installed_app_id':'mycelix-network-seller','app_port':int(sapp)},
  {'prefix':'BUYER','admin_url':f'ws://127.0.0.1:{badmin}','installed_app_id':'mycelix-network-buyer','app_port':int(bapp)}
 ]},open(out,'w'),indent=2)
PY

npm --prefix "$ROOT/frontend-leptos/bridge" ci --ignore-scripts
MARKETPLACE_NETWORK_SETUP_CONFIG="$WORK/setup.json" npm --prefix "$ROOT/frontend-leptos/bridge" run run:network-setup
set -a; source "$WORK/actors.env"; set +a

python3 - "$WORK/topology.json" "$SELLER_ADMIN" "$SELLER_APP" "$SELLER_PID" "$BUYER_ADMIN" "$BUYER_APP" "$BUYER_PID" "$WORK/network-services.json" "$MARKETPLACE_NETWORK_CONTROL" <<'PY'
import hashlib,json,sys
from pathlib import Path
out,sadmin,sapp,spid,badmin,bapp,bpid,services,control=sys.argv[1:]
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
json.dump({
 'schema_version':1,
 'topology':'two_conductor_isolated_network',
 'conductors':{
  'seller':{'admin_url':f'ws://127.0.0.1:{sadmin}','app_port':int(sapp),'pid':int(spid)},
  'buyer':{'admin_url':f'ws://127.0.0.1:{badmin}','app_port':int(bapp),'pid':int(bpid)}
 },
 'service_receipt_sha256':sha(services),
 'control_hook_sha256':sha(control)
},open(out,'w'),indent=2)
PY

EVIDENCE="$ARTIFACT_ROOT/evidence-network"
rm -rf "$EVIDENCE"; mkdir -p "$EVIDENCE"
export MARKETPLACE_HAPP_PATH="$HAPP" MARKETPLACE_DNA_PATH="$DNA" MARKETPLACE_EVIDENCE_DIR="$EVIDENCE"
export MARKETPLACE_SOURCE_REVISION="${MARKETPLACE_SOURCE_REVISION:-$(git -C "$ROOT" rev-parse HEAD)}"
export MARKETPLACE_HAPP_SHA256="$(sha256sum "$HAPP" | awk '{print $1}')"
export MARKETPLACE_DNA_SHA256="$(sha256sum "$DNA" | awk '{print $1}')"
export MARKETPLACE_CLIENT_VERSION="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["dependencies"]["@holochain/client"])' "$ROOT/frontend-leptos/bridge/package.json")"
export MARKETPLACE_NETWORK_TOPOLOGY="$WORK/topology.json"
export MARKETPLACE_NETWORK_SERVICES_RECEIPT="$WORK/network-services.json"

npm --prefix "$ROOT/frontend-leptos/bridge" run test:conductor-network
python3 "$ROOT/scripts/verify-live-evidence.py" "$EVIDENCE" \
  --profile network --source-revision "$MARKETPLACE_SOURCE_REVISION" \
  --happ "$HAPP" --marketplace-dna "$DNA" --output "$EVIDENCE/promotion.json"

kill "$SELLER_PID" "$BUYER_PID"
wait "$SELLER_PID" 2>/dev/null || true
wait "$BUYER_PID" 2>/dev/null || true
SELLER_PID=""; BUYER_PID=""
"$MARKETPLACE_NETWORK_SERVICES" stop "$WORK" "$WORK/network-services.json"
SERVICES_STARTED=0
cp "$WORK/seller-conductor.log" "$EVIDENCE/seller-conductor.log"
cp "$WORK/buyer-conductor.log" "$EVIDENCE/buyer-conductor.log"
cp "$WORK/network-services.json" "$EVIDENCE/network-services.json"
cp "$WORK/topology.json" "$EVIDENCE/network-topology.json"
sha256sum "$WORK/seller-conductor.yaml" > "$EVIDENCE/seller-conductor-config.sha256"
sha256sum "$WORK/buyer-conductor.yaml" > "$EVIDENCE/buyer-conductor-config.sha256"
sha256sum "$MARKETPLACE_NETWORK_SERVICES" > "$EVIDENCE/network-services-hook.sha256"
sha256sum "$MARKETPLACE_NETWORK_CONTROL" > "$EVIDENCE/network-control-hook.sha256"
printf '%s\n' 'two_conductor_isolated_network' > "$EVIDENCE/topology.txt"
sha256sum "$EVIDENCE"/* > "$EVIDENCE/ALL_SHA256SUMS"
echo "$EVIDENCE"
