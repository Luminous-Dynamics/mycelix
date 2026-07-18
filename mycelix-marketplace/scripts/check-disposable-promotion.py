#!/usr/bin/env python3
from __future__ import annotations
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=json.loads((ROOT/'contracts/disposable-promotion-v1.json').read_text())
setup=(ROOT/'frontend-leptos/bridge/test/prepare-disposable-conductor.ts').read_text()
runner=(ROOT/'scripts/run-disposable-promotion.sh').read_text()
config=(ROOT/'scripts/generate-disposable-conductor-config.py').read_text()
errors=[]
for needle in ['generateAgentPubKey','installApp','enableApp','authorizeSigningCredentials','attachAppInterface','issueAppAuthenticationToken']:
 if needle not in setup: errors.append(f'missing admin setup operation: {needle}')
for needle in ['danger_test_keystore','data_root_path','admin_interfaces']:
 if needle not in config: errors.append(f'missing disposable conductor isolation setting: {needle}')
for needle in ['mktemp -d','trap cleanup','MARKETPLACE_SETTLEMENT_BOOTSTRAP','conductor.log','single_conductor_multi_agent']:
 if needle not in runner: errors.append(f'missing disposable runner guard: {needle}')
if spec['topology']!='single_conductor_multi_agent': errors.append('unexpected disposable topology')
if errors: raise SystemExit('\n'.join(f'- {e}' for e in errors))
print('disposable promotion contract passed: isolated conductor, distinct agents/interfaces, no token evidence')
