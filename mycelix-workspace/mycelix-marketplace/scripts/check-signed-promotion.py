#!/usr/bin/env python3
from __future__ import annotations
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=json.loads((ROOT/'contracts/signed-promotion-v1.json').read_text())
workflow=(ROOT/spec['workflow']).read_text()
errors=[]
for label in spec['runner_labels']:
 if label not in workflow: errors.append(f'missing controlled runner label: {label}')
for step in spec['required_steps']:
 if step not in workflow: errors.append(f'missing promotion step: {step}')
for needle in ['persist-credentials: false','MARKETPLACE_RELEASE_SIGNING_KEY_BASE64','shred -u','upload-artifact@v4']:
 if needle not in workflow: errors.append(f'missing workflow security boundary: {needle}')
if 'marketplace-release-key.pem' in workflow.split('uses: actions/upload-artifact@v4',1)[-1]:
 errors.append('private release key appears in upload-artifact section')
if '${{ inputs.finance_dna_path }}' not in workflow or '${{ inputs.settlement_bootstrap_path }}' not in workflow:
 errors.append('settlement external dependencies are not explicit workflow inputs')
if errors: raise SystemExit('\n'.join(f'- {e}' for e in errors))
print('signed promotion workflow contract passed: controlled runner, exact build, live evidence, Ed25519 envelope')
