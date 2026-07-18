#!/usr/bin/env python3
from __future__ import annotations
import hashlib,json,os,subprocess,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
CLIENT=json.loads((ROOT/'frontend-leptos/bridge/package.json').read_text())['dependencies']['@holochain/client']
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
with tempfile.TemporaryDirectory() as raw:
 t=Path(raw); a=t/'artifact'; (a/'artifacts').mkdir(parents=True); e=a/'evidence-base'; e.mkdir()
 happ=a/'artifacts/mycelix_marketplace.happ'; dna=a/'artifacts/mycelix_marketplace.dna'; happ.write_bytes(b'happ'); dna.write_bytes(b'dna')
 source='bundle-test-revision'
 lifecycle={"schema_version":1,"scenario":"lifecycle","result":"pass","fixture":False,"generated_at":"2026-07-15T00:00:00Z","source_revision":source,"happ_sha256":sha(happ),"marketplace_dna_sha256":sha(dna),"finance_dna_sha256":None,"holochain_client_version":CLIENT,"active_roles":{"seller":["marketplace"],"buyer":["marketplace"]},"details":{"delivered_revisions":4,"cancellation_revisions":2}}
 (e/'lifecycle.json').write_text(json.dumps(lifecycle))
 subprocess.run(['python3',str(ROOT/'scripts/verify-live-evidence.py'),str(e),'--profile','base','--source-revision',source,'--happ',str(happ),'--marketplace-dna',str(dna),'--output',str(e/'promotion.json')],check=True,stdout=subprocess.DEVNULL)
 artifact={"schema_version":1,"profile":"base","source_revision":source,"artifacts":{"happ":{"path":str(happ),"sha256":sha(happ),"size":happ.stat().st_size},"marketplace_dna":{"path":str(dna),"sha256":sha(dna),"size":dna.stat().st_size},"finance_dna":None}}
 (a/'artifact-manifest.json').write_text(json.dumps(artifact)); (a/'toolchain.json').write_text('{}'); (a/'zome-artifacts.json').write_text('{}'); (a/'SHA256SUMS').write_text('fixture\n')
 key=t/'key.pem'; subprocess.run(['openssl','genpkey','-algorithm','ED25519','-out',str(key)],check=True,stdout=subprocess.DEVNULL)
 stage=t/'stage'; subprocess.run(['python3',str(ROOT/'scripts/seal-promotion-bundle.py'),'--profile','base','--artifact-root',str(a),'--private-key',str(key),'--output-dir',str(stage)],check=True,stdout=subprocess.DEVNULL)
 bundle=t/'bundle.tar.gz'; subprocess.run(['tar','-C',str(stage),'-czf',str(bundle),'mycelix-promotion-v1'],check=True)
 subprocess.run(['python3',str(ROOT/'scripts/verify-promotion-bundle.py'),str(bundle)],check=True,stdout=subprocess.DEVNULL)
 # Tamper with a copied tree and prove verification rejects it.
 unpack=t/'unpack'; unpack.mkdir(); subprocess.run(['tar','-C',str(unpack),'-xzf',str(bundle)],check=True)
 (unpack/'mycelix-promotion-v1/evidence/lifecycle.json').write_text('{}')
 bad=t/'bad.tar.gz'; subprocess.run(['tar','-C',str(unpack),'-czf',str(bad),'mycelix-promotion-v1'],check=True)
 rejected=subprocess.run(['python3',str(ROOT/'scripts/verify-promotion-bundle.py'),str(bad)],capture_output=True,text=True)
 assert rejected.returncode != 0
print('signed promotion bundle tests passed')
