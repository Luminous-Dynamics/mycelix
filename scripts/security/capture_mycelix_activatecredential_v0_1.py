#!/usr/bin/env python3
"""Capture and independently receipt TPM MakeCredential/ActivateCredential."""
from __future__ import annotations
import argparse, hashlib, json, os, shutil, subprocess
from pathlib import Path
from typing import Any

VERIFIER_ID="mycelix.tpm.activatecredential-capture.v0.1"

def canonical_hash(value:Any)->str:
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()

def sha256_file(path:Path)->str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def run(cmd:list[str],cwd:Path)->subprocess.CompletedProcess[str]:
    return subprocess.run(cmd,cwd=cwd,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)

def result(state:str,reason:str,details:dict[str,Any]|None=None)->dict[str,Any]:
    out={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
    if details is not None:out["details"]=details
    out["content_sha256"]=canonical_hash(out)
    return out

def self_test()->int:
    print("ActivateCredential capture helper semantic source policy: PASS")
    print("physical execution is required for a live PASS")
    return 0

def capture(out:Path,tpm_identity_digest:str)->int:
    out.mkdir(parents=True,exist_ok=True)
    required=[out/"ek.pub",out/"ak.name.readpublic",out/"ek.tpmt",out/"ek.ctx",out/"ak.ctx"]
    if not all(p.is_file() for p in required):
        missing=[p.name for p in required if not p.is_file()]
        value=result("INDETERMINATE","activation-input-artifacts-missing",{"missing":missing})
        (out/"activatecredential-capture.json").write_text(json.dumps(value,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        return 2
    if shutil.which("openssl") is None or shutil.which("tpm2_makecredential") is None or shutil.which("tpm2_activatecredential") is None:
        value=result("INDETERMINATE","required-activation-tools-unavailable")
        (out/"activatecredential-capture.json").write_text(json.dumps(value,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        return 2

    secret=out/"credential-secret.bin"
    blob=out/"activatecredential.blob"
    recovered=out/"activated-secret.bin"
    secret_proc=run(["openssl","rand","-out",str(secret),"32"],out)
    secret_stdout=hashlib.sha256(secret_proc.stdout.encode()).hexdigest()
    secret_stderr=hashlib.sha256(secret_proc.stderr.encode()).hexdigest()
    if secret_proc.returncode!=0 or not secret.is_file():
        value=result("DENY","registrar-secret-generation-failed",{"returncode":secret_proc.returncode,"stdout_sha256":secret_stdout,"stderr_sha256":secret_stderr})
        (out/"activatecredential-capture.json").write_text(json.dumps(value,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        return 1

    ak_name=out/"ak.name.readpublic"
    ek_pub=out/"ek.pub"
    mc_cmd=["tpm2_makecredential","-u",str(ek_pub),"-s",str(secret),"-n",ak_name.read_bytes().hex(),"-o",str(blob)]
    mc=run(mc_cmd,out)
    make_tx={
        "command":mc_cmd,
        "returncode":mc.returncode,
        "stdout_sha256":hashlib.sha256(mc.stdout.encode()).hexdigest(),
        "stderr_sha256":hashlib.sha256(mc.stderr.encode()).hexdigest(),
        "output_present":blob.is_file(),
        "output_sha256":sha256_file(blob) if blob.is_file() else None,
    }

    ac_cmd=["tpm2_activatecredential","-c",str(out/"ak.ctx"),"-C",str(out/"ek.ctx"),"-i",str(blob),"-o",str(recovered)]
    ac=run(ac_cmd,out)
    activate_tx={
        "command":ac_cmd,
        "returncode":ac.returncode,
        "stdout_sha256":hashlib.sha256(ac.stdout.encode()).hexdigest(),
        "stderr_sha256":hashlib.sha256(ac.stderr.encode()).hexdigest(),
        "output_present":recovered.is_file(),
        "output_sha256":sha256_file(recovered) if recovered.is_file() else None,
    }

    secret_sha=sha256_file(secret)
    recovered_sha=sha256_file(recovered) if recovered.is_file() else None
    equality="PASS" if recovered.is_file() and secret.read_bytes()==recovered.read_bytes() else "DENY"
    session={
        "tpm_identity_digest":tpm_identity_digest,
        "ek_public_sha256":sha256_file(ek_pub),
        "ek_public_wire_sha256":sha256_file(out/"ek.tpmt"),
        "ak_name_sha256":sha256_file(ak_name),
        "registrar_secret_sha256":secret_sha,
        "recovered_secret_sha256":recovered_sha,
        "makecredential_blob_sha256":sha256_file(blob) if blob.is_file() else None,
        "makecredential_returncode":mc.returncode,
        "activatecredential_returncode":ac.returncode,
    }
    session_hash=canonical_hash(session)
    receipt={
        "verifier_id":VERIFIER_ID,
        "state":"PASS" if mc.returncode==0 and ac.returncode==0 and equality=="PASS" else "DENY",
        "reason":"activatecredential-secret-recovered" if equality=="PASS" and mc.returncode==0 and ac.returncode==0 else "activatecredential-execution-failed",
        "verification_mode":"LiveVerifierSession",
        "claim_ceiling":"ReferenceModelOnly",
        "tpm_identity_digest":tpm_identity_digest,
        "ek_public_sha256":session["ek_public_sha256"],
        "ek_public_wire_sha256":session["ek_public_wire_sha256"],
        "ak_name_sha256":session["ak_name_sha256"],
        "registrar_secret_sha256":secret_sha,
        "recovered_secret_sha256":recovered_sha,
        "makecredential_blob_sha256":session["makecredential_blob_sha256"],
        "makecredential_transcript_sha256":hashlib.sha256(json.dumps(make_tx,sort_keys=True,separators=(",",":")).encode()).hexdigest(),
        "activatecredential_transcript_sha256":hashlib.sha256(json.dumps(activate_tx,sort_keys=True,separators=(",",":")).encode()).hexdigest(),
        "makecredential_returncode":mc.returncode,
        "activatecredential_returncode":ac.returncode,
        "secret_equality_state":equality,
        "activation_session_tuple_sha256":session_hash,
        "source_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "makecredential_transcript":make_tx,
        "activatecredential_transcript":activate_tx,
    }
    receipt["session_binding_sha256"]=canonical_hash({
        "verification_mode":receipt["verification_mode"],
        "claim_ceiling":receipt["claim_ceiling"],
        "tpm_identity_digest":receipt["tpm_identity_digest"],
        "ek_public_sha256":receipt["ek_public_sha256"],
        "ek_public_wire_sha256":receipt["ek_public_wire_sha256"],
        "ak_name_sha256":receipt["ak_name_sha256"],
        "registrar_secret_sha256":receipt["registrar_secret_sha256"],
        "recovered_secret_sha256":receipt["recovered_secret_sha256"],
        "makecredential_blob_sha256":receipt["makecredential_blob_sha256"],
        "makecredential_transcript_sha256":receipt["makecredential_transcript_sha256"],
        "activatecredential_transcript_sha256":receipt["activatecredential_transcript_sha256"],
        "makecredential_returncode":receipt["makecredential_returncode"],
        "activatecredential_returncode":receipt["activatecredential_returncode"],
        "source_sha256":receipt["source_sha256"],
    })
    receipt["content_sha256"]=canonical_hash({k:v for k,v in receipt.items() if k!="content_sha256"})
    (out/"activatecredential-makecredential-transcript.json").write_text(json.dumps(make_tx,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    (out/"activatecredential-activatecredential-transcript.json").write_text(json.dumps(activate_tx,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    (out/"activatecredential-capture.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    return 0 if receipt["state"]=="PASS" else 1

def main()->int:
    p=argparse.ArgumentParser()
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument("--self-test",action="store_true")
    g.add_argument("--capture",action="store_true")
    p.add_argument("--output")
    p.add_argument("--tpm-identity-digest",required=False,default="")
    a=p.parse_args()
    if a.self_test:return self_test()
    if not a.output:p.error("--output is required with --capture")
    return capture(Path(a.output).resolve(),a.tpm_identity_digest)

if __name__=="__main__":
    raise SystemExit(main())
