#!/usr/bin/env python3
"""Preserve raw EK certificate-chain bytes from TCG-defined TPM NV indices."""
from __future__ import annotations
import argparse, hashlib, json, os, re, shutil, subprocess
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-certificate-chain-nv-capture.v0.1"
CHAIN_MIN = 0x01C00100
CHAIN_MAX = 0x01C001FF

def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()

def parse_handles(text: str) -> list[int]:
    return sorted({int(v, 16) for v in re.findall(r"0x[0-9a-fA-F]+", text)})

def chain_handles(handles: list[int]) -> list[int]:
    return [h for h in handles if CHAIN_MIN <= h <= CHAIN_MAX]

def source_policy() -> dict[str, Any]:
    return {"mode":"TPM_NV_ONLY","network_access":False,"remote_lookup":False,"certificate_parsing":False,"raw_bytes_preserved":True,"range_start":f"0x{CHAIN_MIN:08x}","range_end":f"0x{CHAIN_MAX:08x}"}

def run(command: list[str], env: dict[str, str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command,cwd=cwd,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)

def result(state: str, reason: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    value={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
    if details is not None:value["details"]=details
    value["content_sha256"]=canonical_hash(value)
    return value

def self_test() -> int:
    handles=parse_handles("\n".join(["0x01c00100","0x01c00101","0x01c00107","0x01c00012","0x01c001ff","0x01c00200"]))
    got=[f"0x{x:08x}" for x in chain_handles(handles)]
    expected=["0x01c00100","0x01c00101","0x01c00107","0x01c001ff"]
    if got!=expected:
        print("exact EK certificate-chain NV range classification: FAIL"); return 1
    p=source_policy()
    if p["mode"]!="TPM_NV_ONLY" or p["network_access"] or p["remote_lookup"]:
        print("raw chain source policy: FAIL"); return 1
    print("EK certificate-chain NV capture semantic corpus: PASS")
    print("0x01c00100..0x01c001ff raw-source boundary: PASS")
    return 0

def capture(out: Path, env: dict[str, str]) -> int:
    out.mkdir(parents=True,exist_ok=True)
    inventory_path=out/"ek-cert-chain-nv-index-handles.txt"
    transcript_path=out/"ek-cert-chain-nv-capture-transcript.json"
    result_path=out/"ek-cert-chain-nv-capture.json"
    concatenated_path=out/"ek-cert-chain-nv-concatenated.bin"
    if shutil.which("tpm2_getcap") is None or shutil.which("tpm2_nvread") is None:
        value=result("INDETERMINATE","required-tpm2-tools-unavailable")
        result_path.write_text(json.dumps(value,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        return 2
    inv=run(["tpm2_getcap","handles-nv-index"],env,out)
    inventory_path.write_text(inv.stdout+inv.stderr,encoding="utf-8")
    discovered=parse_handles(inventory_path.read_text(encoding="utf-8"))
    candidates=chain_handles(discovered)
    reads=[]; chunks=[]
    for handle in candidates:
        output=out/f"ek-cert-chain-nv-{handle:08x}.bin"
        if output.exists(): output.unlink()
        proc=run(["tpm2_nvread",f"0x{handle:08x}","-o",str(output)],env,out)
        present=output.is_file()
        data=output.read_bytes() if present else b""
        if present:chunks.append(data)
        reads.append({"handle":f"0x{handle:08x}","returncode":proc.returncode,"stdout_sha256":hashlib.sha256(proc.stdout.encode()).hexdigest(),"stderr_sha256":hashlib.sha256(proc.stderr.encode()).hexdigest(),"present":present,"sha256":hashlib.sha256(data).hexdigest() if present else None,"size":len(data)})
    concatenated_path.write_bytes(b"".join(chunks))
    first_gap=None
    expected=CHAIN_MIN
    discovered_chain=set(candidates)
    while expected<=CHAIN_MAX and expected in discovered_chain: expected+=1
    if expected<=CHAIN_MAX:first_gap=f"0x{expected:08x}"
    transcript={"inventory_command":["tpm2_getcap","handles-nv-index"],"inventory_returncode":inv.returncode,"inventory_stdout_sha256":hashlib.sha256(inv.stdout.encode()).hexdigest(),"inventory_stderr_sha256":hashlib.sha256(inv.stderr.encode()).hexdigest(),"candidate_handles":[f"0x{x:08x}" for x in candidates],"certificate_reads":reads,"first_gap":first_gap,"source_policy":source_policy()}
    transcript_path.write_text(json.dumps(transcript,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    state,reason=("PASS","raw-ek-certificate-chain-bytes-captured-from-tcg-nv-range") if any(r["present"] for r in reads) else ("INDETERMINATE","no-populated-ek-certificate-chain-nv-index-observed")
    value=result(state,reason,{"source_policy":transcript["source_policy"],"inventory_sha256":sha256_file(inventory_path),"transcript_sha256":sha256_file(transcript_path),"candidate_handles":[f"0x{x:08x}" for x in candidates],"first_handle":f"0x{candidates[0]:08x}" if candidates else None,"last_handle":f"0x{candidates[-1]:08x}" if candidates else None,"first_gap":first_gap,"reads":reads,"concatenated_sha256":sha256_file(concatenated_path),"concatenated_size":concatenated_path.stat().st_size})
    result_path.write_text(json.dumps(value,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[state]

def main() -> int:
    parser=argparse.ArgumentParser()
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test",action="store_true")
    mode.add_argument("--capture",action="store_true")
    parser.add_argument("--output")
    args=parser.parse_args()
    if args.self_test:return self_test()
    if not args.output:parser.error("--output is required with --capture")
    return capture(Path(args.output).resolve(),os.environ.copy())

if __name__=="__main__":raise SystemExit(main())
