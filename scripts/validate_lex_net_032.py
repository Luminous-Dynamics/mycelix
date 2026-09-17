#!/usr/bin/env python3
"""Validate LEX-NET-032 R2 canonical interpretation with resource bounds."""
from __future__ import annotations
import argparse, base64, hashlib, json, subprocess
from pathlib import Path
from typing import Any

TRANCHE="LEX-NET-032"
PROFILE_ID="lex-net-json-interpretation-v1"
PARENT="94d9869fd82c8708af3e08b95306586ef0d7b4fb"
R1_HEAD="083890a2cf967cef2fe8294585f3eb32e3a44658"
EXPECTED_PATHS=[
    ".github/workflows/lex-net-032.yml",
    "docs/lex-net/LEX_NET_CANONICAL_INTERPRETATION_V1.md",
    "docs/lex-net/lex_net_032_manifest.json",
    "scripts/validate_lex_net_032.py",
]
MANIFEST_PATH=Path("docs/lex-net/lex_net_032_manifest.json")
DOC_PATH=Path("docs/lex-net/LEX_NET_CANONICAL_INTERPRETATION_V1.md")
MAX_SOURCE_BYTES=65536
MAX_NESTING_DEPTH=32
MAX_CONTAINER_ITEMS=256
MAX_STRING_UTF8_BYTES=16384
MAX_KEY_UTF8_BYTES=1024

class DuplicateKeyError(ValueError): pass
class NumericDomainError(ValueError): pass
class ResourceLimitError(ValueError): pass

def sha256(data:bytes)->str: return hashlib.sha256(data).hexdigest()
def reject_float(token:str)->Any: raise NumericDomainError(f"floating-point JSON number forbidden: {token}")
def reject_constant(token:str)->Any: raise NumericDomainError(f"non-finite JSON number forbidden: {token}")

def object_pairs_no_duplicates(pairs:list[tuple[str,Any]])->dict[str,Any]:
    if len(pairs)>MAX_CONTAINER_ITEMS:
        raise ResourceLimitError(f"object member count {len(pairs)} exceeds {MAX_CONTAINER_ITEMS}")
    out={}
    for key,value in pairs:
        if key in out: raise DuplicateKeyError(key)
        out[key]=value
    return out

def synthetic_bytes(spec:dict[str,Any])->bytes:
    kind=spec.get("kind")
    if kind=="source-size":
        n=int(spec["bytes"])
        if n<2: raise ValueError("source-size synthetic bytes must be >=2")
        return b"{}" + b" "*(n-2)
    if kind=="nesting-depth":
        depth=int(spec["depth"])
        return ('{"x":' + '['*(depth-1) + '0' + ']'*(depth-1) + '}').encode()
    if kind=="object-items":
        count=int(spec["count"])
        return ("{" + ",".join(f'"k{i}":0' for i in range(count)) + "}").encode()
    if kind=="array-items":
        count=int(spec["count"])
        return ('{"x":[' + ",".join("0" for _ in range(count)) + "]}").encode()
    if kind=="string-bytes":
        n=int(spec["bytes"])
        return ('{"x":"' + "a"*n + '"}').encode()
    if kind=="key-bytes":
        n=int(spec["bytes"])
        return ('{"' + "k"*n + '":0}').encode()
    raise ValueError(f"unknown synthetic source kind: {kind!r}")

def fixture_bytes(fixture:dict[str,Any])->bytes:
    modes=[k for k in ("raw_json","raw_base64","synthetic") if k in fixture]
    if len(modes)!=1: raise ValueError(f"{fixture.get('id')}: exactly one source mode required")
    if "raw_json" in fixture: return fixture["raw_json"].encode("utf-8")
    if "raw_base64" in fixture: return base64.b64decode(fixture["raw_base64"],validate=True)
    return synthetic_bytes(fixture["synthetic"])

def lexical_max_depth(text:str)->int:
    depth=0; max_depth=0; in_string=False; escape=False
    for ch in text:
        if in_string:
            if escape: escape=False
            elif ch=="\\": escape=True
            elif ch=='"': in_string=False
            continue
        if ch=='"': in_string=True
        elif ch in "{[":
            depth+=1; max_depth=max(max_depth,depth)
            if max_depth>MAX_NESTING_DEPTH:
                raise ResourceLimitError(f"nesting depth exceeds {MAX_NESTING_DEPTH}")
        elif ch in "}]":
            depth=max(depth-1,0)
    return max_depth

def check_decoded_limits(value:Any)->None:
    if isinstance(value,bool) or value is None: return
    if isinstance(value,int):
        if value < -(2**63) or value > 2**63-1: raise NumericDomainError(f"signed-64 overflow: {value}")
        return
    if isinstance(value,str):
        n=len(value.encode("utf-8",errors="strict"))
        if n>MAX_STRING_UTF8_BYTES: raise ResourceLimitError(f"string UTF-8 length {n} exceeds {MAX_STRING_UTF8_BYTES}")
        return
    if isinstance(value,list):
        if len(value)>MAX_CONTAINER_ITEMS: raise ResourceLimitError(f"array item count {len(value)} exceeds {MAX_CONTAINER_ITEMS}")
        for item in value: check_decoded_limits(item)
        return
    if isinstance(value,dict):
        if len(value)>MAX_CONTAINER_ITEMS: raise ResourceLimitError(f"object member count {len(value)} exceeds {MAX_CONTAINER_ITEMS}")
        for key,item in value.items():
            key_bytes=len(key.encode("utf-8",errors="strict"))
            if key_bytes>MAX_KEY_UTF8_BYTES: raise ResourceLimitError(f"key UTF-8 length {key_bytes} exceeds {MAX_KEY_UTF8_BYTES}")
            check_decoded_limits(item)
        return
    raise NumericDomainError(f"unsupported value domain: {type(value).__name__}")

def result(disposition:str,reason:str,**extra:Any)->dict[str,Any]:
    return {"disposition":disposition,"reason":reason,"grants_local_authority":False,"grants_external_effect_authority":False,**extra}

def interpret(fixture:dict[str,Any])->dict[str,Any]:
    if fixture.get("profile_id")!=PROFILE_ID:
        return result("ProfileUnsupported","exact interpretation profile is not supported")
    try: raw=fixture_bytes(fixture)
    except Exception as exc: return result("MalformedEncoding",f"invalid frozen source encoding: {exc}")
    source_sha=sha256(raw)
    if len(raw)>MAX_SOURCE_BYTES:
        return result("ResourceLimitExceeded",f"source bytes {len(raw)} exceeds {MAX_SOURCE_BYTES}",source_sha256=source_sha)
    try: text=raw.decode("utf-8",errors="strict")
    except UnicodeDecodeError:
        return result("MalformedEncoding","source bytes are not valid UTF-8",source_sha256=source_sha)
    try: lexical_max_depth(text)
    except ResourceLimitError as exc:
        return result("ResourceLimitExceeded",str(exc),source_sha256=source_sha)
    try:
        parsed=json.loads(text,object_pairs_hook=object_pairs_no_duplicates,parse_float=reject_float,parse_constant=reject_constant)
    except DuplicateKeyError as exc:
        return result("DuplicateKeyRejected",f"duplicate object key: {exc}",source_sha256=source_sha)
    except NumericDomainError as exc:
        return result("NumericDomainViolation",str(exc),source_sha256=source_sha)
    except ResourceLimitError as exc:
        return result("ResourceLimitExceeded",str(exc),source_sha256=source_sha)
    except (json.JSONDecodeError,ValueError,RecursionError) as exc:
        return result("MalformedEncoding",f"JSON parse failed: {exc}",source_sha256=source_sha)
    if not isinstance(parsed,dict):
        return result("MalformedEncoding","top-level JSON value must be an object",source_sha256=source_sha)
    try: check_decoded_limits(parsed)
    except UnicodeEncodeError:
        return result("UnicodeViolation","decoded projection contains invalid Unicode scalar data",source_sha256=source_sha)
    except ResourceLimitError as exc:
        return result("ResourceLimitExceeded",str(exc),source_sha256=source_sha)
    except NumericDomainError as exc:
        return result("NumericDomainViolation",str(exc),source_sha256=source_sha)
    critical=parsed.get("critical",[])
    if not isinstance(critical,list) or any(not isinstance(x,str) for x in critical) or len(set(critical))!=len(critical):
        return result("CriticalFieldUnsupported","critical must be an array of unique strings",source_sha256=source_sha)
    if critical:
        return result("CriticalFieldUnsupported","v1 supports no critical extensions",source_sha256=source_sha)
    try:
        canonical_text=json.dumps(parsed,ensure_ascii=False,sort_keys=True,separators=(",",":"),allow_nan=False)
        canonical=canonical_text.encode("utf-8",errors="strict")
    except UnicodeEncodeError:
        return result("UnicodeViolation","decoded projection contains invalid Unicode scalar data",source_sha256=source_sha)
    projection_sha=sha256(canonical)
    claimed=fixture.get("claimed_projection_sha256")
    if claimed is not None and claimed!=projection_sha:
        return result("CanonicalizationMismatch","caller-supplied projection commitment does not match independent interpretation",source_sha256=source_sha,canonical_sha256=projection_sha)
    return result("InterpretationEstablished","exact source bytes interpreted under frozen resource-bounded v1 profile",source_sha256=source_sha,canonical_sha256=projection_sha,projection_sha256=projection_sha,canonical_text=canonical_text,interpretation_profile=PROFILE_ID)

def load_manifest()->dict[str,Any]: return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
def git(*args:str)->str:
    return subprocess.run(["git",*args],check=True,text=True,capture_output=True).stdout.strip()

def validate_scope()->None:
    manifest=load_manifest()
    if manifest.get("qualified_parent")!=PARENT: raise SystemExit("manifest qualified parent drift")
    if manifest.get("expected_paths")!=EXPECTED_PATHS: raise SystemExit("manifest path census drift")
    head=git("rev-parse","HEAD"); parent=git("rev-parse","HEAD^")
    if parent!=PARENT: raise SystemExit(f"wrong parent: {parent}")
    count=int(git("rev-list","--count",f"{PARENT}..HEAD"))
    if count!=1: raise SystemExit(f"expected one authored commit, got {count}")
    changed=git("diff","--name-only",f"{PARENT}..HEAD").splitlines()
    if changed!=sorted(EXPECTED_PATHS): raise SystemExit(f"exact path set mismatch: {changed}")
    print(json.dumps({"tranche":TRANCHE,"head":head,"parent":parent,"commit_count":count,"paths":changed,"scope_result":"PASS"},sort_keys=True))

def validate_shape(manifest:dict[str,Any])->None:
    if manifest.get("tranche")!=TRANCHE or manifest.get("issue")!=1254: raise SystemExit("manifest tranche/issue drift")
    if manifest.get("candidate_revision")!="R2" or manifest.get("profile_version")!=2: raise SystemExit("R2 manifest version drift")
    if manifest.get("historical_r1_head")!=R1_HEAD: raise SystemExit("historical R1 head drift")
    p=manifest.get("interpretation_profile",{})
    expected_limits={"max_source_bytes":MAX_SOURCE_BYTES,"max_nesting_depth":MAX_NESTING_DEPTH,"max_container_items":MAX_CONTAINER_ITEMS,"max_string_utf8_bytes":MAX_STRING_UTF8_BYTES,"max_key_utf8_bytes":MAX_KEY_UTF8_BYTES}
    if p.get("resource_limits")!=expected_limits: raise SystemExit("resource limit profile drift")
    if p.get("canonicalizer_external_standard_claim") is not False: raise SystemExit("external conformance claim drift")
    if len(manifest.get("fixtures",[]))!=28: raise SystemExit("fixture census drift")

def validate_semantic()->None:
    manifest=load_manifest(); validate_shape(manifest)
    outputs={}; seen=set()
    for fixture in manifest["fixtures"]:
        fid=fixture.get("id")
        if not isinstance(fid,str) or fid in seen: raise SystemExit("fixture ids must be unique strings")
        seen.add(fid); out=interpret(fixture); outputs[fid]=out
        if out["disposition"]!=fixture.get("expected"):
            raise SystemExit(f"{fid}: expected {fixture.get('expected')} got {out['disposition']} ({out['reason']})")
        if out["grants_local_authority"] or out["grants_external_effect_authority"]: raise SystemExit(f"{fid}: authority escalation")
        if "expected_canonical" in fixture and out.get("canonical_text")!=fixture["expected_canonical"]: raise SystemExit(f"{fid}: canonical text mismatch")
    duplicate=next(f for f in manifest["fixtures"] if f["id"]=="duplicate_authority_rejected")
    if json.loads(duplicate["raw_json"]).get("authority") is not True: raise SystemExit("naive last-wins demonstration drift")
    if outputs["unicode_composed_preserved"]["projection_sha256"]==outputs["unicode_decomposed_preserved"]["projection_sha256"]: raise SystemExit("Unicode unexpectedly collapsed")
    if outputs["array_order_ab"]["projection_sha256"]==outputs["array_order_ba"]["projection_sha256"]: raise SystemExit("array order unexpectedly collapsed")
    for stem in ("source_size","nesting","object_items","array_items","string_size","key_size"):
        if outputs[f"{stem}_at_limit"]["disposition"]!="InterpretationEstablished": raise SystemExit(f"{stem} at-limit must pass")
        if outputs[f"{stem}_over_limit"]["disposition"]!="ResourceLimitExceeded": raise SystemExit(f"{stem} over-limit must fail")
    doc=DOC_PATH.read_text(encoding="utf-8").casefold()
    for phrase in [
        "signature-valid bytes != parser agreement != canonical-data agreement != semantic agreement != local recognition",
        "maximum source bytes: `65536`","maximum structural nesting depth: `32`","maximum members/items in any one object/array: `256`",
        "resource-limit compliance does not establish safety beyond the frozen profile",
        "grants_local_authority = false","grants_external_effect_authority = false","does not claim rfc 8785",
    ]:
        if phrase not in doc: raise SystemExit(f"normative phrase missing: {phrase}")
    print(json.dumps({"tranche":TRANCHE,"fixture_count":len(manifest["fixtures"]),"observed_dispositions":sorted({x["disposition"] for x in outputs.values()}),"semantic_result":"PASS","grants_local_authority":False,"grants_external_effect_authority":False},sort_keys=True))

def self_test()->None:
    cases=[
        ({"profile_id":PROFILE_ID,"raw_json":'{"x":1,"x":2}'},"DuplicateKeyRejected"),
        ({"profile_id":PROFILE_ID,"synthetic":{"kind":"source-size","bytes":MAX_SOURCE_BYTES+1}},"ResourceLimitExceeded"),
        ({"profile_id":PROFILE_ID,"synthetic":{"kind":"nesting-depth","depth":MAX_NESTING_DEPTH+1}},"ResourceLimitExceeded"),
        ({"profile_id":PROFILE_ID,"synthetic":{"kind":"array-items","count":MAX_CONTAINER_ITEMS+1}},"ResourceLimitExceeded"),
        ({"profile_id":PROFILE_ID,"synthetic":{"kind":"string-bytes","bytes":MAX_STRING_UTF8_BYTES+1}},"ResourceLimitExceeded"),
        ({"profile_id":PROFILE_ID,"synthetic":{"kind":"key-bytes","bytes":MAX_KEY_UTF8_BYTES+1}},"ResourceLimitExceeded"),
        ({"profile_id":PROFILE_ID,"raw_json":'{"x":1.5}'},"NumericDomainViolation"),
        ({"profile_id":"other","raw_json":'{"x":1}'},"ProfileUnsupported"),
    ]
    for fixture,expected in cases:
        actual=interpret(fixture)["disposition"]
        if actual!=expected: raise SystemExit(f"self-test expected {expected}, got {actual}")
    print("LEX-NET-032 R2 self-test PASS")

def main()->None:
    p=argparse.ArgumentParser(); g=p.add_mutually_exclusive_group(required=True)
    g.add_argument("--scope",action="store_true"); g.add_argument("--semantic",action="store_true"); g.add_argument("--self-test",action="store_true")
    a=p.parse_args()
    if a.scope: validate_scope()
    elif a.semantic: validate_semantic()
    else: self_test()
if __name__=="__main__": main()
