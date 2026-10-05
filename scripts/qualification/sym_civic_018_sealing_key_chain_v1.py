#!/usr/bin/env python3
from __future__ import annotations
import base64
import copy
import hashlib
import json
import re
from datetime import datetime
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_018_sealing_key_chain_v1.json"
PROGRAM = "SYM-CIVIC-018"
SCHEMA = "mycelix.sym-civic.sealing-key-chain-preflight.v1"
PARENT_SUBJECT = "c51a51c431bb8775e4d69e05d97ffb24de6f2617"

REJECT = "REJECT_SEALING_KEY_CHAIN"
SUFFICIENT = "SEALING_KEY_CHAIN_SUFFICIENT"
UNRESOLVED = "SEALING_KEY_CHAIN_UNRESOLVED"
INTERNAL_ERROR = "SEALING_KEY_CHAIN_INTERNAL_ERROR"

def canon(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)

def digest(value):
    return "sha256:" + hashlib.sha256(canon(value).encode("utf-8")).hexdigest()

def clone(value):
    return copy.deepcopy(value)

def b64url(value):
    return base64.urlsafe_b64encode(value).decode("ascii").rstrip("=")

def normalize_origin(value):
    try:
        parsed = urlsplit(value)
        if (
            parsed.scheme.lower() != "https"
            or not parsed.hostname
            or parsed.username is not None
            or parsed.password is not None
            or parsed.path not in ("", "/")
            or parsed.query
            or parsed.fragment
        ):
            raise ValueError("invalid origin")
        host = parsed.hostname.lower()
        port = parsed.port
        netloc = host if port in (None, 443) else f"{host}:{port}"
        return urlunsplit(("https", netloc, "", "", ""))
    except Exception as exc:
        raise ValueError("invalid origin") from exc

def origin_or_none(value):
    try:
        return normalize_origin(value)
    except Exception:
        return None

def synthetic_jwk(seed):
    return {
        "kty": "EC",
        "crv": "P-256",
        "x": b64url(hashlib.sha256((seed + ":x").encode("utf-8")).digest()),
        "y": b64url(hashlib.sha256((seed + ":y").encode("utf-8")).digest()),
    }

def jwk_thumbprint(jwk):
    required = {
        "crv": jwk["crv"],
        "kty": jwk["kty"],
        "x": jwk["x"],
        "y": jwk["y"],
    }
    raw = json.dumps(
        required,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")
    return b64url(hashlib.sha256(raw).digest())

def key_material(key):
    return {
        "kid": key["kid"],
        "jwk": key["jwk"],
        "arp-key-status": key["arp-key-status"],
        "arp-key-validity": key["arp-key-validity"],
    }

def make_key(seed, status="active", validity=None):
    validity = validity or [
        "2026-01-01T00:00:00Z",
        "2027-01-01T00:00:00Z",
    ]
    jwk = synthetic_jwk(seed)
    key = {
        "kid": jwk_thumbprint(jwk),
        "jwk": jwk,
        "arp-key-status": status,
        "arp-key-validity": list(validity),
    }
    key["binding_digest"] = digest(key_material(key))
    return key

def key_binding_ok(key):
    try:
        return (
            key["binding_digest"] == digest(key_material(key))
            and jwk_thumbprint(key["jwk"]) == key["kid"]
        )
    except Exception:
        return False

def sign_binding(signer, payload_digest):
    return digest({"signer": signer, "payload_digest": payload_digest})

def make_authorised_origin_document(register_origin, authority_origin, sealing_signer, operator_signer, register_signer):
    entries = [[authority_origin, sealing_signer, operator_signer]]
    entries.sort(key=lambda entry: entry[0].encode("utf-8"))
    payload_digest = digest(entries)
    return {
        "register_origin": register_origin,
        "entries": entries,
        "signed_by": register_signer,
        "payload_digest": payload_digest,
        "signature_digest": sign_binding(register_signer, payload_digest),
    }

def make_keyset(origin, signer_kid, keys):
    payload_digest = digest({"origin": origin, "keys": keys})
    return {
        "origin": origin,
        "signing_kid": signer_kid,
        "publication_timestamp": "2026-10-01T00:00:00Z",
        "keys": keys,
        "payload_digest": payload_digest,
        "signature_digest": sign_binding(signer_kid, payload_digest),
    }

def base():
    authority = "https://server.example"
    sealing = make_key("server-seal")
    signer = make_key("server-set-signer")
    registers = []

    for seed, register_origin, operator_signer in (
        ("a", "https://reg-a.example", "operator-a"),
        ("b", "https://reg-b.example", "operator-b"),
    ):
        register_signer = make_key(seed + "-register")
        auth = make_authorised_origin_document(
            register_origin,
            authority,
            signer["kid"],
            operator_signer,
            register_signer["kid"],
        )
        keyset = make_keyset(authority, signer["kid"], [clone(signer), clone(sealing)])
        registers.append({
            "register_origin": register_origin,
            "register_keys": [register_signer],
            "auth": auth,
            "sealing_keys": keyset,
        })

    kid = sealing["kid"]
    return {
        "registers": registers,
        "agreements": [
            {"register_origin": r["register_origin"], "authority_origin": authority}
            for r in registers
        ],
        "output": {
            "reconciliation_timestamp": "2026-10-02T12:00:00Z",
            "addressed_registers": [r["register_origin"] for r in registers],
            "authority_origin": authority,
            "sealing_key_kid": kid,
            "sealing_key_id": [authority, kid],
            "payload_kid_text": kid,
            "cose_kid_bytes": list(kid.encode("utf-8")),
            "keyset_url": authority + "/.well-known/arp-sealing-keys",
            "single_key_url": authority + "/.well-known/arp-sealing-keys/" + kid,
            "synthetic_sealing_signature": {
                "sealing_key_id": [authority, kid],
                "signed_payload_digest": "sha256:sealed-output",
            },
        },
    }

def mutate(candidate, mutation):
    node = candidate
    parts = mutation["path"].split(".")
    for part in parts[:-1]:
        node = node[int(part)] if isinstance(node, list) else node[part]
    leaf = parts[-1]
    target = int(leaf) if isinstance(node, list) else leaf
    if mutation["op"] == "replace":
        node[target] = mutation["value"]
    elif mutation["op"] == "remove":
        if isinstance(node, list):
            del node[target]
        else:
            node.pop(target, None)
    else:
        raise ValueError(mutation["op"])

def reseal(candidate):
    for register in candidate["registers"]:
        for key in register["register_keys"] + register["sealing_keys"]["keys"]:
            key["binding_digest"] = digest(key_material(key))
        auth = register["auth"]
        auth["payload_digest"] = digest(auth["entries"])
        auth["signature_digest"] = sign_binding(auth["signed_by"], auth["payload_digest"])
        keyset = register["sealing_keys"]
        keyset["payload_digest"] = digest({
            "origin": keyset["origin"],
            "keys": keyset["keys"],
        })
        keyset["signature_digest"] = sign_binding(
            keyset["signing_kid"], keyset["payload_digest"]
        )

def when(value):
    return datetime.fromisoformat(value.replace("Z", "+00:00"))

def usable_at(key, timestamp):
    try:
        start, end = (when(v) for v in key["arp-key-validity"][:2])
        instant = when(timestamp)
        if not (start <= instant < end):
            return False
        status = key["arp-key-status"]
        validity = key["arp-key-validity"]
        if status in ("active", "retired"):
            return len(validity) == 2
        if status == "revoked":
            return len(validity) == 3 and instant < when(validity[2])
        return False
    except Exception:
        return False

def structural_valid(candidate):
    if not isinstance(candidate, dict):
        return False
    output = candidate.get("output")
    agreements = candidate.get("agreements")
    registers = candidate.get("registers")
    if not isinstance(output, dict) or not isinstance(agreements, list) or not isinstance(registers, list):
        return False
    if any(not isinstance(a, dict) for a in agreements):
        return False
    if any(not isinstance(r, dict) for r in registers):
        return False

    required_output = (
        "reconciliation_timestamp",
        "addressed_registers",
        "authority_origin",
        "sealing_key_kid",
        "sealing_key_id",
        "payload_kid_text",
        "cose_kid_bytes",
        "keyset_url",
        "single_key_url",
        "synthetic_sealing_signature",
    )
    if any(field not in output for field in required_output):
        return False
    addressed = output["addressed_registers"]
    if not isinstance(addressed, list) or any(not isinstance(v, str) for v in addressed):
        return False
    signature = output["synthetic_sealing_signature"]
    if not isinstance(signature, dict):
        return False
    if any(field not in signature for field in ("sealing_key_id", "signed_payload_digest")):
        return False

    required_agreement = ("register_origin", "authority_origin")
    required_register = ("register_origin", "register_keys", "auth", "sealing_keys")
    required_auth = ("signed_by", "entries", "payload_digest", "signature_digest")
    required_keyset = (
        "origin",
        "signing_kid",
        "publication_timestamp",
        "keys",
        "payload_digest",
        "signature_digest",
    )
    required_key = (
        "kid",
        "jwk",
        "arp-key-status",
        "arp-key-validity",
        "binding_digest",
    )
    required_jwk = ("kty", "crv", "x", "y")

    for agreement in agreements:
        if any(field not in agreement for field in required_agreement):
            return False
    for register in registers:
        if any(field not in register for field in required_register):
            return False
        register_keys = register["register_keys"]
        if not isinstance(register_keys, list) or any(not isinstance(k, dict) for k in register_keys):
            return False
        auth = register["auth"]
        if not isinstance(auth, dict) or any(field not in auth for field in required_auth):
            return False
        entries = auth["entries"]
        if not isinstance(entries, list):
            return False
        if any(
            not isinstance(entry, list)
            or len(entry) != 3
            or any(not isinstance(value, str) for value in entry)
            for entry in entries
        ):
            return False
        keyset = register["sealing_keys"]
        if not isinstance(keyset, dict) or any(field not in keyset for field in required_keyset):
            return False
        keyset_keys = keyset["keys"]
        if not isinstance(keyset_keys, list) or any(not isinstance(k, dict) for k in keyset_keys):
            return False
        for key in register_keys + keyset_keys:
            if any(field not in key for field in required_key):
                return False
            jwk = key["jwk"]
            if not isinstance(jwk, dict) or any(field not in jwk for field in required_jwk):
                return False
    return True

def _validate_primary(candidate):
    if not isinstance(candidate, dict):
        return REJECT
    output = candidate.get("output")
    agreements = candidate.get("agreements")
    registers = candidate.get("registers")
    if not isinstance(output, dict) or not isinstance(agreements, list) or not isinstance(registers, list):
        return REJECT
    if any(not isinstance(a, dict) for a in agreements):
        return REJECT
    if any(not isinstance(r, dict) for r in registers):
        return REJECT
    try:
        authority = normalize_origin(output["authority_origin"])
        when(output["reconciliation_timestamp"])
    except Exception:
        return REJECT

    try:
        addressed = [normalize_origin(v) for v in output["addressed_registers"]]
    except Exception:
        return REJECT
    if len(addressed) != len(set(addressed)):
        return REJECT
    if output["addressed_registers"] != sorted(
        output["addressed_registers"],
        key=lambda value: value.encode("utf-8"),
    ):
        return REJECT

    agreement_origins = []
    for agreement in candidate["agreements"]:
        try:
            agreement_origins.append(normalize_origin(agreement["register_origin"]))
            if normalize_origin(agreement["authority_origin"]) != authority:
                return REJECT
        except Exception:
            return REJECT
    if sorted(agreement_origins) != sorted(addressed):
        return REJECT

    kid = output.get("sealing_key_kid")
    if output.get("sealing_key_id") != [authority, kid]:
        return REJECT
    if not isinstance(kid, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", kid):
        return REJECT
    if output.get("payload_kid_text") != kid:
        return REJECT
    if output.get("cose_kid_bytes") != list(kid.encode("utf-8")):
        return REJECT
    if output.get("keyset_url") != authority + "/.well-known/arp-sealing-keys":
        return REJECT
    if output.get("single_key_url") != output.get("keyset_url") + "/" + kid:
        return REJECT

    signature = output.get("synthetic_sealing_signature", {})
    if signature.get("sealing_key_id") != [authority, kid]:
        return REJECT
    if signature.get("signed_payload_digest") != "sha256:sealed-output":
        return REJECT

    for agreement in candidate["agreements"]:
        register_origin = normalize_origin(agreement["register_origin"])
        register = next(
            (r for r in candidate["registers"]
             if origin_or_none(r.get("register_origin")) == register_origin),
            None,
        )
        if register is None:
            return REJECT

        auth = register.get("auth")
        entries = auth.get("entries", []) if isinstance(auth, dict) else []
        try:
            if entries != sorted(entries, key=lambda entry: entry[0].encode("utf-8")):
                return REJECT
            if any(not isinstance(entry, list) or len(entry) != 3 for entry in entries):
                return REJECT
            if len({entry[0] for entry in entries}) != len(entries):
                return REJECT
        except Exception:
            return REJECT

        target = [
            entry for entry in entries
            if isinstance(entry, list)
            and len(entry) == 3
            and origin_or_none(entry[0]) == authority
        ]
        if len(target) != 1:
            return REJECT

        sealing_signer, operator_signer = target[0][1], target[0][2]
        if sealing_signer == operator_signer:
            return REJECT

        register_key = next(
            (key for key in register.get("register_keys", [])
             if key.get("kid") == auth.get("signed_by")),
            None,
        )
        if register_key is None or not key_binding_ok(register_key):
            return REJECT
        if (
            auth.get("payload_digest") != digest(entries)
            or auth.get("signature_digest")
            != sign_binding(auth.get("signed_by"), auth.get("payload_digest"))
        ):
            return REJECT

        keyset = register.get("sealing_keys")
        if (
            not keyset
            or origin_or_none(keyset.get("origin", "")) != authority
            or keyset.get("signing_kid") != sealing_signer
        ):
            return REJECT

        signer_keys = [k for k in keyset.get("keys", []) if k.get("kid") == sealing_signer]
        if len(signer_keys) != 1:
            return REJECT
        signer_key = signer_keys[0]
        if not key_binding_ok(signer_key):
            return REJECT
        if not usable_at(signer_key, keyset.get("publication_timestamp", "")):
            return REJECT
        if (
            keyset.get("payload_digest")
            != digest({"origin": keyset.get("origin"), "keys": keyset.get("keys")})
            or keyset.get("signature_digest")
            != sign_binding(keyset.get("signing_kid"), keyset.get("payload_digest"))
        ):
            return REJECT

        sealing_keys = [k for k in keyset.get("keys", []) if k.get("kid") == kid]
        if len(sealing_keys) != 1:
            return REJECT
        sealing_key = sealing_keys[0]
        if not key_binding_ok(sealing_key):
            return REJECT
        if not usable_at(sealing_key, output["reconciliation_timestamp"]):
            return REJECT

    return SUFFICIENT

def validate_reference_impl(candidate):
    output = candidate["output"]
    try:
        authority = normalize_origin(output["authority_origin"])
        timestamp = output["reconciliation_timestamp"]
        when(timestamp)
    except Exception:
        return REJECT

    try:
        stated_values = [normalize_origin(v) for v in output["addressed_registers"]]
    except Exception:
        return REJECT
    if len(stated_values) != len(output["addressed_registers"]):
        return REJECT
    if output["addressed_registers"] != sorted(
        output["addressed_registers"],
        key=lambda value: value.encode("utf-8"),
    ):
        return REJECT
    stated = set(stated_values)

    declared = set()
    for agreement in candidate["agreements"]:
        try:
            register_origin = normalize_origin(agreement["register_origin"])
            if normalize_origin(agreement["authority_origin"]) != authority:
                return REJECT
        except Exception:
            return REJECT
        declared.add(register_origin)
    if declared != stated:
        return REJECT

    kid = output.get("sealing_key_kid")
    if output.get("sealing_key_id") != [authority, kid]:
        return REJECT
    if not isinstance(kid, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", kid):
        return REJECT
    if output.get("payload_kid_text") != kid:
        return REJECT
    if output.get("cose_kid_bytes") != list(kid.encode("utf-8")):
        return REJECT
    if output.get("keyset_url") != authority + "/.well-known/arp-sealing-keys":
        return REJECT
    if output.get("single_key_url") != output.get("keyset_url") + "/" + kid:
        return REJECT

    for register in candidate["registers"]:
        register_origin = origin_or_none(register.get("register_origin"))
        if register_origin is None:
            return REJECT
        auth = register.get("auth")
        if not isinstance(auth, dict):
            return REJECT
        entries = auth.get("entries")
        if not isinstance(entries, list):
            return REJECT
        try:
            if entries != sorted(entries, key=lambda entry: entry[0].encode("utf-8")):
                return REJECT
        except Exception:
            return REJECT
        target = [
            entry for entry in entries
            if isinstance(entry, list) and len(entry) == 3
            and origin_or_none(entry[0]) == authority
        ]
        if len(target) != 1 or target[0][1] == target[0][2]:
            return REJECT

        register_signer = next(
            (key for key in register["register_keys"]
             if key["kid"] == auth.get("signed_by")),
            None,
        )
        if register_signer is None or not key_binding_ok(register_signer):
            return REJECT
        if auth.get("payload_digest") != digest(entries):
            return REJECT
        if auth.get("signature_digest") != sign_binding(
            auth.get("signed_by"), auth.get("payload_digest")
        ):
            return REJECT

        keyset = register.get("sealing_keys")
        if not isinstance(keyset, dict):
            return REJECT
        if origin_or_none(keyset.get("origin", "")) != authority:
            return REJECT
        signer_kid = target[0][1]
        if keyset.get("signing_kid") != signer_kid:
            return REJECT

        signer_candidates = [
            key for key in keyset.get("keys", []) if key.get("kid") == signer_kid
        ]
        if len(signer_candidates) != 1:
            return REJECT
        signer = signer_candidates[0]
        if not key_binding_ok(signer):
            return REJECT
        if not usable_at(signer, keyset.get("publication_timestamp", "")):
            return REJECT
        if keyset.get("payload_digest") != digest({
            "origin": keyset.get("origin"),
            "keys": keyset.get("keys"),
        }):
            return REJECT
        if keyset.get("signature_digest") != sign_binding(
            keyset.get("signing_kid"), keyset.get("payload_digest")
        ):
            return REJECT

        selected = [key for key in keyset.get("keys", []) if key.get("kid") == kid]
        if len(selected) != 1:
            return REJECT
        sealing_key = selected[0]
        try:
            if jwk_thumbprint(sealing_key["jwk"]) != kid:
                return REJECT
        except Exception:
            return REJECT
        if not usable_at(sealing_key, timestamp):
            return REJECT

    return SUFFICIENT

def validate_primary(candidate):
    try:
        if not structural_valid(candidate):
            return REJECT
        return _validate_primary(candidate)
    except Exception:
        return INTERNAL_ERROR

def validate_reference(candidate):
    try:
        if not structural_valid(candidate):
            return REJECT
        return validate_reference_impl(candidate)
    except Exception:
        return INTERNAL_ERROR

def candidate_for(case):
    candidate = base()
    for mutation in case.get("mutations", []):
        mutate(candidate, mutation)
    if any(mutation.get("reseal") for mutation in case.get("mutations", [])):
        reseal(candidate)
    return candidate

def verdict(candidate):
    primary = validate_primary(candidate)
    reference = validate_reference(candidate)
    if INTERNAL_ERROR in (primary, reference):
        return UNRESOLVED, primary, reference
    if primary != reference:
        return UNRESOLVED, primary, reference
    return primary, primary, reference

def main():
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assert document["schema"] == SCHEMA
    assert document["program"] == PROGRAM
    assert document["analysis_role"] == "research_only"
    assert document["parent_subject"] == PARENT_SUBJECT

    cases = document["cases"]
    assert [case["id"] for case in cases] == [f"C-{i:02d}" for i in range(1, 79)]
    for case in cases:
        assert set(case) == {"id", "family", "mutations"}
        lowered = canon(case).lower()
        assert not any(
            token in lowered
            for token in (
                "expected_verdict",
                "expected_disposition",
                "oracle_verdict",
                "candidate_verdict",
            )
        )

    results = {}
    disagreements = []
    for case in cases:
        outcome, primary, reference = verdict(candidate_for(case))
        results[case["id"]] = outcome
        if primary != reference:
            disagreements.append({
                "case": case["id"],
                "primary": primary,
                "reference": reference,
            })

    census = {
        REJECT: sum(v == REJECT for v in results.values()),
        SUFFICIENT: sum(v == SUFFICIENT for v in results.values()),
        UNRESOLVED: sum(v == UNRESOLVED for v in results.values()),
    }
    assert not disagreements, disagreements
    assert census == {
        REJECT: 73,
        SUFFICIENT: 5,
        UNRESOLVED: 0,
    }, census

    assert verdict(candidate_for(cases[0]))[0] == SUFFICIENT

    class ExplodingDict(dict):
        def get(self, *args, **kwargs):
            raise RuntimeError("synthetic unexpected verifier failure")

    exploding = ExplodingDict()
    assert verdict(exploding)[0] == UNRESOLVED
    assert verdict(None)[0] == REJECT

    variant = clone(candidate_for(cases[0]))
    variant["output"]["authority_origin"] = "https://SERVER.EXAMPLE:443/"
    assert verdict(variant)[0] == SUFFICIENT

    variant = clone(candidate_for(cases[0]))
    variant["agreements"][0]["authority_origin"] = "https://SERVER.EXAMPLE:443/"
    assert verdict(variant)[0] == SUFFICIENT

    malformed = clone(candidate_for(cases[0]))
    malformed["registers"][0]["auth"]["entries"][0][0] = "https://server.example:bad"
    assert verdict(malformed)[0] == REJECT

    malformed = clone(candidate_for(cases[0]))
    malformed["registers"][0]["sealing_keys"]["origin"] = "https://server.example:bad"
    assert verdict(malformed)[0] == REJECT

    malformed = clone(candidate_for(cases[0]))
    malformed["registers"][0]["register_origin"] = "https://reg-a.example:bad"
    assert verdict(malformed)[0] == REJECT

    reordered = clone(candidate_for(cases[0]))
    reordered["output"]["addressed_registers"] = list(reversed(reordered["output"]["addressed_registers"]))
    assert verdict(reordered)[0] == REJECT

    structural = [
        {"agreements": None},
        {"registers": None},
        {"agreements.0": "malformed"},
        {"registers.0": "malformed"},
        {"registers.0.auth": []},
        {"registers.0.sealing_keys": "malformed"},
        {"registers.0.sealing_keys.keys": None},
        {"registers.0.register_keys.0": "malformed"},
    ]
    for patch in structural:
        malformed = clone(candidate_for(cases[0]))
        for path, value in patch.items():
            mutation = {"op": "replace", "path": path, "value": value}
            mutate(malformed, mutation)
        assert verdict(malformed)[0] == REJECT

    for case in cases[48:]:
        assert verdict(candidate_for(case))[0] == REJECT

    print("SYM-CIVIC-018 DERIVED=" + canon(census))
    print("SYM-CIVIC-018 METAMORPHIC=PASS")
    print("SYM-CIVIC-018 PASS")

if __name__ == "__main__":
    main()
