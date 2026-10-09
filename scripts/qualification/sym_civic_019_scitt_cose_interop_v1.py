#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

SCHEMA = "SYM-CIVIC-019-SCITT-COSE-INTEROP-V1"
TAG_COSE_SIGN1 = 18
ALG_EDDSA = -8
ALG_ES256 = -7
HP_ALG = 1
HP_CONTENT_TYPE = 3
HP_CWT_CLAIMS = 15
HP_VDS = 395
HP_VDP = 396
VDP_INCLUSION = -1
VDS_RFC9162_SHA256 = 1
PINNED_SOURCE_COMMIT = "41811d1e3d9b32d000b1e7f26cafdb116f572167"

def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\x00" + data).hexdigest()

def assert_surface_bindings():
    root = Path(__file__).resolve().parents[2]
    manifest_path = root / "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_manifest_v1.json"
    surface = strict_json_file(manifest_path, "qualification surface manifest")
    if surface.get("schema") != "SYM-CIVIC-019-RECEIPT-PROOF-BINDING-MANIFEST-V1":
        raise Reject("qualification surface manifest schema mismatch")
    bindings = surface.get("files_git_blob_sha")
    expected = {
        "verifier": "scripts/qualification/sym_civic_019_receipt_proof_binding_v1.py",
        "corpus": "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_v1.json",
        "workflow": ".github/workflows/sym-civic-019-receipt-proof-binding.yml",
        "external_interop_verifier": "scripts/qualification/sym_civic_019_scitt_cose_interop_v1.py",
        "external_interop_manifest": "mycelix-workspace/docs/civic-resilience/scitt-cose-v1/manifest.json",
    }
    if not isinstance(bindings, dict) or set(bindings) != set(expected):
        raise Reject("qualification surface manifest file-binding schema mismatch")
    for label, rel in expected.items():
        path = root / rel
        if not path.is_file():
            raise Reject("qualification surface file missing: " + rel)
        if git_blob_sha(path.read_bytes()) != bindings[label]:
            raise Reject("qualification surface file binding mismatch: " + label)


class Reject(Exception):
    pass

class Unresolved(Exception):
    pass

MAX_TREE_SIZE = 1 << 62

def strict_json_loads(text: str, what: str):
    def reject_duplicates(pairs):
        out = {}
        for key, value in pairs:
            if key in out:
                raise Reject(f"{what} contains duplicate JSON object key: {key}")
            out[key] = value
        return out

    def reject_constant(token):
        raise Reject(f"{what} contains non-standard JSON constant: {token}")

    try:
        return json.loads(text, object_pairs_hook=reject_duplicates, parse_constant=reject_constant)
    except Reject:
        raise
    except json.JSONDecodeError as exc:
        raise Reject(f"{what} is not valid JSON: {exc}") from exc

def strict_json_file(path: Path, what: str):
    return strict_json_loads(path.read_text(), what)

def require_exact_keys(obj: dict, required: set[str], what: str, optional: set[str] | None = None):
    if not isinstance(obj, dict):
        raise Reject(f"{what} must be a JSON object")
    allowed = required | (optional or set())
    missing = required - set(obj)
    unknown = set(obj) - allowed
    if missing:
        raise Reject(f"{what} missing required keys: {sorted(missing)}")
    if unknown:
        raise Reject(f"{what} contains unknown keys: {sorted(unknown)}")

class Node:
    __slots__ = ("mt", "value", "raw", "children", "tag")
    def __init__(self, mt: int, value, raw: bytes, children=(), tag=None):
        self.mt = mt
        self.value = value
        self.raw = raw
        self.children = tuple(children)
        self.tag = tag

class Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.i = 0

    def head(self):
        if self.i >= len(self.data):
            raise Reject("truncated CBOR")
        start = self.i
        first = self.data[self.i]
        self.i += 1
        mt = first >> 5
        ai = first & 31
        if ai < 24:
            value = ai
        elif ai == 24:
            value = int.from_bytes(self._take(1), "big")
            if value < 24:
                raise Reject("non-minimal CBOR additional-information encoding")
        elif ai == 25:
            value = int.from_bytes(self._take(2), "big")
            if value < 256:
                raise Reject("non-minimal CBOR additional-information encoding")
        elif ai == 26:
            value = int.from_bytes(self._take(4), "big")
            if value < 65536:
                raise Reject("non-minimal CBOR additional-information encoding")
        elif ai == 27:
            value = int.from_bytes(self._take(8), "big")
            if value < 4294967296:
                raise Reject("non-minimal CBOR additional-information encoding")
        else:
            raise Reject("indefinite or reserved CBOR encoding")
        return mt, value, start

    def _take(self, width: int) -> bytes:
        end = self.i + width
        if end > len(self.data):
            raise Reject("truncated CBOR argument")
        out = self.data[self.i:end]
        self.i = end
        return out

    def item(self) -> Node:
        mt, value, start = self.head()
        if mt == 0:
            return Node(mt, value, self.data[start:self.i])
        if mt == 1:
            return Node(mt, -1 - value, self.data[start:self.i])
        if mt in (2, 3):
            raw = self._take(value)
            if mt == 2:
                decoded = bytes(raw)
            else:
                try:
                    decoded = raw.decode("utf-8")
                except UnicodeDecodeError as exc:
                    raise Reject("invalid UTF-8 text string") from exc
            return Node(mt, decoded, self.data[start:self.i])
        if mt == 4:
            children = [self.item() for _ in range(value)]
            return Node(mt, tuple(x.value for x in children), self.data[start:self.i], children)
        if mt == 5:
            children = []
            keys = set()
            for _ in range(value):
                k = self.item()
                v = self.item()
                semantic_key = (k.mt, k.value)
                if semantic_key in keys:
                    raise Reject("duplicate CBOR map key")
                keys.add(semantic_key)
                children.extend((k, v))
            pairs = tuple(
                (children[i].value, children[i + 1].value)
                for i in range(0, len(children), 2)
            )
            return Node(mt, pairs, self.data[start:self.i], children)
        if mt == 6:
            child = self.item()
            return Node(mt, child.value, self.data[start:self.i], (child,), tag=value)
        if mt == 7:
            if value in (20, 21):
                return Node(mt, value == 21, self.data[start:self.i])
            if value == 22:
                return Node(mt, None, self.data[start:self.i])
            raise Reject("unsupported CBOR simple/float type")
        raise Reject("unsupported CBOR major type")

    def parse(self) -> Node:
        node = self.item()
        if self.i != len(self.data):
            raise Reject("trailing bytes after CBOR object")
        return node

def parse(data: bytes) -> Node:
    return Reader(data).parse()

def expect_bstr(node: Node, what: str) -> bytes:
    if node.mt != 2:
        raise Reject(f"{what} must be a CBOR byte string")
    return node.value

def expect_text(node: Node, what: str) -> str:
    if node.mt != 3:
        raise Reject(f"{what} must be a CBOR text string")
    return node.value

def expect_int(node: Node, what: str) -> int:
    if node.mt not in (0, 1) or not isinstance(node.value, int):
        raise Reject(f"{what} must be an integer")
    return node.value

def expect_array(node: Node, what: str) -> Node:
    if node.mt != 4:
        raise Reject(f"{what} must be an array")
    return node

def expect_map(node: Node, what: str) -> Node:
    if node.mt != 5:
        raise Reject(f"{what} must be a map")
    return node

def map_pairs(node: Node):
    expect_map(node, "map")
    return [(node.children[i], node.children[i + 1])
            for i in range(0, len(node.children), 2)]

def map_get(node: Node, key: int, what: str) -> Node:
    matches = [v for k, v in map_pairs(node) if k.mt in (0, 1) and k.value == key]
    if len(matches) != 1:
        raise Reject(f"{what} requires exactly one map key {key}")
    return matches[0]

def cbor_uint(major: int, value: int) -> bytes:
    if value < 24:
        return bytes([(major << 5) | value])
    if value < 256:
        return bytes([(major << 5) | 24, value])
    if value < 65536:
        return bytes([(major << 5) | 25]) + value.to_bytes(2, "big")
    if value < 2**32:
        return bytes([(major << 5) | 26]) + value.to_bytes(4, "big")
    return bytes([(major << 5) | 27]) + value.to_bytes(8, "big")

def cbor_encode(value) -> bytes:
    if isinstance(value, bytes):
        return cbor_uint(2, len(value)) + value
    if isinstance(value, str):
        raw = value.encode("utf-8")
        return cbor_uint(3, len(raw)) + raw
    if isinstance(value, int):
        if value >= 0:
            return cbor_uint(0, value)
        return cbor_uint(1, -1 - value)
    if isinstance(value, (list, tuple)):
        return cbor_uint(4, len(value)) + b"".join(cbor_encode(v) for v in value)
    raise TypeError(type(value).__name__)

def sig_structure(protected: bytes, payload: bytes) -> bytes:
    return cbor_encode(["Signature1", protected, b"", payload])

def parse_sign1(raw: bytes, what: str) -> dict:
    top = parse(raw)
    if top.mt != 6 or top.tag != TAG_COSE_SIGN1:
        raise Reject(f"{what} must be tagged COSE_Sign1 (18)")
    body = expect_array(top.children[0], f"{what} body")
    if len(body.children) != 4:
        raise Reject(f"{what} must have four COSE_Sign1 fields")
    protected = expect_bstr(body.children[0], f"{what} protected")
    unprotected = expect_map(body.children[1], f"{what} unprotected")
    payload_node = body.children[2]
    if payload_node.mt == 2:
        payload = payload_node.value
    elif payload_node.mt == 7 and payload_node.value is None:
        payload = None
    else:
        raise Reject(f"{what} payload must be a CBOR byte string or null")
    signature = expect_bstr(body.children[3], f"{what} signature")
    protected_map = expect_map(parse(protected), f"{what} protected map")
    protected_pairs = map_pairs(protected_map)
    unprotected_pairs = map_pairs(unprotected)
    protected_keys = {(k.mt, k.value) for k, _ in protected_pairs}
    unprotected_keys = {(k.mt, k.value) for k, _ in unprotected_pairs}
    if protected_keys & unprotected_keys:
        raise Reject(f"{what} repeats a header label across protected/unprotected buckets")
    if any(k.mt in (0, 1) and k.value == 2 for k, _ in unprotected_pairs):
        raise Reject(f"{what} crit must be protected")
    crit_node = next((v for k, v in protected_pairs if k.mt in (0, 1) and k.value == 2), None)
    if crit_node is not None:
        crit = expect_array(crit_node, f"{what} crit")
        labels = [expect_int(x, f"{what} crit label") for x in crit.children]
        if not labels or len(labels) != len(set(labels)):
            raise Reject(f"{what} critical-label set is empty or duplicated")
        protected_int_labels = {k.value for k, _ in protected_pairs if k.mt in (0, 1)}
        understood = {1, 2, 395}
        for label in labels:
            if label not in protected_int_labels:
                raise Reject(f"{what} critical label is not protected")
            if label not in understood:
                raise Reject(f"{what} critical label is unsupported")
    return {
        "protected": protected,
        "protected_map": protected_map,
        "unprotected": unprotected,
        "payload": payload,
        "signature": signature,
    }

def header_alg(node: dict, what: str) -> int:
    alg = expect_int(map_get(node["protected_map"], HP_ALG, what + " alg"), what + " alg")
    if alg not in (ALG_EDDSA, ALG_ES256):
        raise Reject(f"unsupported {what} COSE algorithm {alg}")
    return alg

def claims_from_statement(sign1: dict) -> tuple[str, str]:
    claims_map = expect_map(
        map_get(sign1["protected_map"], HP_CWT_CLAIMS, "Signed Statement CWT claims"),
        "Signed Statement CWT claims",
    )
    issuer = expect_text(map_get(claims_map, 1, "issuer"), "issuer")
    subject = expect_text(map_get(claims_map, 2, "subject"), "subject")
    return issuer, subject

def der_len(value: int) -> bytes:
    if value < 128:
        return bytes([value])
    raw = value.to_bytes((value.bit_length() + 7) // 8, "big")
    return bytes([0x80 | len(raw)]) + raw

def der_integer(value: bytes) -> bytes:
    value = value.lstrip(b"\x00") or b"\x00"
    if value[0] & 0x80:
        value = b"\x00" + value
    return b"\x02" + der_len(len(value)) + value

def ecdsa_raw_to_der(signature: bytes) -> bytes:
    if len(signature) != 64:
        raise Reject("ES256 signature must be 64-byte raw r||s")
    body = der_integer(signature[:32]) + der_integer(signature[32:])
    return b"\x30" + der_len(len(body)) + body

def openssl_verify(public_key_pem: bytes, algorithm: int, message: bytes, signature: bytes) -> bool:
    with tempfile.TemporaryDirectory(prefix="mycelix019-interop-") as td:
        key_path = Path(td) / "public.pem"
        msg_path = Path(td) / "message.bin"
        sig_path = Path(td) / "signature.bin"
        key_path.write_bytes(public_key_pem)
        msg_path.write_bytes(message)
        if algorithm == ALG_ES256:
            sig_path.write_bytes(ecdsa_raw_to_der(signature))
            cmd = [
                "openssl", "dgst", "-sha256",
                "-verify", str(key_path),
                "-signature", str(sig_path),
                str(msg_path),
            ]
        else:
            sig_path.write_bytes(signature)
            cmd = [
                "openssl", "pkeyutl", "-verify",
                "-pubin", "-inkey", str(key_path),
                "-rawin", "-in", str(msg_path),
                "-sigfile", str(sig_path),
            ]
        result = subprocess.run(cmd, text=True, capture_output=True)
        if result.returncode == 0:
            return True
        combined = (result.stdout + result.stderr).strip().lower()
        if "verification failure" in combined or "invalid signature" in combined:
            return False
        raise Unresolved("OpenSSL verification backend failure: " + combined)

def sha256(data: bytes) -> bytes:
    return hashlib.sha256(data).digest()

def merkle_tree(leaves: list[bytes]) -> bytes:
    if not leaves:
        raise Reject("RFC9162 tree cannot be empty")
    if len(leaves) == 1:
        return leaves[0]
    k = 1 << ((len(leaves) - 1).bit_length() - 1)
    return sha256(b"\x01" + merkle_tree(leaves[:k]) + merkle_tree(leaves[k:]))

def audit_path(leaves: list[bytes], index: int) -> list[bytes]:
    if not leaves or index < 0 or index >= len(leaves):
        raise Reject("invalid audit-path index")
    if len(leaves) == 1:
        return []
    k = 1 << ((len(leaves) - 1).bit_length() - 1)
    if index < k:
        return audit_path(leaves[:k], index) + [merkle_tree(leaves[k:])]
    return audit_path(leaves[k:], index - k) + [merkle_tree(leaves[:k])]

def deterministic_tree(vector_id: str, statement_bytes: bytes, size: int, index: int):
    if size != 8 or index != 2:
        raise Reject("external v1 corpus defines only tree_size=8, leaf_index=2")
    entries = []
    for i in range(size):
        if i == index:
            entries.append(sha256(statement_bytes))
        else:
            filler = f"scitt-cose test vectors v1 :: {vector_id} :: filler leaf {i}".encode("ascii")
            entries.append(sha256(filler))
    leaves = [sha256(b"\x00" + entry) for entry in entries]
    return entries[index], merkle_tree(leaves), audit_path(leaves, index)

def receipt_proof(receipt: dict) -> dict:
    vds = expect_int(map_get(receipt["protected_map"], HP_VDS, "Receipt VDS"), "Receipt VDS")
    if vds != VDS_RFC9162_SHA256:
        return {"valid": False, "reason": "UNSUPPORTED_VDS", "vds": vds}
    vdp = expect_map(map_get(receipt["unprotected"], HP_VDP, "Receipt VDP"), "Receipt VDP")
    proof_list = expect_array(map_get(vdp, VDP_INCLUSION, "Receipt inclusion proofs"), "Receipt inclusion proofs")
    if len(proof_list.children) == 0:
        raise Reject("Receipt inclusion proofs must contain at least one proof")
    if len(proof_list.children) > 16:
        raise Reject("Receipt inclusion proofs exceed the external profile cap")
    if len(proof_list.children) != 1:
        raise Reject("external pinned v1 profile requires exactly one inclusion proof")
    proof_bytes = expect_bstr(proof_list.children[0], "Receipt inclusion proof")
    proof = expect_array(parse(proof_bytes), "Receipt inclusion proof")
    if len(proof.children) != 3:
        raise Reject("Receipt inclusion proof arity must be three")
    size = expect_int(proof.children[0], "Receipt tree_size")
    index = expect_int(proof.children[1], "Receipt leaf_index")
    if size < 1 or size > MAX_TREE_SIZE:
        raise Reject("Receipt tree_size exceeds the supported 2^62 ceiling")
    if index < 0 or index >= size:
        raise Reject("Receipt proof index out of range")
    path_node = expect_array(proof.children[2], "Receipt inclusion path")
    path = [expect_bstr(x, "Receipt inclusion path node") for x in path_node.children]
    if any(len(x) != 32 for x in path):
        raise Reject("Receipt inclusion path nodes must be 32-byte hashes")
    if size > 1:
        expected_len = 0
        n, m = size, index
        while n > 1:
            k = 1 << ((n - 1).bit_length() - 1)
            if m < k:
                n = k
            else:
                n, m = n - k, m - k
            expected_len += 1
        if len(path) != expected_len:
            raise Reject("Receipt inclusion path length does not match tree_size/leaf_index")
    return {"valid": True, "vds": vds, "proof_count": len(proof_list.children), "proof_bytes": proof_bytes, "tree_size": size, "leaf_index": index, "path": path}

def verify_receipt(statement_bytes: bytes, receipt_bytes: bytes, log_key_pem: bytes, vector: dict) -> dict:
    receipt = parse_sign1(receipt_bytes, "Receipt")
    alg = header_alg(receipt, "Receipt")
    expected_header = vector["expected"]["protected_header"]["receipt"]
    if alg != expected_header["alg_code"]:
        raise Reject("Receipt protected alg disagrees with pinned expected receipt algorithm")
    expected_header = vector["expected"]["protected_header"]["receipt"]
    if expected_header["vds_label"] != 395:
        raise Reject("pinned expected Receipt VDS label is not 395")
    proof = receipt_proof(receipt)
    if not proof["valid"]:
        return proof
    expected = vector["expected"]
    if receipt["protected_map"].mt != 5:
        raise Reject("Receipt protected headers must decode as a map")
    expected_leaf_entry = bytes.fromhex(expected["leaf_entry"])
    actual_leaf_entry = sha256(statement_bytes)
    if actual_leaf_entry != expected_leaf_entry:
        raise Reject("leaf_entry does not equal SHA-256(statement.cose)")
    deterministic_entry, deterministic_root, deterministic_path = deterministic_tree(
        vector["id"], statement_bytes, proof["tree_size"], proof["leaf_index"]
    )
    if deterministic_entry != expected_leaf_entry:
        raise Reject("deterministic tree entry disagrees with expected leaf_entry")
    if proof["tree_size"] != expected["tree_size"] or proof["leaf_index"] != expected["leaf_index"]:
        raise Reject("receipt proof coordinates disagree with pinned expected metadata")
    expected_path = [bytes.fromhex(x) for x in expected["inclusion_path"]]
    if proof["path"] != expected_path:
        raise Reject("receipt proof path does not match pinned expected path")
    reconstructed = sha256(b"\x00" + actual_leaf_entry)
    fn = proof["leaf_index"]
    sn = proof["tree_size"] - 1
    p = 0
    while sn:
        take = bool(fn & 1) or fn < sn
        if take:
            if p >= len(proof["path"]):
                raise Reject("Receipt inclusion path exhausted")
            sibling = proof["path"][p]
            reconstructed = (
                sha256(b"\x01" + sibling + reconstructed)
                if (fn & 1)
                else sha256(b"\x01" + reconstructed + sibling)
            )
            p += 1
        fn //= 2
        sn //= 2
    if p != len(proof["path"]):
        raise Reject("Receipt inclusion path has unused nodes")
    proof_root_matches = reconstructed == deterministic_root if receipt["payload"] is None else reconstructed == receipt["payload"]
    tree_root_matches = deterministic_root == reconstructed
    if receipt["payload"] is not None and receipt["payload"] != deterministic_root:
        proof_root_matches = False
    signed_payload = deterministic_root if receipt["payload"] is None else receipt["payload"]
    signature_valid = openssl_verify(
        log_key_pem,
        alg,
        sig_structure(receipt["protected"], signed_payload),
        receipt["signature"],
    )
    deterministic_path_matches = deterministic_path == proof["path"]
    if proof_root_matches and tree_root_matches and signature_valid:
        if expected["reconstructed_root"] != reconstructed.hex():
            raise Reject("expected reconstructed_root is inconsistent with the independently reconstructed root")
        if not deterministic_path_matches:
            raise Reject("independently generated audit path disagrees with receipt path")
        return {
            "valid": True,
            "reason": "VALID",
            "algorithm": alg,
            "payload_mode": "detached_null" if receipt["payload"] is None else "embedded_bstr",
            "tree_root": deterministic_root.hex(),
            "reconstructed_root": reconstructed.hex(),
            "signature_valid": True,
            "payload_mode": "detached_null" if receipt["payload"] is None else "embedded_bstr",
            "proof_path_sha256": hashlib.sha256(b"".join(proof["path"])).hexdigest(),
        }
    if not deterministic_path_matches:
        reason = "TAMPERED_INCLUSION_PATH"
    elif not signature_valid:
        reason = "BAD_RECEIPT_SIGNATURE"
    else:
        reason = "TREE_ROOT_MISMATCH"
    return {
        "valid": False,
        "reason": reason,
        "algorithm": alg,
        "tree_root": deterministic_root.hex(),
        "reconstructed_root": reconstructed.hex(),
        "signature_valid": signature_valid,
        "proof_root_matches": proof_root_matches,
        "tree_root_matches": tree_root_matches,
        "deterministic_path_matches": deterministic_path_matches,
    }

def verify_statement(statement_bytes: bytes, issuer_key_pem: bytes, payload_bytes: bytes, expected: dict) -> dict:
    statement = parse_sign1(statement_bytes, "Signed Statement")
    alg = header_alg(statement, "Signed Statement")
    content_type = expect_text(
        map_get(statement["protected_map"], HP_CONTENT_TYPE, "Signed Statement content-type"),
        "Signed Statement content-type",
    )
    issuer, subject = claims_from_statement(statement)
    payload_hash = hashlib.sha256(statement["payload"]).hexdigest()
    if payload_hash != expected["payload_sha256"]:
        raise Reject("statement payload hash disagrees with pinned expected value")
    if statement["payload"] != payload_bytes:
        raise Reject("statement payload differs from payload.bin")
    header_expected = expected["protected_header"]["statement"]
    if alg != header_expected["alg_code"] or content_type != header_expected["content_type"]:
        raise Reject("statement protected header does not match pinned expected value")
    if issuer != header_expected["issuer"] or subject != header_expected["subject"]:
        raise Reject("statement CWT claims do not match pinned expected value")
    sig_valid = openssl_verify(
        issuer_key_pem,
        alg,
        sig_structure(statement["protected"], statement["payload"]),
        statement["signature"],
    )
    return {
        "signature_valid": sig_valid,
        "algorithm": alg,
        "issuer": issuer,
        "subject": subject,
        "content_type": content_type,
        "payload_sha256": payload_hash,
        "statement_bytes_sha256": hashlib.sha256(statement_bytes).hexdigest(),
    }

def load_checksums(path: Path) -> dict[str, str]:
    result = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            raise Reject(f"malformed upstream SHA256SUMS line: {line}")
        digest, rel = parts
        if len(digest) != 64 or any(ch not in "0123456789abcdefABCDEF" for ch in digest):
            raise Reject(f"invalid SHA-256 digest in upstream SHA256SUMS: {digest}")
        if rel in result:
            raise Reject(f"duplicate path in upstream SHA256SUMS: {rel}")
        result[rel] = digest
    return result

def verify_file_pins(root: Path, checksums: dict[str, str], prefix: str, paths: list[str]) -> None:
    for path in paths:
        rel = f"{prefix}/{path}"
        expected = checksums.get(rel)
        if expected is None:
            raise Reject(f"upstream SHA256SUMS does not list {rel}")
        actual = hashlib.sha256((root / rel).read_bytes()).hexdigest()
        if actual != expected:
            raise Reject(f"upstream byte pin mismatch: {rel}")

def run(corpus_manifest_path: Path, upstream_vectors_root: Path, report_path: Path) -> None:
    corpus = strict_json_file(corpus_manifest_path, "local interop manifest")
    require_exact_keys(corpus, {"schema", "claim_ceiling", "source", "vectors"}, "local interop manifest")
    require_exact_keys(corpus["source"], {
        "binary_vectors_vendored", "clean_room_go_payload_binding", "clean_room_go_vdp_shape",
        "commit", "crit_must_be_protected", "current_ietf_examples_observation",
        "detached_payload_valid_vectors_enforced", "exact_inclusion_path_length_enforced",
        "expected_values_embedded", "failure_artifact_upload", "failure_artifact_upload_note",
        "final_rfc_9942_inclusion_shape", "final_rfc_9942_receipt_payload_scope",
        "final_rfc_9942_reference", "final_rfc_9942_status", "materialization",
        "non_minimal_cbor_rejection", "note", "pinned_v1_exactly_one_proof", "private_keys_vendored",
        "receipt_critical_header_enforcement", "receipt_header_metadata_crosscheck",
        "receipt_proof_count_cap", "receipt_vds_label_395_enforced", "repository",
        "semantic_duplicate_key_rejection", "stability", "synthetic_crit_understood_labels",
        "tampered_path_independence_checked", "tree_size_ceiling", "upstream_checksums_path",
        "upstream_manifest_file", "upstream_manifest_path", "upstream_sha256sum_enforced",
        "upstream_sha256sum_file", "vendored_scope"
    }, "local interop manifest source")
    source = corpus["source"]
    expected_source = {
        "repository": "action-state-group/scitt-cose",
        "commit": PINNED_SOURCE_COMMIT,
        "upstream_manifest_path": "test-vectors/manifest.json",
        "upstream_checksums_path": "test-vectors/SHA256SUMS",
        "upstream_manifest_file": "test-vectors/manifest.json",
        "upstream_sha256sum_file": "test-vectors/SHA256SUMS",
        "stability": "append-only",
        "vendored_scope": "none",
        "materialization": "IMMUTABLE_GIT_CHECKOUT_AT_PINNED_COMMIT",
        "private_keys_vendored": False,
        "binary_vectors_vendored": False,
        "expected_values_embedded": True,
        "upstream_sha256sum_enforced": True,
        "pinned_v1_exactly_one_proof": True,
        "crit_must_be_protected": True,
    }
    for key, value in expected_source.items():
        if source[key] != value:
            raise Reject(f"local interop manifest source binding mismatch: {key}")
    if corpus["schema"] != SCHEMA:
        raise Reject("local interop manifest schema mismatch")
    if corpus["source"]["commit"] != PINNED_SOURCE_COMMIT:
        raise Reject("local interop manifest source commit is not the pinned 41811d source")
    assert_surface_bindings()

    upstream_manifest = strict_json_file(upstream_vectors_root / "manifest.json", "upstream vector manifest")
    require_exact_keys(
        upstream_manifest,
        {"version", "stability", "leaf_entry_definition", "tree_construction", "vectors"},
        "upstream vector manifest",
    )
    if upstream_manifest["version"] != "v1" or upstream_manifest["stability"] != "append-only":
        raise Reject("upstream vector manifest is not the pinned append-only v1 corpus")
    if upstream_manifest.get("leaf_entry_definition") != "SHA-256 digest of the complete Signed Statement (COSE_Sign1) bytes, hex-encoded":
        raise Reject("upstream leaf-entry definition differs from the runner's RFC9942-compatible binding")
    if upstream_manifest.get("tree_construction") != "tree_size=8; statement digest at leaf_index=2; filler leaf i = SHA-256('scitt-cose test vectors v1 :: <vector-id> :: filler leaf <i>'); leaves in index order; RFC 9162 SHA-256 tree":
        raise Reject("upstream tree construction differs from the runner's RFC9942-compatible recipe")
    if len(upstream_manifest["vectors"]) != 5 or len(corpus["vectors"]) != 5:
        raise Reject("external v1 qualification requires exactly five pinned vectors")
    upstream_ids = [(v["id"], v["dir"], v["expected_result"], v.get("failure_code")) for v in upstream_manifest["vectors"]]
    local_ids = [(v["id"], v["dir"], v["expected"]["result"], v["expected"].get("failure_code")) for v in corpus["vectors"]]
    if upstream_ids != local_ids:
        raise Reject("local pin manifest vector index disagrees with upstream manifest")

    checksums = load_checksums(upstream_vectors_root / "SHA256SUMS")
    observations = []
    for vector in corpus["vectors"]:
        require_exact_keys(vector, {"id", "dir", "expected", "sha256"}, f"local vector {vector.get('id', '<missing>')}")
        root = upstream_vectors_root / vector["dir"]
        files = ["expected.json", "statement.cose", "receipt.cose", "issuer-key.pub", "log-key.pub", "payload.bin"]
        verify_file_pins(upstream_vectors_root, checksums, vector["dir"], files)

        # The local vector manifest has its own complete byte pins; independently
        # check them against the same exact upstream files instead of leaving them unused.
        local_hashes = vector["sha256"]
        require_exact_keys(local_hashes, set(files), f"local vector {vector['id']} SHA-256 map")
        for relative_path in files:
            expected_hash = local_hashes[relative_path]
            if (
                not isinstance(expected_hash, str)
                or len(expected_hash) != 64
                or any(ch not in "0123456789abcdef" for ch in expected_hash)
            ):
                raise Reject(f"{vector['id']}: malformed local SHA-256 pin for {relative_path}")
            actual_hash = hashlib.sha256((root / relative_path).read_bytes()).hexdigest()
            if actual_hash != expected_hash:
                raise Reject(f"{vector['id']}: local SHA-256 pin mismatch for {relative_path}")

        expected_upstream = strict_json_file(root / "expected.json", f"{vector['id']} expected.json")
        expected_keys = {
            "description", "payload_sha256", "protected_header", "leaf_entry",
            "leaf_index", "tree_size", "inclusion_path", "reconstructed_root",
            "statement_signature_valid", "receipt_valid", "result",
        }
        optional_keys = {"failure_code"} if expected_upstream.get("result") == "INVALID" else set()
        require_exact_keys(expected_upstream, expected_keys, f"{vector['id']} expected.json", optional=optional_keys)
        require_exact_keys(
            expected_upstream["protected_header"],
            {"statement", "receipt"},
            f"{vector['id']} expected protected_header",
        )
        require_exact_keys(
            expected_upstream["protected_header"]["statement"],
            {"alg", "alg_code", "content_type", "cwt_claims_label", "issuer", "subject"},
            f"{vector['id']} expected statement protected_header",
        )
        require_exact_keys(
            expected_upstream["protected_header"]["receipt"],
            {"alg", "alg_code", "vds_label", "vds"},
            f"{vector['id']} expected receipt protected_header",
        )
        if expected_upstream.get("result") == "VALID" and "failure_code" in expected_upstream:
            raise Reject(f"{vector['id']}: VALID vector unexpectedly declares failure_code")
        if expected_upstream.get("result") == "INVALID" and "failure_code" not in expected_upstream:
            raise Reject(f"{vector['id']}: INVALID vector must declare failure_code")
        if expected_upstream != vector["expected"]:
            raise Reject(f"{vector['id']}: local expected data differs from upstream expected.json")

        statement_bytes = (root / "statement.cose").read_bytes()
        receipt_bytes = (root / "receipt.cose").read_bytes()
        payload_bytes = (root / "payload.bin").read_bytes()
        issuer_key = (root / "issuer-key.pub").read_bytes()
        log_key = (root / "log-key.pub").read_bytes()

        statement = verify_statement(statement_bytes, issuer_key, payload_bytes, expected_upstream)
        receipt = verify_receipt(statement_bytes, receipt_bytes, log_key, vector)

        if statement["signature_valid"] != bool(expected_upstream["statement_signature_valid"]):
            raise Reject(f"{vector['id']}: statement signature verdict mismatch")
        if receipt["valid"] != bool(expected_upstream["receipt_valid"]):
            raise Reject(f"{vector['id']}: receipt validity verdict mismatch")

        if expected_upstream["result"] == "VALID":
            if not (statement["signature_valid"] and receipt["valid"]):
                raise Reject(f"{vector['id']}: VALID vector did not fully verify")
            if receipt.get("payload_mode") != "detached_null":
                raise Reject(f"{vector['id']}: valid pinned Receipt is expected to use detached nil payload")
            expected_receipt_header = expected_upstream["protected_header"]["receipt"]
            if expected_receipt_header["vds_label"] != 395 or expected_receipt_header["vds"] != receipt.get("vds"):
                raise Reject(f"{vector['id']}: Receipt VDS header disagrees with pinned expected metadata")
        elif expected_upstream.get("failure_code") == "BAD_STATEMENT_SIGNATURE":
            if statement["signature_valid"] or not receipt["valid"]:
                raise Reject(f"{vector['id']}: bad-statement-signature isolation failed")
        elif expected_upstream.get("failure_code") == "UNSUPPORTED_VDS":
            if receipt.get("reason") != "UNSUPPORTED_VDS":
                raise Reject(f"{vector['id']}: unsupported-VDS boundary was not exercised")
        elif expected_upstream.get("failure_code") == "TAMPERED_INCLUSION_PATH":
            if receipt.get("reason") != "TAMPERED_INCLUSION_PATH":
                raise Reject(f"{vector['id']}: tampered-path boundary was not exercised")
            if receipt.get("deterministic_path_matches") is not False:
                raise Reject(f"{vector['id']}: tampered-path vector did not diverge from independently generated audit path")
        else:
            raise Reject(f"{vector['id']}: unknown upstream failure_code")

        obs = {
            "id": vector["id"],
            "expected_result": expected_upstream["result"],
            "statement_signature_valid": statement["signature_valid"],
            "receipt_valid": receipt["valid"],
            "proof_count": receipt.get("proof_count"),
            "failure_code": expected_upstream.get("failure_code"),
            "receipt_reason": receipt.get("reason"),
            "statement_bytes_sha256": statement["statement_bytes_sha256"],
            "leaf_entry_sha256": hashlib.sha256(statement_bytes).hexdigest(),
        }
        for key in ("tree_root", "reconstructed_root"):
            if key in receipt:
                obs[key] = receipt[key]
        observations.append(obs)

    result = {
        "schema": SCHEMA,
        "qualification": "PASS",
        "claim_ceiling": "EXTERNAL_INTEROP_RESEARCH_ONLY",
        "source": corpus["source"],
        "vector_count": len(corpus["vectors"]),
        "materialization": "IMMUTABLE_GIT_CHECKOUT_AT_PINNED_COMMIT",
        "upstream_sha256sum_pinning": True,
        "qualification_surface_binding": "VERIFIED",
        "upstream_source_manifest_conformant": True,
        "rfc9942_final_vdp_shape_enforced": True,
    "receipt_vds_label_395_enforced": True,
    "tampered_path_independence_checked": True,
        "receipt_header_metadata_crosscheck": True,
        "semantic_duplicate_key_rejection": True,
        "non_minimal_cbor_rejection": True,
        "detached_payload_valid_vectors_enforced": True,
        "local_expected_crosscheck": True,
        "independent_tree_reconstruction": True,
        "independent_audit_path_reconstruction": True,
        "statement_issuer_signature_verification": True,
        "receipt_log_signature_verification": True,
        "synthetic_019_exact_object_binding": "NOT_EVALUATED",
        "real_transparency_service": False,
        "production_authority": False,
        "vectors": observations,
    }
    report_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print("SYM-CIVIC-019 EXTERNAL SCITT/COSE INTEROP=PASS")
    print("claim_ceiling=EXTERNAL_INTEROP_RESEARCH_ONLY")
    print("vector_count=" + str(len(corpus["vectors"])))
    print("materialization=IMMUTABLE_GIT_CHECKOUT_AT_PINNED_COMMIT")
    print("upstream_sha256sum_pinning=true")
    print("local_expected_crosscheck=true")
    print("independent_tree_reconstruction=true")
    print("independent_audit_path_reconstruction=true")
    print("statement_issuer_signature_verification=true")
    print("receipt_log_signature_verification=true")
    print("synthetic_019_exact_object_binding=NOT_EVALUATED")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--corpus-manifest", required=True)
    ap.add_argument("--upstream-vectors-root", required=True)
    ap.add_argument("--report", required=True)
    args = ap.parse_args()
    try:
        run(Path(args.corpus_manifest), Path(args.upstream_vectors_root), Path(args.report))
    except Reject as exc:
        raise SystemExit("qualification FAIL: " + str(exc))
    except Unresolved as exc:
        raise SystemExit("qualification UNRESOLVED: " + str(exc))
