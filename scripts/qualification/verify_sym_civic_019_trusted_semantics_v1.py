#!/usr/bin/env python3
import argparse
import base64
import hashlib
import json
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path


class Reject(Exception):
    pass


# Local conservative bound for this synthetic research profile; not an RFC 9162 maximum.
TREE_SIZE_MAX_INCLUSIVE = 2**62


@dataclass
class Node:
    kind: str
    value: object
    raw: bytes
    children: tuple


class Reader:
    def __init__(self, raw: bytes):
        self.raw = raw
        self.i = 0

    def _read(self, n: int) -> bytes:
        j = self.i + n
        if j > len(self.raw):
            raise Reject("truncated CBOR")
        out = self.raw[self.i:j]
        self.i = j
        return out

    def _head(self):
        b = self._read(1)[0]
        mt = b >> 5
        ai = b & 31
        if ai < 24:
            return mt, ai
        if ai == 24:
            v = self._read(1)[0]
            if v < 24:
                raise Reject("non-minimal CBOR")
            return mt, v
        if ai == 25:
            v = int.from_bytes(self._read(2), "big")
            if v < 256:
                raise Reject("non-minimal CBOR")
            return mt, v
        if ai == 26:
            v = int.from_bytes(self._read(4), "big")
            if v < 65536:
                raise Reject("non-minimal CBOR")
            return mt, v
        if ai == 27:
            v = int.from_bytes(self._read(8), "big")
            if v < 2**32:
                raise Reject("non-minimal CBOR")
            return mt, v
        raise Reject("indefinite/reserved CBOR additional information")

    def parse(self) -> Node:
        start = self.i
        mt, n = self._head()
        if mt == 0:
            return Node("uint", n, self.raw[start:self.i], ())
        if mt == 1:
            return Node("nint", -1 - n, self.raw[start:self.i], ())
        if mt == 2:
            value = self._read(n)
            return Node("bstr", value, self.raw[start:self.i], ())
        if mt == 3:
            value = self._read(n)
            try:
                text = value.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise Reject("invalid UTF-8 text string") from exc
            return Node("tstr", text, self.raw[start:self.i], ())
        if mt in (4, 5):
            items = []
            if mt == 4:
                for _ in range(n):
                    items.append(self.parse())
            else:
                for _ in range(n * 2):
                    items.append(self.parse())
            value = tuple(items) if mt == 4 else ("map", tuple(items))
            return Node("array" if mt == 4 else "map", value, self.raw[start:self.i], tuple(items))
        if mt == 6:
            child = self.parse()
            return Node("tag", n, self.raw[start:self.i], (child,))
        if mt == 7:
            ai = self.raw[start] & 31
            if ai == 20:
                return Node("bool", False, self.raw[start:self.i], ())
            if ai == 21:
                return Node("bool", True, self.raw[start:self.i], ())
            if ai == 22:
                return Node("null", None, self.raw[start:self.i], ())
            raise Reject("unsupported simple/float CBOR")
        raise Reject("unsupported CBOR major type")


def parse_exact(raw: bytes) -> Node:
    reader = Reader(raw)
    node = reader.parse()
    if reader.i != len(raw):
        raise Reject("trailing bytes after CBOR item")
    return node


def pairs(node: Node):
    if node.kind != "map":
        raise Reject("expected CBOR map")
    if len(node.children) % 2:
        raise Reject("odd CBOR map arity")
    out = []
    seen = set()
    for i in range(0, len(node.children), 2):
        k = node.children[i]
        v = node.children[i + 1]
        if k.kind not in {"uint", "nint", "tstr"}:
            raise Reject("unsupported map-key type")
        key = (k.kind, k.value)
        if key in seen:
            raise Reject("duplicate CBOR map key")
        seen.add(key)
        out.append((k, v))
    return out


def get(node: Node, key, required=True):
    for k, v in pairs(node):
        if k.value == key:
            return v
    if required:
        raise Reject(f"missing map key {key}")
    return None


def arr(node: Node, what: str):
    if node.kind != "array":
        raise Reject(f"{what} must be array")
    return node


def mp(node: Node, what: str):
    if node.kind != "map":
        raise Reject(f"{what} must be map")
    pairs(node)
    return node


def integer(node: Node, what: str):
    if node.kind not in {"uint", "nint"}:
        raise Reject(f"{what} must be integer")
    return node.value


def uint(node: Node, what: str):
    if node.kind != "uint":
        raise Reject(f"{what} must be uint")
    return node.value


def bstr(node: Node, what: str):
    if node.kind != "bstr":
        raise Reject(f"{what} must be bstr")
    return node.value


def tstr(node: Node, what: str):
    if node.kind != "tstr":
        raise Reject(f"{what} must be tstr")
    return node.value


def cbor_head(mt: int, n: int) -> bytes:
    if n < 24:
        return bytes([(mt << 5) | n])
    if n < 256:
        return bytes([(mt << 5) | 24, n])
    if n < 65536:
        return bytes([(mt << 5) | 25]) + n.to_bytes(2, "big")
    if n < 2**32:
        return bytes([(mt << 5) | 26]) + n.to_bytes(4, "big")
    return bytes([(mt << 5) | 27]) + n.to_bytes(8, "big")


def cbor_uint(n: int) -> bytes:
    if n < 0:
        return cbor_head(1, -1 - n)
    return cbor_head(0, n)


def cbor_bstr(value: bytes) -> bytes:
    return cbor_head(2, len(value)) + value


def cbor_tstr(value: str) -> bytes:
    raw = value.encode("utf-8")
    return cbor_head(3, len(raw)) + raw


def cbor_array(values) -> bytes:
    return cbor_head(4, len(values)) + b"".join(values)


def cbor_map(pairs_list) -> bytes:
    return cbor_head(5, len(pairs_list)) + b"".join(
        cbor_uint(k) + raw for k, raw in pairs_list
    )


def cbor_tag(tag: int, value: bytes) -> bytes:
    return cbor_head(6, tag) + value


def sha256(value: bytes) -> bytes:
    return hashlib.sha256(value).digest()


def git_blob_sha(value: bytes) -> str:
    header = f"blob {len(value)}".encode() + b"\0"
    return hashlib.sha1(header + value).hexdigest()


def strict_json(path: Path):
    def reject_duplicate_members(items):
        result = {}
        for key, value in items:
            if key in result:
                raise Reject("duplicate JSON member: " + key)
            result[key] = value
        return result

    try:
        return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=reject_duplicate_members)
    except json.JSONDecodeError as exc:
        raise Reject("invalid JSON: " + str(exc)) from exc


def b64url(value: bytes) -> str:
    return base64.urlsafe_b64encode(value).rstrip(b"=").decode("ascii")


def sig_structure(protected_raw: bytes, payload: bytes) -> bytes:
    return cbor_array(
        [
            cbor_tstr("Signature1"),
            cbor_bstr(protected_raw),
            cbor_bstr(b""),
            cbor_bstr(payload),
        ]
    )


def openssl_ed25519_verify(public_key_pem: str, message: bytes, signature: bytes) -> bool:
    with tempfile.TemporaryDirectory(prefix="mycelix019-trusted-") as td:
        root = Path(td)
        key_path = root / "key.pem"
        msg_path = root / "message.bin"
        sig_path = root / "signature.bin"
        key_path.write_text(public_key_pem, encoding="utf-8")
        msg_path.write_bytes(message)
        sig_path.write_bytes(signature)
        result = subprocess.run(
            [
                "openssl",
                "pkeyutl",
                "-verify",
                "-pubin",
                "-inkey",
                str(key_path),
                "-rawin",
                "-in",
                str(msg_path),
                "-sigfile",
                str(sig_path),
            ],
            text=True,
            capture_output=True,
        )
        if result.returncode == 0:
            return True
        if "Signature Verification Failure" in result.stdout + result.stderr:
            return False
        raise Reject("OpenSSL verification backend unresolved")


def inclusion_root(leaf: bytes, leaf_index: int, tree_size: int, path):
    if tree_size < 1:
        raise Reject("tree_size must be positive")
    if not 0 <= leaf_index < tree_size:
        raise Reject("leaf_index out of range")
    if any(not isinstance(node, bytes) or len(node) != 32 for node in path):
        raise Reject("path node must be 32 bytes")
    cur = leaf
    fn = leaf_index
    sn = tree_size - 1
    pos = 0
    while sn:
        need = (fn & 1) or fn < sn
        if need:
            if pos >= len(path):
                raise Reject("inclusion path exhausted")
            sibling = path[pos]
            cur = (
                sha256(b"\x01" + sibling + cur)
                if fn & 1
                else sha256(b"\x01" + cur + sibling)
            )
            pos += 1
        fn //= 2
        sn //= 2
    if pos != len(path):
        raise Reject("unused inclusion path nodes")
    return cur


def merkle_tree_hash(entries):
    if not entries:
        return sha256(b"")
    leaves = [sha256(b"\x00" + entry) for entry in entries]

    def mth(nodes):
        if len(nodes) == 1:
            return nodes[0]
        k = 1 << ((len(nodes) - 1).bit_length() - 1)
        return sha256(b"\x01" + mth(nodes[:k]) + mth(nodes[k:]))

    return mth(leaves)


def reference_inclusion_path(entries, leaf_index):
    """Build proof paths recursively, independently of the iterative verifier."""
    if not entries or not 0 <= leaf_index < len(entries):
        raise Reject("reference proof index out of range")
    if len(entries) == 1:
        return []
    split = 1 << ((len(entries) - 1).bit_length() - 1)
    if leaf_index < split:
        return reference_inclusion_path(entries[:split], leaf_index) + [
            merkle_tree_hash(entries[split:])
        ]
    return reference_inclusion_path(entries[split:], leaf_index - split) + [
        merkle_tree_hash(entries[:split])
    ]


def verify_positive_inclusion_matrix(tree_sizes, expected_case_count):
    cases = []
    for tree_size in tree_sizes:
        entries = [
            f"SYM-CIVIC-019-INCLUSION-MATRIX-V1/tree={tree_size}/leaf={i}".encode("ascii")
            for i in range(tree_size)
        ]
        expected_root = merkle_tree_hash(entries)
        for leaf_index, entry in enumerate(entries):
            path = reference_inclusion_path(entries, leaf_index)
            try:
                observed_root = inclusion_root(
                    sha256(b"\x00" + entry), leaf_index, tree_size, path
                )
                passed = observed_root == expected_root
                cases.append({
                    "tree_size": tree_size,
                    "leaf_index": leaf_index,
                    "path_length": len(path),
                    "expected_root_sha256": expected_root.hex(),
                    "observed_root_sha256": observed_root.hex(),
                    "verdict": "PASS" if passed else "FAIL",
                })
            except Reject as exc:
                cases.append({
                    "tree_size": tree_size,
                    "leaf_index": leaf_index,
                    "path_length": len(path),
                    "expected_root_sha256": expected_root.hex(),
                    "verdict": "FAIL",
                    "error": str(exc),
                })
    if len(cases) != expected_case_count:
        raise Reject(
            f"positive inclusion matrix case count mismatch: {len(cases)} != {expected_case_count}"
        )
    return cases


def parse_sign1(raw: bytes, what: str):
    top = parse_exact(raw)
    if top.kind != "tag" or top.value != 18 or len(top.children) != 1:
        raise Reject(f"{what} must be tagged COSE_Sign1")
    body = arr(top.children[0], f"{what} body")
    if len(body.children) != 4:
        raise Reject(f"{what} body arity")
    protected = bstr(body.children[0], f"{what} protected")
    unprotected = mp(body.children[1], f"{what} unprotected")
    payload = bstr(body.children[2], f"{what} payload")
    signature = bstr(body.children[3], f"{what} signature")
    protected_map = mp(parse_exact(protected), f"{what} protected map")
    protected_pairs = pairs(protected_map)
    unprotected_pairs = pairs(unprotected)
    if {k.value for k, _ in protected_pairs} & {k.value for k, _ in unprotected_pairs}:
        raise Reject(f"{what} protected/unprotected overlap")
    if get(protected_map, 2, False) is not None or get(unprotected, 2, False) is not None:
        raise Reject(f"{what} critical header outside fixed synthetic profile")
    return {
        "raw": raw,
        "body": body,
        "protected": protected,
        "protected_map": protected_map,
        "unprotected": unprotected,
        "payload": payload,
        "signature": signature,
    }


def statement_claims(sign1):
    claims = mp(get(sign1["protected_map"], 15), "CWT claims")
    return tstr(get(claims, 1), "issuer"), tstr(get(claims, 2), "subject")


def parse_inclusion_receipt(raw: bytes, entry: bytes, common: dict, identity: dict, receipt_label: str):
    s = parse_sign1(raw, receipt_label)
    ph = s["protected_map"]
    uh = s["unprotected"]

    if integer(get(ph, 1), "Receipt alg") != -8:
        raise Reject("Receipt alg mismatch")
    vds = uint(get(ph, 395), "Receipt VDS")
    if vds != common["vds_id"]:
        raise Reject("Receipt VDS mismatch")
    if {k.value for k, _ in pairs(ph)} != {1, 4, 15, 395}:
        raise Reject("Receipt protected-header schema mismatch")
    if {k.value for k, _ in pairs(uh)} != {396}:
        raise Reject("Receipt unprotected-header schema mismatch")
    kid = bstr(get(ph, 4), "Receipt kid")
    iss, sub = statement_claims(s)

    vdp = mp(get(uh, 396), "Receipt VDP")
    if {k.value for k, _ in pairs(vdp)} != {-1}:
        raise Reject("Receipt VDP schema mismatch")
    proofs = arr(get(vdp, -1), "Receipt inclusion proofs")
    if len(proofs.children) != 1:
        raise Reject("trusted profile requires exactly one inclusion proof")
    proof_bytes = bstr(proofs.children[0], "Receipt proof")
    proof = arr(parse_exact(proof_bytes), "Receipt proof content")
    if len(proof.children) != 3:
        raise Reject("Receipt proof arity")
    tree_size = uint(proof.children[0], "tree_size")
    leaf_index = uint(proof.children[1], "leaf_index")
    path_node = arr(proof.children[2], "inclusion path")
    path = [bstr(node, "path node") for node in path_node.children]

    root = s["payload"]
    if len(root) != 32:
        raise Reject("Receipt payload must be SHA-256 root")
    if kid.hex() != identity["kid_hex"]:
        raise Reject("Receipt KID mismatch")
    if iss != identity["issuer"]:
        raise Reject("Receipt issuer mismatch")
    if sub != common["subject"]:
        raise Reject("Receipt subject mismatch")
    if tree_size > TREE_SIZE_MAX_INCLUSIVE:
        raise Reject("tree_size exceeds local synthetic profile limit")
    if tree_size < 1:
        raise Reject("tree_size must be positive")
    if not 0 <= leaf_index < tree_size:
        raise Reject("leaf_index out of range")
    if tree_size != common["tree_size"] or leaf_index != common["leaf_index"]:
        raise Reject("proof metadata mismatch")

    leaf = sha256(b"\x00" + entry)
    if inclusion_root(leaf, leaf_index, tree_size, path) != root:
        raise Reject("inclusion root mismatch")
    if root.hex() != common["root_hash"]:
        raise Reject("root does not match trusted expectation")

    if not openssl_ed25519_verify(
        identity["public_key_pem"],
        sig_structure(s["protected"], root),
        s["signature"],
    ):
        raise Reject("Receipt signature verification failed")

    return {
        "receipt_sha256": hashlib.sha256(raw).hexdigest(),
        "protected_sha256": hashlib.sha256(s["protected"]).hexdigest(),
        "kid_hex": kid.hex(),
        "issuer": iss,
        "subject": sub,
        "tree_size": tree_size,
        "leaf_index": leaf_index,
        "root_hash": root.hex(),
        "proof_sha256": hashlib.sha256(proof_bytes).hexdigest(),
    }, s, proof_bytes

def rebuild_map(node: Node, replacements: dict) -> bytes:
    out = []
    for k, v in pairs(node):
        value = replacements.get(k.value, v.raw)
        out.append((k.value, value))
    return cbor_map(out)


def rebuild_sign1(parsed, protected_raw=None, unprotected_raw=None, payload=None, signature=None):
    prot = parsed["protected"] if protected_raw is None else protected_raw
    uh = parsed["unprotected"].raw if unprotected_raw is None else unprotected_raw
    pl = cbor_bstr(parsed["payload"]) if payload is None else cbor_bstr(payload)
    sig = cbor_bstr(parsed["signature"]) if signature is None else cbor_bstr(signature)
    body = cbor_array([cbor_bstr(prot), uh, pl, sig])
    return cbor_tag(18, body)


def expect_reject(fn, label: str, expected_reason: str):
    try:
        fn()
    except Reject as exc:
        observed_reason = str(exc)
        if observed_reason != expected_reason:
            return {
                "name": label,
                "verdict": "UNRESOLVED",
                "error": "expected rejection reason " + repr(expected_reason)
                         + ", observed " + repr(observed_reason),
            }
        return {"name": label, "verdict": "REJECT", "reason": observed_reason}
    except Exception as exc:
        return {
            "name": label,
            "verdict": "UNRESOLVED",
            "error": type(exc).__name__ + ":" + str(exc),
        }
    return {"name": label, "verdict": "PASS"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--candidate-root", required=True)
    ap.add_argument("--trusted-manifest", required=True)
    ap.add_argument("--report", required=True)
    args = ap.parse_args()

    trusted_manifest_path = Path(args.trusted_manifest)
    trusted = strict_json(trusted_manifest_path)
    if set(trusted) != {
        "schema", "qualification_workflow", "trusted_admission_workflow",
        "pr_number", "parent_subject", "candidate_subject",
        "expected_changed_files", "expected_blob_sha", "required_successful_jobs",
        "semantic_expectations", "trusted_semantic_oracle",
    }:
        raise Reject("trusted admission manifest schema drift")
    if trusted["schema"] != "MYCELIX-SYM-CIVIC-019-TRUSTED-ADMISSION-V1":
        raise Reject("trusted admission manifest schema mismatch")

    oracle_profile = trusted["trusted_semantic_oracle"]
    if oracle_profile != {
        "path": "scripts/qualification/verify_sym_civic_019_trusted_semantics_v1.py",
        "claim_ceiling": "TRUSTED_SEMANTIC_ORACLE_RESEARCH_ONLY",
        "candidate_code_executed": False,
        "negative_control_count": 11,
        "tree_size_max_inclusive": TREE_SIZE_MAX_INCLUSIVE,
        "positive_inclusion_matrix": {
            "tree_sizes": [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
            "expected_case_count": 136,
        },
    }:
        raise Reject("trusted oracle profile mismatch")

    common = trusted["semantic_expectations"]
    if set(common) != {
        "profile", "vds_id", "vdp_id", "entry_sha256", "statement_sha256",
        "receipt_a_sha256", "receipt_b_sha256", "root_hash", "merkle_root",
        "tree_size", "leaf_index", "subject", "statement_issuer", "identities",
    }:
        raise Reject("trusted semantic expectation schema drift")
    if common["profile"] != "RFC9162_SHA256_SYNTHETIC_V1":
        raise Reject("trusted semantic profile mismatch")
    if common["vds_id"] != 1 or common["vdp_id"] != -1:
        raise Reject("trusted VDS/VDP mismatch")
    if common["tree_size"] != 1 or common["leaf_index"] != 0:
        raise Reject("trusted proof metadata mismatch")
    if common["subject"] != "stmt-019-001":
        raise Reject("trusted subject mismatch")
    identities = common["identities"]
    if set(identities) != {"A", "B"}:
        raise Reject("trusted receipt identity schema mismatch")

    root = Path(args.candidate_root)
    corpus_path = root / "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_v1.json"
    manifest_path = root / "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_manifest_v1.json"
    if not corpus_path.is_file() or not manifest_path.is_file():
        raise Reject("candidate qualification files missing")

    corpus = strict_json(corpus_path)
    if corpus.get("schema") != "SYM-CIVIC-019-RECEIPT-PROOF-BINDING-CORPUS-V1":
        raise Reject("candidate corpus schema mismatch")
    if set(corpus) != {
        "profile", "receipt_a_hex", "receipt_b_hex", "registered_statement_hex",
        "schema", "transparent_statement_hex", "tree", "ts_key_registry",
        "vdp_registry", "vds_registry", "vds_entry_bytes_hex", "merkle_vectors",
    }:
        raise Reject("candidate corpus has unexpected top-level fields")

    entry = bytes.fromhex(corpus["vds_entry_bytes_hex"])
    statement = bytes.fromhex(corpus["registered_statement_hex"])
    receipt_a = bytes.fromhex(corpus["receipt_a_hex"])
    receipt_b = bytes.fromhex(corpus["receipt_b_hex"])
    transparent = bytes.fromhex(corpus["transparent_statement_hex"])

    if hashlib.sha256(entry).hexdigest() != common["entry_sha256"]:
        raise Reject("candidate entry hash differs from trusted expectation")
    if hashlib.sha256(statement).hexdigest() != common["statement_sha256"]:
        raise Reject("candidate Signed Statement hash differs from trusted expectation")
    if entry != statement:
        raise Reject("candidate VDS entry is not exact Signed Statement bytes")
    if hashlib.sha256(receipt_a).hexdigest() != common["receipt_a_sha256"]:
        raise Reject("candidate Receipt A hash differs from trusted expectation")
    if hashlib.sha256(receipt_b).hexdigest() != common["receipt_b_sha256"]:
        raise Reject("candidate Receipt B hash differs from trusted expectation")

    key_registry = corpus["ts_key_registry"]
    if not isinstance(key_registry, list) or len(key_registry) != 2:
        raise Reject("candidate TS key registry shape")
    parsed_identities = {}
    for label, expected_identity in identities.items():
        matches = [
            x for x in key_registry
            if x.get("raw_kid_hex") == expected_identity["kid_hex"]
        ]
        if len(matches) != 1:
            raise Reject(f"candidate key registry does not resolve trusted {label} KID uniquely")
        item = matches[0]
        if item.get("issuer") != expected_identity["issuer"]:
            raise Reject(f"candidate {label} issuer differs from trusted identity")
        if item.get("public_key_pem") != expected_identity["public_key_pem"]:
            raise Reject(f"candidate {label} public key differs from trusted identity")
        parsed_identities[label] = expected_identity

    statement_s = parse_sign1(statement, "Signed Statement")
    statement_iss, statement_sub = statement_claims(statement_s)
    if statement_iss != common["statement_issuer"]:
        raise Reject(
            "trusted Signed Statement issuer mismatch: expected "
            + repr(common["statement_issuer"]) + ", observed " + repr(statement_iss)
        )
    if statement_sub != common["subject"]:
        raise Reject(
            "trusted Signed Statement subject mismatch: expected "
            + repr(common["subject"]) + ", observed " + repr(statement_sub)
        )
    if {k.value for k, _ in pairs(statement_s["protected_map"])} != {1, 4, 15, 1000, 1001}:
        raise Reject("Signed Statement protected-header schema mismatch")
    if pairs(statement_s["unprotected"]):
        raise Reject("Signed Statement unprotected headers must be empty")

    transparent_s = parse_sign1(transparent, "Transparent Statement")
    if transparent_s["protected"] != statement_s["protected"]:
        raise Reject("Transparent Statement protected bytes differ")
    if transparent_s["payload"] != statement_s["payload"]:
        raise Reject("Transparent Statement payload differs")
    if transparent_s["signature"] != statement_s["signature"]:
        raise Reject("Transparent Statement signature differs")
    if {k.value for k, _ in pairs(transparent_s["unprotected"])} != {394}:
        raise Reject("Transparent Statement unprotected-header schema mismatch")
    receipts = arr(get(transparent_s["unprotected"], 394), "receipt sequence")
    sequence = [bstr(x, "receipt sequence entry") for x in receipts.children]
    if sequence != [receipt_a, receipt_b]:
        raise Reject("Receipt sequence does not equal trusted A,B ordering")

    info_a, parsed_a, proof_a = parse_inclusion_receipt(
        receipt_a, entry, common, parsed_identities["A"], "Receipt A"
    )
    info_b, parsed_b, proof_b = parse_inclusion_receipt(
        receipt_b, entry, common, parsed_identities["B"], "Receipt B"
    )

    if info_a["root_hash"] != info_b["root_hash"] or info_a["root_hash"] != common["root_hash"]:
        raise Reject("Receipt roots disagree with trusted root")

    vectors = corpus["merkle_vectors"]
    entries = [bytes.fromhex(x) for x in vectors["entries_hex"]]
    if merkle_tree_hash(entries).hex() != vectors["root_hash_hex"]:
        raise Reject("candidate Merkle fixture does not independently reconstruct")
    if vectors["root_hash_hex"] != common["merkle_root"]:
        raise Reject("candidate Merkle fixture differs from trusted Merkle root")

    negatives = []
    negatives.append(
        expect_reject(
            lambda: parse_inclusion_receipt(
                receipt_a, entry + b"\x00", common, parsed_identities["A"], "wrong-entry"
            ),
            "WRONG_ENTRY_BYTES", "inclusion root mismatch",
        )
    )

    mutated_sig = bytearray(parsed_a["signature"])
    mutated_sig[-1] ^= 1
    negatives.append(
        expect_reject(
            lambda: parse_inclusion_receipt(
                rebuild_sign1(parsed_a, signature=bytes(mutated_sig)),
                entry, common, parsed_identities["A"], "signature mutation"
            ),
            "RECEIPT_SIGNATURE_MUTATION", "Receipt signature verification failed",
        )
    )

    mutated_proof = cbor_array(
        [cbor_uint(common["tree_size"]), cbor_uint(common["tree_size"]), cbor_array([])]
    )
    needle = cbor_bstr(proof_a)
    if receipt_a.count(needle) != 1:
        raise Reject("proof mutation fixture is not uniquely locatable")
    mutated_receipt = receipt_a.replace(needle, cbor_bstr(mutated_proof), 1)
    negatives.append(
        expect_reject(
            lambda: parse_inclusion_receipt(
                mutated_receipt, entry, common, parsed_identities["A"], "proof mutation"
            ),
            "LEAF_INDEX_EQUALS_TREE_SIZE", "leaf_index out of range",
        )
    )

    path_mutated_proof = cbor_array([
        cbor_uint(common["tree_size"]),
        cbor_uint(common["leaf_index"]),
        cbor_array([cbor_bstr(bytes([0x5A]) * 32)]),
    ])
    path_mutated_receipt = receipt_a.replace(needle, cbor_bstr(path_mutated_proof), 1)
    negatives.append(
        expect_reject(
            lambda: parse_inclusion_receipt(
                path_mutated_receipt, entry, common, parsed_identities["A"], "path node mutation"
            ),
            "PROOF_PATH_EXTRA_NODE", "unused inclusion path nodes",
        )
    )

    mutated_payload = bytearray(parsed_a["payload"])
    mutated_payload[0] ^= 1
    negatives.append(
        expect_reject(
            lambda: parse_inclusion_receipt(
                rebuild_sign1(parsed_a, payload=bytes(mutated_payload)),
                entry, common, parsed_identities["A"], "embedded root mutation"
            ),
            "EMBEDDED_ROOT_PAYLOAD_MUTATION", "inclusion root mismatch",
        )
    )

    mutated_protected = rebuild_map(parsed_a["protected_map"], {395: cbor_uint(2)})
    negatives.append(
        expect_reject(
            lambda: parse_inclusion_receipt(
                rebuild_sign1(parsed_a, protected_raw=mutated_protected),
                entry, common, parsed_identities["A"], "VDS selector mutation"
            ),
            "VDS_SELECTOR_MUTATION", "Receipt VDS mismatch",
        )
    )

    malformed_vdp = rebuild_map(parsed_a["unprotected"], {396: cbor_array([])})
    negatives.append(
        expect_reject(
            lambda: parse_inclusion_receipt(
                rebuild_sign1(parsed_a, unprotected_raw=malformed_vdp),
                entry, common, parsed_identities["A"], "malformed VDP"
            ),
            "MALFORMED_VDP_SHAPE", "Receipt VDP schema mismatch",
        )
    )

    protected_pairs = [(k.value, v.raw) for k, v in pairs(parsed_a["protected_map"])]
    duplicate_protected = cbor_map(protected_pairs + [(395, cbor_uint(1))])
    negatives.append(
        expect_reject(
            lambda: parse_inclusion_receipt(
                rebuild_sign1(parsed_a, protected_raw=duplicate_protected),
                entry, common, parsed_identities["A"], "duplicate protected label"
            ),
            "DUPLICATE_COSE_HEADER_LABEL", "duplicate CBOR map key",
        )
    )

    negatives.append(
        expect_reject(
            lambda: parse_exact(b"\xa1\x01\x18\x01"),
            "NON_MINIMAL_CBOR", "non-minimal CBOR",
        )
    )

    negatives.append(
        expect_reject(
            lambda: inclusion_root(sha256(b"synthetic leaf"), 0, 2, [bytes([0x5A]) * 31]),
            "PATH_NODE_WRONG_LENGTH", "path node must be 32 bytes",
        )
    )

    over_limit_proof = cbor_array([
        cbor_uint(TREE_SIZE_MAX_INCLUSIVE + 1),
        cbor_uint(0),
        cbor_array([]),
    ])
    over_limit_receipt = receipt_a.replace(needle, cbor_bstr(over_limit_proof), 1)
    negatives.append(
        expect_reject(
            lambda: parse_inclusion_receipt(
                over_limit_receipt, entry, common, parsed_identities["A"], "tree size over limit"
            ),
            "TREE_SIZE_ABOVE_LIMIT", "tree_size exceeds local synthetic profile limit",
        )
    )

    failures = [x for x in negatives if x["verdict"] != "REJECT"]
    matrix_profile = oracle_profile["positive_inclusion_matrix"]
    inclusion_cases = verify_positive_inclusion_matrix(
        matrix_profile["tree_sizes"], matrix_profile["expected_case_count"]
    )
    inclusion_failures = [x for x in inclusion_cases if x["verdict"] != "PASS"]
    result = {
        "schema": "MYCELIX-SYM-CIVIC-019-TRUSTED-SEMANTIC-ORACLE-V5",
        "claim_ceiling": "TRUSTED_SEMANTIC_ORACLE_RESEARCH_ONLY",
        "candidate_input_only": True,
        "candidate_code_executed": False,
        "profile": common["profile"],
        "vds_id": common["vds_id"],
        "vdp_id": common["vdp_id"],
        "receipt_a": info_a,
        "receipt_b": info_b,
        "statement_sha256": common["statement_sha256"],
        "statement_issuer": statement_iss,
        "statement_subject": statement_sub,
        "entry_sha256": common["entry_sha256"],
        "independent_merkle_fixture": "PASS",
        "positive_inclusion_matrix": {
            "tree_sizes_tested": matrix_profile["tree_sizes"],
            "case_count": len(inclusion_cases),
            "all_pass": not inclusion_failures,
            "cases": inclusion_cases,
        },
        "negative_controls": negatives,
        "negative_control_count": len(negatives),
        "tree_size_max_inclusive": TREE_SIZE_MAX_INCLUSIVE,
        "tree_size_limit_scope": "LOCAL_SYNTHETIC_PROFILE_NOT_RFC_REQUIREMENT",
        "negative_controls_all_rejected": not failures,
        "candidate_qualification_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "candidate_corpus_git_blob_sha": git_blob_sha(corpus_path.read_bytes()),
        "issuer_signature_verification": "NOT_EVALUATED",
        "result": "PASS" if not failures and not inclusion_failures else "FAIL",
    }
    Path(args.report).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if failures or inclusion_failures:
        raise SystemExit(
            "trusted semantic oracle failure: "
            + repr({"negative_control_failures": failures, "positive_inclusion_failures": inclusion_failures})
        )
    print("SYM-CIVIC-019 TRUSTED SEMANTIC ORACLE=PASS")
    print("candidate_code_executed=false")
    print("independent_merkle_fixture=PASS")
    print(f"positive_inclusion_matrix_cases={len(inclusion_cases)}")
    print("positive_inclusion_matrix_all_pass=true")
    print("negative_controls_all_rejected=true")
    print("issuer_signature_verification=NOT_EVALUATED")
    print("claim_ceiling=TRUSTED_SEMANTIC_ORACLE_RESEARCH_ONLY")


if __name__ == "__main__":
    main()
