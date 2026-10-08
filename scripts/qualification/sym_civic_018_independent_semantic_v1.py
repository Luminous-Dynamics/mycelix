#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path

TRUSTED_MANIFEST = Path(__file__).with_name("sym_civic_018_independent_semantic_v1.json")
CANDIDATE_MANIFEST_REL = "mycelix-workspace/docs/civic-resilience/sym_civic_018_cbor_encoding_v1.json"
CANDIDATE_WORKFLOW_REL = ".github/workflows/sym-civic-018-cbor-encoding.yml"
CANDIDATE_DOC_REL = "mycelix-workspace/docs/civic-resilience/SYM_CIVIC_018_CBOR_ENCODING_V1.md"
CANDIDATE_QUALIFIER_REL = "scripts/qualification/sym_civic_018_cbor_encoding_v1.py"

SUFFICIENT = "CBOR_ENCODING_SUFFICIENT"
MESSAGE_REJECT = "CBOR_MESSAGE_REJECT"
ENCODING_REJECT = "CBOR_ENCODING_REJECT"
PARSE_ERROR = "CBOR_PARSE_ERROR"
UNRESOLVED = "CBOR_ENCODING_UNRESOLVED"

DEPTH_WITHIN = "DEPTH_PROBE_WITHIN_LIMIT"
DEPTH_EXCEEDED = "DEPTH_PROBE_EXCEEDED"
DEPTH_UNRESOLVED = "DEPTH_PROBE_UNRESOLVED"
RESOURCE_WITHIN = "RESOURCE_PROBE_WITHIN_LIMIT"
RESOURCE_EXCEEDED = "RESOURCE_PROBE_EXCEEDED"
RESOURCE_UNRESOLVED = "RESOURCE_PROBE_UNRESOLVED"

MAX_DEPTH = 32
MAX_STRING_BYTES = 1024
MAX_CONTAINER_ITEMS = 64


class ParseFault(Exception):
    pass


class DepthFault(ParseFault):
    pass


class ResourceFault(ParseFault):
    pass


class ShapeFault(Exception):
    pass


@dataclass(frozen=True)
class Node:
    kind: str
    value: object
    raw: bytes
    preferred: bool
    indefinite: bool
    depth: int


def reject_duplicate_json_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON object key")
        result[key] = value
    return result


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=reject_duplicate_json_keys)


def git(root: Path, *args: str):
    return subprocess.check_output(["git", *args], cwd=root, text=True).strip()


def read_arg(data: bytes, pos: int, ai: int):
    if ai < 24:
        return ai, pos, True
    if ai == 31:
        return None, pos, False
    width = {24: 1, 25: 2, 26: 4, 27: 8}.get(ai)
    if width is None or pos + width > len(data):
        raise ParseFault("bad argument")
    value = int.from_bytes(data[pos:pos + width], "big")
    minimum = {1: 24, 2: 256, 4: 65536, 8: 4294967296}[width]
    return value, pos + width, value >= minimum


def parse_item(data: bytes, pos: int = 0, depth: int = 0):
    if depth > MAX_DEPTH:
        raise DepthFault("maximum depth exceeded")
    if pos >= len(data):
        raise ParseFault("missing item")

    start = pos
    initial = data[pos]
    pos += 1
    major = initial >> 5
    ai = initial & 31

    if major in (0, 1):
        value, pos, preferred = read_arg(data, pos, ai)
        if value is None:
            raise ParseFault("indefinite integer")
        if major == 1:
            value = -1 - value
        return Node("int", value, data[start:pos], preferred, False, depth), pos

    if major in (2, 3):
        length, pos, preferred = read_arg(data, pos, ai)
        if length is None:
            parts = []
            total = 0
            while True:
                if pos >= len(data):
                    raise ParseFault("unterminated string")
                if data[pos] == 0xFF:
                    pos += 1
                    break
                child, pos = parse_item(data, pos, depth + 1)
                if child.kind != ("bytes" if major == 2 else "text") or child.indefinite:
                    raise ParseFault("invalid indefinite string chunk")
                chunk = child.value if major == 2 else child.value.encode("utf-8")
                total += len(chunk)
                if total > MAX_STRING_BYTES:
                    raise ResourceFault("string resource bound")
                parts.append(chunk)
            merged = b"".join(parts)
            if major == 2:
                return Node("bytes", merged, data[start:pos], False, True, depth), pos
            try:
                text = merged.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise ParseFault("invalid UTF-8") from exc
            return Node("text", text, data[start:pos], False, True, depth), pos

        if length > MAX_STRING_BYTES:
            raise ResourceFault("string resource bound")
        end = pos + length
        if end > len(data):
            raise ParseFault("short string")
        payload = data[pos:end]
        if major == 2:
            return Node("bytes", payload, data[start:end], preferred, False, depth), end
        try:
            text = payload.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ParseFault("invalid UTF-8") from exc
        return Node("text", text, data[start:end], preferred, False, depth), end

    if major == 4:
        length, pos, preferred = read_arg(data, pos, ai)
        if length is not None and length > MAX_CONTAINER_ITEMS:
            raise ResourceFault("array resource bound")
        items = []
        if length is None:
            count = 0
            while True:
                if pos >= len(data):
                    raise ParseFault("unterminated array")
                if data[pos] == 0xFF:
                    pos += 1
                    break
                if count >= MAX_CONTAINER_ITEMS:
                    raise ResourceFault("array resource bound")
                item, pos = parse_item(data, pos, depth + 1)
                items.append(item)
                count += 1
            return Node("array", items, data[start:pos], False, True, depth), pos
        for _ in range(length):
            item, pos = parse_item(data, pos, depth + 1)
            items.append(item)
        return Node("array", items, data[start:pos], preferred, False, depth), pos

    if major == 5:
        length, pos, preferred = read_arg(data, pos, ai)
        if length is not None and length > MAX_CONTAINER_ITEMS:
            raise ResourceFault("map resource bound")
        pairs = []
        if length is None:
            count = 0
            while True:
                if pos >= len(data):
                    raise ParseFault("unterminated map")
                if data[pos] == 0xFF:
                    pos += 1
                    break
                if count >= MAX_CONTAINER_ITEMS:
                    raise ResourceFault("map resource bound")
                key, pos = parse_item(data, pos, depth + 1)
                value, pos = parse_item(data, pos, depth + 1)
                pairs.append((key, value))
                count += 1
            return Node("map", pairs, data[start:pos], False, True, depth), pos
        for _ in range(length):
            key, pos = parse_item(data, pos, depth + 1)
            value, pos = parse_item(data, pos, depth + 1)
            pairs.append((key, value))
        return Node("map", pairs, data[start:pos], preferred, False, depth), pos

    if major == 6:
        tag, pos, preferred = read_arg(data, pos, ai)
        if tag is None:
            raise ParseFault("indefinite tag")
        child, pos = parse_item(data, pos, depth + 1)
        return Node("tag", (tag, child), data[start:pos], preferred, False, depth), pos

    if major == 7:
        if ai == 24:
            if pos >= len(data):
                raise ParseFault("short simple")
            value = data[pos]
            pos += 1
            if value < 32:
                raise ParseFault("non-well-formed simple")
            return Node("simple", value, data[start:pos], True, False, depth), pos
        if ai == 25:
            end = pos + 2
            if end > len(data):
                raise ParseFault("short binary16")
            return Node("float", data[start:end], data[start:end], True, False, depth), end
        if ai == 26:
            end = pos + 4
            if end > len(data):
                raise ParseFault("short binary32")
            return Node("float", data[start:end], data[start:end], True, False, depth), end
        if ai == 27:
            end = pos + 8
            if end > len(data):
                raise ParseFault("short binary64")
            return Node("float", data[start:end], data[start:end], True, False, depth), end
        if ai >= 28:
            raise ParseFault("reserved simple/break")
        return Node("simple", ai, data[start:pos], True, False, depth), pos

    raise ParseFault("unsupported major type")


def float_fields(raw: bytes):
    layouts = {
        3: (5, 10, 15),
        5: (8, 23, 127),
        9: (11, 52, 1023),
    }
    spec = layouts.get(len(raw))
    if spec is None:
        return None
    exponent_bits, fraction_bits, bias = spec
    bits = int.from_bytes(raw[1:], "big")
    fraction_mask = (1 << fraction_bits) - 1
    exponent_mask = (1 << exponent_bits) - 1
    exponent = (bits >> fraction_bits) & exponent_mask
    fraction = bits & fraction_mask
    return exponent_bits, fraction_bits, bias, exponent, fraction


def decode_dyadic(raw: bytes):
    fields = float_fields(raw)
    if fields is None:
        return None
    exponent_bits, fraction_bits, bias, exponent, fraction = fields
    if exponent == (1 << exponent_bits) - 1:
        return None
    if exponent == 0:
        significand = fraction
        power = 1 - bias - fraction_bits
    else:
        significand = (1 << fraction_bits) | fraction
        power = exponent - bias - fraction_bits
    if raw[1] & 0x80:
        significand = -significand
    return significand, power


def exact_representable_in(raw: bytes, target_len: int):
    decoded = decode_dyadic(raw)
    if decoded is None:
        return False
    significand, power = decoded
    if significand == 0:
        return True

    n = abs(significand)
    while (n & 1) == 0:
        n >>= 1
        power += 1

    _, fraction_bits, bias = {
        3: (5, 10, 15),
        5: (8, 23, 127),
        9: (11, 52, 1023),
    }[target_len]
    precision = fraction_bits + 1
    emin = 1 - bias
    emax = bias
    bit_length = n.bit_length()
    value_exponent = power + bit_length - 1

    if bit_length <= precision and emin <= value_exponent <= emax:
        return True

    subnormal_power = emin - (precision - 1)
    if power >= subnormal_power:
        scaled = n << (power - subnormal_power)
        return 0 < scaled < (1 << fraction_bits)

    return False


def nan_is_shortest(raw: bytes):
    fields = float_fields(raw)
    if fields is None:
        return False
    exponent_bits, fraction_bits, _, exponent, fraction = fields
    if exponent != (1 << exponent_bits) - 1 or fraction == 0:
        return False
    if len(raw) == 3:
        return True
    for target_len, target_fraction_bits in ((3, 10), (5, 23)):
        if target_len >= len(raw):
            continue
        shift = fraction_bits - target_fraction_bits
        if fraction & ((1 << shift) - 1) == 0:
            return False
    return True


def float_deterministic(raw: bytes):
    fields = float_fields(raw)
    if fields is None:
        return False
    exponent_bits, _, _, exponent, fraction = fields
    max_exponent = (1 << exponent_bits) - 1

    if exponent == max_exponent:
        if fraction != 0:
            return nan_is_shortest(raw)
        # IEEE infinities have the same value in all three widths; binary16
        # is therefore always the shortest valid representation.
        return len(raw) == 3

    if len(raw) == 3:
        return True
    if len(raw) == 5:
        return not exact_representable_in(raw, 3)
    return not exact_representable_in(raw, 3) and not exact_representable_in(raw, 5)


def has_nan(node: Node):
    if node.kind == "float":
        fields = float_fields(node.raw)
        return fields is not None and fields[3] == (1 << fields[0]) - 1 and fields[4] != 0
    if node.kind == "array":
        return any(has_nan(child) for child in node.value)
    if node.kind == "map":
        return any(has_nan(key) or has_nan(value) for key, value in node.value)
    if node.kind == "tag":
        return has_nan(node.value[1])
    return False


def deterministic(node: Node):
    if node.indefinite or not node.preferred:
        return False
    if node.kind == "float":
        return float_deterministic(node.raw)
    if node.kind == "array":
        return all(deterministic(child) for child in node.value)
    if node.kind == "map":
        if not all(deterministic(k) and deterministic(v) for k, v in node.value):
            return False
        keys = [k.raw for k, _ in node.value]
        return keys == sorted(keys)
    if node.kind == "tag":
        return deterministic(node.value[1])
    return True


def parse_complete(data: bytes):
    node, end = parse_item(data)
    if end != len(data):
        raise ParseFault("trailing bytes")
    return node


def protected_map(node: Node):
    if node.kind != "bytes" or node.indefinite:
        raise ShapeFault("protected header must be a definite bstr")
    inner, end = parse_item(node.value, 0, node.depth + 1)
    if end != len(node.value) or inner.kind != "map":
        raise ShapeFault("protected header must contain exactly one map")
    return inner


def label_valid(node: Node):
    return node.kind in ("int", "text")


def label_identity(node: Node):
    if node.kind == "int":
        return ("int", node.value)
    if node.kind == "text":
        return ("text", node.value)
    return (node.kind, node.raw)


def classify(data: bytes):
    try:
        root = parse_complete(data)
        if root.kind != "array" or len(root.value) != 4:
            return MESSAGE_REJECT

        protected, unprotected, payload, signature = root.value
        if protected.kind != "bytes" or unprotected.kind != "map" or signature.kind != "bytes":
            return MESSAGE_REJECT
        if payload.kind not in ("bytes", "simple") or (
            payload.kind == "simple" and payload.value != 22
        ):
            return MESSAGE_REJECT

        pmap = protected_map(protected)

        if not deterministic(root) or not deterministic(pmap) or not deterministic(unprotected):
            return ENCODING_REJECT

        for current in (pmap, unprotected):
            for key, _ in current.value:
                if not label_valid(key):
                    return MESSAGE_REJECT

        for current in (pmap, unprotected):
            seen = set()
            for key, _ in current.value:
                ident = label_identity(key)
                if ident in seen:
                    return MESSAGE_REJECT
                seen.add(ident)

        left = {label_identity(key) for key, _ in pmap.value}
        right = {label_identity(key) for key, _ in unprotected.value}
        if left.intersection(right):
            return MESSAGE_REJECT

        if any(has_nan(value) for _, value in pmap.value):
            return MESSAGE_REJECT
        if any(has_nan(value) for _, value in unprotected.value):
            return MESSAGE_REJECT

        values = {label_identity(key): value for key, value in pmap.value}
        if ("int", 1) not in values or ("int", 4) not in values:
            return MESSAGE_REJECT

        if values[("int", 1)].kind not in ("int", "text"):
            return MESSAGE_REJECT
        if values[("int", 4)].kind != "bytes":
            return MESSAGE_REJECT

        return SUFFICIENT
    except ShapeFault:
        return MESSAGE_REJECT
    except (ParseFault, ResourceFault, DepthFault):
        return PARSE_ERROR
    except Exception:
        return UNRESOLVED


def corpus_digest(vectors):
    digest = hashlib.sha256()
    for vector in vectors:
        digest.update(vector["id"].encode("ascii"))
        digest.update(b"\x00")
        digest.update(vector["family"].encode("utf-8"))
        digest.update(b"\x00")
        digest.update(bytes.fromhex(vector["hex"]))
    return digest.hexdigest()


def result_digest(per_case):
    digest = hashlib.sha256()
    for index in range(1, 52):
        case_id = f"C-{index:02d}"
        digest.update(case_id.encode("ascii"))
        digest.update(b"\x00")
        digest.update(per_case[case_id].encode("ascii"))
        digest.update(b"\x00")
    return digest.hexdigest()


def verify_subject(candidate_root: Path, run_meta: dict, jobs_meta: dict, trusted: dict):
    target = trusted["candidate_sha"]
    parent = trusted["parent_sha"]

    if run_meta["head_sha"] != target:
        raise AssertionError("trigger run head mismatch")
    if run_meta["event"] != "pull_request":
        raise AssertionError("trigger run must be a pull_request")
    if run_meta["workflow_id"] != 375144995 or run_meta["path"] != CANDIDATE_WORKFLOW_REL:
        raise AssertionError("trigger workflow identity mismatch")
    if run_meta["repository"]["id"] != 1176351975 or run_meta["repository"]["full_name"] != "Luminous-Dynamics/mycelix":
        raise AssertionError("trigger repository mismatch")
    if run_meta["head_repository"]["id"] != 1176351975 or run_meta["head_repository"]["full_name"] != "Luminous-Dynamics/mycelix":
        raise AssertionError("trigger head repository mismatch")
    if run_meta["conclusion"] != "success":
        raise AssertionError("trigger run did not succeed")

    if not any(
        item.get("number") == trusted["candidate_pr"]
        and item.get("head", {}).get("sha") == target
        and item.get("base", {}).get("sha") == parent
        for item in run_meta.get("pull_requests", [])
    ):
        raise AssertionError("trigger PR topology mismatch")

    jobs = [
        job for job in jobs_meta.get("jobs", [])
        if job.get("run_id") == run_meta["id"]
        and job.get("name") == "CBOR encoding boundary"
        and job.get("status") == "completed"
        and job.get("conclusion") == "success"
    ]
    if len(jobs) != 1:
        raise AssertionError("expected exactly one successful semantic qualification job")

    if git(candidate_root, "rev-parse", "HEAD") != target:
        raise AssertionError("candidate checkout mismatch")
    if git(candidate_root, "rev-parse", f"{target}^") != parent:
        raise AssertionError("candidate parent mismatch")
    if git(candidate_root, "rev-list", "--count", f"{parent}..{target}") != "1":
        raise AssertionError("candidate topology is not exactly one commit")

    expected_files = sorted(trusted["expected_changed_files"])
    actual_files = git(candidate_root, "diff", "--name-only", parent, target).splitlines()
    if actual_files != expected_files:
        raise AssertionError({"expected_files": expected_files, "actual_files": actual_files})

    expected_blobs = {
        CANDIDATE_WORKFLOW_REL: trusted["candidate_workflow_blob_sha"],
        CANDIDATE_DOC_REL: trusted["candidate_doc_blob_sha"],
        CANDIDATE_MANIFEST_REL: trusted["candidate_manifest_blob_sha"],
        CANDIDATE_QUALIFIER_REL: trusted["candidate_qualifier_blob_sha"],
    }
    for path, expected in expected_blobs.items():
        actual = git(candidate_root, "rev-parse", f"{target}:{path}")
        if actual != expected:
            raise AssertionError(f"candidate blob mismatch: {path}")

    manifest = load_json(candidate_root / CANDIDATE_MANIFEST_REL)
    if manifest.get("schema") != trusted["candidate_schema"]:
        raise AssertionError("candidate schema mismatch")
    if manifest.get("program") != trusted["candidate_program"]:
        raise AssertionError("candidate program mismatch")
    if manifest.get("parent_subject") != parent or manifest.get("analysis_role") != "research_only":
        raise AssertionError("candidate manifest identity mismatch")
    if set(manifest) != {
        "schema", "program", "parent_subject", "analysis_role",
        "cases", "depth_vectors", "resource_profile", "resource_vectors",
    }:
        raise AssertionError("unexpected candidate manifest keys")
    if manifest["resource_profile"] != trusted["resource_profile"]:
        raise AssertionError("resource profile mismatch")

    cases = manifest["cases"]
    depths = manifest["depth_vectors"]
    resources = manifest["resource_vectors"]

    if [x["id"] for x in cases] != [f"C-{i:02d}" for i in range(1, 52)]:
        raise AssertionError("message id sequence mismatch")
    if [x["id"] for x in depths] != [f"D-{i:02d}" for i in range(1, 5)]:
        raise AssertionError("depth id sequence mismatch")
    if [x["id"] for x in resources] != [f"R-{i:02d}" for i in range(1, 13)]:
        raise AssertionError("resource id sequence mismatch")

    vectors = cases + depths + resources
    if len({v["hex"] for v in vectors}) != len(vectors):
        raise AssertionError("duplicate exact vector bytes")

    for vector in vectors:
        if set(vector) != {"id", "family", "hex"}:
            raise AssertionError("vector shape mismatch")
        if not isinstance(vector["family"], str) or not isinstance(vector["hex"], str):
            raise AssertionError("vector field type mismatch")
        if vector["hex"] != vector["hex"].lower() or len(vector["hex"]) % 2:
            raise AssertionError("non-canonical vector hex")
        raw = bytes.fromhex(vector["hex"])
        if raw.hex() != vector["hex"] or not raw:
            raise AssertionError("vector bytes do not round-trip")

    message_digest = corpus_digest(cases)
    depth_digest = corpus_digest(depths)
    resource_digest = corpus_digest(resources)
    if message_digest != trusted["expected_message_corpus_sha256"]:
        raise AssertionError("message corpus digest mismatch")
    if depth_digest != trusted["expected_depth_corpus_sha256"]:
        raise AssertionError("depth corpus digest mismatch")
    if resource_digest != trusted["expected_resource_corpus_sha256"]:
        raise AssertionError("resource corpus digest mismatch")

    counts = {
        SUFFICIENT: 0,
        MESSAGE_REJECT: 0,
        ENCODING_REJECT: 0,
        PARSE_ERROR: 0,
        UNRESOLVED: 0,
    }
    per_case = {}
    for case in cases:
        outcome = classify(bytes.fromhex(case["hex"]))
        per_case[case["id"]] = outcome
        counts[outcome] += 1

    if counts != trusted["expected_counts"]:
        raise AssertionError({"expected_counts": trusted["expected_counts"], "actual_counts": counts})

    message_results_digest = result_digest(per_case)
    if message_results_digest != trusted["expected_message_results_sha256"]:
        raise AssertionError({
            "expected_message_results_sha256": trusted["expected_message_results_sha256"],
            "actual_message_results_sha256": message_results_digest,
        })

    # High-value semantic anchors: these are deliberately distinct boundaries
    # that have historically exposed classification leakage.
    assert per_case["C-03"] == MESSAGE_REJECT
    assert per_case["C-04"] == MESSAGE_REJECT
    assert per_case["C-11"] == ENCODING_REJECT
    assert per_case["C-12"] == ENCODING_REJECT
    assert per_case["C-28"] == SUFFICIENT
    assert per_case["C-29"] == ENCODING_REJECT
    assert per_case["C-30"] == SUFFICIENT
    assert per_case["C-31"] == ENCODING_REJECT

    depth_results = {}
    for vector in depths:
        data = bytes.fromhex(vector["hex"])
        try:
            node = parse_complete(data)
            while node.kind == "array" and len(node.value) == 1:
                node = node.value[0]
            if node.kind != "bytes":
                raise ShapeFault("depth vector does not terminate at protected bstr")
            parse_item(node.value, 0, node.depth + 1)
            depth_results[vector["id"]] = DEPTH_WITHIN
        except DepthFault:
            depth_results[vector["id"]] = DEPTH_EXCEEDED
        except Exception:
            depth_results[vector["id"]] = DEPTH_UNRESOLVED

    if depth_results != trusted["depth_results"]:
        raise AssertionError({"expected_depth": trusted["depth_results"], "actual_depth": depth_results})

    resource_results = {}
    for vector in resources:
        data = bytes.fromhex(vector["hex"])
        try:
            _, end = parse_item(data)
            if end != len(data):
                raise ParseFault("resource probe trailing bytes")
            resource_results[vector["id"]] = RESOURCE_WITHIN
        except ResourceFault:
            resource_results[vector["id"]] = RESOURCE_EXCEEDED
        except Exception:
            resource_results[vector["id"]] = RESOURCE_UNRESOLVED

    if resource_results != trusted["resource_results"]:
        raise AssertionError({"expected_resource": trusted["resource_results"], "actual_resource": resource_results})

    print("SYM-CIVIC-018-INDEPENDENT-SEMANTIC=PASS")
    print(f"candidate={target}")
    print(f"parent={parent}")
    print(f"run_id={run_meta['id']}")
    print(f"job_id={jobs[0]['id']}")
    print(f"cases={len(cases)}")
    print(f"message_corpus_sha256={message_digest}")
    print(f"depth_corpus_sha256={depth_digest}")
    print(f"resource_corpus_sha256={resource_digest}")
    print(f"message_results_sha256={message_results_digest}")
    print("counts=" + json.dumps(counts, sort_keys=True, separators=(",", ":")))
    print("depth_results=" + json.dumps(depth_results, sort_keys=True, separators=(",", ":")))
    print("resource_results=" + json.dumps(resource_results, sort_keys=True, separators=(",", ":")))
    print("boundary=trusted_candidate_independent_semantic_recomputation")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-root", required=True, type=Path)
    parser.add_argument("--run-json", required=True, type=Path)
    parser.add_argument("--jobs-json", required=True, type=Path)
    args = parser.parse_args()

    trusted = load_json(TRUSTED_MANIFEST)
    if trusted["schema"] != "MYCELIX-SYM-CIVIC-018-INDEPENDENT-SEMANTIC-V1":
        raise SystemExit("trusted manifest schema mismatch")
    verify_subject(args.candidate_root.resolve(), load_json(args.run_json), load_json(args.jobs_json), trusted)


if __name__ == "__main__":
    main()
