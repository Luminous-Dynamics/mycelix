#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import math
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_018_cbor_encoding_v1.json"
SCHEMA = "mycelix.sym-civic.cbor-encoding-preflight.v3"
PROGRAM = "SYM-CIVIC-018-CBOR-FP-DEPTH"
PARENT_SUBJECT = "ec2eb36f58ea9dede58e4071228d3fa5d42ffffc"

SUFFICIENT = "CBOR_ENCODING_SUFFICIENT"
MESSAGE_REJECT = "CBOR_MESSAGE_REJECT"
ENCODING_REJECT = "CBOR_ENCODING_REJECT"
PARSE_ERROR = "CBOR_PARSE_ERROR"
UNRESOLVED = "CBOR_ENCODING_UNRESOLVED"


class Fault(Exception):
    pass


class InvalidText(Fault):
    pass


class DepthExceeded(Fault):
    pass


MAX_DEPTH = 32


def head(major, value):
    if value < 24:
        return bytes([(major << 5) | value])
    if value < 256:
        return bytes([(major << 5) | 24, value])
    if value < 65536:
        return bytes([(major << 5) | 25]) + value.to_bytes(2, "big")
    if value < 4294967296:
        return bytes([(major << 5) | 26]) + value.to_bytes(4, "big")
    return bytes([(major << 5) | 27]) + value.to_bytes(8, "big")


def read_arg(data, pos, ai):
    if ai < 24:
        return ai, pos, True
    widths = {24: 1, 25: 2, 26: 4, 27: 8}
    if ai == 31:
        return None, pos, False
    width = widths.get(ai)
    if width is None or pos + width > len(data):
        raise Fault("bad argument")
    value = int.from_bytes(data[pos:pos + width], "big")
    minimum = {1: 24, 2: 256, 4: 65536, 8: 4294967296}[width]
    return value, pos + width, value >= minimum


def parse(data, pos=0, depth=0):
    if depth > MAX_DEPTH:
        raise DepthExceeded("maximum parser depth exceeded")
    start = pos
    if pos >= len(data):
        raise Fault("missing initial byte")
    initial = data[pos]
    pos += 1
    major = initial >> 5
    ai = initial & 31

    if major in (0, 1):
        value, pos, preferred = read_arg(data, pos, ai)
        if value is None:
            raise Fault("indefinite integer")
        value = value if major == 0 else -1 - value
        return ("i", value, preferred, data[start:pos], False), pos

    if major in (2, 3):
        length, pos, preferred = read_arg(data, pos, ai)
        if length is None:
            chunks = []
            while True:
                if pos >= len(data):
                    raise Fault("open string")
                if data[pos] == 255:
                    pos += 1
                    break
                child, pos = parse(data, pos, depth + 1)
                if child[0] != ("b" if major == 2 else "t") or child[4]:
                    raise Fault("bad string chunk")
                chunks.append(child[1] if major == 2 else child[1].encode("utf-8"))
            raw = data[start:pos]
            if major == 2:
                return ("b", b"".join(chunks), False, raw, True, depth), pos
            try:
                return ("t", b"".join(chunks).decode("utf-8"), False, raw, True), pos
            except UnicodeDecodeError as exc:
                raise InvalidText from exc
        end = pos + length
        if end > len(data):
            raise Fault("short string")
        raw = data[start:end]
        if major == 2:
            return ("b", data[pos:end], preferred, raw, False, depth), end
        try:
            return ("t", data[pos:end].decode("utf-8"), preferred, raw, False), end
        except UnicodeDecodeError as exc:
            raise InvalidText from exc

    if major == 4:
        length, pos, preferred = read_arg(data, pos, ai)
        items = []
        if length is None:
            while True:
                if pos >= len(data):
                    raise Fault("open array")
                if data[pos] == 255:
                    pos += 1
                    break
                item, pos = parse(data, pos, depth + 1)
                items.append(item)
            return ("a", items, False, data[start:pos], True), pos
        for _ in range(length):
            item, pos = parse(data, pos, depth + 1)
            items.append(item)
        return ("a", items, preferred, data[start:pos], False), pos

    if major == 5:
        length, pos, preferred = read_arg(data, pos, ai)
        pairs = []
        if length is None:
            while True:
                if pos >= len(data):
                    raise Fault("open map")
                if data[pos] == 255:
                    pos += 1
                    break
                key, pos = parse(data, pos, depth + 1)
                value, pos = parse(data, pos, depth + 1)
                pairs.append((key, value))
            return ("m", pairs, False, data[start:pos], True), pos
        for _ in range(length):
            key, pos = parse(data, pos, depth + 1)
            value, pos = parse(data, pos, depth + 1)
            pairs.append((key, value))
        return ("m", pairs, preferred, data[start:pos], False), pos

    if major == 6:
        tag, pos, preferred = read_arg(data, pos, ai)
        if tag is None:
            raise Fault("indefinite tag")
        child, pos = parse(data, pos, depth + 1)
        return ("g", (tag, child), preferred, data[start:pos], False), pos

    if major == 7:
        if ai in (28, 29, 30, 31):
            raise Fault("reserved/simple break")
        if ai == 24:
            if pos >= len(data):
                raise Fault("short simple")
            value = data[pos]
            pos += 1
            if value < 32:
                raise Fault("non-preferred simple")
            return ("s", value, True, data[start:pos], False), pos
        if ai == 25:
            end = pos + 2
            if end > len(data):
                raise Fault("short float")
            return ("f", data[pos:end], True, data[start:end], False), end
        if ai == 26:
            end = pos + 4
            if end > len(data):
                raise Fault("short float")
            return ("f", data[pos:end], True, data[start:end], False), end
        if ai == 27:
            end = pos + 8
            if end > len(data):
                raise Fault("short float")
            return ("f", data[pos:end], True, data[start:end], False), end
        return ("s", ai, True, data[start:pos], False), pos

    raise Fault("unsupported major type")



def _exact_float_representation(fmt, value):
    try:
        packed = struct.pack(fmt, value)
        decoded = struct.unpack(fmt, packed)[0]
    except (OverflowError, struct.error):
        return False
    if decoded != value:
        return False
    if value == 0.0:
        return math.copysign(1.0, decoded) == math.copysign(1.0, value)
    return True


def float_deterministic(raw):
    if not raw:
        return False
    ai = raw[0] & 31
    width_fmt = {
        25: (2, ">e"),
        26: (4, ">f"),
        27: (8, ">d"),
    }.get(ai)
    if width_fmt is None or len(raw) != width_fmt[0] + 1:
        return False
    try:
        value = struct.unpack(width_fmt[1], raw[1:])[0]
    except struct.error:
        return False
    if math.isnan(value):
        return False
    width = width_fmt[0]
    if width == 2:
        return True
    if _exact_float_representation(">e", value):
        return False
    if width == 4:
        return True
    if _exact_float_representation(">f", value):
        return False
    return True


def canonical(node):
    kind, value, preferred, raw, indefinite = node[:5]
    if indefinite or not preferred:
        raise Fault("non-deterministic")
    if kind == "i":
        return head(0, value) if value >= 0 else head(1, -1 - value)
    if kind == "b":
        return head(2, len(value)) + value
    if kind == "t":
        raw_text = value.encode("utf-8")
        return head(3, len(raw_text)) + raw_text
    if kind == "a":
        return head(4, len(value)) + b"".join(canonical(item) for item in value)
    if kind == "m":
        encoded = [(canonical(k), canonical(v)) for k, v in value]
        keys = [k for k, _ in encoded]
        if keys != sorted(keys) or len(keys) != len(set(keys)):
            raise Fault("map ordering")
        return head(5, len(encoded)) + b"".join(k + v for k, v in encoded)
    if kind == "g":
        return head(6, value[0]) + canonical(value[1])
    if kind == "s":
        if value in (20, 21, 22, 23):
            return bytes([0xE0 | value])
        return b"\xF8" + bytes([value])
    if kind == "f":
        if not float_deterministic(raw):
            raise Fault("non-deterministic float")
        return raw
    raise Fault("unknown kind")


def deterministic(node):
    try:
        return canonical(node) == node[3]
    except Fault:
        return False


def kid(node):
    if node[0] == "i":
        return ("i", node[1])
    if node[0] == "b":
        return ("b", node[1])
    if node[0] == "t":
        return ("t", node[1])
    return (node[0], node[3])


def dup(pairs):
    seen = set()
    for key, _ in pairs:
        ident = kid(key)
        if ident in seen:
            return True
        seen.add(ident)
    return False


def protected_map(node):
    if node[0] != "b":
        raise Fault("protected not bytes")
    if len(node) != 6:
        raise Fault("protected depth metadata missing")
    inner, end = parse(node[1], depth=node[5] + 1)
    if end != len(node[1]) or inner[0] != "m":
        raise Fault("protected bytes not one map")
    return inner


def primary(data):
    try:
        root, end = parse(data)
        if end != len(data):
            return PARSE_ERROR
        if root[0] != "a" or len(root[1]) != 4:
            return MESSAGE_REJECT
        protected, unprotected, payload, signature = root[1]
        if protected[0] != "b" or unprotected[0] != "m" or signature[0] != "b":
            return MESSAGE_REJECT
        pmap = protected_map(protected)
        if dup(pmap[1]) or dup(unprotected[1]):
            return MESSAGE_REJECT
        if set(kid(k) for k, _ in pmap[1]).intersection(set(kid(k) for k, _ in unprotected[1])):
            return MESSAGE_REJECT
        if not deterministic(root) or not deterministic(pmap) or not deterministic(unprotected):
            return ENCODING_REJECT
        labels = {kid(k) for k, _ in pmap[1]}
        if ("i", 1) not in labels or ("i", 4) not in labels:
            return MESSAGE_REJECT
        if payload[0] not in ("b", "s") or (payload[0] == "s" and payload[1] != 22):
            return MESSAGE_REJECT
        return SUFFICIENT
    except InvalidText:
        return PARSE_ERROR
    except Fault:
        return PARSE_ERROR


def read_arg_ref(data, pos, ai):
    if ai < 24:
        return ai, pos, True
    widths = {24: 1, 25: 2, 26: 4, 27: 8}
    if ai == 31:
        return None, pos, False
    width = widths.get(ai)
    if width is None or pos + width > len(data):
        raise Fault("ref bad argument")
    value = int.from_bytes(data[pos:pos+width], "big")
    minimum = {1:24, 2:256, 4:65536, 8:4294967296}[width]
    return value, pos+width, value >= minimum


def scan(data, pos=0, depth=0):
    if depth > MAX_DEPTH:
        raise DepthExceeded("ref maximum parser depth exceeded")
    start = pos
    if pos >= len(data):
        raise Fault("ref missing byte")
    initial = data[pos]
    pos += 1
    major = initial >> 5
    ai = initial & 31
    if major in (0, 1, 2, 3, 4, 5, 6):
        arg, pos, preferred = read_arg_ref(data, pos, ai)
        if major in (0, 1):
            if arg is None:
                raise Fault("ref indefinite int")
            return {"m":major,"v":arg if major == 0 else -1-arg,"p":preferred,"r":data[start:pos]}, pos
        if major in (2, 3):
            if arg is None:
                chunks = []
                while True:
                    if pos >= len(data):
                        raise Fault("ref open string")
                    if data[pos] == 255:
                        pos += 1
                        break
                    child, pos = scan(data, pos, depth + 1)
                    if child["m"] != major or child.get("i"):
                        raise Fault("ref bad string chunk")
                    chunks.append(data[child["s"]:child["e"]])
                raw = data[start:pos]
                result = {"m":major,"v":b"".join(chunks),"p":False,"r":raw,"i":True,"s":start,"e":pos}
                if major == 2:
                    result["d"] = depth
                return result, pos
            end = pos + arg
            if end > len(data):
                raise Fault("ref short string")
            if major == 3:
                data[pos:end].decode("utf-8")
            result = {"m":major,"v":data[pos:end],"p":preferred,"r":data[start:end],"s":start,"e":end}
            if major == 2:
                result["d"] = depth
            return result, end
        if major == 4:
            items = []
            if arg is None:
                while True:
                    if pos >= len(data):
                        raise Fault("ref open array")
                    if data[pos] == 255:
                        pos += 1
                        break
                    child, pos = scan(data, pos, depth + 1)
                    items.append(child)
                return {"m":4,"items":items,"p":False,"i":True,"r":data[start:pos],"s":start,"e":pos}, pos
            for _ in range(arg):
                child, pos = scan(data, pos, depth + 1)
                items.append(child)
            return {"m":4,"items":items,"p":preferred,"r":data[start:pos],"s":start,"e":pos}, pos
        if major == 5:
            pairs = []
            if arg is None:
                while True:
                    if pos >= len(data):
                        raise Fault("ref open map")
                    if data[pos] == 255:
                        pos += 1
                        break
                    key, pos = scan(data, pos, depth + 1)
                    value, pos = scan(data, pos, depth + 1)
                    pairs.append((key, value))
                return {"m":5,"pairs":pairs,"p":False,"i":True,"r":data[start:pos],"s":start,"e":pos}, pos
            for _ in range(arg):
                key, pos = scan(data, pos, depth + 1)
                value, pos = scan(data, pos, depth + 1)
                pairs.append((key, value))
            return {"m":5,"pairs":pairs,"p":preferred,"r":data[start:pos],"s":start,"e":pos}, pos
        if arg is None:
            raise Fault("ref indefinite tag")
        child, pos = scan(data, pos, depth + 1)
        return {"m":6,"v":(arg,child),"p":preferred,"r":data[start:pos],"s":start,"e":pos}, pos
    if major == 7:
        if ai in (28,29,30,31):
            raise Fault("ref reserved")
        if ai == 24:
            if pos >= len(data):
                raise Fault("ref short simple")
            value = data[pos]
            pos += 1
            if value < 32:
                raise Fault("ref non-preferred simple")
            return {"m":7,"v":value,"p":True,"r":data[start:pos],"s":start,"e":pos}, pos
        widths = {25:2,26:4,27:8}
        if ai in widths:
            end = pos + widths[ai]
            if end > len(data):
                raise Fault("ref short float")
            return {"m":7,"v":data[pos:end],"p":True,"r":data[start:end],"s":start,"e":end}, end
        return {"m":7,"v":ai,"p":True,"r":data[start:pos],"s":start,"e":pos}, pos
    raise Fault("ref unsupported major")


def ref_key(node):
    if node["m"] == 0:
        return ("i", node["v"])
    if node["m"] == 1:
        return ("i", -1 - node["v"])
    if node["m"] == 2:
        return ("b", node["v"])
    if node["m"] == 3:
        return ("t", node["v"].decode("utf-8"))
    return ("raw", node["r"])



def ref_float_deterministic(raw):
    ai = raw[0] & 31
    widths = {25: (2, ">e"), 26: (4, ">f"), 27: (8, ">d")}
    width_fmt = widths.get(ai)
    if width_fmt is None or len(raw) != width_fmt[0] + 1:
        return False
    try:
        value = struct.unpack(width_fmt[1], raw[1:])[0]
    except struct.error:
        return False
    if math.isnan(value):
        return False
    if width_fmt[0] == 2:
        return True
    try:
        half = struct.unpack(">e", struct.pack(">e", value))[0]
    except (OverflowError, struct.error):
        half = None
    if half == value and (value != 0.0 or math.copysign(1.0, half) == math.copysign(1.0, value)):
        return False
    if width_fmt[0] == 4:
        return True
    try:
        single = struct.unpack(">f", struct.pack(">f", value))[0]
    except (OverflowError, struct.error):
        single = None
    if single == value and (value != 0.0 or math.copysign(1.0, single) == math.copysign(1.0, value)):
        return False
    return True


def ref_deterministic(node):
    if node.get("i") or not node["p"]:
        return False
    if node["m"] in (0,1,2,3):
        return True
    if node["m"] == 7:
        ai = node["r"][0] & 31
        if ai in (25, 26, 27):
            return ref_float_deterministic(node["r"])
        return True
    if node["m"] == 4:
        return all(ref_deterministic(child) for child in node["items"])
    if node["m"] == 5:
        if not all(ref_deterministic(k) and ref_deterministic(v) for k,v in node["pairs"]):
            return False
        keys = [k["r"] for k,_ in node["pairs"]]
        return keys == sorted(keys) and len({ref_key(k) for k,_ in node["pairs"]}) == len(node["pairs"])
    if node["m"] == 6:
        return ref_deterministic(node["v"][1])
    return False


def ref_protected(node):
    if node["m"] != 2 or node.get("i"):
        raise Fault("ref protected")
    depth = node.get("d")
    if depth is None:
        raise Fault("ref protected depth metadata missing")
    inner, end = scan(node["v"], depth=depth + 1)
    if end != len(node["v"]) or inner["m"] != 5:
        raise Fault("ref protected map")
    return inner


DEPTH_PROBE_WITHIN_LIMIT = "DEPTH_PROBE_WITHIN_LIMIT"
DEPTH_PROBE_EXCEEDED = "DEPTH_PROBE_EXCEEDED"


def _find_unary_bstr_primary(node):
    while node[0] == "a" and len(node[1]) == 1:
        node = node[1][0]
    if node[0] != "b":
        raise Fault("depth probe did not terminate at bstr")
    return node


def _find_unary_bstr_reference(node):
    while node["m"] == 4 and len(node["items"]) == 1:
        node = node["items"][0]
    if node["m"] != 2:
        raise Fault("ref depth probe did not terminate at bstr")
    return node


def depth_probe_primary(data):
    root, end = parse(data)
    if end != len(data):
        raise Fault("depth probe trailing bytes")
    protected_map(_find_unary_bstr_primary(root))


def depth_probe_reference(data):
    root, end = scan(data)
    if end != len(data):
        raise Fault("ref depth probe trailing bytes")
    ref_protected(_find_unary_bstr_reference(root))


def run_depth_probe(fn, data):
    try:
        fn(data)
    except DepthExceeded:
        return DEPTH_PROBE_EXCEEDED
    return DEPTH_PROBE_WITHIN_LIMIT


def reference(data):
    try:
        root, end = scan(data)
        if end != len(data):
            return PARSE_ERROR
        if root["m"] != 4 or len(root["items"]) != 4:
            return MESSAGE_REJECT
        protected, unprotected, payload, signature = root["items"]
        if protected["m"] != 2 or unprotected["m"] != 5 or signature["m"] != 2:
            return MESSAGE_REJECT
        pmap = ref_protected(protected)
        for current in (pmap, unprotected):
            seen = set()
            for key, _ in current["pairs"]:
                ident = ref_key(key)
                if ident in seen:
                    return MESSAGE_REJECT
                seen.add(ident)
        left = {ref_key(k) for k,_ in pmap["pairs"]}
        right = {ref_key(k) for k,_ in unprotected["pairs"]}
        if left.intersection(right):
            return MESSAGE_REJECT
        if not ref_deterministic(root) or not ref_deterministic(pmap) or not ref_deterministic(unprotected):
            return ENCODING_REJECT
        if ("i",1) not in left or ("i",4) not in left:
            return MESSAGE_REJECT
        if payload["m"] not in (2,7) or (payload["m"] == 7 and payload["v"] != 22):
            return MESSAGE_REJECT
        return SUFFICIENT
    except (UnicodeDecodeError, InvalidText):
        return PARSE_ERROR
    except Fault:
        return PARSE_ERROR


def classify(data):
    try:
        a = primary(data)
    except Exception:
        a = UNRESOLVED
    try:
        b = reference(data)
    except Exception:
        b = UNRESOLVED
    if a != b:
        return UNRESOLVED, a, b
    return a, a, b


def main():
    doc = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assert doc["schema"] == SCHEMA
    assert doc["program"] == PROGRAM
    assert doc["analysis_role"] == "research_only"
    assert doc["parent_subject"] == PARENT_SUBJECT
    cases = doc["cases"]
    depth_vectors = doc["depth_vectors"]
    assert [c["id"] for c in cases] == [f"C-{i:02d}" for i in range(1,38)]
    assert [v["id"] for v in depth_vectors] == ["D-01", "D-02"]
    for case in cases:
        assert set(case) == {"id","family","hex"}
        raw = bytes.fromhex(case["hex"])
        assert raw
        lowered = json.dumps(case, sort_keys=True).lower()
        assert "expected_verdict" not in lowered
        assert "oracle_verdict" not in lowered
    for vector in depth_vectors:
        assert set(vector) == {"id","family","hex"}
        raw = bytes.fromhex(vector["hex"])
        assert raw
        lowered = json.dumps(vector, sort_keys=True).lower()
        assert "expected_verdict" not in lowered
        assert "oracle_verdict" not in lowered

    census = {}
    corpus = bytearray()
    disagreements = []
    for case in cases:
        raw = bytes.fromhex(case["hex"])
        corpus.extend(case["id"].encode("ascii"))
        corpus.extend(raw)
        outcome, primary_outcome, reference_outcome = classify(raw)
        census[outcome] = census.get(outcome, 0) + 1
        if primary_outcome != reference_outcome:
            disagreements.append((case["id"], primary_outcome, reference_outcome))

    assert not disagreements, disagreements
    assert census == {
        SUFFICIENT: 8,
        MESSAGE_REJECT: 8,
        ENCODING_REJECT: 16,
        PARSE_ERROR: 5,
        UNRESOLVED: 0,
    }, census

    class ExplodingBytes(bytes):
        def __len__(self):
            raise RuntimeError("synthetic unexpected decoder failure")

    assert classify(ExplodingBytes(b"\x84"))[0] == UNRESOLVED

    canonical_float = bytes.fromhex(cases[24]["hex"])
    nonminimal_single = bytes.fromhex(cases[25]["hex"])
    nonminimal_double = bytes.fromhex(cases[26]["hex"])
    canonical_single = bytes.fromhex(cases[35]["hex"])
    canonical_double = bytes.fromhex(cases[36]["hex"])
    assert classify(canonical_float)[0] == SUFFICIENT
    assert classify(nonminimal_single)[0] == ENCODING_REJECT
    assert classify(nonminimal_double)[0] == ENCODING_REJECT
    assert classify(canonical_single)[0] == SUFFICIENT
    assert classify(canonical_double)[0] == SUFFICIENT
    assert classify(bytes.fromhex(cases[34]["hex"]))[0] == PARSE_ERROR

    probe_results = {}
    for vector in depth_vectors:
        raw = bytes.fromhex(vector["hex"])
        primary_probe = run_depth_probe(depth_probe_primary, raw)
        reference_probe = run_depth_probe(depth_probe_reference, raw)
        assert primary_probe == reference_probe, (vector["id"], primary_probe, reference_probe)
        probe_results[vector["id"]] = primary_probe
    assert probe_results == {
        "D-01": DEPTH_PROBE_EXCEEDED,
        "D-02": DEPTH_PROBE_WITHIN_LIMIT,
    }, probe_results

    depth_corpus = bytearray()
    for vector in depth_vectors:
        depth_corpus.extend(vector["id"].encode("ascii"))
        depth_corpus.extend(bytes.fromhex(vector["hex"]))

    print(f"SYM-CIVIC-018-CBOR CASES={len(cases)}")
    print("SYM-CIVIC-018-CORPUS_SHA256=" + hashlib.sha256(bytes(corpus)).hexdigest())
    print("SYM-CIVIC-018-CBOR DEPTH_VECTORS=" + json.dumps(probe_results, sort_keys=True, separators=(",",":")))
    print("SYM-CIVIC-018-DEPTH_CORPUS_SHA256=" + hashlib.sha256(bytes(depth_corpus)).hexdigest())
    print("SYM-CIVIC-018-CBOR DERIVED=" + json.dumps(census, sort_keys=True, separators=(",",":")))
    print("SYM-CIVIC-018-CBOR METAMORPHIC=PASS")
    print("SYM-CIVIC-018-CBOR PASS")


if __name__ == "__main__":
    main()
