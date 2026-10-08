#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import math
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_018_cbor_encoding_v1.json"
SCHEMA = "mycelix.sym-civic.cbor-fp-depth-preflight.v16"
PROGRAM = "SYM-CIVIC-018-CBOR-FP-DEPTH-V5"
PARENT_SUBJECT = "1196889eb2a849ddc359d8d05edf063aa9537699"

if not __debug__:
    raise SystemExit("SYM-CIVIC-018 refuses optimized Python execution; assertion-backed qualification is fail-closed")

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


class ResourceExceeded(Fault):
    pass


def _reject_duplicate_json_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise Fault("duplicate JSON object key")
        result[key] = value
    return result


MAX_DEPTH = 32
MAX_STRING_BYTES = 1024
MAX_CONTAINER_ITEMS = 64
RESOURCE_PROFILE = {"max_depth": 32, "max_string_bytes": 1024, "max_container_items": 64}


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
            total_length = 0
            while True:
                if pos >= len(data):
                    raise Fault("open string")
                if data[pos] == 255:
                    pos += 1
                    break
                child, pos = parse(data, pos, depth + 1)
                if child[0] != ("b" if major == 2 else "t") or child[4]:
                    raise Fault("bad string chunk")
                chunk = child[1] if major == 2 else child[1].encode("utf-8")
                total_length += len(chunk)
                if total_length > MAX_STRING_BYTES:
                    raise ResourceExceeded("string resource bound")
                chunks.append(chunk)
            raw = data[start:pos]
            if major == 2:
                return ("b", b"".join(chunks), False, raw, True, depth), pos
            try:
                return ("t", b"".join(chunks).decode("utf-8"), False, raw, True), pos
            except UnicodeDecodeError as exc:
                raise InvalidText from exc
        if length > MAX_STRING_BYTES:
            raise ResourceExceeded("string resource bound")
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
        if length is not None and length > MAX_CONTAINER_ITEMS:
            raise ResourceExceeded("array resource bound")
        items = []
        if length is None:
            count = 0
            while True:
                if pos >= len(data):
                    raise Fault("open array")
                if data[pos] == 255:
                    pos += 1
                    break
                if count >= MAX_CONTAINER_ITEMS:
                    raise ResourceExceeded("array resource bound")
                item, pos = parse(data, pos, depth + 1)
                items.append(item)
                count += 1
            return ("a", items, False, data[start:pos], True), pos
        for _ in range(length):
            item, pos = parse(data, pos, depth + 1)
            items.append(item)
        return ("a", items, preferred, data[start:pos], False), pos

    if major == 5:
        length, pos, preferred = read_arg(data, pos, ai)
        if length is not None and length > MAX_CONTAINER_ITEMS:
            raise ResourceExceeded("map resource bound")
        pairs = []
        if length is None:
            count = 0
            while True:
                if pos >= len(data):
                    raise Fault("open map")
                if data[pos] == 255:
                    pos += 1
                    break
                if count >= MAX_CONTAINER_ITEMS:
                    raise ResourceExceeded("map resource bound")
                key, pos = parse(data, pos, depth + 1)
                value, pos = parse(data, pos, depth + 1)
                pairs.append((key, value))
                count += 1
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


def nan_deterministic_primary(raw):
    layout = {3: (10, 5), 5: (23, 8), 9: (52, 11)}
    spec = layout.get(len(raw))
    if spec is None:
        return False
    fraction_bits, exponent_bits = spec
    bits = int.from_bytes(raw[1:], "big")
    exponent_mask = (1 << exponent_bits) - 1
    exponent = (bits >> fraction_bits) & exponent_mask
    fraction = bits & ((1 << fraction_bits) - 1)
    if exponent != exponent_mask or fraction == 0:
        return False
    for target_len, target_fraction_bits in ((3, 10), (5, 23)):
        if target_len >= len(raw):
            continue
        shift = fraction_bits - target_fraction_bits
        if fraction & ((1 << shift) - 1) == 0:
            return False
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
        return nan_deterministic_primary(raw)
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
        if keys != sorted(keys):
            raise Fault("map ordering")
        return head(5, len(encoded)) + b"".join(k + v for k, v in encoded)
    if kind == "g":
        return head(6, value[0]) + canonical(value[1])
    if kind == "s":
        if 0 <= value <= 23:
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


def labels_are_valid(pairs):
    return all(key[0] in ("i", "t") for key, _ in pairs)


def node_contains_nan(node):
    kind = node[0]
    if kind == "f":
        raw = node[3]
        fmt = {25: ">e", 26: ">f", 27: ">d"}.get(raw[0] & 31)
        if fmt is None:
            return False
        try:
            return math.isnan(struct.unpack(fmt, raw[1:])[0])
        except struct.error:
            return False
    if kind == "a":
        return any(node_contains_nan(item) for item in node[1])
    if kind == "m":
        return any(node_contains_nan(key) or node_contains_nan(value) for key, value in node[1])
    if kind == "g":
        return node_contains_nan(node[1][1])
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
        if not deterministic(root) or not deterministic(pmap) or not deterministic(unprotected):
            return ENCODING_REJECT
        if not labels_are_valid(pmap[1]) or not labels_are_valid(unprotected[1]):
            return MESSAGE_REJECT
        if dup(pmap[1]) or dup(unprotected[1]):
            return MESSAGE_REJECT
        if set(kid(k) for k, _ in pmap[1]).intersection(set(kid(k) for k, _ in unprotected[1])):
            return MESSAGE_REJECT
        if any(node_contains_nan(value) for _, value in pmap[1]) or any(node_contains_nan(value) for _, value in unprotected[1]):
            return MESSAGE_REJECT
        labels = {kid(k) for k, _ in pmap[1]}
        protected_values = {kid(k): value for k, value in pmap[1]}
        if ("i", 1) not in labels or ("i", 4) not in labels:
            return MESSAGE_REJECT
        if protected_values[("i", 1)][0] not in ("i", "t"):
            return MESSAGE_REJECT
        if protected_values[("i", 4)][0] != "b":
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
                total_length = 0
                while True:
                    if pos >= len(data):
                        raise Fault("ref open string")
                    if data[pos] == 255:
                        pos += 1
                        break
                    child, pos = scan(data, pos, depth + 1)
                    if child["m"] != major or child.get("i"):
                        raise Fault("ref bad string chunk")
                    chunk = child["v"]
                    total_length += len(chunk)
                    if total_length > MAX_STRING_BYTES:
                        raise ResourceExceeded("ref string resource bound")
                    chunks.append(chunk)
                raw = data[start:pos]
                result = {"m":major,"v":b"".join(chunks),"p":False,"r":raw,"i":True,"s":start,"e":pos}
                if major == 2:
                    result["d"] = depth
                return result, pos
            if arg > MAX_STRING_BYTES:
                raise ResourceExceeded("ref string resource bound")
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
            if arg is not None and arg > MAX_CONTAINER_ITEMS:
                raise ResourceExceeded("ref array resource bound")
            items = []
            if arg is None:
                count = 0
                while True:
                    if pos >= len(data):
                        raise Fault("ref open array")
                    if data[pos] == 255:
                        pos += 1
                        break
                    if count >= MAX_CONTAINER_ITEMS:
                        raise ResourceExceeded("ref array resource bound")
                    child, pos = scan(data, pos, depth + 1)
                    items.append(child)
                    count += 1
                return {"m":4,"items":items,"p":False,"i":True,"r":data[start:pos],"s":start,"e":pos}, pos
            for _ in range(arg):
                child, pos = scan(data, pos, depth + 1)
                items.append(child)
            return {"m":4,"items":items,"p":preferred,"r":data[start:pos],"s":start,"e":pos}, pos
        if major == 5:
            if arg is not None and arg > MAX_CONTAINER_ITEMS:
                raise ResourceExceeded("ref map resource bound")
            pairs = []
            if arg is None:
                count = 0
                while True:
                    if pos >= len(data):
                        raise Fault("ref open map")
                    if data[pos] == 255:
                        pos += 1
                        break
                    if count >= MAX_CONTAINER_ITEMS:
                        raise ResourceExceeded("ref map resource bound")
                    key, pos = scan(data, pos, depth + 1)
                    value, pos = scan(data, pos, depth + 1)
                    pairs.append((key, value))
                    count += 1
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


def ref_labels_are_valid(pairs):
    return all(key["m"] in (0, 3) for key, _ in pairs)



REF_FLOAT_LAYOUT = {
    25: {"bytes": 3, "fraction_bits": 10, "exponent_bits": 5, "bias": 15},
    26: {"bytes": 5, "fraction_bits": 23, "exponent_bits": 8, "bias": 127},
    27: {"bytes": 9, "fraction_bits": 52, "exponent_bits": 11, "bias": 1023},
}

def ref_float_fields(raw):
    if not raw:
        return None
    spec = REF_FLOAT_LAYOUT.get(raw[0] & 31)
    if spec is None or len(raw) != spec["bytes"]:
        return None
    bits = int.from_bytes(raw[1:], "big")
    fraction_mask = (1 << spec["fraction_bits"]) - 1
    exponent_mask = (1 << spec["exponent_bits"]) - 1
    fraction = bits & fraction_mask
    exponent = (bits >> spec["fraction_bits"]) & exponent_mask
    sign = (bits >> (spec["fraction_bits"] + spec["exponent_bits"])) & 1
    return spec, sign, exponent, fraction

def ref_nan_deterministic(raw):
    fields = ref_float_fields(raw)
    if fields is None:
        return False
    spec, _, exponent, fraction = fields
    if exponent != (1 << spec["exponent_bits"]) - 1 or fraction == 0:
        return False
    for target_fraction_bits in (10, 23):
        if target_fraction_bits >= spec["fraction_bits"]:
            continue
        shift = spec["fraction_bits"] - target_fraction_bits
        if fraction & ((1 << shift) - 1) == 0:
            return False
    return True

def ref_exact_representable(raw, target_ai):
    source = ref_float_fields(raw)
    target = REF_FLOAT_LAYOUT.get(target_ai)
    if source is None or target is None:
        return False
    spec, _, exponent_field, fraction = source
    target_fraction_bits = target["fraction_bits"]
    target_exponent_bits = target["exponent_bits"]
    target_bias = target["bias"]
    target_exponent_max = (1 << target_exponent_bits) - 2
    target_emin = 1 - target_bias
    target_emax = target_exponent_max - target_bias
    source_exponent_all_ones = (1 << spec["exponent_bits"]) - 1
    if exponent_field == source_exponent_all_ones:
        return True
    if exponent_field == 0 and fraction == 0:
        return True
    if exponent_field == 0:
        mantissa = fraction
        value_exponent = 1 - spec["bias"] - spec["fraction_bits"]
    else:
        mantissa = (1 << spec["fraction_bits"]) | fraction
        value_exponent = exponent_field - spec["bias"] - spec["fraction_bits"]
    trailing = (mantissa & -mantissa).bit_length() - 1
    mantissa >>= trailing
    value_exponent += trailing
    binary_exponent = mantissa.bit_length() - 1 + value_exponent
    if target_emin <= binary_exponent <= target_emax:
        return mantissa.bit_length() <= target_fraction_bits + 1
    if binary_exponent < target_emin:
        quantum_exponent = target_emin - target_fraction_bits
        if value_exponent < quantum_exponent:
            return False
        scaled = mantissa << (value_exponent - quantum_exponent)
        return 1 <= scaled < (1 << target_fraction_bits)
    return False

def ref_float_deterministic(raw):
    fields = ref_float_fields(raw)
    if fields is None:
        return False
    spec, _, exponent, fraction = fields
    all_ones = (1 << spec["exponent_bits"]) - 1
    if exponent == all_ones:
        if fraction == 0:
            return len(raw) == 3
        return ref_nan_deterministic(raw)
    if len(raw) == 3:
        return True
    if ref_exact_representable(raw, 25):
        return False
    if len(raw) == 5:
        return True
    if ref_exact_representable(raw, 26):
        return False
    return True

def ref_node_contains_nan(node):
    if node["m"] == 7:
        fields = ref_float_fields(node["r"])
        if fields is not None:
            spec, _, exponent, fraction = fields
            return exponent == (1 << spec["exponent_bits"]) - 1 and fraction != 0
        return False
    if node["m"] == 4:
        return any(ref_node_contains_nan(item) for item in node["items"])
    if node["m"] == 5:
        return any(ref_node_contains_nan(key) or ref_node_contains_nan(value) for key, value in node["pairs"])
    if node["m"] == 6:
        return ref_node_contains_nan(node["v"][1])
    return False
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
        return keys == sorted(keys)
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
DEPTH_PROBE_UNRESOLVED = "DEPTH_PROBE_UNRESOLVED"

RESOURCE_PROBE_WITHIN_LIMIT = "RESOURCE_PROBE_WITHIN_LIMIT"
RESOURCE_PROBE_EXCEEDED = "RESOURCE_PROBE_EXCEEDED"
RESOURCE_PROBE_UNRESOLVED = "RESOURCE_PROBE_UNRESOLVED"


def resource_probe_primary(data):
    root, end = parse(data)
    if end != len(data):
        raise Fault("resource probe trailing bytes")
    return root


def resource_probe_reference(data):
    root, end = scan(data)
    if end != len(data):
        raise Fault("ref resource probe trailing bytes")
    return root


def run_resource_probe(fn, data):
    try:
        fn(data)
    except ResourceExceeded:
        return RESOURCE_PROBE_EXCEEDED
    except Fault:
        return RESOURCE_PROBE_UNRESOLVED
    except Exception:
        return RESOURCE_PROBE_UNRESOLVED
    return RESOURCE_PROBE_WITHIN_LIMIT




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
    except Fault:
        return DEPTH_PROBE_UNRESOLVED
    except Exception:
        return DEPTH_PROBE_UNRESOLVED
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
        if not ref_deterministic(root) or not ref_deterministic(pmap) or not ref_deterministic(unprotected):
            return ENCODING_REJECT
        if not ref_labels_are_valid(pmap["pairs"]) or not ref_labels_are_valid(unprotected["pairs"]):
            return MESSAGE_REJECT
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
        if any(ref_node_contains_nan(value) for _, value in pmap["pairs"]) or any(ref_node_contains_nan(value) for _, value in unprotected["pairs"]):
            return MESSAGE_REJECT
        if ("i",1) not in left or ("i",4) not in left:
            return MESSAGE_REJECT
        protected_values = {ref_key(k): v for k, v in pmap["pairs"]}
        if protected_values[("i",1)]["m"] not in (0,1,3):
            return MESSAGE_REJECT
        if protected_values[("i",4)]["m"] != 2:
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
    doc = json.loads(MANIFEST.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_json_keys)
    assert doc["schema"] == SCHEMA
    assert doc["program"] == PROGRAM
    assert doc["analysis_role"] == "research_only"
    assert doc["parent_subject"] == PARENT_SUBJECT
    assert doc["resource_profile"] == RESOURCE_PROFILE
    assert type(doc["resource_profile"]) is dict
    assert all(type(value) is int for value in doc["resource_profile"].values())
    assert set(doc) == {"schema","program","parent_subject","analysis_role","cases","depth_vectors","resource_profile","resource_vectors"}
    cases = doc["cases"]
    depth_vectors = doc["depth_vectors"]
    resource_vectors = doc["resource_vectors"]
    assert [c["id"] for c in cases] == [f"C-{i:02d}" for i in range(1,52)]
    assert len({c["hex"] for c in cases}) == len(cases)
    assert [v["id"] for v in depth_vectors] == ["D-01", "D-02", "D-03", "D-04"]
    assert [v["family"] for v in depth_vectors] == ['combined_protected_depth_exceeded','protected_depth_within_limit','exact_depth_limit_32','depth_limit_33_exceeded']
    assert len({v["hex"] for v in depth_vectors}) == len(depth_vectors)
    assert [v["id"] for v in resource_vectors] == [f"R-{i:02d}" for i in range(1,13)]
    assert [v["family"] for v in resource_vectors] == ['definite_byte_string_at_limit','indefinite_byte_string_at_limit','indefinite_byte_string_exceeded','definite_text_string_at_limit','indefinite_text_string_at_limit','indefinite_text_string_exceeded','definite_array_at_limit','indefinite_array_at_limit','indefinite_array_exceeded','definite_map_at_limit','indefinite_map_at_limit','indefinite_map_exceeded']
    assert len({v["hex"] for v in resource_vectors}) == len(resource_vectors)
    all_vectors = cases + depth_vectors + resource_vectors
    assert len({v["hex"] for v in all_vectors}) == len(all_vectors)
    for case in cases:
        assert set(case) == {"id","family","hex"}
        assert isinstance(case["family"], str)
        assert isinstance(case["hex"], str)
        assert case["hex"] == case["hex"].lower()
        assert len(case["hex"]) % 2 == 0
        raw = bytes.fromhex(case["hex"])
        assert raw.hex() == case["hex"]
        assert raw
        lowered = json.dumps(case, sort_keys=True).lower()
        assert "expected_verdict" not in lowered
        assert "oracle_verdict" not in lowered
    for vector in depth_vectors:
        assert set(vector) == {"id","family","hex"}
        assert isinstance(vector["family"], str)
        assert isinstance(vector["hex"], str)
        assert vector["hex"] == vector["hex"].lower()
        assert len(vector["hex"]) % 2 == 0
        raw = bytes.fromhex(vector["hex"])
        assert raw.hex() == vector["hex"]
        assert raw
        lowered = json.dumps(vector, sort_keys=True).lower()
        assert "expected_verdict" not in lowered
        assert "oracle_verdict" not in lowered

    for vector in resource_vectors:
        assert set(vector) == {"id","family","hex"}
        assert isinstance(vector["hex"], str)
        assert vector["hex"] == vector["hex"].lower()
        assert len(vector["hex"]) % 2 == 0
        raw = bytes.fromhex(vector["hex"])
        assert raw.hex() == vector["hex"]
        assert raw
        lowered = json.dumps(vector, sort_keys=True).lower()
        assert "expected_verdict" not in lowered
        assert "oracle_verdict" not in lowered

    census = {
        SUFFICIENT: 0,
        MESSAGE_REJECT: 0,
        ENCODING_REJECT: 0,
        PARSE_ERROR: 0,
        UNRESOLVED: 0,
    }
    corpus = bytearray()
    disagreements = []
    for case in cases:
        raw = bytes.fromhex(case["hex"])
        corpus.extend(case["id"].encode("ascii"))
        corpus.extend(b"\x00")
        corpus.extend(case["family"].encode("utf-8"))
        corpus.extend(b"\x00")
        corpus.extend(raw)
        outcome, primary_outcome, reference_outcome = classify(raw)
        census[outcome] = census.get(outcome, 0) + 1
        if primary_outcome != reference_outcome:
            disagreements.append((case["id"], primary_outcome, reference_outcome))

    assert not disagreements, disagreements
    assert census == {
        SUFFICIENT: 9,
        MESSAGE_REJECT: 13,
        ENCODING_REJECT: 17,
        PARSE_ERROR: 12,
        UNRESOLVED: 0,
    }, census
    assert sum(census.values()) == len(cases)

    class ExplodingBytes(bytes):
        def __len__(self):
            raise RuntimeError("synthetic unexpected decoder failure")

    assert classify(ExplodingBytes(b"\x84"))[0] == UNRESOLVED

    by_family = {case["family"]: bytes.fromhex(case["hex"]) for case in cases}
    assert len(by_family) == len(cases)
    assert [c["family"] for c in cases] == ['canonical_sign1','canonical_sign1_payload_bytes','duplicate_protected_label','duplicate_unprotected_label','float_header_label_application_reject','sign1_wrong_arity','protected_not_bstr','unprotected_not_map','missing_protected_alg','missing_protected_kid','protected_map_order_reversed','unprotected_map_order_reversed','nonminimal_integer_argument','nonminimal_bstr_length','nonminimal_map_length','indefinite_sign1_array','indefinite_protected_map','indefinite_signature_bytes','indefinite_text_key','nonminimal_negative_integer','truncated_item','trailing_bytes','invalid_utf8_text_key','reserved_simple_value_01','canonical_float_half','nonminimal_float_single','nonminimal_float_double','canonical_positive_infinity_half','nonminimal_positive_infinity_single','canonical_negative_infinity_half','nonminimal_negative_infinity_double','canonical_negative_zero_half','nonminimal_negative_zero_single','canonical_nan_application_reject','recursion_depth_exceeded','canonical_float_single','canonical_float_double','float_header_label_application_reject_2','nonminimal_nan_single','nonminimal_nan_double','reserved_simple_value_00','reserved_simple_value_18','reserved_simple_value_1f','oversized_byte_string_resource','oversized_text_string_resource','oversized_array_resource','oversized_map_resource','canonical_simple_value_0','alg_value_wrong_float_type','alg_value_wrong_bstr_type','kid_value_wrong_text_type']
    assert classify(by_family["canonical_float_half"])[0] == SUFFICIENT
    assert classify(by_family["nonminimal_float_single"])[0] == ENCODING_REJECT
    assert classify(by_family["nonminimal_float_double"])[0] == ENCODING_REJECT
    assert classify(by_family["canonical_float_single"])[0] == SUFFICIENT
    assert classify(by_family["canonical_float_double"])[0] == SUFFICIENT
    assert classify(by_family["canonical_simple_value_0"])[0] == SUFFICIENT
    assert canonical(("s", 0, True, b"\xe0", False)) == b"\xe0"
    assert canonical(("s", 19, True, b"\xf3", False)) == b"\xf3"
    assert canonical(("s", 23, True, b"\xf7", False)) == b"\xf7"
    assert ref_float_deterministic(bytes.fromhex("f93e00"))
    assert not ref_float_deterministic(bytes.fromhex("fa3fc00000"))
    assert not ref_float_deterministic(bytes.fromhex("fb3ff8000000000000"))
    assert ref_float_deterministic(bytes.fromhex("fa45ad9c00"))
    assert ref_float_deterministic(bytes.fromhex("fa00000001"))
    assert not ref_float_deterministic(bytes.fromhex("fa800000"))
    assert not ref_float_deterministic(bytes.fromhex("fa7f800000"))
    assert ref_float_deterministic(bytes.fromhex("f97c00"))
    assert not ref_float_deterministic(bytes.fromhex("fb7ff0000000000000"))
    assert classify(by_family["float_header_label_application_reject"])[0] == MESSAGE_REJECT
    assert classify(by_family["canonical_nan_application_reject"])[0] == MESSAGE_REJECT
    alternate_float_header_label = bytes.fromhex("8446a20126044101a1f97e0001f64100")
    assert classify(alternate_float_header_label)[0] == MESSAGE_REJECT
    assert classify(by_family["canonical_nan_application_reject"])[0] == MESSAGE_REJECT
    alternate_unprotected_nan = bytes.fromhex("8446a20126044101a16178f97e00f64100")
    assert classify(alternate_unprotected_nan)[0] == MESSAGE_REJECT
    assert classify(by_family["nonminimal_nan_single"])[0] == ENCODING_REJECT
    assert classify(by_family["nonminimal_nan_double"])[0] == ENCODING_REJECT
    assert classify(by_family["alg_value_wrong_float_type"])[0] == MESSAGE_REJECT
    assert classify(by_family["alg_value_wrong_bstr_type"])[0] == MESSAGE_REJECT
    assert classify(by_family["kid_value_wrong_text_type"])[0] == MESSAGE_REJECT
    assert classify(bytes.fromhex("844aa3012604410118044101a0f64100"))[0] == ENCODING_REJECT
    assert classify(by_family["duplicate_protected_label"])[0] == MESSAGE_REJECT
    for family in (
        "recursion_depth_exceeded",
        "reserved_simple_value_01",
        "reserved_simple_value_00",
        "reserved_simple_value_18",
        "reserved_simple_value_1f",
        "oversized_byte_string_resource",
        "oversized_text_string_resource",
        "oversized_array_resource",
        "oversized_map_resource",
    ):
        assert classify(by_family[family])[0] == PARSE_ERROR
    assert len(by_family["oversized_byte_string_resource"]) == 1028
    assert len(by_family["oversized_text_string_resource"]) == 1028

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
        "D-03": DEPTH_PROBE_WITHIN_LIMIT,
        "D-04": DEPTH_PROBE_EXCEEDED,
    }, probe_results

    def synthetic_probe_fault(_data):
        raise Fault("synthetic probe fault")

    def synthetic_probe_exception(_data):
        raise RuntimeError("synthetic probe exception")

    assert run_depth_probe(synthetic_probe_fault, b"") == DEPTH_PROBE_UNRESOLVED
    assert run_resource_probe(synthetic_probe_fault, b"") == RESOURCE_PROBE_UNRESOLVED
    assert run_depth_probe(synthetic_probe_exception, b"") == DEPTH_PROBE_UNRESOLVED
    assert run_resource_probe(synthetic_probe_exception, b"") == RESOURCE_PROBE_UNRESOLVED

    resource_probe_results = {}
    for vector in resource_vectors:
        raw = bytes.fromhex(vector["hex"])
        primary_probe = run_resource_probe(resource_probe_primary, raw)
        reference_probe = run_resource_probe(resource_probe_reference, raw)
        assert primary_probe == reference_probe, (vector["id"], primary_probe, reference_probe)
        resource_probe_results[vector["id"]] = primary_probe

    assert resource_probe_results == {
        "R-01": RESOURCE_PROBE_WITHIN_LIMIT,
        "R-02": RESOURCE_PROBE_WITHIN_LIMIT,
        "R-03": RESOURCE_PROBE_EXCEEDED,
        "R-04": RESOURCE_PROBE_WITHIN_LIMIT,
        "R-05": RESOURCE_PROBE_WITHIN_LIMIT,
        "R-06": RESOURCE_PROBE_EXCEEDED,
        "R-07": RESOURCE_PROBE_WITHIN_LIMIT,
        "R-08": RESOURCE_PROBE_WITHIN_LIMIT,
        "R-09": RESOURCE_PROBE_EXCEEDED,
        "R-10": RESOURCE_PROBE_WITHIN_LIMIT,
        "R-11": RESOURCE_PROBE_WITHIN_LIMIT,
        "R-12": RESOURCE_PROBE_EXCEEDED,
    }, resource_probe_results

    resource_corpus = bytearray()
    for vector in resource_vectors:
        resource_corpus.extend(vector["id"].encode("ascii"))
        resource_corpus.extend(b"\x00")
        resource_corpus.extend(vector["family"].encode("utf-8"))
        resource_corpus.extend(b"\x00")
        resource_corpus.extend(bytes.fromhex(vector["hex"]))

    depth_corpus = bytearray()
    for vector in depth_vectors:
        depth_corpus.extend(vector["id"].encode("ascii"))
        depth_corpus.extend(b"\x00")
        depth_corpus.extend(vector["family"].encode("utf-8"))
        depth_corpus.extend(b"\x00")
        depth_corpus.extend(bytes.fromhex(vector["hex"]))

    print(f"SYM-CIVIC-018-CBOR CASES={len(cases)}")
    print("SYM-CIVIC-018-CORPUS_SHA256=" + hashlib.sha256(bytes(corpus)).hexdigest())
    print("SYM-CIVIC-018-CBOR DEPTH_VECTORS=" + json.dumps(probe_results, sort_keys=True, separators=(",",":")))
    print("SYM-CIVIC-018-DEPTH_CORPUS_SHA256=" + hashlib.sha256(bytes(depth_corpus)).hexdigest())
    print("SYM-CIVIC-018-CBOR RESOURCE_VECTORS=" + json.dumps(resource_probe_results, sort_keys=True, separators=(",",":")))
    print("SYM-CIVIC-018-RESOURCE_CORPUS_SHA256=" + hashlib.sha256(bytes(resource_corpus)).hexdigest())
    print("SYM-CIVIC-018-CBOR DERIVED=" + json.dumps(census, sort_keys=True, separators=(",",":")))
    print("SYM-CIVIC-018-CBOR METAMORPHIC=PASS")
    print("SYM-CIVIC-018-CBOR PASS")


if __name__ == "__main__":
    main()
