#!/usr/bin/env python3
"""Independent, semantic-only reference evaluator for temporal reconciliation."""
from enum import Enum

class Comparability(str, Enum):
    COMPARABLE = "comparable"
    INCOMPARABLE = "incomparable"
    UNKNOWN = "unknown"

class Compatibility(str, Enum):
    COMPATIBLE = "compatible"
    INCOMPATIBLE = "incompatible"
    UNKNOWN = "unknown"

class Classification(str, Enum):
    COEXISTENT = "coexistent"
    CONFLICTING = "conflicting"
    SEQUENTIAL = "sequential"
    INCOMPARABLE = "incomparable"
    INDETERMINATE = "indeterminate"
    SUPERSEDED = "superseded"

def overlap(a_start, a_end, b_start, b_end):
    if None in (a_start, a_end, b_start, b_end):
        return None
    if a_end < a_start or b_end < b_start:
        raise ValueError("invalid interval")
    return a_start <= b_end and b_start <= a_end

def classify(comparability, compatibility, a_start, a_end, b_start, b_end,
             explicitly_superseded=False, disputed=False):
    c = Comparability(comparability)
    k = Compatibility(compatibility)
    if explicitly_superseded:
        return {"classification": Classification.SUPERSEDED.value, "disputed": False}
    if c is Comparability.INCOMPARABLE:
        return {"classification": Classification.INCOMPARABLE.value, "disputed": False}
    if c is Comparability.UNKNOWN or k is Compatibility.UNKNOWN:
        return {"classification": Classification.INDETERMINATE.value, "disputed": False}
    if k is Compatibility.COMPATIBLE:
        return {"classification": Classification.COEXISTENT.value, "disputed": False}
    temporal_overlap = overlap(a_start, a_end, b_start, b_end)
    if temporal_overlap is None:
        return {"classification": Classification.INDETERMINATE.value, "disputed": False}
    return {
        "classification": Classification.CONFLICTING.value if temporal_overlap else Classification.SEQUENTIAL.value,
        "disputed": bool(disputed and temporal_overlap),
    }

def self_test():
    assert classify("comparable","compatible",0,5,3,8)["classification"] == "coexistent"
    assert classify("comparable","incompatible",0,5,3,8)["classification"] == "conflicting"
    assert classify("comparable","incompatible",0,2,3,5)["classification"] == "sequential"
    assert classify("comparable","incompatible",0,None,3,5)["classification"] == "indeterminate"
    assert classify("incomparable","incompatible",0,5,3,8)["classification"] == "incomparable"
    assert classify("unknown","incompatible",0,5,3,8)["classification"] == "indeterminate"
    assert classify("comparable","unknown",0,5,3,8)["classification"] == "indeterminate"
    assert classify("comparable","incompatible",0,5,3,8,True)["classification"] == "superseded"
    disputed = classify("comparable","incompatible",0,5,3,8,False,True)
    assert disputed == {"classification":"conflicting","disputed":True}
    assert classify("comparable","compatible",None,None,3,8)["classification"] == "coexistent"
    assert classify("incomparable","incompatible",0,5,3,8)["classification"] == "incomparable"
    assert classify("comparable","incompatible",0,5,3,8)["classification"] == "conflicting"
    try:
        overlap(5, 1, 0, 2)
    except ValueError:
        pass
    else:
        raise AssertionError("invalid interval accepted")
    print("temporal reconciliation reference: 12 vectors + invalid interval PASS")

if __name__ == "__main__":
    self_test()
