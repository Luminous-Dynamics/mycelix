#!/usr/bin/env python3
from __future__ import annotations
import json
import sys
from pathlib import Path

class DuplicateKey(ValueError):
    pass

def pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise DuplicateKey(key)
        result[key] = value
    return result

def reject_int(token):
    if token == "-0":
        raise ValueError("negative zero")
    return int(token)

def reject_float(token):
    if token in ("-0.0", "-0e0", "-0E0"):
        raise ValueError("negative zero")
    return float(token)

def reject_constant(token):
    raise ValueError("non-finite constant: " + token)

def main():
    if len(sys.argv) != 2:
        print("usage: verify_continual_adaptation_raw.py CORPUS.json", file=sys.stderr)
        return 2
    corpus = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    failures = []
    for case in corpus["cases"]:
        accepted = True
        try:
            json.loads(
                case["raw"],
                object_pairs_hook=pairs,
                parse_int=reject_int,
                parse_float=reject_float,
                parse_constant=reject_constant,
            )
        except (json.JSONDecodeError, DuplicateKey, ValueError, TypeError):
            accepted = False
        actual = "accept" if accepted else "reject"
        if actual != case["expected"]:
            failures.append((case["case_id"], case["expected"], actual))
    print(f"cases={len(corpus['cases'])} failures={len(failures)}")
    for failure in failures:
        print("FAIL", failure)
    return 1 if failures else 0

if __name__ == "__main__":
    raise SystemExit(main())
