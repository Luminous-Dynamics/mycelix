#!/usr/bin/env python3
from __future__ import annotations
import copy
import hashlib
import json
from urllib.parse import urlsplit, urlunsplit

ROOT = __import__("pathlib").Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_016_effective_witness_set_v1.json"

PROGRAM = "SYM-CIVIC-016"
SCHEMA = "mycelix.sym-civic.effective-witness-set-preflight.v1"
REJECT = "REJECT_EFFECTIVE_WITNESS_PROVENANCE"
SUFFICIENT = "EFFECTIVE_WITNESS_SET_SUFFICIENT"
UNSAT = "EFFECTIVE_WITNESS_SET_UNSATISFIABLE"
UNRESOLVED = "EFFECTIVE_WITNESS_SET_UNRESOLVED"

PARENT_SUBJECT = "ed66bf43ac498316da53386752e9db7efb07a0ab"
BASE_TIME = "2026-10-04T00:00:00Z"

CRITICAL = {"effective-witness-v1": {"mode": "INTERSECTION_MAX_QUORUM"}}
POLICY = {"required_quorum_floor": 2}
