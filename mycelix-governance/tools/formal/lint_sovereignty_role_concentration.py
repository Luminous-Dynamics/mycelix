#!/usr/bin/env python3
"""Candidate-only semantic linter for role-concentration formal artifacts."""
from __future__ import annotations
import json
import re
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
MODEL=ROOT/"mycelix-governance/specs/ArtificialSovereigntyRoleConcentrationV1.tla"
CFG=ROOT/"mycelix-governance/specs/ArtificialSovereigntyRoleConcentrationV1.cfg"
NEG=ROOT/"mycelix-governance/specs/ArtificialSovereigntyRoleConcentrationV1NegativeControls.tla"
ALLOY=ROOT/"mycelix-governance/specs/alloy/ArtificialSovereigntyRoleConcentrationV1.als"
REF=ROOT/"mycelix-governance/tools/formal/sovereignty_role_concentration_reference_explorer.py"
PROFILE=ROOT/"docs/qualification/SOVEREIGNTY_ROLE_CONCENTRATION_FORMAL_QUALIFICATION_PROFILE_V2.json"

def fail(msg): raise SystemExit("ROLE_FORMAL_LINT_FAIL: "+msg)

model=MODEL.read_text(encoding="utf-8")
cfg=CFG.read_text(encoding="utf-8")
neg=NEG.read_text(encoding="utf-8")
alloy=ALLOY.read_text(encoding="utf-8")
ref=REF.read_text(encoding="utf-8")
profile=json.loads(PROFILE.read_text(encoding="utf-8"))

if "externalReview =" in model or "externalReview'" in model:
    fail("mutable externalReview boolean remains; use reviewer identities")
for token in ("IndependentReviewRecorded(s)","ReviewerRoleDisjointness","reviewer # s","reviewer \\notin roleHolder[role]"):
    if token not in model:
        fail("TLA missing required independence token: "+token)

for token in ("RoleConcentrationRequiresFinding","FullControlRequiresIndependentExternalReview","ReviewerRoleDisjointness"):
    if token not in cfg:
        fail("TLA config omits "+token)

expected_controls=set(profile["models"]["negative_tla"]["controls"])
actual_controls=set(re.findall(r'Control = "([^"]+)"', neg))
# Controls are generated in the profile, not in the source as constants; inspect branches instead.
branch_controls=set(re.findall(r'Control = "([^"]+)" THEN', neg))
if branch_controls != expected_controls:
    fail(f"negative-control set mismatch: {branch_controls!r} != {expected_controls!r}")

exacts=["BadAssignSecondWithoutFinding","BadFullControlWithoutIndependentReview","BadSelfReviewFullControl",
        "BadSameRoleReviewFullControl","BadAssignRoleToActiveReviewer"]
for name in exacts:
    if name not in neg: fail("negative module missing "+name)

# Detect duplicate field selectors inside individual TLA+ EXCEPT expressions.
for block in re.findall(r'[roleHolder EXCEPT([sS]*?)]', neg):
    selectors=re.findall(r'!\[([A-Za-z][A-Za-z0-9_]*)\]', block)
    if len(selectors)!=len(set(selectors)):
        fail("duplicate roleHolder EXCEPT selector: "+repr(selectors))

for token in (
    "externalReviewers: set Subject",
    "pred independentReview",
    "no r.roles",
    "SelfReviewOnlyFullControl",
    "SameRoleReviewerFullControl",
    "ReviewerRoleDriftState",
    "fact ReviewerRoleDisjointness",
):
    if token not in alloy:
        fail("Alloy missing required independence token: "+token)

for label in profile["models"]["alloy"]["expected_unsat_runs"]:
    if f"run {label}" not in alloy:
        fail("Alloy missing expected UNSAT run "+label)
for label in profile["models"]["alloy"]["expected_unsat_checks"]:
    if f"check {label}" not in alloy:
        fail("Alloy missing expected UNSAT check "+label)

for token in (
    "def independent_review",
    "all(reviewer not in holders[role] for role in ROLES)",
    "reviewer-role-drift",
    "CANONICAL PASS",
):
    if token not in ref:
        fail("reference oracle missing "+token)

print("ROLE FORMAL LINT PASS: independence semantics, controls, EXCEPT uniqueness, and oracle crosswalk aligned")
print("NON-AUTHORITATIVE: static candidate preflight only")
