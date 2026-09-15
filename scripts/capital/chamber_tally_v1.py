#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import stewardship_decision_gate_v1 as g1

PROFILE_VERSION = "mycelix-chamber-tally-v1"
RECEIPT_VERSION = "mycelix-chamber-tally-receipt-v1"
VOTES = {"APPROVE", "REJECT", "ABSTAIN"}
RECUSAL_SEMANTICS = "EXCLUDE_FROM_ELIGIBLE_AND_TALLY"

PROFILE_KEYS = {
    "profile_version", "project_id", "governance_profile_sha256", "eligibility_registry_id",
    "designated_registry_epoch", "recusal_semantics", "max_participants",
}
CONTEXT_KEYS = {
    "project_id", "governance_profile_sha256", "decision_id", "action_code", "declared_class",
}
SNAPSHOT_KEYS = {
    "registry_id", "registry_epoch", "project_id", "governance_profile_sha256", "chambers", "evidence_ref",
}
BALLOT_KEYS = {
    "ballot_id", "project_id", "governance_profile_sha256", "decision_id", "action_code", "registry_id",
    "registry_epoch", "chamber", "participant_ref", "vote", "evidence_ref",
}
CONFLICT_KEYS = {"participant_ref", "chamber", "conflict_code", "disclosed", "evidence_ref"}
CASE_KEYS = {"governance_profile", "tally_profile", "decision_context", "eligibility_snapshot", "ballots", "conflicts"}


class TallyError(ValueError):
    pass


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def sha256_hex(value: Any) -> str:
    raw = value if isinstance(value, (bytes, bytearray)) else canonical_bytes(value)
    return hashlib.sha256(raw).hexdigest()


def _exact(obj: dict[str, Any], expected: set[str], label: str) -> None:
    got = set(obj)
    if got != expected:
        raise TallyError(f"{label} keys mismatch missing={sorted(expected-got)} unknown={sorted(got-expected)}")


def _str(v: Any, label: str) -> str:
    if not isinstance(v, str) or not v or len(v) > 256:
        raise TallyError(f"invalid {label}")
    return v


def _int(v: Any, label: str, maximum: int) -> int:
    if type(v) is not int or v < 0 or v > maximum:
        raise TallyError(f"invalid {label}")
    return v


def validate_tally_profile(profile: dict[str, Any], governance_profile: dict[str, Any]) -> None:
    _exact(profile, PROFILE_KEYS, "tally_profile")
    if profile["profile_version"] != PROFILE_VERSION:
        raise TallyError("unsupported tally profile")
    g1.validate_profile(governance_profile)
    if profile["project_id"] != governance_profile["project_id"]:
        raise TallyError("tally/governance project mismatch")
    if profile["governance_profile_sha256"] != g1.sha256_hex(governance_profile):
        raise TallyError("governance profile substitution")
    _str(profile["eligibility_registry_id"], "eligibility_registry_id")
    _int(profile["designated_registry_epoch"], "designated_registry_epoch", 10**12)
    if profile["recusal_semantics"] != RECUSAL_SEMANTICS:
        raise TallyError("unsupported recusal semantics")
    if _int(profile["max_participants"], "max_participants", 10**9) == 0:
        raise TallyError("max_participants must be positive")


def _threshold(n: int, d: int, ppm: int) -> bool:
    return d > 0 and n * g1.PPM >= ppm * d


@dataclass(frozen=True)
class Result:
    value: dict[str, Any]
    def receipt(self) -> dict[str, Any]: return self.value


def qualify(case: dict[str, Any]) -> Result:
    if not isinstance(case, dict):
        raise TallyError("case must be object")
    _exact(case, CASE_KEYS, "case")
    gov = case["governance_profile"]
    tp = case["tally_profile"]
    validate_tally_profile(tp, gov)
    tp_sha = sha256_hex(tp)

    ctx = case["decision_context"]
    if not isinstance(ctx, dict): raise TallyError("invalid decision_context")
    _exact(ctx, CONTEXT_KEYS, "decision_context")
    if ctx["project_id"] != tp["project_id"]: raise TallyError("decision project substitution")
    if ctx["governance_profile_sha256"] != tp["governance_profile_sha256"]: raise TallyError("decision profile substitution")
    decision_id = _str(ctx["decision_id"], "decision_id")
    action_code = _str(ctx["action_code"], "action_code")
    if action_code not in gov["actions"]: raise TallyError("unknown action code")
    rule = gov["actions"][action_code]
    if ctx["declared_class"] != rule["decision_class"]: raise TallyError("decision class substitution")

    snapshot = case["eligibility_snapshot"]
    if not isinstance(snapshot, dict): raise TallyError("invalid eligibility_snapshot")
    _exact(snapshot, SNAPSHOT_KEYS, "eligibility_snapshot")
    if snapshot["registry_id"] != tp["eligibility_registry_id"]: raise TallyError("registry substitution")
    if snapshot["registry_epoch"] != tp["designated_registry_epoch"]: raise TallyError("stale eligibility epoch")
    if snapshot["project_id"] != tp["project_id"]: raise TallyError("snapshot project substitution")
    if snapshot["governance_profile_sha256"] != tp["governance_profile_sha256"]: raise TallyError("snapshot profile substitution")
    _str(snapshot["evidence_ref"], "snapshot.evidence_ref")
    chambers = snapshot["chambers"]
    if not isinstance(chambers, dict): raise TallyError("invalid snapshot chambers")
    if set(chambers) != set(gov["chambers"]): raise TallyError("snapshot chamber set mismatch")

    max_participants = tp["max_participants"]
    participant_chamber: dict[str, str] = {}
    chamber_members: dict[str, set[str]] = {}
    for chamber in gov["chambers"]:
        members = chambers[chamber]
        if not isinstance(members, list) or len(members) > max_participants:
            raise TallyError("invalid member list")
        seen: set[str] = set()
        for participant in members:
            participant = _str(participant, "participant_ref")
            if participant in seen: raise TallyError("duplicate participant in chamber")
            if participant in participant_chamber: raise TallyError("participant appears in multiple chambers")
            seen.add(participant); participant_chamber[participant] = chamber
        chamber_members[chamber] = seen

    required_conflicts = set(rule["required_recusal_conflicts"])
    conflicts = case["conflicts"]
    if not isinstance(conflicts, list) or len(conflicts) > max_participants:
        raise TallyError("invalid conflicts")
    seen_conflicts: set[tuple[str,str]] = set()
    recused: dict[str, set[str]] = {c:set() for c in gov["chambers"]}
    conflict_commit_records: list[dict[str, Any]] = []
    for rec in conflicts:
        if not isinstance(rec, dict): raise TallyError("invalid conflict record")
        _exact(rec, CONFLICT_KEYS, "conflict")
        participant = _str(rec["participant_ref"], "conflict.participant_ref")
        chamber = _str(rec["chamber"], "conflict.chamber")
        code = _str(rec["conflict_code"], "conflict_code")
        disclosed = rec["disclosed"]
        if type(disclosed) is not bool: raise TallyError("invalid conflict.disclosed")
        _str(rec["evidence_ref"], "conflict.evidence_ref")
        if code not in gov["conflict_codes"]: raise TallyError("unknown conflict code")
        if participant not in participant_chamber: raise TallyError("conflict participant not eligible")
        if participant_chamber[participant] != chamber: raise TallyError("conflict chamber substitution")
        key=(participant,code)
        if key in seen_conflicts: raise TallyError("duplicate conflict identity")
        seen_conflicts.add(key)
        if code in required_conflicts:
            if not disclosed: raise TallyError("required conflict not disclosed")
            recused[chamber].add(participant)
        conflict_commit_records.append(dict(rec))
    conflict_commit_records.sort(key=lambda r:(r["chamber"],r["participant_ref"],r["conflict_code"]))

    ballots = case["ballots"]
    if not isinstance(ballots, list) or len(ballots) > max_participants:
        raise TallyError("invalid ballots")
    seen_ballot_ids:set[str]=set(); seen_participants:set[str]=set(); valid_ballots: list[dict[str,Any]]=[]
    for ballot in ballots:
        if not isinstance(ballot,dict): raise TallyError("invalid ballot")
        _exact(ballot,BALLOT_KEYS,"ballot")
        ballot_id=_str(ballot["ballot_id"],"ballot_id")
        if ballot_id in seen_ballot_ids: raise TallyError("duplicate ballot id")
        seen_ballot_ids.add(ballot_id)
        if ballot["project_id"] != tp["project_id"]: raise TallyError("ballot project substitution")
        if ballot["governance_profile_sha256"] != tp["governance_profile_sha256"]: raise TallyError("ballot profile substitution")
        if ballot["decision_id"] != decision_id or ballot["action_code"] != action_code: raise TallyError("ballot decision substitution")
        if ballot["registry_id"] != tp["eligibility_registry_id"] or ballot["registry_epoch"] != tp["designated_registry_epoch"]:
            raise TallyError("ballot registry currentness failure")
        chamber=_str(ballot["chamber"],"ballot.chamber")
        participant=_str(ballot["participant_ref"],"ballot.participant_ref")
        if participant not in participant_chamber: raise TallyError("ineligible ballot")
        if participant_chamber[participant] != chamber: raise TallyError("cross-chamber ballot")
        if participant in seen_participants: raise TallyError("participant voted more than once")
        seen_participants.add(participant)
        if participant in recused[chamber]: raise TallyError("recused participant submitted ballot")
        if ballot["vote"] not in VOTES: raise TallyError("invalid vote value")
        _str(ballot["evidence_ref"],"ballot.evidence_ref")
        valid_ballots.append(dict(ballot))
    valid_ballots.sort(key=lambda b:(b["chamber"],b["participant_ref"],b["ballot_id"]))

    required_chambers=set(rule["required_chambers"])
    chamber_results:dict[str,Any]={}; blockers:list[str]=[]
    for chamber in sorted(required_chambers):
        effective_eligible = chamber_members[chamber] - recused[chamber]
        cb = [b for b in valid_ballots if b["chamber"] == chamber]
        participating=len(cb)
        approvals=sum(1 for b in cb if b["vote"]=="APPROVE")
        rejects=sum(1 for b in cb if b["vote"]=="REJECT")
        abstains=sum(1 for b in cb if b["vote"]=="ABSTAIN")
        thresholds=rule["required_chambers"][chamber]
        quorum_pass=_threshold(participating,len(effective_eligible),thresholds["quorum_ppm"])
        approval_pass=_threshold(approvals,participating,thresholds["approval_ppm"])
        if not quorum_pass: blockers.append(f"QUORUM_FAIL:{chamber}")
        if not approval_pass: blockers.append(f"APPROVAL_FAIL:{chamber}")
        chamber_results[chamber]={
            "eligible_count":len(effective_eligible),"participating_count":participating,"approval_count":approvals,
            "reject_count":rejects,"abstain_count":abstains,"recused_count":len(recused[chamber]),
            "quorum_pass":quorum_pass,"approval_pass":approval_pass,
        }

    normalized_snapshot = dict(snapshot)
    normalized_snapshot["chambers"] = {c: sorted(chamber_members[c]) for c in sorted(chamber_members)}
    receipt={
        "receipt_version":RECEIPT_VERSION,"profile_version":PROFILE_VERSION,"project_id":tp["project_id"],
        "tally_profile_sha256":tp_sha,"governance_profile_sha256":tp["governance_profile_sha256"],
        "decision_id":decision_id,"action_code":action_code,"decision_class":rule["decision_class"],
        "eligibility_snapshot_sha256":sha256_hex(normalized_snapshot),"ballot_set_sha256":sha256_hex(valid_ballots),
        "conflict_set_sha256":sha256_hex(conflict_commit_records),"chamber_results":chamber_results,
        "recusal_semantics":RECUSAL_SEMANTICS,"blockers":sorted(blockers),
        "chamber_threshold_state":"PASS" if not blockers else "BLOCKED",
        "governance_authorization_established":False,"identity_authenticity_established":False,
        "democratic_legitimacy_established":False,
        "nonclaims":[
            "chamber tally PASS is not overall governance authorization",
            "pseudonymous participant references are not real-world identity authentication",
            "tally reconstruction is not ballot secrecy or coercion resistance",
            "absence of a supplied conflict record is not proof that no real-world conflict exists",
            "tally reconstruction is not democratic legitimacy or legal validity",
        ],
    }
    return Result(receipt)


def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument("case",type=Path); ap.add_argument("--receipt-out",type=Path); args=ap.parse_args()
    case=json.loads(args.case.read_text(encoding="utf-8")); receipt=qualify(case).receipt(); out=canonical_bytes(receipt)
    if args.receipt_out: args.receipt_out.write_bytes(out)
    else: print(out.decode(),end="")
    return 0

if __name__=="__main__": raise SystemExit(main())
