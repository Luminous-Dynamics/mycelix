#!/usr/bin/env python3
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sup_civ_002c_revision_current_state.json"
DOC = ROOT / "mycelix-workspace/docs/civic-resilience/SUP_CIV_002C_REVISION_CURRENT_STATE_V1.md"

d = json.loads(MANIFEST.read_text())
doc = DOC.read_text()

assert d["schema"] == "mycelix.support-civic.revision-current-state.v1"
assert d["program"] == "SUP-CIV-002C"
assert d["parent_subject"] == "1733cb6d6e281e16530335c118d951ad939d45d7"
assert d["parent_qualification_required"] is True
assert d["runtime_owner"] == "Deferred"

assert d["current_state_results"] == [
    "Absent",
    "LegacyUnmigrated",
    "UniqueAdmittedHead",
    "Conflict",
    "InsufficientLineageEvidence",
    "InvalidLineage",
    "RetiredOrDeletedUnderProfile",
]

expected_lineage_requirements = {
    "same_stable_ticket_identity",
    "actual_valid_holochain_update_of_declared_parent",
    "admitted_typed_mutation",
    "qualified_write_set_and_transition",
    "immutable_identity_fields_preserved",
    "mutation_provenance_bound",
    "declared_expected_parent_matches",
}
assert set(d["lineage_edge_requirements"]) == expected_lineage_requirements

head = d["head_policy"]
assert head["derive_from_admitted_graph"] is True
assert head["unique_head_result"] == "UniqueAdmittedHead"
assert head["multiple_heads_result"] == "Conflict"
for forbidden in [
    "timestamp_auto_winner",
    "action_sequence_auto_winner",
    "hash_order_auto_winner",
    "index_membership_auto_winner",
]:
    assert head[forbidden] is False

assert d["conflict_history_preserved"] is True
assert d["index_is_source_of_truth"] is False
assert d["delete_is_closed"] is False
assert d["delete_is_administrative_finality"] is False

required_non_equivalences = {
    "HolochainUpdate != AdmittedSemanticRevision",
    "LatestRecord != ProvenCurrentState",
    "GetEntryResult != ProvenCurrentState",
    "NewestTimestamp != CurrentRevisionAuthority",
    "HighestActionSequence != CurrentRevisionAuthority",
    "NoVisibleCompetingHead != NoCompetingHeadExistsAnywhere",
    "UniqueAdmittedHead != UniversalRealWorldCurrentTruth",
    "ConcurrentAdmittedHeads != AutomaticWinner",
    "ConflictResolution != DeleteHistory",
    "IndexMembership != AdmittedOperationalTicketState",
    "StatusLink != CurrentStatusProof",
    "DeleteAction != TicketStatus::Closed",
    "DeleteAction != AdministrativeFinality",
}
assert required_non_equivalences.issubset(set(d["non_equivalences"]))

expected_cases = {
    "concurrent_children_yield_conflict",
    "newest_timestamp_does_not_auto_win",
    "highest_action_sequence_does_not_auto_win",
    "cross_ticket_lineage_invalid",
    "legacy_whole_record_update_not_auto_admitted",
    "identity_field_mutation_invalid",
    "forbidden_transition_invalid",
    "missing_parent_insufficient_evidence",
    "incomplete_cut_not_global_no_conflict_claim",
    "conflict_resolution_retains_heads",
    "stale_index_does_not_override_lineage",
    "delete_not_closed_or_admin_finality",
}
assert set(d["required_runtime_cases"]) == expected_cases

for phrase in [
    "HolochainUpdate != AdmittedSemanticRevision",
    "ConcurrentAdmittedHeads -> Conflict",
    "NoVisibleCompetingHead != NoCompetingHeadExistsAnywhere",
    "UniqueAdmittedHead != UniversalRealWorldCurrentTruth",
    "IndexMembership != AdmittedOperationalTicketState",
    "DeleteAction != AdministrativeFinality",
    "InsufficientLineageEvidence",
    "InvalidLineage",
]:
    assert phrase in doc, phrase

print("SUP-CIV-002C semantic contract: PASS")
