#!/usr/bin/env python3
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sup_civ_002a_typed_ticket_mutation.json"
DOC = ROOT / "mycelix-workspace/docs/civic-resilience/SUP_CIV_002A_TYPED_TICKET_MUTATION_V1.md"

d = json.loads(MANIFEST.read_text())
doc = DOC.read_text()

assert d["schema"] == "mycelix.support-civic.typed-ticket-mutation.v1"
assert d["program"] == "SUP-CIV-002A"
assert d["parent_subject"] == "9a5148b9927c630d4073d486f464824fb429d8c6"
assert d["parent_qualification_required"] is True
assert d["runtime_owner"] == "Deferred"

expected_commands = [
    "StartWork",
    "RequestUserInput",
    "ResumeWork",
    "MarkOperationallyResolved",
    "CloseOperationalTicket",
    "ReopenOperationalTicket",
    "AssignTicket",
    "ChangeOperationalPriority",
    "ChangeSupportCategory",
    "AmendDescriptiveMetadata",
]
assert d["command_vocabulary"] == expected_commands
assert d["forbidden_generic_commands"] == [
    "SetStatus",
    "ReplaceTicket",
    "PatchArbitraryFields",
    "CallerSelectedToStatus",
]

expected_transitions = [
    ["Open", "StartWork", "InProgress"],
    ["Open", "RequestUserInput", "AwaitingUser"],
    ["InProgress", "RequestUserInput", "AwaitingUser"],
    ["AwaitingUser", "ResumeWork", "InProgress"],
    ["Open", "MarkOperationallyResolved", "Resolved"],
    ["InProgress", "MarkOperationallyResolved", "Resolved"],
    ["AwaitingUser", "MarkOperationallyResolved", "Resolved"],
    ["Resolved", "CloseOperationalTicket", "Closed"],
    ["Resolved", "ReopenOperationalTicket", "InProgress"],
    ["Closed", "ReopenOperationalTicket", "InProgress"],
]
assert d["transition_relation"] == expected_transitions
assert d["default_transition_policy"] == "deny_unlisted"

assert d["immutable_identity_bound_fields"] == ["requester", "created_at"]
assert set(d["historical_provenance_fields"]) == {"title", "description", "category"}

currentness = d["currentness_policy"]
assert currentness["last_write_wins_admitted"] is False
assert currentness["stale_base_must_refuse"] is True
assert currentness["unrelated_revision_identity_pair_must_refuse"] is True
assert currentness["caller_supplied_updated_at_is_currentness_proof"] is False

legacy = d["legacy_api_policy"]
assert legacy["whole_record_update_is_civic_safe_surface"] is False
assert legacy["civ_res_must_use_typed_surface"] is True
assert legacy["legacy_api_retroactively_qualified"] is False

required_non_equivalences = {
    "WholeRecordReplacement != AuthorizedSemanticTransition",
    "CallerSelectedToStatus != LifecycleAuthority",
    "StableSupportTicketId != ExpectedRevisionRef",
    "LatestSeenRevision != ProvenCurrentState",
    "TicketStatus::Resolved != CompletionVerification",
    "TicketStatus::Closed != AdministrativeFinality",
    "TicketStatus::Closed != ExternalMunicipalClosure",
    "TicketStatus::Closed != PaymentAuthority",
    "TicketPriority != CivicPriorityAuthority",
    "SupportCategory != UniversalCivicTaxonomy",
    "Assignee != ExecutionAuthority",
    "TypedCommand != AuthorityToMutate",
    "SupportCapability != MunicipalAuthority",
    "Closed != Cancelled",
    "Closed != Duplicate",
    "Closed != Rejected",
    "Closed != Transferred",
    "CallerSuppliedUpdatedAt != TrustedCurrentness",
}
assert required_non_equivalences.issubset(set(d["non_equivalences"]))

required_refusals = {
    "arbitrary_open_to_closed",
    "caller_selected_to_status",
    "requester_rewrite",
    "created_at_rewrite",
    "combined_untyped_status_priority_patch",
    "caller_timestamp_as_currentness",
    "stale_expected_revision",
    "unrelated_revision_identity_pair",
    "resolved_as_completion_verification",
    "closed_as_administrative_finality",
    "critical_priority_as_civic_priority_authority",
    "assignment_as_execution_authority",
    "reopen_erases_prior_resolution_history",
}
assert set(d["required_runtime_refusals"]) == required_refusals

# Documentation must state the load-bearing boundaries in human-readable form.
for phrase in [
    "whole-record replacement",
    "caller-supplied to_status",
    "stale base -> explicit conflict/refusal",
    "TicketStatus::Resolved != CompletionVerification",
    "TicketStatus::Closed != AdministrativeFinality",
    "TicketPriority != CivicPriorityAuthority",
    "Assignee != ExecutionAuthority",
    "Closed != Cancelled",
    "CIV-RES must consume only the qualified typed surface",
]:
    assert phrase in doc, phrase

print("SUP-CIV-002A semantic contract: PASS")
