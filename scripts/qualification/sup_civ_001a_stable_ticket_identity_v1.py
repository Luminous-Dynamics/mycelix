#!/usr/bin/env python3
"""Validate SUP-CIV-001A stable support-ticket identity contract."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace/docs/civic-resilience/SUP_CIV_001A_STABLE_TICKET_IDENTITY_V1.md"
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sup_civ_001a_stable_ticket_identity.json"

REQUIRED_NON_EQUIVALENCES = {
    "SupportTicketActionHash != StableSupportTicketId",
    "StableSupportTicketId != CurrentTicketState",
    "StableSupportTicketId != TicketStatus",
    "StableSupportTicketId != CivicAuthority",
    "IdentityEntry != CurrentRevision",
    "IdentityEntry != CompletionVerification",
    "IdentityEntry != AdministrativeFinality",
    "IdentityEntry != ExternalMunicipalClosure",
    "IdentityEntry != OutcomeImprovement",
    "IdentityEntry != CausalEffect",
    "LegacyTicketWithoutIdentity != MigratedTicket",
    "LinkToIdentity != ValidatedLineage",
    "LatestSeenRevision != ProvenCurrentState",
    "TicketPriority != CivicPriorityAuthority",
    "SupportCategory != UniversalCivicTaxonomy",
    "AutonomyLevel::FullAutonomous != CivicAuthority",
    "AutonomousAction.approved != InstitutionalAuthority",
    "AutonomousAction.executed != ConfirmedExternalEffect",
    "AutonomousAction.success != OutcomeEffect",
    "PredictionConfidence != ObservationTruth",
}

REQUIRED_REFUSALS = {
    "update_action_as_genesis",
    "non_ticket_action_as_genesis",
    "requester_mismatch",
    "intake_created_at_mismatch",
    "identity_entry_update",
    "fabricated_identity_for_legacy_absence",
}

REQUIRED_DEFERRED = {
    "revision_lineage",
    "current_state_projection",
    "status_transition_algebra",
    "current_index_maintenance",
    "civic_adapter_refs",
    "completion_verification_binding",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def main() -> None:
    require(DOC.is_file(), f"missing contract: {DOC}")
    require(MANIFEST.is_file(), f"missing manifest: {MANIFEST}")

    text = DOC.read_text(encoding="utf-8")
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))

    require(data["schema"] == "mycelix.support-civic.stable-ticket-identity.v1", "unexpected schema")
    require(data["program"] == "SUP-CIV-001A", "unexpected program")
    require(data["tracking_issue"] == 2044, "unexpected tracking issue")
    require(data["parent_discovery_issue"] == 2030, "unexpected parent discovery")
    require(data["civic_dependency_issue"] == 2028, "unexpected civic dependency")
    require(data["parent_subject"] == "a85369699099d4c7524e502e531735eed4ab36f4", "unexpected parent subject")
    require(data["runtime_owner"] == "Deferred", "runtime ownership must remain deferred")

    stable = data["stable_id_definition"]
    require(stable["logical_type"] == "StableSupportTicketId", "logical type drift")
    require(stable["representation"] == "EntryHash(CanonicalSupportTicketIdentity)", "stable ID representation drift")
    require(stable["action_hash_is_stable_id"] is False, "action hash must not become stable ID")
    require(stable["legacy_support_ticket_schema_change_required"] is False, "001A must preserve legacy SupportTicket schema")

    require(
        data["canonical_identity_fields"]
        == ["genesis_ticket_action_ref", "requester_ref", "intake_created_at_ref"],
        "canonical identity field set/order drift",
    )

    requirements = set(data["canonicality_requirements"])
    for needle in (
        "genesis_ticket_action_ref resolves to a Create action",
        "the created application entry is SupportTicket",
        "canonical identity content is immutable",
        "same canonical identity content yields the same EntryHash",
    ):
        require(needle in requirements, f"missing canonicality requirement: {needle}")

    require(set(data["required_executable_refusals"]) == REQUIRED_REFUSALS, "required executable refusal registry drift")
    require(REQUIRED_DEFERRED.issubset(set(data["deferred_theorems"])), "required deferred theorem missing")
    require(set(data["non_equivalences"]) == REQUIRED_NON_EQUIVALENCES, "non-equivalence registry drift")

    require(
        data["legacy_identity_states"]
        == [
            "validated_identity_present",
            "legacy_unmigrated_identity_absent",
            "migration_failed_or_refused",
        ],
        "legacy identity states must remain explicit and closed",
    )

    forbidden_claims = set(data["forbidden_claims"])
    for required in (
        "runtime_identity_implemented",
        "legacy_migration_complete",
        "current_state_proven",
        "civic_authority_established",
        "completion_verified",
        "johannesburg_deployment_ready",
    ):
        require(required in forbidden_claims, f"missing forbidden claim: {required}")

    require(data["claim_ceiling"] == "semantic identity and migration boundary only", "claim ceiling drift")

    for statement in REQUIRED_NON_EQUIVALENCES:
        require(statement in text, f"contract missing non-equivalence: {statement}")

    for phrase in (
        "StableSupportTicketId := EntryHash(CanonicalSupportTicketIdentity)",
        "legacy/unmigrated identity absent",
        "SUP-CIV-001B",
        "This tranche changes no Rust",
        "does not establish runtime identity code",
    ):
        require(phrase in text, f"contract missing load-bearing phrase: {phrase}")

    banned_positive_fields = ("completed=true", "official=true", "civic_authority=true")
    for banned in banned_positive_fields:
        require(banned not in text, f"caller-selected positive authority field leaked into contract: {banned}")

    print("SUP-CIV-001A stable ticket identity contract: PASS")


if __name__ == "__main__":
    main()
