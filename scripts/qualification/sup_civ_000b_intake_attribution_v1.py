#!/usr/bin/env python3
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sup_civ_000b_intake_attribution.json"
DOC = ROOT / "mycelix-workspace/docs/civic-resilience/SUP_CIV_000B_INTAKE_ATTRIBUTION_V1.md"

d = json.loads(MANIFEST.read_text())
doc = DOC.read_text()

assert d["schema"] == "mycelix.support-civic.intake-attribution.v1"
assert d["program"] == "SUP-CIV-000B"
assert d["parent_subject"] == "a85369699099d4c7524e502e531735eed4ab36f4"
assert d["runtime_owner"] == "Deferred"
assert d["intake_modes"] == [
    "SelfIntake",
    "DelegatedIntake",
    "AutomatedOrPreemptiveIntake",
    "LegacyIntake",
]
assert d["attribution_dispositions"] == [
    "SelfAuthoredVerified",
    "DelegatedVerified",
    "AutomatedSourceVerified",
    "LegacyAttributionUnverified",
    "AttributionConflict",
    "AttributionInvalid",
    "InsufficientAttributionEvidence",
]
assert d["self_intake_rule"] == "SupportTicket.requester == create_action.author"
assert d["legacy_default_disposition"] == "LegacyAttributionUnverified"
assert d["agent_to_ticket_link_is_attribution_proof"] is False
assert d["civic_gate_is_impersonation_authority"] is False
assert d["verified_attribution_is_public_disclosure_authority"] is False

required_non_equivalences = {
    "RequesterField != RequesterAuthorization",
    "CanonicalIdentityMatchesGenesis != GenesisRequesterAuthorized",
    "AgentToTicketLink != RequesterAuthorization",
    "CivicGatePassed != AuthorityToImpersonateRequester",
    "Submitter != RepresentedRequester",
    "DelegationExists != DelegationInScope",
    "SymthaeaPredictedTicket != HumanRequestedTicket",
    "AutomatedSourceVerified != HumanConsent",
    "LegacyRequesterField != VerifiedRequesterAttribution",
    "SelfAuthoredVerified != LegalIdentityVerified",
    "SelfAuthoredVerified != ServiceEntitlement",
    "InsufficientEvidence != AttributionInvalid",
    "VerifiedRequesterAttribution != PublicRequesterDisclosureAuthority",
}
assert set(d["non_equivalences"]) == required_non_equivalences

expected_refusals = {
    "self_intake_requester_author_mismatch_without_delegation",
    "direct_integrity_path_bypasses_requester_binding",
    "coordinator_silently_substitutes_requester",
    "expired_revoked_or_out_of_scope_delegation",
    "delegation_subject_mismatch",
    "automated_intake_masquerades_as_human_self_intake",
    "agent_to_ticket_link_upgrades_attribution",
    "legacy_unknown_attribution_silently_verified",
    "conflicting_attribution_silently_selects_winner",
}
assert set(d["required_runtime_refusals"]) == expected_refusals

for phrase in [
    "SupportTicket.requester == create_action.author",
    "CanonicalIdentityMatchesGenesis != GenesisRequesterAuthorized",
    "AgentToTicketLink != RequesterAuthorization",
    "CivicGatePassed != AuthorityToImpersonateRequester",
    "LegacyAttributionUnverified",
    "AttributionConflict",
    "VerifiedRequesterAttribution != PublicRequesterDisclosureAuthority",
]:
    assert phrase in doc, phrase

print("SUP-CIV-000B semantic contract: PASS")
