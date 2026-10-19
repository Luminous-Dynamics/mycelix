#!/usr/bin/env python3
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sup_civ_000c_protected_public_boundary.json"
DOC = ROOT / "mycelix-workspace/docs/civic-resilience/SUP_CIV_000C_PROTECTED_PUBLIC_BOUNDARY_V1.md"

d = json.loads(MANIFEST.read_text())
doc = DOC.read_text()

assert d["schema"] == "mycelix.support-civic.protected-public-boundary.v1"
assert d["program"] == "SUP-CIV-000C"
assert d["parent_subject"] == "a85369699099d4c7524e502e531735eed4ab36f4"
assert d["runtime_owner"] == "Deferred"
assert d["data_planes"] == [
    "PublicOperationalEnvelope",
    "ProtectedSupportPayload",
    "ProtectedAccessReceiptRef",
    "ReleaseProjectionRef",
]
assert d["storage_mechanism"] == "Deferred"
assert d["universal_public_safe_field_set"] is False
assert d["universal_legal_basis"] is False
assert d["protected_access_may_imply_public_release"] is False
assert d["public_release_may_imply_access_legitimacy"] is False
assert d["application_delete_is_global_erasure"] is False

expected_legacy = [
    "LegacyPayloadNotClassifiedForCivicDisclosure",
    "PublicEnvelopeQualifiedUnderProfile",
    "ProtectedPayloadQualifiedUnderProfile",
    "ReleaseProjectionQualifiedUnderProfile",
]
assert d["legacy_classification_states"] == expected_legacy

required_non_equivalences = {
    "ServiceRequest != PublicDisclosureConsent",
    "OperationalRouting != PublicDisclosureAuthority",
    "PublicDhtReplication != PrivacyProtection",
    "RequesterAgentPubKey != AnonymousIdentity",
    "RequesterAgentPubKey != PublicIdentityConsent",
    "PseudonymousIdentifier != AnonymousData",
    "FreeTextDescription != PublicSafeSummary",
    "NeedForRouting != PermissionForPublicPreciseLocation",
    "OperationalDiagnosticNeed != PublicSystemDisclosure",
    "PublicTicketEnvelope != AllLinkedMaterialPublic",
    "ProtectedPayloadAccess != PublicReleaseAuthority",
    "PublicRelease != AccessLegitimacy",
    "ApplicationDeleteAction != GuaranteedGlobalErasure",
    "LegacySupportPayload != CivicPublicSafePayload",
    "PredictionConfidence != PublicDisclosureAuthority",
    "PreemptiveAlert != PublicIncidentTruth",
}
assert set(d["non_equivalences"]) == required_non_equivalences

expected_refusals = {
    "raw_phone_or_name_in_free_text_to_public_envelope",
    "exact_home_address_to_public_projection_without_release_policy",
    "sensitive_comment_inherits_public_ticket_visibility",
    "credential_token_ip_or_internal_topology_to_public_projection",
    "linkable_identifier_precise_place_time_without_profile_review",
    "protected_access_without_required_purpose_scope_evidence",
    "public_dashboard_direct_protected_payload_read",
    "release_when_required_history_or_mosaic_policy_missing_or_stale",
    "delete_action_as_guaranteed_global_erasure",
    "legacy_payload_silently_upgraded_to_civic_public_safe",
}
assert set(d["required_runtime_refusals"]) == expected_refusals

for phrase in [
    "FreeTextDescription != PublicSafeSummary",
    "NeedForRouting != PermissionForPublicPreciseLocation",
    "RequesterAgentPubKey != AnonymousIdentity",
    "ProtectedPayloadAccess != PublicReleaseAuthority",
    "ApplicationDeleteAction != GuaranteedGlobalErasure",
    "PublicTicketEnvelope != AllLinkedMaterialPublic",
    "storage mechanism",
    "does not mandate private Holochain entries",
]:
    assert phrase in doc, phrase

print("SUP-CIV-000C semantic contract: PASS")
