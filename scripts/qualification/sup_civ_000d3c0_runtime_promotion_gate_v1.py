#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace/docs/civic-resilience/SUP_CIV_000D3C0_RUNTIME_PROMOTION_GATE_V1.md"
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sup_civ_000d3c0_runtime_promotion_gate.json"

d = json.loads(MANIFEST.read_text())
doc = DOC.read_text()

assert d["schema"] == "mycelix.support-civic.d3c-runtime-promotion-gate.v1"
assert d["program"] == "SUP-CIV-000D3C0"
assert d["tracking_issue"] == 2505
assert d["parent_subject"] == "e356226ed2d0b3b2eeea4f8aeb854656d5d2788c"
assert d["selected_profile"] == "EncryptedMultiRecipientProtectedReadV1"
assert d["profile_options"] == {
    "offline_mode_enabled": False,
    "break_glass_enabled": False,
    "public_publication_enabled": False,
}
assert d["frozen_evidence_cut"] is True
assert d["live_ci_auto_promotion"] is False
assert d["future_promotion_requires_new_subject"] is True

roles = d["required_roles"]
assert len(roles) == 14
assert len({r["role"] for r in roles}) == 14
assert all(r["satisfied"] is False for r in roles)
assert all(r["qualification_evidence_ref"] is None for r in roles)

by_role = {r["role"]: r for r in roles}
assert by_role["protected_public_classification_boundary"]["subject_exact_head"] == "5abbbb19c644924738b1de45d28758bd45209533"
assert by_role["protected_access_composition_theorem"]["subject_exact_head"] == "2ccb30692fb56ec555528860fcffb9421053d547"
assert by_role["protected_read_admission_oracle"]["subject_exact_head"] == "e356226ed2d0b3b2eeea4f8aeb854656d5d2788c"
assert by_role["public_release_projection_owner"]["subject_exact_head"] == "306aee19526c4e88130372152d64365e509f79eb"

c2 = by_role["pq_multi_recipient_wrap_capability"]
assert c2["subject_exact_head"] is None
assert c2["observed_evidence_state"] == "no_exact_capability_subject"
assert c2["non_satisfying_preflight"]["subject_exact_head"] == "805f68b6c9151bc390896b77bf108c00e33bfc93"
assert c2["non_satisfying_preflight"]["reason"] == "toolchain_preflight_is_not_recipient_wrap_capability"

for role in [
    "current_authorization_source",
    "purpose_scope_currentness_source",
    "durable_accountability_commit_or_escrow",
]:
    assert by_role[role]["subject_exact_head"] is None
    assert by_role[role]["owner"] == "Deferred"

acct = by_role["reciprocal_accountability_semantics"]
assert acct["subject_exact_head"] == "165997366ec47fe9b9e5863649766f9b913dcc2c"
assert acct["observed_evidence_state"] == "format_gate_failed_before_tests_and_clippy"

assert d["all_required_roles_satisfied"] is False
assert d["runtime_promotion"] == "Refused"

expected_non_eq = {
    "ReviewSubjectExists != DependencySatisfied",
    "QueuedQualification != QualifiedEvidence",
    "HarnessFailure != SemanticFailure",
    "FormatGateFailure != SemanticFailure",
    "ToolchainPreflight != ProtocolCapabilityQualified",
    "D3APassPlusD3BPass != RuntimeDependenciesSatisfied",
    "AuthorizationEvidenceShape != CurrentAuthorizationSource",
    "AccessReceiptSchema != DurableAccountabilityCommit",
    "LaterExternalPass != FrozenManifestChanged",
    "BaseProtectedReadQualified != OfflineLeaseQualified",
    "BaseProtectedReadQualified != BreakGlassAuthorityQualified",
}
assert set(d["required_non_equivalences"]) == expected_non_eq

for phrase in [
    "ReviewSubjectExists != DependencySatisfied",
    "C2AToolchainPreflight != C2RecipientWrapQualified",
    "AccessReceiptSchema != DurableAccountabilityCommit",
    "all_required_roles_satisfied = false",
    "runtime_promotion = Refused",
    "later external PASS",
    "new exact subject",
]:
    assert phrase in doc, phrase

print("SUP-CIV-000D3C0 blocked runtime promotion gate: PASS")
