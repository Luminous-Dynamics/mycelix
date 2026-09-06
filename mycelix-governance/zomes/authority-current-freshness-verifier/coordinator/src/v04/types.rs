// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Fail-closed runtime composition for deployment-bound current operational authority.

use hdk::prelude::*;
use mycelix_authority_bootstrap_root_adoption_verifier::{
    build_adoption_claim, qualify_bootstrap_root_adoption, BootstrapRootAdoptionClaim,
    VerifiedBootstrapRootAdoptionProof, ADOPTION_CLAIM_PROFILE,
};
use mycelix_authority_control_plane_freshness::{
    qualify_control_plane_subject_freshness, QualifiedControlPlaneSubjectFreshness,
};
use mycelix_authority_evidence_lease::{
    qualify_evidence_lease_manifest, EvidenceLease, EvidenceLeaseContribution,
    EvidenceLeaseRole, LeasedEvidence, QualifiedEvidenceLeaseManifest,
};
use mycelix_authority_freshness::{AuthoritySubjectRef, VerifiedAuthorityFreshness};
use mycelix_authority_operational_context::qualify_operational_policy_context;
use mycelix_authority_operational_deployment_fence::{
    qualify_operational_freshness_for_deployment_with_provenance, HostLocalDnaContext,
};
use mycelix_authority_operational_freshness::qualify_operational_subject_freshness;
use mycelix_authority_state_bootstrap_root::{
    qualify_bootstrap_root, AuthorityStateBootstrapRootManifest,
    VerifiedCurrentConstitutionReceipt, CURRENT_CONSTITUTION_RECEIPT_PROTOCOL,
};
use mycelix_authority_state_coverage::{
    VerifiedAuthorityCoveragePolicy, VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead,
    POLICY_IDENTITY_PROFILE, SOURCE_HEAD_IDENTITY_PROFILE, WITNESS_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    VerifiedCoverageChallenge, VerifiedCoverageTrustContextPolicy, VerifiedWitnessTrustBinding,
    CONTEXT_POLICY_PROFILE, WITNESS_TRUST_BINDING_PROFILE,
};
use mycelix_authority_state_source::{
    AuthorityStateTransition, VerifiedAuthorityStateTransition, TRANSITION_IDENTITY_PROFILE,
};
use mycelix_authority_state_transition_verifier::{
    qualify_authority_state_transition, VerifiedTransitionAuthorityProof,
    VerifiedTransitionRecordProof,
};
use mycelix_governance_constitution::{
    ConstitutionStatement, Digest32 as ConstitutionDigest32, STATEMENT_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

const RUNTIME_PROTOCOL: &str = "mycelix-authority-current-freshness-verifier-v0.8";
const ROOT_MANIFEST_PROVIDER_ZOME: &str = "authority_state_bootstrap_root_manifest_provider";
const CONSTITUTION_TRANSITION_ZOME: &str = "constitution_transition";
const CURRENT_CONSTITUTION_FUNCTION: &str = "get_verified_current_constitution";
const ROOT_ADOPTION_PROOF_VERIFIER_ZOME: &str =
    "authority_bootstrap_root_adoption_proof_verifier";
const ROOT_ADOPTION_PROOF_VERIFIER_FUNCTION: &str = "verify_bootstrap_root_adoption_proof";
const POLICY_PROVIDER_ZOME: &str = "authority_operational_policy_leased_provider";
const EVIDENCE_PLAN_ZOME: &str = "authority_state_evidence_plan_provider";
const PROBE_VERIFIER_ZOME: &str = "authority_state_challenge";
const SOURCE_HEAD_VERIFIER_ZOME: &str = "authority_state_source_head_verifier";
const WITNESS_VERIFIER_ZOME: &str = "authority_state_witness_verifier";
const TRANSITION_CANDIDATE_PROVIDER_ZOME: &str = "authority_state_transition_candidate_provider";
const TRANSITION_RECORD_PROOF_VERIFIER_ZOME: &str =
    "authority_state_transition_record_proof_verifier";
const TRANSITION_AUTHORITY_PROOF_VERIFIER_ZOME: &str =
    "authority_state_transition_authority_proof_verifier";
const MAX_CONTROL_PLANE_PROBES: usize = 3;
const MAX_WITNESSES: usize = 64;
const MAX_TRUST_BINDINGS: usize = 64;
const MAX_TRANSITIONS: usize = 256;
const CURRENT_CONSTITUTION_COMPOSITION_LEASE_MS: u64 = 30_000;
const DEPLOYMENT_RETURN_LEASE_MS: u64 = 5_000;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct VerifiedCurrentConstitutionMirror {
    dna_hash: String,
    statement: ConstitutionStatement,
    statement_digest: ConstitutionDigest32,
    verified_transition_count: u64,
    candidate_count: u64,
    legacy_constitution_authoritative: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RootAdoptionProofVerificationRequest {
    pub claim: BootstrapRootAdoptionClaim,
    pub claim_digest: Digest32,
    pub claim_profile: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalPolicyCandidateBundle {
    pub coverage: VerifiedAuthorityCoveragePolicy,
    pub context: VerifiedCoverageTrustContextPolicy,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ControlPlaneProbePlanRequest {
    pub target_subject: AuthoritySubjectRef,
    pub root_manifest_digest: Digest32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ControlPlaneProbePlan {
    pub probe_actions: Vec<ActionHash>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalProbePlanRequest {
    pub target_subject: AuthoritySubjectRef,
    pub context_policy_digest: Digest32,
    pub coverage_policy_digest: Digest32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OperationalProbePlan {
    pub probe_action: ActionHash,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SourceHeadVerificationRequest {
    pub probe_action: ActionHash,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WitnessVerificationRequest {
    pub challenge: VerifiedCoverageChallenge,
    pub source_head: VerifiedAuthoritySourceHead,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedWitnessEvidenceBundle {
    pub witnesses: Vec<VerifiedAuthorityHeadWitness>,
    pub trust_bindings: Vec<VerifiedWitnessTrustBinding>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TransitionCandidateDiscoveryRequest {
    pub subject: AuthoritySubjectRef,
    pub authoritative_source_ref: String,
    pub head_generation: u64,
    pub head_transition_digest: Digest32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TransitionRecordProofVerificationRequest {
    pub transition: AuthorityStateTransition,
    pub transition_digest: Digest32,
    pub transition_profile: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TransitionAuthorityProofVerificationRequest {
    pub transition: AuthorityStateTransition,
    pub transition_digest: Digest32,
    pub transition_profile: String,
}

struct ResolvedTransitionLineage {
    transitions: Vec<VerifiedAuthorityStateTransition>,
    lease: EvidenceLease,
    contributions: Vec<EvidenceLeaseContribution>,
}

struct ResolvedAuthorityEvidence {
    challenge: VerifiedCoverageChallenge,
    source_head: VerifiedAuthoritySourceHead,
    witnesses: Vec<VerifiedAuthorityHeadWitness>,
    trust_bindings: Vec<VerifiedWitnessTrustBinding>,
    transitions: Vec<VerifiedAuthorityStateTransition>,
    lease: EvidenceLease,
    contributions: Vec<EvidenceLeaseContribution>,
}

struct ResolvedControlPlaneFreshness {
    qualified: Vec<QualifiedControlPlaneSubjectFreshness>,
    lease: EvidenceLease,
    contributions: Vec<EvidenceLeaseContribution>,
    probe_count: usize,
    witness_count: usize,
    transition_count: usize,
}

struct ResolvedBootstrapRoot {
    root: mycelix_authority_state_bootstrap_root::QualifiedAuthorityStateBootstrapRoot,
    constitution: VerifiedCurrentConstitutionMirror,
    lease: EvidenceLease,
    contributions: Vec<EvidenceLeaseContribution>,
}

/// Serializable cross-zome/API projection for audit, diagnostics and local-provider
/// consumption. Deserialization is never positive authority by itself.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CurrentOperationalFreshnessAuditReceipt {
    pub protocol: String,
    pub subject: AuthoritySubjectRef,
    pub bootstrap_root_digest: Digest32,
    pub bootstrap_root_profile: String,
    pub operational_context_digest: Digest32,
    pub operational_context_profile: String,
    pub authority_digest: Digest32,
    pub authority_profile: String,
    pub evidence_digest: Digest32,
    pub evidence_profile: String,
    pub composition_evidence_manifest_digest: Digest32,
    pub composition_evidence_manifest_profile: String,
    pub composition_evidence_contributor_count: u32,
    pub composition_evidence_lease_protocol: String,
    pub composition_evidence_verified_at_ms: u64,
    pub composition_evidence_valid_until_ms: u64,
    pub local_dna_hash: String,
    pub constitution_statement_digest: Digest32,
    pub constitution_statement_profile: String,
    pub deployment_authority_digest: Digest32,
    pub deployment_authority_profile: String,
    pub deployment_evidence_digest: Digest32,
    pub deployment_evidence_profile: String,
    pub current_freshness: VerifiedAuthorityFreshness,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub lease_until_ms: u64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CurrentFreshnessRuntimeStatus {
    pub protocol: String,
    pub composition_only: bool,
    pub root_manifest_provider_grants_authority: bool,
    pub current_constitution_source_is_binding_plane: bool,
    pub constitution_rechecked_after_adoption: bool,
    pub constitution_rechecked_before_return: bool,
    pub root_adoption_proof_verifier_separate: bool,
    pub root_adoption_constructed_locally: bool,
    pub current_constitution_provenance_explicit: bool,
    pub root_adoption_provenance_explicit: bool,
    pub root_provenance_lease_constructed_at_root_boundary: bool,
    pub transition_discovery_grants_authority: bool,
    pub transition_record_proof_verifier_separate: bool,
    pub transition_authority_proof_verifier_separate: bool,
    pub transition_source_bound_from_source_head: bool,
    pub leased_source_head_consumed: bool,
    pub leased_witness_evidence_consumed: bool,
    pub leased_operational_policy_consumed: bool,
    pub transition_leases_intersected: bool,
    pub global_evidence_lease_enforced: bool,
    pub provenance_contributor_set_constructed_locally: bool,
    pub provenance_required_role_cardinality_closed: bool,
    pub provenance_manifest_qualified_locally: bool,
    pub provenance_manifest_matches_global_lease: bool,
    pub provenance_bound_into_deployment_evidence: bool,
    pub deployment_lease_capped_by_global_evidence: bool,
    pub wire_receipt_transport_only: bool,
    pub caller_supplied_positive_currentness_accepted: bool,
    pub wire_receipt_grants_execution_authority: bool,
    pub host_local_dna_derived: bool,
    pub constitutional_dna_cross_checked: bool,
    pub deployment_authority_constructed_locally: bool,
    pub semantic_only_output: bool,
    pub evidence_plan_grants_authority: bool,
    pub latest_dht_record_authority: bool,
    pub provider_positive_object_authority: bool,
    pub probe_authority: bool,
    pub qualification_time_after_evidence: bool,
    pub external_effects_enabled: bool,
    pub operational: bool,
}
