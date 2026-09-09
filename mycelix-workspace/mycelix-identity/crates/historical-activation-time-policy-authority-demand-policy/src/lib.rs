// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Closed Identity-side authority demand set for historical time-policy governance.
//!
//! This theorem does not import the generic authority implementation and cannot establish
//! historical or current authority. It freezes exactly what the generic authority plane must
//! prove after ancestry convergence:
//!
//! - one generic historical subject + signer-committed causal coordinate for every #454
//!   Identity time-policy transition; and
//! - one exact generic current signing-policy subject for #476's terminal #395 policy.
//! 
//! Missing or extra signer mappings fail closed. The terminal current-policy subject is bound to
//! the exact #476 candidate and full #467 mapping, so the later adapter is left only with equality
//! checks against independently qualified generic authority capabilities.

#![forbid(unsafe_code)]

use mycelix_historical_activation_time_authority_policy_generic_authority_subject_mapping_policy::QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1;
use mycelix_historical_activation_time_authority_policy_terminal_candidate_policy::QualifiedHistoricalActivationTimeAuthorityPolicyTerminalCandidateV1;
use mycelix_historical_activation_time_policy_authority_anchored_authenticated_lineage_policy::{
    HistoricalActivationTimePolicyCausalAuthorityRequirementV2,
    QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2,
};
use mycelix_historical_activation_time_policy_transition_signer_generic_authority_subject_mapping_policy::QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const SHA256_DIGEST_LEN_V1: usize = 32;
pub const MAX_HISTORICAL_AUTHORITY_DEMANDS_V1: usize = 4096;
pub const GENERIC_SIGNING_POLICY_KIND_V1: &str = "SigningPolicy";
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_DEMAND_PROFILE_V1: &str =
    "mycelix-identity-time-policy-authority-demand-v1-sha256-framed";
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_DEMAND_DOMAIN_V1: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-authority-demand:v1\0";

#[derive(Debug)]
pub struct HistoricalTimePolicyAuthorityDemandV1 {
    transition_generation: u64,
    base_transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
    anchored_transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
    authenticity_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
    signer_authority_subject_sha256: [u8; SHA256_DIGEST_LEN_V1],
    signer_mapping_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
    generic_subject_kind: String,
    generic_namespace: String,
    generic_subject_id: String,
    generic_identity_profile: String,
    generic_identity_digest: [u8; SHA256_DIGEST_LEN_V1],
    authority_state_generation: u64,
    authority_state_transition_digest: [u8; SHA256_DIGEST_LEN_V1],
}

impl HistoricalTimePolicyAuthorityDemandV1 {
    pub fn transition_generation(&self) -> u64 {
        self.transition_generation
    }

    pub fn base_transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.base_transition_signing_digest_sha256
    }

    pub fn anchored_transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.anchored_transition_signing_digest_sha256
    }

    pub fn authenticity_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.authenticity_digest_sha256
    }

    pub fn signer_authority_subject_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.signer_authority_subject_sha256
    }

    pub fn signer_mapping_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.signer_mapping_digest_sha256
    }

    pub fn generic_subject_kind(&self) -> &str {
        &self.generic_subject_kind
    }

    pub fn generic_namespace(&self) -> &str {
        &self.generic_namespace
    }

    pub fn generic_subject_id(&self) -> &str {
        &self.generic_subject_id
    }

    pub fn generic_identity_profile(&self) -> &str {
        &self.generic_identity_profile
    }

    pub fn generic_identity_digest(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.generic_identity_digest
    }

    pub fn authority_state_generation(&self) -> u64 {
        self.authority_state_generation
    }

    pub fn authority_state_transition_digest(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.authority_state_transition_digest
    }
}

#[derive(Debug)]
pub struct CurrentTimeAuthorityPolicyDemandV1 {
    terminal_policy_sha256: [u8; SHA256_DIGEST_LEN_V1],
    terminal_candidate_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
    policy_mapping_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
    generic_subject_kind: String,
    generic_namespace: String,
    generic_subject_id: String,
    generic_identity_profile: String,
    generic_identity_digest: [u8; SHA256_DIGEST_LEN_V1],
}

impl CurrentTimeAuthorityPolicyDemandV1 {
    pub fn terminal_policy_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.terminal_policy_sha256
    }

    pub fn terminal_candidate_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.terminal_candidate_digest_sha256
    }

    pub fn policy_mapping_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.policy_mapping_digest_sha256
    }

    pub fn generic_subject_kind(&self) -> &str {
        &self.generic_subject_kind
    }

    pub fn generic_namespace(&self) -> &str {
        &self.generic_namespace
    }

    pub fn generic_subject_id(&self) -> &str {
        &self.generic_subject_id
    }

    pub fn generic_identity_profile(&self) -> &str {
        &self.generic_identity_profile
    }

    pub fn generic_identity_digest(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.generic_identity_digest
    }
}

#[derive(Debug)]
pub struct QualifiedHistoricalActivationTimePolicyAuthorityDemandV1 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V1],
    lineage_qualification_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
    historical_demands: Vec<HistoricalTimePolicyAuthorityDemandV1>,
    distinct_signer_count: u32,
    current_policy_demand: CurrentTimeAuthorityPolicyDemandV1,
    demand_digest_sha256: [u8; SHA256_DIGEST_LEN_V1],
}

impl QualifiedHistoricalActivationTimePolicyAuthorityDemandV1 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.authority_domain_sha256
    }

    pub fn lineage_qualification_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.lineage_qualification_digest_sha256
    }

    pub fn historical_demands(&self) -> &[HistoricalTimePolicyAuthorityDemandV1] {
        &self.historical_demands
    }

    pub fn distinct_signer_count(&self) -> u32 {
        self.distinct_signer_count
    }

    pub fn current_policy_demand(&self) -> &CurrentTimeAuthorityPolicyDemandV1 {
        &self.current_policy_demand
    }

    pub fn demand_profile(&self) -> &'static str {
        HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_DEMAND_PROFILE_V1
    }

    pub fn demand_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V1] {
        &self.demand_digest_sha256
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyAuthorityDemandErrorV1 {
    InvalidHistoricalDemandCount,
    CandidateAuthorityDomainMismatch,
    CandidateLineageMismatch,
    CandidateTransitionCountMismatch,
    CandidateTerminalTransitionMismatch,
    CandidateTerminalPolicyMismatch,
    TerminalMappingDigestMismatch,
    TerminalGenericIdentityMismatch,
    TerminalGenericKindMismatch,
    DuplicateSignerMapping,
    MissingSignerMapping,
    ExtraSignerMapping,
    SignerMappingKindMismatch,
    DistinctSignerCountOverflow,
}

pub fn qualify_historical_activation_time_policy_authority_demand_v1(
    lineage: &QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2,
    signer_mappings: &[&QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1],
    terminal_candidate: &QualifiedHistoricalActivationTimeAuthorityPolicyTerminalCandidateV1,
    terminal_mapping: &QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1,
) -> Result<
    QualifiedHistoricalActivationTimePolicyAuthorityDemandV1,
    HistoricalActivationTimePolicyAuthorityDemandErrorV1,
> {
    if lineage.requirements().is_empty()
        || lineage.requirements().len() > MAX_HISTORICAL_AUTHORITY_DEMANDS_V1
    {
        return Err(
            HistoricalActivationTimePolicyAuthorityDemandErrorV1::InvalidHistoricalDemandCount,
        );
    }

    validate_terminal_binding(lineage, terminal_candidate, terminal_mapping)?;

    let mut mappings = BTreeMap::<
        [u8; SHA256_DIGEST_LEN_V1],
        &QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1,
    >::new();
    for mapping in signer_mappings {
        let key = *mapping.source_signer_subject_sha256();
        if mappings.insert(key, *mapping).is_some() {
            return Err(
                HistoricalActivationTimePolicyAuthorityDemandErrorV1::DuplicateSignerMapping,
            );
        }
        if mapping.generic_subject_kind() != GENERIC_SIGNING_POLICY_KIND_V1 {
            return Err(
                HistoricalActivationTimePolicyAuthorityDemandErrorV1::SignerMappingKindMismatch,
            );
        }
    }

    let mut used_signers = BTreeSet::<[u8; SHA256_DIGEST_LEN_V1]>::new();
    let mut historical_demands = Vec::with_capacity(lineage.requirements().len());
    for requirement in lineage.requirements() {
        historical_demands.push(bind_historical_requirement(
            requirement,
            &mappings,
            &mut used_signers,
        )?);
    }

    if mappings.len() != used_signers.len() {
        return Err(HistoricalActivationTimePolicyAuthorityDemandErrorV1::ExtraSignerMapping);
    }
    let distinct_signer_count = u32::try_from(used_signers.len()).map_err(|_| {
        HistoricalActivationTimePolicyAuthorityDemandErrorV1::DistinctSignerCountOverflow
    })?;

    let current_policy_demand = CurrentTimeAuthorityPolicyDemandV1 {
        terminal_policy_sha256: *terminal_candidate.terminal_policy_sha256(),
        terminal_candidate_digest_sha256: *terminal_candidate.candidate_digest_sha256(),
        policy_mapping_digest_sha256: *terminal_mapping.mapping_digest_sha256(),
        generic_subject_kind: terminal_mapping.generic_subject_kind().to_string(),
        generic_namespace: terminal_mapping.generic_namespace().to_string(),
        generic_subject_id: terminal_mapping.generic_subject_id().to_string(),
        generic_identity_profile: terminal_mapping.generic_identity_profile().to_string(),
        generic_identity_digest: *terminal_mapping.generic_identity_digest(),
    };

    let demand_digest_sha256 = derive_demand_digest_v1(
        lineage.authority_domain_sha256(),
        lineage.qualification_digest_sha256(),
        &historical_demands,
        &current_policy_demand,
    );

    Ok(QualifiedHistoricalActivationTimePolicyAuthorityDemandV1 {
        authority_domain_sha256: *lineage.authority_domain_sha256(),
        lineage_qualification_digest_sha256: *lineage.qualification_digest_sha256(),
        historical_demands,
        distinct_signer_count,
        current_policy_demand,
        demand_digest_sha256,
    })
}

fn validate_terminal_binding(
    lineage: &QualifiedAuthorityAnchoredAuthenticatedHistoricalActivationTimePolicyLineageV2,
    terminal_candidate: &QualifiedHistoricalActivationTimeAuthorityPolicyTerminalCandidateV1,
    terminal_mapping: &QualifiedHistoricalActivationTimeAuthorityPolicyGenericAuthoritySubjectMappingV1,
) -> Result<(), HistoricalActivationTimePolicyAuthorityDemandErrorV1> {
    if terminal_candidate.authority_domain_sha256() != lineage.authority_domain_sha256() {
        return Err(
            HistoricalActivationTimePolicyAuthorityDemandErrorV1::CandidateAuthorityDomainMismatch,
        );
    }
    if terminal_candidate.lineage_qualification_digest_sha256()
        != lineage.qualification_digest_sha256()
    {
        return Err(HistoricalActivationTimePolicyAuthorityDemandErrorV1::CandidateLineageMismatch);
    }
    if terminal_candidate.transition_count() != lineage.transition_count() {
        return Err(
            HistoricalActivationTimePolicyAuthorityDemandErrorV1::CandidateTransitionCountMismatch,
        );
    }
    if terminal_candidate.terminal_transition_generation()
        != lineage.terminal_transition_generation()
        || terminal_candidate.terminal_transition_sha256() != lineage.terminal_transition_sha256()
    {
        return Err(
            HistoricalActivationTimePolicyAuthorityDemandErrorV1::CandidateTerminalTransitionMismatch,
        );
    }
    let Some(terminal_policy) = lineage.terminal_policy_sha256() else {
        return Err(
            HistoricalActivationTimePolicyAuthorityDemandErrorV1::CandidateTerminalPolicyMismatch,
        );
    };
    if terminal_candidate.terminal_policy_sha256() != terminal_policy {
        return Err(
            HistoricalActivationTimePolicyAuthorityDemandErrorV1::CandidateTerminalPolicyMismatch,
        );
    }
    if terminal_candidate.generic_policy_mapping_digest_sha256()
        != terminal_mapping.mapping_digest_sha256()
    {
        return Err(
            HistoricalActivationTimePolicyAuthorityDemandErrorV1::TerminalMappingDigestMismatch,
        );
    }
    if terminal_candidate.generic_policy_subject_identity_sha256()
        != terminal_mapping.generic_identity_digest()
    {
        return Err(
            HistoricalActivationTimePolicyAuthorityDemandErrorV1::TerminalGenericIdentityMismatch,
        );
    }
    if terminal_mapping.generic_subject_kind() != GENERIC_SIGNING_POLICY_KIND_V1 {
        return Err(
            HistoricalActivationTimePolicyAuthorityDemandErrorV1::TerminalGenericKindMismatch,
        );
    }
    Ok(())
}

fn bind_historical_requirement(
    requirement: &HistoricalActivationTimePolicyCausalAuthorityRequirementV2,
    mappings: &BTreeMap<
        [u8; SHA256_DIGEST_LEN_V1],
        &QualifiedHistoricalActivationTimePolicySignerGenericAuthoritySubjectMappingV1,
    >,
    used_signers: &mut BTreeSet<[u8; SHA256_DIGEST_LEN_V1]>,
) -> Result<HistoricalTimePolicyAuthorityDemandV1, HistoricalActivationTimePolicyAuthorityDemandErrorV1>
{
    let signer_key = *requirement.signer_authority_subject_sha256();
    let mapping = mappings
        .get(&signer_key)
        .ok_or(HistoricalActivationTimePolicyAuthorityDemandErrorV1::MissingSignerMapping)?;
    used_signers.insert(signer_key);

    Ok(HistoricalTimePolicyAuthorityDemandV1 {
        transition_generation: requirement.transition_generation(),
        base_transition_signing_digest_sha256: *requirement.base_transition_signing_digest_sha256(),
        anchored_transition_signing_digest_sha256: *requirement
            .anchored_transition_signing_digest_sha256(),
        authenticity_digest_sha256: *requirement.authenticity_digest_sha256(),
        signer_authority_subject_sha256: signer_key,
        signer_mapping_digest_sha256: *mapping.mapping_digest_sha256(),
        generic_subject_kind: mapping.generic_subject_kind().to_string(),
        generic_namespace: mapping.generic_namespace().to_string(),
        generic_subject_id: mapping.generic_subject_id().to_string(),
        generic_identity_profile: mapping.generic_identity_profile().to_string(),
        generic_identity_digest: *mapping.generic_identity_digest(),
        authority_state_generation: requirement.authority_state_generation(),
        authority_state_transition_digest: *requirement.authority_state_transition_digest(),
    })
}

fn derive_demand_digest_v1(
    authority_domain_sha256: &[u8; SHA256_DIGEST_LEN_V1],
    lineage_qualification_digest_sha256: &[u8; SHA256_DIGEST_LEN_V1],
    historical_demands: &[HistoricalTimePolicyAuthorityDemandV1],
    current_policy: &CurrentTimeAuthorityPolicyDemandV1,
) -> [u8; SHA256_DIGEST_LEN_V1] {
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_DEMAND_DOMAIN_V1);
    hasher.update([0x01]);
    hasher.update(authority_domain_sha256);
    hasher.update([0x02]);
    hasher.update(lineage_qualification_digest_sha256);
    hasher.update([0x03]);
    hasher.update((historical_demands.len() as u32).to_be_bytes());
    for demand in historical_demands {
        hasher.update([0x10]);
        hasher.update(demand.transition_generation.to_be_bytes());
        hasher.update([0x11]);
        hasher.update(demand.base_transition_signing_digest_sha256);
        hasher.update([0x12]);
        hasher.update(demand.anchored_transition_signing_digest_sha256);
        hasher.update([0x13]);
        hasher.update(demand.authenticity_digest_sha256);
        hasher.update([0x14]);
        hasher.update(demand.signer_authority_subject_sha256);
        hasher.update([0x15]);
        hasher.update(demand.signer_mapping_digest_sha256);
        frame_str(&mut hasher, 0x16, &demand.generic_subject_kind);
        frame_str(&mut hasher, 0x17, &demand.generic_namespace);
        frame_str(&mut hasher, 0x18, &demand.generic_subject_id);
        frame_str(&mut hasher, 0x19, &demand.generic_identity_profile);
        hasher.update([0x1a]);
        hasher.update(demand.generic_identity_digest);
        hasher.update([0x1b]);
        hasher.update(demand.authority_state_generation.to_be_bytes());
        hasher.update([0x1c]);
        hasher.update(demand.authority_state_transition_digest);
    }
    hasher.update([0x20]);
    hasher.update(current_policy.terminal_policy_sha256);
    hasher.update([0x21]);
    hasher.update(current_policy.terminal_candidate_digest_sha256);
    hasher.update([0x22]);
    hasher.update(current_policy.policy_mapping_digest_sha256);
    frame_str(&mut hasher, 0x23, &current_policy.generic_subject_kind);
    frame_str(&mut hasher, 0x24, &current_policy.generic_namespace);
    frame_str(&mut hasher, 0x25, &current_policy.generic_subject_id);
    frame_str(&mut hasher, 0x26, &current_policy.generic_identity_profile);
    hasher.update([0x27]);
    hasher.update(current_policy.generic_identity_digest);
    hasher.finalize().into()
}

fn frame_str(hasher: &mut Sha256, tag: u8, value: &str) {
    let bytes = value.as_bytes();
    let len = u32::try_from(bytes.len()).expect("qualified authority subject strings fit u32");
    hasher.update([tag]);
    hasher.update(len.to_be_bytes());
    hasher.update(bytes);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn historical(byte: u8, generation: u64) -> HistoricalTimePolicyAuthorityDemandV1 {
        HistoricalTimePolicyAuthorityDemandV1 {
            transition_generation: generation,
            base_transition_signing_digest_sha256: [byte; 32],
            anchored_transition_signing_digest_sha256: [byte.wrapping_add(1); 32],
            authenticity_digest_sha256: [byte.wrapping_add(2); 32],
            signer_authority_subject_sha256: [byte.wrapping_add(3); 32],
            signer_mapping_digest_sha256: [byte.wrapping_add(4); 32],
            generic_subject_kind: GENERIC_SIGNING_POLICY_KIND_V1.into(),
            generic_namespace: "identity:time-policy:transition-authority".into(),
            generic_subject_id: format!("signer-{byte}@generation:{generation}"),
            generic_identity_profile: "signer-profile-v1".into(),
            generic_identity_digest: [byte.wrapping_add(5); 32],
            authority_state_generation: generation + 10,
            authority_state_transition_digest: [byte.wrapping_add(6); 32],
        }
    }

    fn current(byte: u8) -> CurrentTimeAuthorityPolicyDemandV1 {
        CurrentTimeAuthorityPolicyDemandV1 {
            terminal_policy_sha256: [byte; 32],
            terminal_candidate_digest_sha256: [byte.wrapping_add(1); 32],
            policy_mapping_digest_sha256: [byte.wrapping_add(2); 32],
            generic_subject_kind: GENERIC_SIGNING_POLICY_KIND_V1.into(),
            generic_namespace: "identity:historical-activation:time-authority-policy".into(),
            generic_subject_id: "mycelix-identity-v2@epoch:1:time-policy:primary-v2@version:1"
                .into(),
            generic_identity_profile: "policy-profile-v1".into(),
            generic_identity_digest: [byte.wrapping_add(3); 32],
        }
    }

    #[test]
    fn demand_digest_commits_historical_and_current_subjects() {
        let domain = [0x11; 32];
        let lineage = [0x22; 32];
        let first = vec![historical(0x30, 1)];
        let second = vec![historical(0x31, 1)];
        let current_a = current(0x40);
        let current_b = current(0x41);
        let baseline = derive_demand_digest_v1(&domain, &lineage, &first, &current_a);
        assert_ne!(
            baseline,
            derive_demand_digest_v1(&domain, &lineage, &second, &current_a)
        );
        assert_ne!(
            baseline,
            derive_demand_digest_v1(&domain, &lineage, &first, &current_b)
        );
    }

    #[test]
    fn historical_demand_order_is_part_of_identity() {
        let domain = [0x11; 32];
        let lineage = [0x22; 32];
        let current = current(0x40);
        let ordered = vec![historical(0x30, 1), historical(0x31, 2)];
        let reversed = vec![historical(0x31, 2), historical(0x30, 1)];
        assert_ne!(
            derive_demand_digest_v1(&domain, &lineage, &ordered, &current),
            derive_demand_digest_v1(&domain, &lineage, &reversed, &current)
        );
    }
}
