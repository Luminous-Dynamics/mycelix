//! ELECT-018A pure verifier-key lifecycle core for Mycelix public elections.
//!
//! This crate proves deterministic genesis and structurally valid rotate/disable projections.
//! It does not prove governance authorization, durable currentness, or signature validity.
//! A projected or materialized state is not authoritative until later ELECT-018 authority and
//! transparency/witness tranches qualify it.

use election_integrity_types::Digest32;
use election_verifier_key_authorization::{
    VerifierKeyAuthorizationPolicyV1,
    VerifierKeyAuthorizationPolicyViolation, VerifierKeyAuthorizationRootV1,
    VerifierKeyAuthorizationRootViolation, XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256,
    XENIA_ED25519_AUTHENTICATION_SUITE_ID, XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
    validate_verifier_key_authorization_root, verifier_key_authorization_policy_digest,
    verifier_key_authorization_root_digest,
};
use election_verifier_public_key_binding::{
    VerifierAuthenticationPublicKeyBundleV1,
    VerifierAuthenticationPublicKeyEvidenceV1, VerifierPublicKeyBundleViolation,
    XENIA_AUTHENTICATION_PROFILE_V1_SHA256, XeniaSignerKeyIdViolation,
    validate_verifier_authentication_public_key_bundle, verifier_authentication_public_key_bundle_digest,
    xenia_authentication_signer_key_id_v1,
};
use sha2::{Digest as ShaDigest, Sha256};
use std::collections::{HashMap, HashSet};

pub const VERIFIER_KEY_LIFECYCLE_CORE_PROFILE_ID: &str =
    "mycelix-public-election-verifier-key-lifecycle-core-v1";
pub const MAX_LIFECYCLE_TRANSITIONS_V1: u32 = 256;
pub const MAX_CANONICAL_STRING_BYTES: usize = 256;

const POLICY_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:VERIFIER-KEY-LIFECYCLE-CORE-POLICY:V1\0";
const STATE_DOMAIN: &[u8] = b"MYCELIX:PUBLIC-ELECTION:VERIFIER-KEY-LIFECYCLE-STATE:V1\0";
const PROJECTION_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:VERIFIER-KEY-LIFECYCLE-SUCCESSOR-PROJECTION:V1\0";
const PROPOSAL_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:VERIFIER-KEY-LIFECYCLE-PROPOSAL:V1\0";

fn sha256(bytes: &[u8]) -> Digest32 {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize().into()
}

fn append_len_prefixed_utf8(bytes: &mut Vec<u8>, value: &str) -> Result<(), ()> {
    let raw = value.as_bytes();
    if raw.len() > MAX_CANONICAL_STRING_BYTES {
        return Err(());
    }
    let length = u32::try_from(raw.len()).map_err(|_| ())?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(raw);
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifierKeyLifecycleCorePolicyV1 {
    pub lifecycle_profile_id: String,
    pub verifier_key_authorization_root_digest: Digest32,
    pub authorization_policy_digest: Digest32,
    pub verifier_public_key_bundle_digest: Digest32,
    pub xenia_authentication_profile_digest: Digest32,
    pub xenia_authentication_suite_registry_digest: Digest32,
    pub max_transitions: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LifecycleCorePolicyViolation {
    ParentAuthorizationRoot(VerifierKeyAuthorizationRootViolation),
    AuthorizationPolicy(VerifierKeyAuthorizationPolicyViolation),
    PublicKeyBundle(VerifierPublicKeyBundleViolation),
    WrongLifecycleProfile,
    CanonicalStringTooLong,
    AuthorizationRootDigestMismatch,
    AuthorizationPolicyDigestMismatch,
    PublicKeyBundleDigestMismatch,
    WrongXeniaAuthenticationProfile,
    WrongXeniaAuthenticationSuiteRegistry,
    WrongTransitionLimit,
}

pub fn validate_verifier_key_lifecycle_core_policy(
    policy: &VerifierKeyLifecycleCorePolicyV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Result<(), LifecycleCorePolicyViolation> {
    validate_verifier_key_authorization_root(authorization_root, authorization_policy)
        .map_err(LifecycleCorePolicyViolation::ParentAuthorizationRoot)?;
    validate_verifier_authentication_public_key_bundle(
        public_key_bundle,
        authorization_root,
        authorization_policy,
    )
    .map_err(LifecycleCorePolicyViolation::PublicKeyBundle)?;

    if policy.lifecycle_profile_id != VERIFIER_KEY_LIFECYCLE_CORE_PROFILE_ID {
        return Err(LifecycleCorePolicyViolation::WrongLifecycleProfile);
    }
    if policy.lifecycle_profile_id.len() > MAX_CANONICAL_STRING_BYTES {
        return Err(LifecycleCorePolicyViolation::CanonicalStringTooLong);
    }

    let expected_root_digest =
        verifier_key_authorization_root_digest(authorization_root, authorization_policy)
            .map_err(LifecycleCorePolicyViolation::ParentAuthorizationRoot)?;
    if policy.verifier_key_authorization_root_digest != expected_root_digest {
        return Err(LifecycleCorePolicyViolation::AuthorizationRootDigestMismatch);
    }

    let expected_authorization_policy_digest =
        verifier_key_authorization_policy_digest(authorization_policy)
            .map_err(LifecycleCorePolicyViolation::AuthorizationPolicy)?;
    if policy.authorization_policy_digest != expected_authorization_policy_digest
        || authorization_root.authorization_policy_digest != expected_authorization_policy_digest
    {
        return Err(LifecycleCorePolicyViolation::AuthorizationPolicyDigestMismatch);
    }

    let expected_bundle_digest = verifier_authentication_public_key_bundle_digest(
        public_key_bundle,
        authorization_root,
        authorization_policy,
    )
    .map_err(LifecycleCorePolicyViolation::PublicKeyBundle)?;
    if policy.verifier_public_key_bundle_digest != expected_bundle_digest {
        return Err(LifecycleCorePolicyViolation::PublicKeyBundleDigestMismatch);
    }

    if policy.xenia_authentication_profile_digest != XENIA_AUTHENTICATION_PROFILE_V1_SHA256 {
        return Err(LifecycleCorePolicyViolation::WrongXeniaAuthenticationProfile);
    }
    if policy.xenia_authentication_suite_registry_digest
        != XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256
    {
        return Err(LifecycleCorePolicyViolation::WrongXeniaAuthenticationSuiteRegistry);
    }
    if policy.max_transitions != MAX_LIFECYCLE_TRANSITIONS_V1 {
        return Err(LifecycleCorePolicyViolation::WrongTransitionLimit);
    }

    Ok(())
}

pub fn canonical_verifier_key_lifecycle_core_policy_bytes(
    policy: &VerifierKeyLifecycleCorePolicyV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Result<Vec<u8>, LifecycleCorePolicyViolation> {
    validate_verifier_key_lifecycle_core_policy(
        policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )?;

    let mut bytes = Vec::with_capacity(320);
    bytes.extend_from_slice(POLICY_DOMAIN);
    append_len_prefixed_utf8(&mut bytes, &policy.lifecycle_profile_id)
        .map_err(|_| LifecycleCorePolicyViolation::CanonicalStringTooLong)?;
    bytes.extend_from_slice(&policy.verifier_key_authorization_root_digest);
    bytes.extend_from_slice(&policy.authorization_policy_digest);
    bytes.extend_from_slice(&policy.verifier_public_key_bundle_digest);
    bytes.extend_from_slice(&policy.xenia_authentication_profile_digest);
    bytes.extend_from_slice(&policy.xenia_authentication_suite_registry_digest);
    bytes.extend_from_slice(&policy.max_transitions.to_be_bytes());
    Ok(bytes)
}

pub fn verifier_key_lifecycle_core_policy_digest(
    policy: &VerifierKeyLifecycleCorePolicyV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Result<Digest32, LifecycleCorePolicyViolation> {
    Ok(sha256(
        &canonical_verifier_key_lifecycle_core_policy_bytes(
            policy,
            authorization_root,
            authorization_policy,
            public_key_bundle,
        )?,
    ))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VerifierKeyLifecycleStatusV1 {
    Active {
        ed25519_signer_key_id: Digest32,
        ed25519_public_key: Vec<u8>,
        ml_dsa_65_signer_key_id: Digest32,
        ml_dsa_65_public_key: Vec<u8>,
    },
    Disabled,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifierKeyLifecycleRecordV1 {
    verifier_release_digest: Digest32,
    verifier_lineage_digest: Digest32,
    builder_control_domain_digest: Digest32,
    status: VerifierKeyLifecycleStatusV1,
}

impl VerifierKeyLifecycleRecordV1 {
    pub fn verifier_release_digest(&self) -> Digest32 {
        self.verifier_release_digest
    }

    pub fn verifier_lineage_digest(&self) -> Digest32 {
        self.verifier_lineage_digest
    }

    pub fn builder_control_domain_digest(&self) -> Digest32 {
        self.builder_control_domain_digest
    }

    pub fn status(&self) -> &VerifierKeyLifecycleStatusV1 {
        &self.status
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifierKeyLifecycleStateV1 {
    lifecycle_profile_id: String,
    lifecycle_policy_digest: Digest32,
    verifier_key_authorization_root_digest: Digest32,
    xenia_authentication_profile_digest: Digest32,
    sequence: u32,
    predecessor_state_digest: Digest32,
    authorization_event_digest: Digest32,
    records: Vec<VerifierKeyLifecycleRecordV1>,
    retired_ed25519_signer_key_ids: Vec<Digest32>,
    retired_ml_dsa_65_signer_key_ids: Vec<Digest32>,
}

impl VerifierKeyLifecycleStateV1 {
    pub fn sequence(&self) -> u32 {
        self.sequence
    }

    pub fn predecessor_state_digest(&self) -> Digest32 {
        self.predecessor_state_digest
    }

    pub fn authorization_event_digest(&self) -> Digest32 {
        self.authorization_event_digest
    }

    pub fn records(&self) -> &[VerifierKeyLifecycleRecordV1] {
        &self.records
    }

    pub fn retired_ed25519_signer_key_ids(&self) -> &[Digest32] {
        &self.retired_ed25519_signer_key_ids
    }

    pub fn retired_ml_dsa_65_signer_key_ids(&self) -> &[Digest32] {
        &self.retired_ml_dsa_65_signer_key_ids
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LifecycleStateViolation {
    Policy(LifecycleCorePolicyViolation),
    WrongLifecycleProfile,
    LifecyclePolicyDigestMismatch,
    AuthorizationRootDigestMismatch,
    WrongXeniaAuthenticationProfile,
    SequenceExceedsLimit,
    GenesisPredecessorMustBeZero,
    GenesisAuthorizationEventMustBeZero,
    NonGenesisPredecessorMustBeNonzero,
    NonGenesisAuthorizationEventMustBeNonzero,
    RetiredHistoryLengthMismatch,
    RetiredEd25519SetNotCanonical,
    RetiredMlDsa65SetNotCanonical,
    RecordCountMismatch,
    DuplicateVerifierRelease,
    UnexpectedVerifierRelease,
    FrozenVerifierIdentityMismatch,
    MissingVerifierRelease,
    UnknownAuthenticationSuite,
    ActiveEd25519SignerIdMismatch,
    ActiveMlDsa65SignerIdMismatch,
    DuplicateActiveEd25519SignerId,
    DuplicateActiveMlDsa65SignerId,
    ActiveEd25519SignerIdWasRetired,
    ActiveMlDsa65SignerIdWasRetired,
    GenesisRecordsMismatch,
    RecordCountOverflow,
    RetiredCountOverflow,
    PublicKeyLengthOverflow,
}

fn evidence_by_release_and_suite<'a>(
    bundle: &'a VerifierAuthenticationPublicKeyBundleV1,
) -> HashMap<(Digest32, u16), &'a VerifierAuthenticationPublicKeyEvidenceV1> {
    bundle
        .evidence
        .iter()
        .map(|record| {
            (
                (record.verifier_release_digest, record.authentication_suite_id),
                record,
            )
        })
        .collect()
}

fn genesis_records(
    authorization_root: &VerifierKeyAuthorizationRootV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Vec<VerifierKeyLifecycleRecordV1> {
    let evidence = evidence_by_release_and_suite(public_key_bundle);
    let mut records = authorization_root
        .authorized_verifiers
        .iter()
        .map(|verifier| {
            let ed = evidence
                .get(&(
                    verifier.verifier_release_digest,
                    XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                ))
                .expect("validated bundle must contain Ed25519 evidence");
            let ml = evidence
                .get(&(
                    verifier.verifier_release_digest,
                    XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                ))
                .expect("validated bundle must contain ML-DSA-65 evidence");
            VerifierKeyLifecycleRecordV1 {
                verifier_release_digest: verifier.verifier_release_digest,
                verifier_lineage_digest: verifier.verifier_lineage_digest,
                builder_control_domain_digest: verifier.builder_control_domain_digest,
                status: VerifierKeyLifecycleStatusV1::Active {
                    ed25519_signer_key_id: ed.signer_key_id,
                    ed25519_public_key: ed.public_key_bytes.clone(),
                    ml_dsa_65_signer_key_id: ml.signer_key_id,
                    ml_dsa_65_public_key: ml.public_key_bytes.clone(),
                },
            }
        })
        .collect::<Vec<_>>();
    records.sort_by_key(|record| record.verifier_release_digest);
    records
}

fn is_strictly_sorted_unique(values: &[Digest32]) -> bool {
    values.windows(2).all(|pair| pair[0] < pair[1])
}

fn validate_record_population(
    state: &VerifierKeyLifecycleStateV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
) -> Result<(), LifecycleStateViolation> {
    if state.records.len() != authorization_root.authorized_verifiers.len() {
        return Err(LifecycleStateViolation::RecordCountMismatch);
    }

    let root_by_release = authorization_root
        .authorized_verifiers
        .iter()
        .map(|verifier| (verifier.verifier_release_digest, verifier))
        .collect::<HashMap<_, _>>();

    let mut releases = HashSet::with_capacity(state.records.len());
    let mut active_ed = HashSet::with_capacity(state.records.len());
    let mut active_ml = HashSet::with_capacity(state.records.len());
    let retired_ed = state
        .retired_ed25519_signer_key_ids
        .iter()
        .copied()
        .collect::<HashSet<_>>();
    let retired_ml = state
        .retired_ml_dsa_65_signer_key_ids
        .iter()
        .copied()
        .collect::<HashSet<_>>();

    for record in &state.records {
        if !releases.insert(record.verifier_release_digest) {
            return Err(LifecycleStateViolation::DuplicateVerifierRelease);
        }
        let Some(frozen) = root_by_release.get(&record.verifier_release_digest) else {
            return Err(LifecycleStateViolation::UnexpectedVerifierRelease);
        };
        if record.verifier_lineage_digest != frozen.verifier_lineage_digest
            || record.builder_control_domain_digest != frozen.builder_control_domain_digest
        {
            return Err(LifecycleStateViolation::FrozenVerifierIdentityMismatch);
        }

        if let VerifierKeyLifecycleStatusV1::Active {
            ed25519_signer_key_id,
            ed25519_public_key,
            ml_dsa_65_signer_key_id,
            ml_dsa_65_public_key,
        } = &record.status
        {
            let recomputed_ed = xenia_authentication_signer_key_id_v1(
                XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                ed25519_public_key,
            )
            .map_err(|_| LifecycleStateViolation::ActiveEd25519SignerIdMismatch)?;
            if recomputed_ed != *ed25519_signer_key_id {
                return Err(LifecycleStateViolation::ActiveEd25519SignerIdMismatch);
            }

            let recomputed_ml = xenia_authentication_signer_key_id_v1(
                XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                ml_dsa_65_public_key,
            )
            .map_err(|_| LifecycleStateViolation::ActiveMlDsa65SignerIdMismatch)?;
            if recomputed_ml != *ml_dsa_65_signer_key_id {
                return Err(LifecycleStateViolation::ActiveMlDsa65SignerIdMismatch);
            }

            if !active_ed.insert(*ed25519_signer_key_id) {
                return Err(LifecycleStateViolation::DuplicateActiveEd25519SignerId);
            }
            if !active_ml.insert(*ml_dsa_65_signer_key_id) {
                return Err(LifecycleStateViolation::DuplicateActiveMlDsa65SignerId);
            }
            if retired_ed.contains(ed25519_signer_key_id) {
                return Err(LifecycleStateViolation::ActiveEd25519SignerIdWasRetired);
            }
            if retired_ml.contains(ml_dsa_65_signer_key_id) {
                return Err(LifecycleStateViolation::ActiveMlDsa65SignerIdWasRetired);
            }
        }
    }

    for frozen in &authorization_root.authorized_verifiers {
        if !releases.contains(&frozen.verifier_release_digest) {
            return Err(LifecycleStateViolation::MissingVerifierRelease);
        }
    }

    Ok(())
}

pub fn validate_verifier_key_lifecycle_state(
    state: &VerifierKeyLifecycleStateV1,
    lifecycle_policy: &VerifierKeyLifecycleCorePolicyV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Result<(), LifecycleStateViolation> {
    validate_verifier_key_lifecycle_core_policy(
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )
    .map_err(LifecycleStateViolation::Policy)?;

    if state.lifecycle_profile_id != VERIFIER_KEY_LIFECYCLE_CORE_PROFILE_ID {
        return Err(LifecycleStateViolation::WrongLifecycleProfile);
    }

    let expected_policy_digest = verifier_key_lifecycle_core_policy_digest(
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )
    .map_err(LifecycleStateViolation::Policy)?;
    if state.lifecycle_policy_digest != expected_policy_digest {
        return Err(LifecycleStateViolation::LifecyclePolicyDigestMismatch);
    }
    if state.verifier_key_authorization_root_digest
        != lifecycle_policy.verifier_key_authorization_root_digest
    {
        return Err(LifecycleStateViolation::AuthorizationRootDigestMismatch);
    }
    if state.xenia_authentication_profile_digest != XENIA_AUTHENTICATION_PROFILE_V1_SHA256 {
        return Err(LifecycleStateViolation::WrongXeniaAuthenticationProfile);
    }
    if state.sequence > MAX_LIFECYCLE_TRANSITIONS_V1 {
        return Err(LifecycleStateViolation::SequenceExceedsLimit);
    }

    let expected_retired_len =
        usize::try_from(state.sequence).map_err(|_| LifecycleStateViolation::RetiredCountOverflow)?;
    if state.retired_ed25519_signer_key_ids.len() != expected_retired_len
        || state.retired_ml_dsa_65_signer_key_ids.len() != expected_retired_len
    {
        return Err(LifecycleStateViolation::RetiredHistoryLengthMismatch);
    }
    if !is_strictly_sorted_unique(&state.retired_ed25519_signer_key_ids)
        && !state.retired_ed25519_signer_key_ids.is_empty()
    {
        return Err(LifecycleStateViolation::RetiredEd25519SetNotCanonical);
    }
    if !is_strictly_sorted_unique(&state.retired_ml_dsa_65_signer_key_ids)
        && !state.retired_ml_dsa_65_signer_key_ids.is_empty()
    {
        return Err(LifecycleStateViolation::RetiredMlDsa65SetNotCanonical);
    }

    if state.sequence == 0 {
        if state.predecessor_state_digest != [0_u8; 32] {
            return Err(LifecycleStateViolation::GenesisPredecessorMustBeZero);
        }
        if state.authorization_event_digest != [0_u8; 32] {
            return Err(LifecycleStateViolation::GenesisAuthorizationEventMustBeZero);
        }
    } else {
        if state.predecessor_state_digest == [0_u8; 32] {
            return Err(LifecycleStateViolation::NonGenesisPredecessorMustBeNonzero);
        }
        if state.authorization_event_digest == [0_u8; 32] {
            return Err(LifecycleStateViolation::NonGenesisAuthorizationEventMustBeNonzero);
        }
    }

    validate_record_population(state, authorization_root)?;

    if state.sequence == 0 && state.records != genesis_records(authorization_root, public_key_bundle) {
        return Err(LifecycleStateViolation::GenesisRecordsMismatch);
    }

    Ok(())
}

fn append_record_population(
    bytes: &mut Vec<u8>,
    records: &[VerifierKeyLifecycleRecordV1],
) -> Result<(), LifecycleStateViolation> {
    let mut ordered = records.to_vec();
    ordered.sort_by_key(|record| record.verifier_release_digest);
    let count =
        u16::try_from(ordered.len()).map_err(|_| LifecycleStateViolation::RecordCountOverflow)?;
    bytes.extend_from_slice(&count.to_be_bytes());

    for record in ordered {
        bytes.extend_from_slice(&record.verifier_release_digest);
        bytes.extend_from_slice(&record.verifier_lineage_digest);
        bytes.extend_from_slice(&record.builder_control_domain_digest);
        match record.status {
            VerifierKeyLifecycleStatusV1::Active {
                ed25519_signer_key_id,
                ed25519_public_key,
                ml_dsa_65_signer_key_id,
                ml_dsa_65_public_key,
            } => {
                bytes.push(1);
                bytes.extend_from_slice(&ed25519_signer_key_id);
                let ed_len = u32::try_from(ed25519_public_key.len())
                    .map_err(|_| LifecycleStateViolation::PublicKeyLengthOverflow)?;
                bytes.extend_from_slice(&ed_len.to_be_bytes());
                bytes.extend_from_slice(&ed25519_public_key);

                bytes.extend_from_slice(&ml_dsa_65_signer_key_id);
                let ml_len = u32::try_from(ml_dsa_65_public_key.len())
                    .map_err(|_| LifecycleStateViolation::PublicKeyLengthOverflow)?;
                bytes.extend_from_slice(&ml_len.to_be_bytes());
                bytes.extend_from_slice(&ml_dsa_65_public_key);
            }
            VerifierKeyLifecycleStatusV1::Disabled => bytes.push(2),
        }
    }

    Ok(())
}

fn append_retired_set(
    bytes: &mut Vec<u8>,
    retired: &[Digest32],
) -> Result<(), LifecycleStateViolation> {
    let count =
        u16::try_from(retired.len()).map_err(|_| LifecycleStateViolation::RetiredCountOverflow)?;
    bytes.extend_from_slice(&count.to_be_bytes());
    for digest in retired {
        bytes.extend_from_slice(digest);
    }
    Ok(())
}

pub fn canonical_verifier_key_lifecycle_state_bytes(
    state: &VerifierKeyLifecycleStateV1,
    lifecycle_policy: &VerifierKeyLifecycleCorePolicyV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Result<Vec<u8>, LifecycleStateViolation> {
    validate_verifier_key_lifecycle_state(
        state,
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )?;

    let mut bytes = Vec::with_capacity(8192);
    bytes.extend_from_slice(STATE_DOMAIN);
    append_len_prefixed_utf8(&mut bytes, &state.lifecycle_profile_id)
        .map_err(|_| LifecycleStateViolation::WrongLifecycleProfile)?;
    bytes.extend_from_slice(&state.lifecycle_policy_digest);
    bytes.extend_from_slice(&state.verifier_key_authorization_root_digest);
    bytes.extend_from_slice(&state.xenia_authentication_profile_digest);
    bytes.extend_from_slice(&state.sequence.to_be_bytes());
    bytes.extend_from_slice(&state.predecessor_state_digest);
    bytes.extend_from_slice(&state.authorization_event_digest);
    append_record_population(&mut bytes, &state.records)?;
    append_retired_set(&mut bytes, &state.retired_ed25519_signer_key_ids)?;
    append_retired_set(&mut bytes, &state.retired_ml_dsa_65_signer_key_ids)?;
    Ok(bytes)
}

pub fn verifier_key_lifecycle_state_digest(
    state: &VerifierKeyLifecycleStateV1,
    lifecycle_policy: &VerifierKeyLifecycleCorePolicyV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Result<Digest32, LifecycleStateViolation> {
    Ok(sha256(&canonical_verifier_key_lifecycle_state_bytes(
        state,
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )?))
}

pub fn derive_root_lifecycle_state(
    lifecycle_policy: &VerifierKeyLifecycleCorePolicyV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Result<VerifierKeyLifecycleStateV1, LifecycleStateViolation> {
    validate_verifier_key_lifecycle_core_policy(
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )
    .map_err(LifecycleStateViolation::Policy)?;

    let lifecycle_policy_digest = verifier_key_lifecycle_core_policy_digest(
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )
    .map_err(LifecycleStateViolation::Policy)?;

    let state = VerifierKeyLifecycleStateV1 {
        lifecycle_profile_id: VERIFIER_KEY_LIFECYCLE_CORE_PROFILE_ID.to_owned(),
        lifecycle_policy_digest,
        verifier_key_authorization_root_digest: lifecycle_policy
            .verifier_key_authorization_root_digest,
        xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
        sequence: 0,
        predecessor_state_digest: [0_u8; 32],
        authorization_event_digest: [0_u8; 32],
        records: genesis_records(authorization_root, public_key_bundle),
        retired_ed25519_signer_key_ids: Vec::new(),
        retired_ml_dsa_65_signer_key_ids: Vec::new(),
    };

    validate_verifier_key_lifecycle_state(
        &state,
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )?;
    Ok(state)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LifecycleTransitionOperationV1 {
    RotateKeyPair {
        new_ed25519_signer_key_id: Digest32,
        new_ed25519_public_key: Vec<u8>,
        new_ml_dsa_65_signer_key_id: Digest32,
        new_ml_dsa_65_public_key: Vec<u8>,
    },
    DisableVerifier,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LifecycleTransitionRequestV1 {
    pub lifecycle_policy_digest: Digest32,
    pub current_state_digest: Digest32,
    pub next_sequence: u32,
    pub target_verifier_release_digest: Digest32,
    pub expected_current_ed25519_signer_key_id: Digest32,
    pub expected_current_ml_dsa_65_signer_key_id: Digest32,
    pub operation: LifecycleTransitionOperationV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CertificationCapacityV1 {
    Sufficient,
    Insufficient {
        active_verifiers: u16,
        distinct_implementation_lineages: u16,
        distinct_builder_control_domains: u16,
        required_verifiers: u16,
        required_implementation_lineages: u16,
        required_builder_control_domains: u16,
    },
}

pub fn assess_certification_capacity(
    records: &[VerifierKeyLifecycleRecordV1],
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
) -> CertificationCapacityV1 {
    let active = records
        .iter()
        .filter(|record| matches!(record.status, VerifierKeyLifecycleStatusV1::Active { .. }))
        .collect::<Vec<_>>();
    let lineages = active
        .iter()
        .map(|record| record.verifier_lineage_digest)
        .collect::<HashSet<_>>();
    let builders = active
        .iter()
        .map(|record| record.builder_control_domain_digest)
        .collect::<HashSet<_>>();

    let active_count = u16::try_from(active.len()).unwrap_or(u16::MAX);
    let lineage_count = u16::try_from(lineages.len()).unwrap_or(u16::MAX);
    let builder_count = u16::try_from(builders.len()).unwrap_or(u16::MAX);

    if active_count >= authorization_policy.minimum_total_verifiers
        && lineage_count >= authorization_policy.minimum_distinct_implementation_lineages
        && builder_count >= authorization_policy.minimum_distinct_builder_control_domains
    {
        CertificationCapacityV1::Sufficient
    } else {
        CertificationCapacityV1::Insufficient {
            active_verifiers: active_count,
            distinct_implementation_lineages: lineage_count,
            distinct_builder_control_domains: builder_count,
            required_verifiers: authorization_policy.minimum_total_verifiers,
            required_implementation_lineages: authorization_policy
                .minimum_distinct_implementation_lineages,
            required_builder_control_domains: authorization_policy
                .minimum_distinct_builder_control_domains,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProjectedLifecycleSuccessorV1 {
    lifecycle_profile_id: String,
    lifecycle_policy_digest: Digest32,
    verifier_key_authorization_root_digest: Digest32,
    xenia_authentication_profile_digest: Digest32,
    sequence: u32,
    predecessor_state_digest: Digest32,
    records: Vec<VerifierKeyLifecycleRecordV1>,
    retired_ed25519_signer_key_ids: Vec<Digest32>,
    retired_ml_dsa_65_signer_key_ids: Vec<Digest32>,
    certification_capacity: CertificationCapacityV1,
}

impl ProjectedLifecycleSuccessorV1 {
    pub fn sequence(&self) -> u32 {
        self.sequence
    }

    pub fn predecessor_state_digest(&self) -> Digest32 {
        self.predecessor_state_digest
    }

    pub fn records(&self) -> &[VerifierKeyLifecycleRecordV1] {
        &self.records
    }

    pub fn retired_ed25519_signer_key_ids(&self) -> &[Digest32] {
        &self.retired_ed25519_signer_key_ids
    }

    pub fn retired_ml_dsa_65_signer_key_ids(&self) -> &[Digest32] {
        &self.retired_ml_dsa_65_signer_key_ids
    }

    pub fn certification_capacity(&self) -> CertificationCapacityV1 {
        self.certification_capacity
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValidLifecycleProposalV1 {
    proposal_digest: Digest32,
    lifecycle_policy_digest: Digest32,
    current_state_digest: Digest32,
    next_sequence: u32,
    target_verifier_release_digest: Digest32,
    successor_projection_digest: Digest32,
}

impl ValidLifecycleProposalV1 {
    pub fn proposal_digest(&self) -> Digest32 {
        self.proposal_digest
    }

    pub fn lifecycle_policy_digest(&self) -> Digest32 {
        self.lifecycle_policy_digest
    }

    pub fn current_state_digest(&self) -> Digest32 {
        self.current_state_digest
    }

    pub fn next_sequence(&self) -> u32 {
        self.next_sequence
    }

    pub fn target_verifier_release_digest(&self) -> Digest32 {
        self.target_verifier_release_digest
    }

    pub fn successor_projection_digest(&self) -> Digest32 {
        self.successor_projection_digest
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LifecycleTransitionViolation {
    Policy(LifecycleCorePolicyViolation),
    CurrentState(LifecycleStateViolation),
    LifecyclePolicyDigestMismatch,
    CurrentStateDigestMismatch,
    SequenceLimitReached,
    WrongNextSequence,
    UnknownTargetVerifierRelease,
    TargetVerifierDisabled,
    CurrentKeyExpectationMismatch,
    WrongEd25519ReplacementSignerId,
    WrongMlDsa65ReplacementSignerId,
    ActiveEd25519KeyReuse,
    ActiveMlDsa65KeyReuse,
    RetiredEd25519KeyReuse,
    RetiredMlDsa65KeyReuse,
    SignerKeyId(XeniaSignerKeyIdViolation),
    ProjectionEncoding(LifecycleStateViolation),
    ProposalPublicKeyLengthOverflow,
}

fn active_key_ids(
    records: &[VerifierKeyLifecycleRecordV1],
) -> (HashSet<Digest32>, HashSet<Digest32>) {
    let mut ed = HashSet::new();
    let mut ml = HashSet::new();
    for record in records {
        if let VerifierKeyLifecycleStatusV1::Active {
            ed25519_signer_key_id,
            ml_dsa_65_signer_key_id,
            ..
        } = &record.status
        {
            ed.insert(*ed25519_signer_key_id);
            ml.insert(*ml_dsa_65_signer_key_id);
        }
    }
    (ed, ml)
}

fn canonical_successor_projection_bytes(
    projection: &ProjectedLifecycleSuccessorV1,
) -> Result<Vec<u8>, LifecycleStateViolation> {
    let mut bytes = Vec::with_capacity(8192);
    bytes.extend_from_slice(PROJECTION_DOMAIN);
    append_len_prefixed_utf8(&mut bytes, &projection.lifecycle_profile_id)
        .map_err(|_| LifecycleStateViolation::WrongLifecycleProfile)?;
    bytes.extend_from_slice(&projection.lifecycle_policy_digest);
    bytes.extend_from_slice(&projection.verifier_key_authorization_root_digest);
    bytes.extend_from_slice(&projection.xenia_authentication_profile_digest);
    bytes.extend_from_slice(&projection.sequence.to_be_bytes());
    bytes.extend_from_slice(&projection.predecessor_state_digest);
    append_record_population(&mut bytes, &projection.records)?;
    append_retired_set(&mut bytes, &projection.retired_ed25519_signer_key_ids)?;
    append_retired_set(&mut bytes, &projection.retired_ml_dsa_65_signer_key_ids)?;
    Ok(bytes)
}

pub fn successor_projection_digest(
    projection: &ProjectedLifecycleSuccessorV1,
) -> Result<Digest32, LifecycleStateViolation> {
    Ok(sha256(&canonical_successor_projection_bytes(projection)?))
}

fn canonical_valid_proposal_bytes(
    request: &LifecycleTransitionRequestV1,
    successor_projection_digest: Digest32,
) -> Result<Vec<u8>, LifecycleTransitionViolation> {
    let mut bytes = Vec::with_capacity(4096);
    bytes.extend_from_slice(PROPOSAL_DOMAIN);
    append_len_prefixed_utf8(&mut bytes, VERIFIER_KEY_LIFECYCLE_CORE_PROFILE_ID)
        .map_err(|_| LifecycleTransitionViolation::ProposalPublicKeyLengthOverflow)?;
    bytes.extend_from_slice(&request.lifecycle_policy_digest);
    bytes.extend_from_slice(&request.current_state_digest);
    bytes.extend_from_slice(&request.next_sequence.to_be_bytes());
    bytes.extend_from_slice(&request.target_verifier_release_digest);
    bytes.extend_from_slice(&request.expected_current_ed25519_signer_key_id);
    bytes.extend_from_slice(&request.expected_current_ml_dsa_65_signer_key_id);

    match &request.operation {
        LifecycleTransitionOperationV1::RotateKeyPair {
            new_ed25519_signer_key_id,
            new_ed25519_public_key,
            new_ml_dsa_65_signer_key_id,
            new_ml_dsa_65_public_key,
        } => {
            bytes.push(1);
            bytes.extend_from_slice(new_ed25519_signer_key_id);
            let ed_len = u32::try_from(new_ed25519_public_key.len())
                .map_err(|_| LifecycleTransitionViolation::ProposalPublicKeyLengthOverflow)?;
            bytes.extend_from_slice(&ed_len.to_be_bytes());
            bytes.extend_from_slice(new_ed25519_public_key);
            bytes.extend_from_slice(new_ml_dsa_65_signer_key_id);
            let ml_len = u32::try_from(new_ml_dsa_65_public_key.len())
                .map_err(|_| LifecycleTransitionViolation::ProposalPublicKeyLengthOverflow)?;
            bytes.extend_from_slice(&ml_len.to_be_bytes());
            bytes.extend_from_slice(new_ml_dsa_65_public_key);
        }
        LifecycleTransitionOperationV1::DisableVerifier => bytes.push(2),
    }
    bytes.extend_from_slice(&successor_projection_digest);
    Ok(bytes)
}

pub fn validate_and_project_lifecycle_transition(
    lifecycle_policy: &VerifierKeyLifecycleCorePolicyV1,
    current_state: &VerifierKeyLifecycleStateV1,
    request: &LifecycleTransitionRequestV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Result<(ValidLifecycleProposalV1, ProjectedLifecycleSuccessorV1), LifecycleTransitionViolation> {
    validate_verifier_key_lifecycle_core_policy(
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )
    .map_err(LifecycleTransitionViolation::Policy)?;
    validate_verifier_key_lifecycle_state(
        current_state,
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )
    .map_err(LifecycleTransitionViolation::CurrentState)?;

    let expected_policy_digest = verifier_key_lifecycle_core_policy_digest(
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )
    .map_err(LifecycleTransitionViolation::Policy)?;
    if request.lifecycle_policy_digest != expected_policy_digest {
        return Err(LifecycleTransitionViolation::LifecyclePolicyDigestMismatch);
    }

    let expected_current_state_digest = verifier_key_lifecycle_state_digest(
        current_state,
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )
    .map_err(LifecycleTransitionViolation::CurrentState)?;
    if request.current_state_digest != expected_current_state_digest {
        return Err(LifecycleTransitionViolation::CurrentStateDigestMismatch);
    }
    if current_state.sequence >= MAX_LIFECYCLE_TRANSITIONS_V1 {
        return Err(LifecycleTransitionViolation::SequenceLimitReached);
    }
    if request.next_sequence != current_state.sequence + 1 {
        return Err(LifecycleTransitionViolation::WrongNextSequence);
    }

    let target_index = current_state
        .records
        .iter()
        .position(|record| {
            record.verifier_release_digest == request.target_verifier_release_digest
        })
        .ok_or(LifecycleTransitionViolation::UnknownTargetVerifierRelease)?;

    let (
        current_ed25519_signer_key_id,
        current_ml_dsa_65_signer_key_id,
    ) = match &current_state.records[target_index].status {
        VerifierKeyLifecycleStatusV1::Active {
            ed25519_signer_key_id,
            ml_dsa_65_signer_key_id,
            ..
        } => (*ed25519_signer_key_id, *ml_dsa_65_signer_key_id),
        VerifierKeyLifecycleStatusV1::Disabled => {
            return Err(LifecycleTransitionViolation::TargetVerifierDisabled);
        }
    };

    if request.expected_current_ed25519_signer_key_id != current_ed25519_signer_key_id
        || request.expected_current_ml_dsa_65_signer_key_id != current_ml_dsa_65_signer_key_id
    {
        return Err(LifecycleTransitionViolation::CurrentKeyExpectationMismatch);
    }

    let mut records = current_state.records.clone();
    let mut retired_ed = current_state.retired_ed25519_signer_key_ids.clone();
    let mut retired_ml = current_state.retired_ml_dsa_65_signer_key_ids.clone();
    let (active_ed, active_ml) = active_key_ids(&current_state.records);
    let retired_ed_set = retired_ed.iter().copied().collect::<HashSet<_>>();
    let retired_ml_set = retired_ml.iter().copied().collect::<HashSet<_>>();

    match &request.operation {
        LifecycleTransitionOperationV1::RotateKeyPair {
            new_ed25519_signer_key_id,
            new_ed25519_public_key,
            new_ml_dsa_65_signer_key_id,
            new_ml_dsa_65_public_key,
        } => {
            let recomputed_ed = xenia_authentication_signer_key_id_v1(
                XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                new_ed25519_public_key,
            )
            .map_err(LifecycleTransitionViolation::SignerKeyId)?;
            if recomputed_ed != *new_ed25519_signer_key_id {
                return Err(LifecycleTransitionViolation::WrongEd25519ReplacementSignerId);
            }

            let recomputed_ml = xenia_authentication_signer_key_id_v1(
                XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                new_ml_dsa_65_public_key,
            )
            .map_err(LifecycleTransitionViolation::SignerKeyId)?;
            if recomputed_ml != *new_ml_dsa_65_signer_key_id {
                return Err(LifecycleTransitionViolation::WrongMlDsa65ReplacementSignerId);
            }

            if active_ed.contains(new_ed25519_signer_key_id) {
                return Err(LifecycleTransitionViolation::ActiveEd25519KeyReuse);
            }
            if active_ml.contains(new_ml_dsa_65_signer_key_id) {
                return Err(LifecycleTransitionViolation::ActiveMlDsa65KeyReuse);
            }
            if retired_ed_set.contains(new_ed25519_signer_key_id) {
                return Err(LifecycleTransitionViolation::RetiredEd25519KeyReuse);
            }
            if retired_ml_set.contains(new_ml_dsa_65_signer_key_id) {
                return Err(LifecycleTransitionViolation::RetiredMlDsa65KeyReuse);
            }

            records[target_index].status = VerifierKeyLifecycleStatusV1::Active {
                ed25519_signer_key_id: *new_ed25519_signer_key_id,
                ed25519_public_key: new_ed25519_public_key.clone(),
                ml_dsa_65_signer_key_id: *new_ml_dsa_65_signer_key_id,
                ml_dsa_65_public_key: new_ml_dsa_65_public_key.clone(),
            };
        }
        LifecycleTransitionOperationV1::DisableVerifier => {
            records[target_index].status = VerifierKeyLifecycleStatusV1::Disabled;
        }
    }

    retired_ed.push(current_ed25519_signer_key_id);
    retired_ed.sort_unstable();
    retired_ml.push(current_ml_dsa_65_signer_key_id);
    retired_ml.sort_unstable();

    let projection = ProjectedLifecycleSuccessorV1 {
        lifecycle_profile_id: VERIFIER_KEY_LIFECYCLE_CORE_PROFILE_ID.to_owned(),
        lifecycle_policy_digest: expected_policy_digest,
        verifier_key_authorization_root_digest: lifecycle_policy
            .verifier_key_authorization_root_digest,
        xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
        sequence: request.next_sequence,
        predecessor_state_digest: expected_current_state_digest,
        certification_capacity: assess_certification_capacity(&records, authorization_policy),
        records,
        retired_ed25519_signer_key_ids: retired_ed,
        retired_ml_dsa_65_signer_key_ids: retired_ml,
    };

    let projection_digest = successor_projection_digest(&projection)
        .map_err(LifecycleTransitionViolation::ProjectionEncoding)?;
    let proposal_digest = sha256(&canonical_valid_proposal_bytes(request, projection_digest)?);
    let valid_proposal = ValidLifecycleProposalV1 {
        proposal_digest,
        lifecycle_policy_digest: expected_policy_digest,
        current_state_digest: expected_current_state_digest,
        next_sequence: request.next_sequence,
        target_verifier_release_digest: request.target_verifier_release_digest,
        successor_projection_digest: projection_digest,
    };

    Ok((valid_proposal, projection))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LifecycleMaterializationViolation {
    ZeroAuthorizationEventDigest,
    State(LifecycleStateViolation),
}

pub fn materialize_structural_lifecycle_state(
    projection: &ProjectedLifecycleSuccessorV1,
    authorization_event_digest: Digest32,
    lifecycle_policy: &VerifierKeyLifecycleCorePolicyV1,
    authorization_root: &VerifierKeyAuthorizationRootV1,
    authorization_policy: &VerifierKeyAuthorizationPolicyV1,
    public_key_bundle: &VerifierAuthenticationPublicKeyBundleV1,
) -> Result<VerifierKeyLifecycleStateV1, LifecycleMaterializationViolation> {
    if authorization_event_digest == [0_u8; 32] {
        return Err(LifecycleMaterializationViolation::ZeroAuthorizationEventDigest);
    }

    let state = VerifierKeyLifecycleStateV1 {
        lifecycle_profile_id: projection.lifecycle_profile_id.clone(),
        lifecycle_policy_digest: projection.lifecycle_policy_digest,
        verifier_key_authorization_root_digest: projection.verifier_key_authorization_root_digest,
        xenia_authentication_profile_digest: projection.xenia_authentication_profile_digest,
        sequence: projection.sequence,
        predecessor_state_digest: projection.predecessor_state_digest,
        authorization_event_digest,
        records: projection.records.clone(),
        retired_ed25519_signer_key_ids: projection.retired_ed25519_signer_key_ids.clone(),
        retired_ml_dsa_65_signer_key_ids: projection.retired_ml_dsa_65_signer_key_ids.clone(),
    };
    validate_verifier_key_lifecycle_state(
        &state,
        lifecycle_policy,
        authorization_root,
        authorization_policy,
        public_key_bundle,
    )
    .map_err(LifecycleMaterializationViolation::State)?;
    Ok(state)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LifecycleForkEvidenceV1 {
    pub predecessor_state_digest: Digest32,
    pub sequence: u32,
    pub left_proposal_digest: Digest32,
    pub right_proposal_digest: Digest32,
    pub left_successor_projection_digest: Digest32,
    pub right_successor_projection_digest: Digest32,
}

pub fn detect_lifecycle_fork(
    left: &ValidLifecycleProposalV1,
    right: &ValidLifecycleProposalV1,
) -> Option<LifecycleForkEvidenceV1> {
    if left.current_state_digest == right.current_state_digest
        && left.next_sequence == right.next_sequence
        && left.proposal_digest != right.proposal_digest
    {
        Some(LifecycleForkEvidenceV1 {
            predecessor_state_digest: left.current_state_digest,
            sequence: left.next_sequence,
            left_proposal_digest: left.proposal_digest,
            right_proposal_digest: right.proposal_digest,
            left_successor_projection_digest: left.successor_projection_digest,
            right_successor_projection_digest: right.successor_projection_digest,
        })
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use election_verifier_key_authorization::{
        AuthorizedVerifierKeysetV1, VERIFIER_KEY_AUTHORIZATION_PROFILE_ID,
        verifier_key_authorization_policy_digest,
    };
    use election_verifier_public_key_binding::VERIFIER_PUBLIC_KEY_BINDING_PROFILE_ID;

    const GOLDEN_PARENT_ROOT_DIGEST: Digest32 = [
        0x4e, 0x52, 0xe8, 0xb4, 0x27, 0x61, 0x1e, 0x11, 0xb3, 0x6b, 0x0a, 0x22, 0xb8, 0xc8,
        0xab, 0x7b, 0x62, 0xbf, 0x80, 0x8d, 0x42, 0x23, 0xdb, 0x19, 0x55, 0xf5, 0x0c, 0xa9,
        0xb0, 0x2e, 0x65, 0xb5,
    ];
    const GOLDEN_PARENT_BUNDLE_DIGEST: Digest32 = [
        0x97, 0x56, 0x5d, 0x55, 0x4f, 0xfa, 0xdd, 0xbc, 0xc3, 0x5d, 0x2a, 0x22, 0xd2, 0x09,
        0xec, 0x1f, 0xb2, 0x64, 0x6c, 0x0f, 0x43, 0x03, 0x14, 0x6b, 0xb7, 0x3c, 0x3d, 0x96,
        0x40, 0xfd, 0xbe, 0xfc,
    ];
    const GOLDEN_LIFECYCLE_POLICY_DIGEST: Digest32 = [
        0xa0, 0x16, 0xda, 0xb4, 0x77, 0x25, 0x7e, 0x83, 0x14, 0x70, 0x87, 0xe8, 0x78, 0x79,
        0x44, 0xea, 0x38, 0xfd, 0xd0, 0x8f, 0xb1, 0x41, 0xb0, 0x76, 0xa7, 0xdb, 0x6c, 0xca,
        0x07, 0x83, 0xdd, 0xac,
    ];
    const GOLDEN_GENESIS_STATE_DIGEST: Digest32 = [
        0x3f, 0x63, 0xf5, 0x0f, 0xf8, 0x14, 0xe7, 0xd4, 0x40, 0xce, 0x10, 0x8c, 0x98, 0x1a,
        0x76, 0x16, 0xcc, 0x1c, 0xab, 0x78, 0xe5, 0x3d, 0x8f, 0x0b, 0x24, 0x95, 0x0d, 0x2e,
        0x99, 0x4a, 0xd8, 0xd1,
    ];
    const GOLDEN_ROTATION_PROJECTION_DIGEST: Digest32 = [
        0xca, 0x71, 0x07, 0xc5, 0xe6, 0x29, 0xa5, 0x99, 0x4b, 0x4c, 0xfd, 0x4a, 0x49, 0x41,
        0x3a, 0x92, 0x81, 0x40, 0x41, 0xd4, 0x80, 0x69, 0x6e, 0x4b, 0xcd, 0x58, 0x60, 0x7a,
        0x54, 0x95, 0xb1, 0x9c,
    ];
    const GOLDEN_ROTATION_PROPOSAL_DIGEST: Digest32 = [
        0x94, 0x8e, 0x8b, 0xac, 0x9e, 0x77, 0xaf, 0x3d, 0xf6, 0xfc, 0x76, 0x66, 0x8a, 0x2f,
        0x8d, 0x1e, 0x9a, 0x8c, 0x63, 0x0b, 0x70, 0x8a, 0x78, 0x16, 0x67, 0x8e, 0xa0, 0x5d,
        0xba, 0xa4, 0x9d, 0x61,
    ];

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn decode_hex(value: &str) -> Vec<u8> {
        (0..value.len())
            .step_by(2)
            .map(|index| u8::from_str_radix(&value[index..index + 2], 16).unwrap())
            .collect()
    }

    fn ed25519_public_key(index: usize) -> Vec<u8> {
        [
            "2152f8d19b791d24453242e15f2eab6cb7cffa7b6a5ed30097960e069881db12",
            "22fc297792f0b6ffc0bfcfdb7edb0c0aa14e025a365ec0e342e86e3829cb74b6",
            "d759793bbc13a2819a827c76adb6fba8a49aee007f49f2d0992d99b825ad2c48",
        ]
        .get(index)
        .map(|value| decode_hex(value))
        .unwrap()
    }

    fn ml_dsa_public_key(index: usize) -> Vec<u8> {
        vec![0x42 + u8::try_from(index).unwrap(); 1952]
    }

    fn authorization_policy() -> VerifierKeyAuthorizationPolicyV1 {
        VerifierKeyAuthorizationPolicyV1::default()
    }

    fn authorization_root() -> VerifierKeyAuthorizationRootV1 {
        let policy = authorization_policy();
        let mut authorized_verifiers = Vec::new();
        for index in 0..3 {
            let ed = ed25519_public_key(index);
            let ml = ml_dsa_public_key(index);
            authorized_verifiers.push(AuthorizedVerifierKeysetV1 {
                verifier_release_digest: digest(0x21 + u8::try_from(index).unwrap()),
                verifier_lineage_digest: digest(0x31 + u8::try_from(index).unwrap()),
                builder_control_domain_digest: digest(if index == 1 { 0x42 } else { 0x41 }),
                ed25519_signer_key_id: xenia_authentication_signer_key_id_v1(
                    XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                    &ed,
                )
                .unwrap(),
                ml_dsa_65_signer_key_id: xenia_authentication_signer_key_id_v1(
                    XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                    &ml,
                )
                .unwrap(),
            });
        }
        VerifierKeyAuthorizationRootV1 {
            verifier_key_authorization_profile_id: VERIFIER_KEY_AUTHORIZATION_PROFILE_ID.to_owned(),
            election_definition_digest: digest(0x11),
            jurisdiction_snapshot_digest: digest(0x12),
            authorization_policy_digest: verifier_key_authorization_policy_digest(&policy).unwrap(),
            authorized_verifiers,
        }
    }

    fn public_key_bundle() -> VerifierAuthenticationPublicKeyBundleV1 {
        let policy = authorization_policy();
        let root = authorization_root();
        let root_digest = verifier_key_authorization_root_digest(&root, &policy).unwrap();
        let mut evidence = Vec::new();

        for (index, verifier) in root.authorized_verifiers.iter().enumerate() {
            let ed = ed25519_public_key(index);
            let ml = ml_dsa_public_key(index);
            evidence.push(VerifierAuthenticationPublicKeyEvidenceV1 {
                verifier_release_digest: verifier.verifier_release_digest,
                authentication_suite_id: XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
                signer_key_id: xenia_authentication_signer_key_id_v1(
                    XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                    &ed,
                )
                .unwrap(),
                public_key_bytes: ed,
            });
            evidence.push(VerifierAuthenticationPublicKeyEvidenceV1 {
                verifier_release_digest: verifier.verifier_release_digest,
                authentication_suite_id: XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
                signer_key_id: xenia_authentication_signer_key_id_v1(
                    XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                    &ml,
                )
                .unwrap(),
                public_key_bytes: ml,
            });
        }

        VerifierAuthenticationPublicKeyBundleV1 {
            verifier_public_key_binding_profile_id: VERIFIER_PUBLIC_KEY_BINDING_PROFILE_ID.to_owned(),
            verifier_key_authorization_root_digest: root_digest,
            election_definition_digest: root.election_definition_digest,
            jurisdiction_snapshot_digest: root.jurisdiction_snapshot_digest,
            xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
            xenia_authentication_suite_registry_digest:
                XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256,
            evidence,
        }
    }

    fn lifecycle_policy() -> VerifierKeyLifecycleCorePolicyV1 {
        let authorization_policy = authorization_policy();
        let root = authorization_root();
        let bundle = public_key_bundle();
        VerifierKeyLifecycleCorePolicyV1 {
            lifecycle_profile_id: VERIFIER_KEY_LIFECYCLE_CORE_PROFILE_ID.to_owned(),
            verifier_key_authorization_root_digest: verifier_key_authorization_root_digest(
                &root,
                &authorization_policy,
            )
            .unwrap(),
            authorization_policy_digest: verifier_key_authorization_policy_digest(
                &authorization_policy,
            )
            .unwrap(),
            verifier_public_key_bundle_digest: verifier_authentication_public_key_bundle_digest(
                &bundle,
                &root,
                &authorization_policy,
            )
            .unwrap(),
            xenia_authentication_profile_digest: XENIA_AUTHENTICATION_PROFILE_V1_SHA256,
            xenia_authentication_suite_registry_digest:
                XENIA_AUTHENTICATION_SUITE_REGISTRY_V1_SHA256,
            max_transitions: MAX_LIFECYCLE_TRANSITIONS_V1,
        }
    }

    fn fixture() -> (
        VerifierKeyAuthorizationPolicyV1,
        VerifierKeyAuthorizationRootV1,
        VerifierAuthenticationPublicKeyBundleV1,
        VerifierKeyLifecycleCorePolicyV1,
        VerifierKeyLifecycleStateV1,
    ) {
        let authorization_policy = authorization_policy();
        let root = authorization_root();
        let bundle = public_key_bundle();
        let lifecycle_policy = lifecycle_policy();
        let genesis = derive_root_lifecycle_state(
            &lifecycle_policy,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();
        (
            authorization_policy,
            root,
            bundle,
            lifecycle_policy,
            genesis,
        )
    }

    fn current_ids(state: &VerifierKeyLifecycleStateV1, release: Digest32) -> (Digest32, Digest32) {
        let record = state
            .records()
            .iter()
            .find(|record| record.verifier_release_digest() == release)
            .unwrap();
        match record.status() {
            VerifierKeyLifecycleStatusV1::Active {
                ed25519_signer_key_id,
                ml_dsa_65_signer_key_id,
                ..
            } => (*ed25519_signer_key_id, *ml_dsa_65_signer_key_id),
            VerifierKeyLifecycleStatusV1::Disabled => panic!("fixture target unexpectedly disabled"),
        }
    }

    fn rotation_request(
        lifecycle_policy: &VerifierKeyLifecycleCorePolicyV1,
        state: &VerifierKeyLifecycleStateV1,
        authorization_policy: &VerifierKeyAuthorizationPolicyV1,
        root: &VerifierKeyAuthorizationRootV1,
        bundle: &VerifierAuthenticationPublicKeyBundleV1,
        release: Digest32,
        ed_byte: u8,
        ml_byte: u8,
    ) -> LifecycleTransitionRequestV1 {
        let current_state_digest = verifier_key_lifecycle_state_digest(
            state,
            lifecycle_policy,
            root,
            authorization_policy,
            bundle,
        )
        .unwrap();
        let lifecycle_policy_digest = verifier_key_lifecycle_core_policy_digest(
            lifecycle_policy,
            root,
            authorization_policy,
            bundle,
        )
        .unwrap();
        let (expected_ed, expected_ml) = current_ids(state, release);
        let ed = vec![ed_byte; 32];
        let ml = vec![ml_byte; 1952];

        LifecycleTransitionRequestV1 {
            lifecycle_policy_digest,
            current_state_digest,
            next_sequence: state.sequence() + 1,
            target_verifier_release_digest: release,
            expected_current_ed25519_signer_key_id: expected_ed,
            expected_current_ml_dsa_65_signer_key_id: expected_ml,
            operation: LifecycleTransitionOperationV1::RotateKeyPair {
                new_ed25519_signer_key_id: xenia_authentication_signer_key_id_v1(
                    XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                    &ed,
                )
                .unwrap(),
                new_ed25519_public_key: ed,
                new_ml_dsa_65_signer_key_id: xenia_authentication_signer_key_id_v1(
                    XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                    &ml,
                )
                .unwrap(),
                new_ml_dsa_65_public_key: ml,
            },
        }
    }

    fn disable_request(
        lifecycle_policy: &VerifierKeyLifecycleCorePolicyV1,
        state: &VerifierKeyLifecycleStateV1,
        authorization_policy: &VerifierKeyAuthorizationPolicyV1,
        root: &VerifierKeyAuthorizationRootV1,
        bundle: &VerifierAuthenticationPublicKeyBundleV1,
        release: Digest32,
    ) -> LifecycleTransitionRequestV1 {
        let current_state_digest = verifier_key_lifecycle_state_digest(
            state,
            lifecycle_policy,
            root,
            authorization_policy,
            bundle,
        )
        .unwrap();
        let lifecycle_policy_digest = verifier_key_lifecycle_core_policy_digest(
            lifecycle_policy,
            root,
            authorization_policy,
            bundle,
        )
        .unwrap();
        let (expected_ed, expected_ml) = current_ids(state, release);
        LifecycleTransitionRequestV1 {
            lifecycle_policy_digest,
            current_state_digest,
            next_sequence: state.sequence() + 1,
            target_verifier_release_digest: release,
            expected_current_ed25519_signer_key_id: expected_ed,
            expected_current_ml_dsa_65_signer_key_id: expected_ml,
            operation: LifecycleTransitionOperationV1::DisableVerifier,
        }
    }

    #[test]
    fn parent_vectors_and_lifecycle_policy_are_exact() {
        let authorization_policy = authorization_policy();
        let root = authorization_root();
        let bundle = public_key_bundle();
        let lifecycle_policy = lifecycle_policy();

        assert_eq!(
            verifier_key_authorization_root_digest(&root, &authorization_policy).unwrap(),
            GOLDEN_PARENT_ROOT_DIGEST
        );
        assert_eq!(
            verifier_authentication_public_key_bundle_digest(
                &bundle,
                &root,
                &authorization_policy
            )
            .unwrap(),
            GOLDEN_PARENT_BUNDLE_DIGEST
        );
        assert_eq!(
            verifier_key_lifecycle_core_policy_digest(
                &lifecycle_policy,
                &root,
                &authorization_policy,
                &bundle
            )
            .unwrap(),
            GOLDEN_LIFECYCLE_POLICY_DIGEST
        );
    }

    #[test]
    fn sequence_zero_is_deterministically_root_derived() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        assert_eq!(genesis.sequence(), 0);
        assert!(genesis.retired_ed25519_signer_key_ids().is_empty());
        assert!(genesis.retired_ml_dsa_65_signer_key_ids().is_empty());
        assert_eq!(
            verifier_key_lifecycle_state_digest(
                &genesis,
                &lifecycle_policy,
                &root,
                &authorization_policy,
                &bundle
            )
            .unwrap(),
            GOLDEN_GENESIS_STATE_DIGEST
        );

        let mut reordered_bundle = bundle.clone();
        reordered_bundle.evidence.reverse();
        let reordered_genesis = derive_root_lifecycle_state(
            &lifecycle_policy,
            &root,
            &authorization_policy,
            &reordered_bundle,
        )
        .unwrap();
        assert_eq!(
            verifier_key_lifecycle_state_digest(
                &reordered_genesis,
                &lifecycle_policy,
                &root,
                &authorization_policy,
                &reordered_bundle
            )
            .unwrap(),
            GOLDEN_GENESIS_STATE_DIGEST
        );
    }

    #[test]
    fn valid_pair_rotation_matches_language_neutral_vectors() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let request = rotation_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
            0x91,
            0x92,
        );
        let (proposal, projection) = validate_and_project_lifecycle_transition(
            &lifecycle_policy,
            &genesis,
            &request,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();

        assert_eq!(
            proposal.successor_projection_digest(),
            GOLDEN_ROTATION_PROJECTION_DIGEST
        );
        assert_eq!(proposal.proposal_digest(), GOLDEN_ROTATION_PROPOSAL_DIGEST);
        assert_eq!(projection.sequence(), 1);
        assert_eq!(
            projection.certification_capacity(),
            CertificationCapacityV1::Sufficient
        );

        let state = materialize_structural_lifecycle_state(
            &projection,
            digest(0xa1),
            &lifecycle_policy,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();
        assert_eq!(state.sequence(), 1);
        assert_eq!(state.retired_ed25519_signer_key_ids().len(), 1);
        assert_eq!(state.retired_ml_dsa_65_signer_key_ids().len(), 1);
    }

    #[test]
    fn replacement_signer_ids_are_recomputed_from_complete_public_keys() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let mut request = rotation_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
            0x91,
            0x92,
        );
        if let LifecycleTransitionOperationV1::RotateKeyPair {
            new_ed25519_signer_key_id,
            ..
        } = &mut request.operation
        {
            *new_ed25519_signer_key_id = digest(0xee);
        }
        assert_eq!(
            validate_and_project_lifecycle_transition(
                &lifecycle_policy,
                &genesis,
                &request,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleTransitionViolation::WrongEd25519ReplacementSignerId)
        );
    }

    #[test]
    fn wrong_suite_specific_public_key_length_fails_before_rotation() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let mut request = rotation_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
            0x91,
            0x92,
        );
        if let LifecycleTransitionOperationV1::RotateKeyPair {
            new_ml_dsa_65_public_key,
            ..
        } = &mut request.operation
        {
            new_ml_dsa_65_public_key.pop();
        }
        assert!(matches!(
            validate_and_project_lifecycle_transition(
                &lifecycle_policy,
                &genesis,
                &request,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleTransitionViolation::SignerKeyId(
                XeniaSignerKeyIdViolation::WrongMlDsa65PublicKeyLength { .. }
            ))
        ));
    }

    #[test]
    fn another_active_verifiers_key_cannot_be_reused() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let mut request = rotation_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
            0x91,
            0x92,
        );
        let other = genesis
            .records()
            .iter()
            .find(|record| record.verifier_release_digest() == digest(0x22))
            .unwrap();
        let VerifierKeyLifecycleStatusV1::Active {
            ed25519_signer_key_id,
            ed25519_public_key,
            ..
        } = other.status()
        else {
            panic!("fixture verifier disabled")
        };
        if let LifecycleTransitionOperationV1::RotateKeyPair {
            new_ed25519_signer_key_id,
            new_ed25519_public_key,
            ..
        } = &mut request.operation
        {
            *new_ed25519_signer_key_id = *ed25519_signer_key_id;
            *new_ed25519_public_key = ed25519_public_key.clone();
        }
        assert_eq!(
            validate_and_project_lifecycle_transition(
                &lifecycle_policy,
                &genesis,
                &request,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleTransitionViolation::ActiveEd25519KeyReuse)
        );
    }

    #[test]
    fn retired_keys_cannot_reappear_later_or_on_another_release() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let first = rotation_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
            0x91,
            0x92,
        );
        let (_, first_projection) = validate_and_project_lifecycle_transition(
            &lifecycle_policy,
            &genesis,
            &first,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();
        let state1 = materialize_structural_lifecycle_state(
            &first_projection,
            digest(0xa1),
            &lifecycle_policy,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();

        let old_ed = ed25519_public_key(0);
        let old_ml = ml_dsa_public_key(0);
        let mut second = rotation_request(
            &lifecycle_policy,
            &state1,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x22),
            0x93,
            0x94,
        );
        if let LifecycleTransitionOperationV1::RotateKeyPair {
            new_ed25519_signer_key_id,
            new_ed25519_public_key,
            ..
        } = &mut second.operation
        {
            *new_ed25519_signer_key_id = xenia_authentication_signer_key_id_v1(
                XENIA_ED25519_AUTHENTICATION_SUITE_ID,
                &old_ed,
            )
            .unwrap();
            *new_ed25519_public_key = old_ed;
        }
        assert_eq!(
            validate_and_project_lifecycle_transition(
                &lifecycle_policy,
                &state1,
                &second,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleTransitionViolation::RetiredEd25519KeyReuse)
        );

        let mut second_ml = rotation_request(
            &lifecycle_policy,
            &state1,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x22),
            0x93,
            0x94,
        );
        if let LifecycleTransitionOperationV1::RotateKeyPair {
            new_ml_dsa_65_signer_key_id,
            new_ml_dsa_65_public_key,
            ..
        } = &mut second_ml.operation
        {
            *new_ml_dsa_65_signer_key_id = xenia_authentication_signer_key_id_v1(
                XENIA_ML_DSA_65_AUTHENTICATION_SUITE_ID,
                &old_ml,
            )
            .unwrap();
            *new_ml_dsa_65_public_key = old_ml;
        }
        assert_eq!(
            validate_and_project_lifecycle_transition(
                &lifecycle_policy,
                &state1,
                &second_ml,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleTransitionViolation::RetiredMlDsa65KeyReuse)
        );
    }

    #[test]
    fn disable_is_terminal_and_capacity_loss_does_not_weaken_thresholds() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let request = disable_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
        );
        let (_, projection) = validate_and_project_lifecycle_transition(
            &lifecycle_policy,
            &genesis,
            &request,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();

        assert_eq!(
            projection.certification_capacity(),
            CertificationCapacityV1::Insufficient {
                active_verifiers: 2,
                distinct_implementation_lineages: 2,
                distinct_builder_control_domains: 2,
                required_verifiers: 3,
                required_implementation_lineages: 3,
                required_builder_control_domains: 2,
            }
        );

        let disabled_state = materialize_structural_lifecycle_state(
            &projection,
            digest(0xa2),
            &lifecycle_policy,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();

        let retry = LifecycleTransitionRequestV1 {
            lifecycle_policy_digest: verifier_key_lifecycle_core_policy_digest(
                &lifecycle_policy,
                &root,
                &authorization_policy,
                &bundle,
            )
            .unwrap(),
            current_state_digest: verifier_key_lifecycle_state_digest(
                &disabled_state,
                &lifecycle_policy,
                &root,
                &authorization_policy,
                &bundle,
            )
            .unwrap(),
            next_sequence: 2,
            target_verifier_release_digest: digest(0x21),
            expected_current_ed25519_signer_key_id: digest(0x55),
            expected_current_ml_dsa_65_signer_key_id: digest(0x66),
            operation: LifecycleTransitionOperationV1::DisableVerifier,
        };
        assert_eq!(
            validate_and_project_lifecycle_transition(
                &lifecycle_policy,
                &disabled_state,
                &retry,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleTransitionViolation::TargetVerifierDisabled)
        );
    }

    #[test]
    fn sequence_and_predecessor_are_exact_not_time_selected() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let mut request = disable_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
        );
        request.next_sequence = 2;
        assert_eq!(
            validate_and_project_lifecycle_transition(
                &lifecycle_policy,
                &genesis,
                &request,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleTransitionViolation::WrongNextSequence)
        );

        request.next_sequence = 1;
        request.current_state_digest = digest(0xfe);
        assert_eq!(
            validate_and_project_lifecycle_transition(
                &lifecycle_policy,
                &genesis,
                &request,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleTransitionViolation::CurrentStateDigestMismatch)
        );
    }

    #[test]
    fn two_competing_successors_are_explicit_fork_evidence() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let left_request = disable_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
        );
        let right_request = disable_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x22),
        );
        let (left, _) = validate_and_project_lifecycle_transition(
            &lifecycle_policy,
            &genesis,
            &left_request,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();
        let (right, _) = validate_and_project_lifecycle_transition(
            &lifecycle_policy,
            &genesis,
            &right_request,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();

        let fork = detect_lifecycle_fork(&left, &right).unwrap();
        assert_eq!(fork.predecessor_state_digest, GOLDEN_GENESIS_STATE_DIGEST);
        assert_ne!(fork.left_proposal_digest, fork.right_proposal_digest);
        assert!(detect_lifecycle_fork(&left, &left).is_none());
    }

    #[test]
    fn frozen_xenia_profile_and_parent_bundle_cannot_be_substituted() {
        let authorization_policy = authorization_policy();
        let root = authorization_root();
        let bundle = public_key_bundle();
        let mut lifecycle_policy = lifecycle_policy();

        lifecycle_policy.xenia_authentication_profile_digest = digest(0x99);
        assert_eq!(
            validate_verifier_key_lifecycle_core_policy(
                &lifecycle_policy,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleCorePolicyViolation::WrongXeniaAuthenticationProfile)
        );

        let mut lifecycle_policy = lifecycle_policy();
        lifecycle_policy.verifier_public_key_bundle_digest = digest(0x99);
        assert_eq!(
            validate_verifier_key_lifecycle_core_policy(
                &lifecycle_policy,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleCorePolicyViolation::PublicKeyBundleDigestMismatch)
        );
    }

    #[test]
    fn transition_limit_blocks_history_truncation_pressure() {
        let (authorization_policy, root, bundle, lifecycle_policy, mut state) = fixture();
        state.sequence = MAX_LIFECYCLE_TRANSITIONS_V1;
        state.predecessor_state_digest = digest(0xf1);
        state.authorization_event_digest = digest(0xf2);
        state.retired_ed25519_signer_key_ids = (0..MAX_LIFECYCLE_TRANSITIONS_V1)
            .map(|index| {
                let mut value = [0_u8; 32];
                value[0] = 0xd0;
                value[1..5].copy_from_slice(&index.to_be_bytes());
                value
            })
            .collect();
        state.retired_ml_dsa_65_signer_key_ids = (0..MAX_LIFECYCLE_TRANSITIONS_V1)
            .map(|index| {
                let mut value = [0_u8; 32];
                value[0] = 0xe0;
                value[1..5].copy_from_slice(&index.to_be_bytes());
                value
            })
            .collect();

        validate_verifier_key_lifecycle_state(
            &state,
            &lifecycle_policy,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();

        let request = rotation_request(
            &lifecycle_policy,
            &state,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
            0x91,
            0x92,
        );
        assert_eq!(
            validate_and_project_lifecycle_transition(
                &lifecycle_policy,
                &state,
                &request,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleTransitionViolation::SequenceLimitReached)
        );
    }

    #[test]
    fn retired_history_cannot_be_dropped_from_materialized_state() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let request = rotation_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
            0x91,
            0x92,
        );
        let (_, projection) = validate_and_project_lifecycle_transition(
            &lifecycle_policy,
            &genesis,
            &request,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();
        let mut state = materialize_structural_lifecycle_state(
            &projection,
            digest(0xa3),
            &lifecycle_policy,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();
        state.retired_ed25519_signer_key_ids.clear();
        assert_eq!(
            validate_verifier_key_lifecycle_state(
                &state,
                &lifecycle_policy,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleStateViolation::RetiredHistoryLengthMismatch)
        );
    }

    #[test]
    fn materialization_requires_external_nonzero_event_but_makes_no_authority_claim() {
        let (authorization_policy, root, bundle, lifecycle_policy, genesis) = fixture();
        let request = disable_request(
            &lifecycle_policy,
            &genesis,
            &authorization_policy,
            &root,
            &bundle,
            digest(0x21),
        );
        let (_, projection) = validate_and_project_lifecycle_transition(
            &lifecycle_policy,
            &genesis,
            &request,
            &root,
            &authorization_policy,
            &bundle,
        )
        .unwrap();

        assert_eq!(
            materialize_structural_lifecycle_state(
                &projection,
                [0_u8; 32],
                &lifecycle_policy,
                &root,
                &authorization_policy,
                &bundle,
            ),
            Err(LifecycleMaterializationViolation::ZeroAuthorizationEventDigest)
        );
    }
}
