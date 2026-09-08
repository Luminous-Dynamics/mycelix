// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Record-bound historical verifier-key generation lineage theorem for Identity V2.
//!
//! This crate resolves the exact #316 key generation named by a #324 signed record from
//! one already-observed immutable generation lineage. It does not gather Holochain data,
//! infer generations from DID-document timestamps, decide revocation/currentness, verify
//! signatures, or establish verifier-policy authority.
//!
//! The lineage is scoped to one canonical `(verifier DID, verifier key ID)` namespace.
//! Historical records may resolve a non-terminal generation. Terminal/current generation
//! authority is deliberately a separate theorem.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

use mycelix_crypto::AlgorithmId;
use mycelix_kvector_verification_record_policy::{
    validate_kvector_verification_record_key_generation_binding_v2,
    KVectorProofVerificationRecordBodyV2, KVectorVerificationRecordErrorV2,
};
use mycelix_kvector_verifier_key_generation_policy::{
    derive_kvector_verifier_key_generation_digest_v2, KVectorVerifierKeyGenerationV2,
    SHA256_DIGEST_LEN_V2,
};

pub const ACTION_ID_MAX_LEN_V2: usize = 256;
pub const MAX_OBSERVED_KEY_GENERATIONS_V2: usize = 4096;

#[derive(Debug, Clone, Copy)]
pub struct ObservedVerifierKeyGenerationRecordV2<'a> {
    pub action_id: &'a str,
    /// `None` identifies generation 1. Every later generation must name the exact
    /// immutable action that asserted its direct predecessor generation.
    pub previous_generation_action_id: Option<&'a str>,
    pub generation: KVectorVerifierKeyGenerationV2<'a>,
}

/// Opaque, verifier-owned proof that the exact key generation named by one signed record
/// exists in one unambiguous observed generation lineage and was historically valid at
/// the record's asserted verification time.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedHistoricalVerifierKeyGenerationV2 {
    generation_action_id: String,
    root_generation_action_id: String,
    verifier_did: String,
    verifier_key_id: String,
    algorithm: AlgorithmId,
    public_key_bytes: Vec<u8>,
    key_generation: u64,
    generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    valid_from_micros: i64,
    valid_until_micros: i64,
    observed_generation_count: usize,
}

impl ResolvedHistoricalVerifierKeyGenerationV2 {
    pub fn generation_action_id(&self) -> &str {
        &self.generation_action_id
    }

    pub fn root_generation_action_id(&self) -> &str {
        &self.root_generation_action_id
    }

    pub fn verifier_did(&self) -> &str {
        &self.verifier_did
    }

    pub fn verifier_key_id(&self) -> &str {
        &self.verifier_key_id
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn public_key_bytes(&self) -> &[u8] {
        &self.public_key_bytes
    }

    pub fn key_generation(&self) -> u64 {
        self.key_generation
    }

    pub fn generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.generation_sha256
    }

    pub fn valid_from_micros(&self) -> i64 {
        self.valid_from_micros
    }

    pub fn valid_until_micros(&self) -> i64 {
        self.valid_until_micros
    }

    pub fn observed_generation_count(&self) -> usize {
        self.observed_generation_count
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerifierKeyGenerationLineageErrorV2 {
    EmptyHistory,
    HistoryTooLarge,
    ActionIdInvalid,
    DuplicateActionId,
    DuplicateGenerationNumber,
    DuplicateGenerationDigest,
    GenerationNamespaceMismatch,
    RootGenerationMissing,
    MultipleRootGenerations,
    RootGenerationNumberInvalid,
    ParentMissing,
    GenerationNotDirectSuccessor,
    GenerationCounterOverflow,
    BranchingGenerationHistory,
    DisconnectedGenerationHistory,
    ValidityWindowsOverlap,
    IssuedAtRegressed,
    TargetGenerationMissing,
    RecordGenerationBinding(KVectorVerificationRecordErrorV2),
}

fn valid_action_id(value: &str) -> bool {
    !value.is_empty() && value.len() <= ACTION_ID_MAX_LEN_V2
}

/// Resolve the exact historical verifier-key generation committed by one #324 record.
///
/// Lineage authority is topological and generation-number based, never timestamp winner
/// selection:
///
/// 1. all supplied generations belong to the record's exact DID + canonical key ID;
/// 2. exactly one root exists and it is generation 1;
/// 3. every later generation names one observed direct predecessor action;
/// 4. each child is exactly `parent.generation + 1`;
/// 5. no parent has competing children;
/// 6. generation numbers and generation digests are unique;
/// 7. all supplied generations are reachable from the root;
/// 8. child validity may start after a gap but may not overlap the parent validity;
/// 9. generation `issued_at` may not regress along a parent edge;
/// 10. the record-bound generation digest resolves exactly once and #324 historical-use
///     binding succeeds at `record.verified_at_micros`.
///
/// Success does not require the selected historical generation to be terminal/current.
pub fn resolve_record_bound_verifier_key_generation_v2<'a>(
    history: &'a [ObservedVerifierKeyGenerationRecordV2<'a>],
    record: KVectorProofVerificationRecordBodyV2<'a>,
) -> Result<ResolvedHistoricalVerifierKeyGenerationV2, VerifierKeyGenerationLineageErrorV2> {
    if history.is_empty() {
        return Err(VerifierKeyGenerationLineageErrorV2::EmptyHistory);
    }
    if history.len() > MAX_OBSERVED_KEY_GENERATIONS_V2 {
        return Err(VerifierKeyGenerationLineageErrorV2::HistoryTooLarge);
    }

    let mut action_to_index: BTreeMap<&str, usize> = BTreeMap::new();
    let mut generation_to_index: BTreeMap<u64, usize> = BTreeMap::new();
    let mut digest_to_index: BTreeMap<[u8; SHA256_DIGEST_LEN_V2], usize> = BTreeMap::new();
    let mut root_index: Option<usize> = None;

    for (index, observed) in history.iter().enumerate() {
        if !valid_action_id(observed.action_id) {
            return Err(VerifierKeyGenerationLineageErrorV2::ActionIdInvalid);
        }
        if let Some(parent) = observed.previous_generation_action_id {
            if !valid_action_id(parent) {
                return Err(VerifierKeyGenerationLineageErrorV2::ActionIdInvalid);
            }
        }

        let generation = observed.generation;
        if generation.verifier_did != record.verifier_did
            || generation.verifier_key_id != record.verifier_key_id
        {
            return Err(VerifierKeyGenerationLineageErrorV2::GenerationNamespaceMismatch);
        }

        let digest = derive_kvector_verifier_key_generation_digest_v2(generation)
            .map_err(|error| {
                VerifierKeyGenerationLineageErrorV2::RecordGenerationBinding(
                    KVectorVerificationRecordErrorV2::VerifierKeyGeneration(error),
                )
            })?;

        if action_to_index.insert(observed.action_id, index).is_some() {
            return Err(VerifierKeyGenerationLineageErrorV2::DuplicateActionId);
        }
        if generation_to_index
            .insert(generation.key_generation, index)
            .is_some()
        {
            return Err(VerifierKeyGenerationLineageErrorV2::DuplicateGenerationNumber);
        }
        if digest_to_index.insert(digest, index).is_some() {
            return Err(VerifierKeyGenerationLineageErrorV2::DuplicateGenerationDigest);
        }

        if observed.previous_generation_action_id.is_none() {
            if root_index.replace(index).is_some() {
                return Err(VerifierKeyGenerationLineageErrorV2::MultipleRootGenerations);
            }
        }
    }

    let root_index = root_index.ok_or(VerifierKeyGenerationLineageErrorV2::RootGenerationMissing)?;
    if history[root_index].generation.key_generation != 1 {
        return Err(VerifierKeyGenerationLineageErrorV2::RootGenerationNumberInvalid);
    }

    let mut child_of: BTreeMap<&str, usize> = BTreeMap::new();
    for (index, observed) in history.iter().enumerate() {
        let Some(parent_action_id) = observed.previous_generation_action_id else {
            continue;
        };

        let parent_index = *action_to_index
            .get(parent_action_id)
            .ok_or(VerifierKeyGenerationLineageErrorV2::ParentMissing)?;
        let parent = history[parent_index].generation;
        let child = observed.generation;

        let expected_generation = parent
            .key_generation
            .checked_add(1)
            .ok_or(VerifierKeyGenerationLineageErrorV2::GenerationCounterOverflow)?;
        if child.key_generation != expected_generation {
            return Err(VerifierKeyGenerationLineageErrorV2::GenerationNotDirectSuccessor);
        }
        if child.valid_from_micros < parent.valid_until_micros {
            return Err(VerifierKeyGenerationLineageErrorV2::ValidityWindowsOverlap);
        }
        if child.issued_at_micros < parent.issued_at_micros {
            return Err(VerifierKeyGenerationLineageErrorV2::IssuedAtRegressed);
        }
        if child_of.insert(parent_action_id, index).is_some() {
            return Err(VerifierKeyGenerationLineageErrorV2::BranchingGenerationHistory);
        }
    }

    let mut visited: BTreeSet<&str> = BTreeSet::new();
    let mut current_index = root_index;
    loop {
        let current_action_id = history[current_index].action_id;
        if !visited.insert(current_action_id) {
            return Err(VerifierKeyGenerationLineageErrorV2::DisconnectedGenerationHistory);
        }
        match child_of.get(current_action_id) {
            Some(next_index) => current_index = *next_index,
            None => break,
        }
    }
    if visited.len() != history.len() {
        return Err(VerifierKeyGenerationLineageErrorV2::DisconnectedGenerationHistory);
    }

    let target_digest: [u8; SHA256_DIGEST_LEN_V2] = record
        .verifier_key_generation_sha256
        .try_into()
        .map_err(|_| {
            VerifierKeyGenerationLineageErrorV2::RecordGenerationBinding(
                KVectorVerificationRecordErrorV2::VerifierKeyGenerationDigestLengthInvalid,
            )
        })?;
    let target_index = *digest_to_index
        .get(&target_digest)
        .ok_or(VerifierKeyGenerationLineageErrorV2::TargetGenerationMissing)?;
    let target = history[target_index];

    validate_kvector_verification_record_key_generation_binding_v2(record, target.generation)
        .map_err(VerifierKeyGenerationLineageErrorV2::RecordGenerationBinding)?;

    Ok(ResolvedHistoricalVerifierKeyGenerationV2 {
        generation_action_id: target.action_id.to_string(),
        root_generation_action_id: history[root_index].action_id.to_string(),
        verifier_did: target.generation.verifier_did.to_string(),
        verifier_key_id: target.generation.verifier_key_id.to_string(),
        algorithm: target.generation.algorithm,
        public_key_bytes: target.generation.public_key_bytes.to_vec(),
        key_generation: target.generation.key_generation,
        generation_sha256: target_digest,
        valid_from_micros: target.generation.valid_from_micros,
        valid_until_micros: target.generation.valid_until_micros,
        observed_generation_count: history.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_kvector_verification_record_policy::KVectorProofVerificationOutcomeV2;

    const DID: &str = "did:mycelix:verifier";
    const KEY_ID: &str = "did:mycelix:verifier#key-1";

    fn generation<'a>(
        key: &'a [u8],
        key_generation: u64,
        valid_from_micros: i64,
        valid_until_micros: i64,
    ) -> KVectorVerifierKeyGenerationV2<'a> {
        KVectorVerifierKeyGenerationV2 {
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            algorithm: AlgorithmId::Ed25519,
            public_key_bytes: key,
            key_generation,
            issued_at_micros: valid_from_micros - 100,
            valid_from_micros,
            valid_until_micros,
        }
    }

    fn observed<'a>(
        action_id: &'a str,
        previous_generation_action_id: Option<&'a str>,
        generation: KVectorVerifierKeyGenerationV2<'a>,
    ) -> ObservedVerifierKeyGenerationRecordV2<'a> {
        ObservedVerifierKeyGenerationRecordV2 {
            action_id,
            previous_generation_action_id,
            generation,
        }
    }

    fn record<'a>(
        generation_digest: &'a [u8],
        verified_at_micros: i64,
    ) -> KVectorProofVerificationRecordBodyV2<'a> {
        static FULFILLMENT: [u8; 32] = [0x11; 32];
        static STATEMENT: [u8; 32] = [0x22; 32];
        static PROOF: [u8; 32] = [0x33; 32];
        static POLICY: [u8; 32] = [0x44; 32];
        KVectorProofVerificationRecordBodyV2 {
            fulfillment_id: &FULFILLMENT,
            proof_statement_sha256: &STATEMENT,
            proof_sha256: &PROOF,
            backend_id: "candidate-backend",
            circuit_id: "identity-kvector-v2",
            circuit_version: "0.1.0",
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            verifier_key_generation_sha256: generation_digest,
            signature_scheme_id: "ed25519-v1",
            verification_policy_sha256: &POLICY,
            outcome: KVectorProofVerificationOutcomeV2::Accepted,
            verified_at_micros,
            valid_until_micros: verified_at_micros + 500,
        }
    }

    #[test]
    fn old_record_resolves_old_generation_even_when_newer_generation_exists() {
        let key1 = [0x31; 32];
        let key2 = [0x52; 32];
        let gen1 = generation(&key1, 1, 1_000, 2_000);
        let gen2 = generation(&key2, 2, 2_000, 3_000);
        let digest1 = derive_kvector_verifier_key_generation_digest_v2(gen1).unwrap();
        let history = [
            observed("g1", None, gen1),
            observed("g2", Some("g1"), gen2),
        ];

        let resolved = resolve_record_bound_verifier_key_generation_v2(
            &history,
            record(&digest1, 1_500),
        )
        .expect("historical record should resolve generation 1, not terminal generation 2");

        assert_eq!(resolved.generation_action_id(), "g1");
        assert_eq!(resolved.key_generation(), 1);
        assert_eq!(resolved.public_key_bytes(), &key1);
        assert_eq!(resolved.observed_generation_count(), 2);
    }

    #[test]
    fn same_key_id_reuse_cannot_substitute_newer_generation() {
        let key1 = [0x31; 32];
        let key2 = [0x52; 32];
        let gen1 = generation(&key1, 1, 1_000, 2_000);
        let gen2 = generation(&key2, 2, 2_000, 3_000);
        let digest1 = derive_kvector_verifier_key_generation_digest_v2(gen1).unwrap();
        let digest2 = derive_kvector_verifier_key_generation_digest_v2(gen2).unwrap();
        assert_ne!(digest1, digest2);

        let history = [
            observed("g1", None, gen1),
            observed("g2", Some("g1"), gen2),
        ];
        let resolved = resolve_record_bound_verifier_key_generation_v2(
            &history,
            record(&digest1, 1_500),
        )
        .unwrap();
        assert_eq!(resolved.generation_sha256(), &digest1);
        assert_ne!(resolved.generation_sha256(), &digest2);
    }

    #[test]
    fn generation_numbers_must_be_exact_successors() {
        let key1 = [0x31; 32];
        let key3 = [0x53; 32];
        let gen1 = generation(&key1, 1, 1_000, 2_000);
        let gen3 = generation(&key3, 3, 2_000, 3_000);
        let digest1 = derive_kvector_verifier_key_generation_digest_v2(gen1).unwrap();
        let history = [
            observed("g1", None, gen1),
            observed("g3", Some("g1"), gen3),
        ];
        assert_eq!(
            resolve_record_bound_verifier_key_generation_v2(
                &history,
                record(&digest1, 1_500),
            ),
            Err(VerifierKeyGenerationLineageErrorV2::GenerationNotDirectSuccessor)
        );
    }

    #[test]
    fn competing_successor_generations_fail_closed() {
        let key1 = [0x31; 32];
        let key2a = [0x52; 32];
        let key2b = [0x62; 32];
        let gen1 = generation(&key1, 1, 1_000, 2_000);
        let gen2a = generation(&key2a, 2, 2_000, 3_000);
        let gen2b = generation(&key2b, 2, 3_000, 4_000);
        let digest1 = derive_kvector_verifier_key_generation_digest_v2(gen1).unwrap();
        let history = [
            observed("g1", None, gen1),
            observed("g2a", Some("g1"), gen2a),
            observed("g2b", Some("g1"), gen2b),
        ];
        assert_eq!(
            resolve_record_bound_verifier_key_generation_v2(
                &history,
                record(&digest1, 1_500),
            ),
            Err(VerifierKeyGenerationLineageErrorV2::DuplicateGenerationNumber)
        );
    }

    #[test]
    fn validity_windows_cannot_overlap() {
        let key1 = [0x31; 32];
        let key2 = [0x52; 32];
        let gen1 = generation(&key1, 1, 1_000, 2_000);
        let gen2 = generation(&key2, 2, 1_999, 3_000);
        let digest1 = derive_kvector_verifier_key_generation_digest_v2(gen1).unwrap();
        let history = [
            observed("g1", None, gen1),
            observed("g2", Some("g1"), gen2),
        ];
        assert_eq!(
            resolve_record_bound_verifier_key_generation_v2(
                &history,
                record(&digest1, 1_500),
            ),
            Err(VerifierKeyGenerationLineageErrorV2::ValidityWindowsOverlap)
        );
    }

    #[test]
    fn record_digest_must_exist_in_observed_lineage() {
        let key1 = [0x31; 32];
        let absent_key = [0x7A; 32];
        let gen1 = generation(&key1, 1, 1_000, 2_000);
        let absent = generation(&absent_key, 1, 1_000, 2_000);
        let absent_digest = derive_kvector_verifier_key_generation_digest_v2(absent).unwrap();
        let history = [observed("g1", None, gen1)];
        assert_eq!(
            resolve_record_bound_verifier_key_generation_v2(
                &history,
                record(&absent_digest, 1_500),
            ),
            Err(VerifierKeyGenerationLineageErrorV2::TargetGenerationMissing)
        );
    }

    #[test]
    fn target_generation_must_cover_record_verification_time() {
        let key1 = [0x31; 32];
        let gen1 = generation(&key1, 1, 2_000, 3_000);
        let digest1 = derive_kvector_verifier_key_generation_digest_v2(gen1).unwrap();
        let history = [observed("g1", None, gen1)];
        assert!(matches!(
            resolve_record_bound_verifier_key_generation_v2(
                &history,
                record(&digest1, 1_500),
            ),
            Err(VerifierKeyGenerationLineageErrorV2::RecordGenerationBinding(
                KVectorVerificationRecordErrorV2::VerifierKeyGeneration(_)
            ))
        ));
    }

    #[test]
    fn qualified_result_fields_are_verifier_owned() {
        let source = include_str!("lib.rs");
        for public_field in [
            "pub generation_action_id:",
            "pub root_generation_action_id:",
            "pub verifier_did:",
            "pub verifier_key_id:",
            "pub algorithm:",
            "pub public_key_bytes:",
            "pub key_generation:",
            "pub generation_sha256:",
            "pub valid_from_micros:",
            "pub valid_until_micros:",
        ] {
            assert!(!source.contains(public_field));
        }
    }
}
