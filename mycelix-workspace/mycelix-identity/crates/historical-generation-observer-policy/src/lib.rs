// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Generation-aware normalized observer assembly for historical Identity V2 verifier keys.
//!
//! This pure crate composes already-qualified viewpoint-scoped activity coverage with one
//! exact normalized fetched security-record set. It additionally requires the complete
//! observed DID-document set to pass the policy-neutral DID lineage theorem before any
//! verifier-key generation may use a referenced DID document as key-material provenance.
//!
//! The word "normalized" is deliberate: this crate cannot prove that caller-supplied
//! fields are the exact bytes committed by a Holochain `Record`. A later concrete HDK
//! adapter must perform exact action/entry-type decoding and call this theorem without
//! exposing caller-controlled normalization.
//!
//! DID deactivation observations are retained as independent evidence. A later
//! deactivation does not by itself erase whether an older signature was historically
//! authentic.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

use mycelix_agent_activity_coverage_policy::QualifiedAgentActivityCoverageV2;
use mycelix_did_authentication_method_material_policy::{
    resolve_admitted_did_authentication_method_v2, DidAuthenticationMethodMaterialErrorV2,
};
use mycelix_did_document_crypto_policy::{
    DidDocumentCryptoAdmissionViewV2, DidVerificationMethodAdmissionViewV2,
};
use mycelix_did_document_lineage_policy::{
    qualify_did_document_lineage_v2, DidDocumentLineageErrorV2,
    ObservedDidDocumentLineageActionV2, QualifiedDidDocumentLineageV2,
};
use mycelix_kvector_verification_record_policy::KVectorProofVerificationRecordBodyV2;
use mycelix_kvector_verifier_key_generation_lineage_policy::{
    resolve_record_bound_verifier_key_generation_v2, ObservedVerifierKeyGenerationRecordV2,
    ResolvedHistoricalVerifierKeyGenerationV2, VerifierKeyGenerationLineageErrorV2,
};
use mycelix_verifier_key_generation_material_policy::{
    assemble_verifier_key_generation_material_v2, AssembledVerifierKeyGenerationV2,
    VerifierKeyGenerationLifecycleAssertionV2, VerifierKeyGenerationMaterialErrorV2,
};

#[derive(Debug, Clone, Copy)]
pub struct FetchedDidVerificationMethodV2<'a> {
    pub id: &'a str,
    pub type_: &'a str,
    pub controller: &'a str,
    pub public_key_multibase: &'a str,
    pub algorithm: Option<u16>,
}

#[derive(Debug, Clone, Copy)]
pub struct FetchedDidDocumentRecordV2<'a> {
    pub action_seq: u32,
    pub action_id: &'a str,
    /// `None` for create; exact original DID-document action for update.
    pub previous_action_id: Option<&'a str>,
    pub version: u32,
    pub did: &'a str,
    pub verification_methods: &'a [FetchedDidVerificationMethodV2<'a>],
    pub authentication: &'a [&'a str],
    pub key_agreement: &'a [&'a str],
}

#[derive(Debug, Clone, Copy)]
pub struct FetchedDidDeactivationRecordV2<'a> {
    pub action_seq: u32,
    pub action_id: &'a str,
    pub did: &'a str,
}

#[derive(Debug, Clone, Copy)]
pub struct FetchedVerifierKeyGenerationRecordV2<'a> {
    pub action_seq: u32,
    pub action_id: &'a str,
    /// Immutable author/source-chain provenance timestamp, not trusted wall-clock time.
    pub action_timestamp_micros: i64,
    pub verifier_did: &'a str,
    pub verifier_key_id: &'a str,
    pub did_document_action_id: &'a str,
    pub key_generation: u64,
    pub valid_from_micros: i64,
    pub valid_until_micros: i64,
    pub previous_generation_action_id: Option<&'a str>,
}

#[derive(Debug, Clone, Copy)]
pub enum FetchedIdentitySecurityRecordV2<'a> {
    DidDocument(FetchedDidDocumentRecordV2<'a>),
    DidDeactivation(FetchedDidDeactivationRecordV2<'a>),
    VerifierKeyGeneration(FetchedVerifierKeyGenerationRecordV2<'a>),
}

impl FetchedIdentitySecurityRecordV2<'_> {
    fn action_seq(&self) -> u32 {
        match self {
            Self::DidDocument(record) => record.action_seq,
            Self::DidDeactivation(record) => record.action_seq,
            Self::VerifierKeyGeneration(record) => record.action_seq,
        }
    }

    fn action_id(&self) -> &str {
        match self {
            Self::DidDocument(record) => record.action_id,
            Self::DidDeactivation(record) => record.action_id,
            Self::VerifierKeyGeneration(record) => record.action_id,
        }
    }

    fn did(&self) -> &str {
        match self {
            Self::DidDocument(record) => record.did,
            Self::DidDeactivation(record) => record.did,
            Self::VerifierKeyGeneration(record) => record.verifier_did,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservedDidDeactivationFactV2 {
    action_seq: u32,
    action_id: String,
    did: String,
}

impl ObservedDidDeactivationFactV2 {
    pub fn action_seq(&self) -> u32 {
        self.action_seq
    }

    pub fn action_id(&self) -> &str {
        &self.action_id
    }

    pub fn did(&self) -> &str {
        &self.did
    }
}

/// Opaque composition of exact activity coverage, exact DID-document lineage membership,
/// and #333's record-bound historical generation resolution.
#[derive(Debug)]
pub struct QualifiedNormalizedHistoricalGenerationObservationV2 {
    coverage: QualifiedAgentActivityCoverageV2,
    did_lineage: QualifiedDidDocumentLineageV2,
    resolved_generation: ResolvedHistoricalVerifierKeyGenerationV2,
    selected_did_document_action_id: String,
    assembled_generation_count: usize,
    deactivations: Vec<ObservedDidDeactivationFactV2>,
}

impl QualifiedNormalizedHistoricalGenerationObservationV2 {
    pub fn coverage(&self) -> &QualifiedAgentActivityCoverageV2 {
        &self.coverage
    }

    pub fn did_lineage(&self) -> &QualifiedDidDocumentLineageV2 {
        &self.did_lineage
    }

    pub fn resolved_generation(&self) -> &ResolvedHistoricalVerifierKeyGenerationV2 {
        &self.resolved_generation
    }

    pub fn selected_did_document_action_id(&self) -> &str {
        &self.selected_did_document_action_id
    }

    pub fn assembled_generation_count(&self) -> usize {
        self.assembled_generation_count
    }

    pub fn deactivations(&self) -> &[ObservedDidDeactivationFactV2] {
        &self.deactivations
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalGenerationObserverErrorV2 {
    DuplicateFetchedRecord,
    FetchedRecordNotInQualifiedActivity,
    QualifiedActivityRecordMissing,
    SecurityRecordDidMismatch,
    DidDocumentLineage(DidDocumentLineageErrorV2),
    ReferencedDidDocumentMissing,
    ReferencedDidDocumentOutsideQualifiedLineage,
    ReferencedDidDocumentNotEarlier,
    DidMethod(DidAuthenticationMethodMaterialErrorV2),
    GenerationMaterial(VerifierKeyGenerationMaterialErrorV2),
    GenerationLineage(VerifierKeyGenerationLineageErrorV2),
    SelectedGenerationAssemblyMissing,
}

/// Resolve one signed record's exact historical verifier-key generation from one complete
/// normalized security observation.
pub fn assemble_normalized_historical_generation_observation_v2<'a>(
    coverage: QualifiedAgentActivityCoverageV2,
    records: &'a [FetchedIdentitySecurityRecordV2<'a>],
    signed_record: KVectorProofVerificationRecordBodyV2<'a>,
) -> Result<
    QualifiedNormalizedHistoricalGenerationObservationV2,
    HistoricalGenerationObserverErrorV2,
> {
    let mut remaining: BTreeSet<(u32, String)> = coverage
        .matching_valid_actions()
        .iter()
        .map(|action| (action.action_seq(), action.action_id().to_string()))
        .collect();
    let mut seen: BTreeSet<(u32, &str)> = BTreeSet::new();
    let mut documents: Vec<&FetchedDidDocumentRecordV2<'a>> = Vec::new();
    let mut documents_by_id: BTreeMap<&str, &FetchedDidDocumentRecordV2<'a>> = BTreeMap::new();
    let mut generation_records: Vec<&FetchedVerifierKeyGenerationRecordV2<'a>> = Vec::new();
    let mut deactivations = Vec::new();

    for record in records {
        let seq = record.action_seq();
        let action_id = record.action_id();
        if !coverage.contains_action(seq, action_id) {
            return Err(HistoricalGenerationObserverErrorV2::FetchedRecordNotInQualifiedActivity);
        }
        if !seen.insert((seq, action_id)) {
            return Err(HistoricalGenerationObserverErrorV2::DuplicateFetchedRecord);
        }
        remaining.remove(&(seq, action_id.to_string()));

        if record.did() != signed_record.verifier_did {
            return Err(HistoricalGenerationObserverErrorV2::SecurityRecordDidMismatch);
        }

        match record {
            FetchedIdentitySecurityRecordV2::DidDocument(document) => {
                documents.push(document);
                documents_by_id.insert(document.action_id, document);
            }
            FetchedIdentitySecurityRecordV2::DidDeactivation(deactivation) => {
                deactivations.push(ObservedDidDeactivationFactV2 {
                    action_seq: deactivation.action_seq,
                    action_id: deactivation.action_id.to_string(),
                    did: deactivation.did.to_string(),
                });
            }
            FetchedIdentitySecurityRecordV2::VerifierKeyGeneration(generation) => {
                generation_records.push(generation);
            }
        }
    }

    if !remaining.is_empty() {
        return Err(HistoricalGenerationObserverErrorV2::QualifiedActivityRecordMissing);
    }

    let did_history: Vec<ObservedDidDocumentLineageActionV2<'_>> = documents
        .iter()
        .map(|document| ObservedDidDocumentLineageActionV2 {
            action_id: document.action_id,
            previous_action_id: document.previous_action_id,
            version: document.version,
            did: document.did,
        })
        .collect();
    let did_lineage = qualify_did_document_lineage_v2(signed_record.verifier_did, &did_history)
        .map_err(HistoricalGenerationObserverErrorV2::DidDocumentLineage)?;

    let mut assembled_generations: Vec<AssembledVerifierKeyGenerationV2> = Vec::new();
    for generation in generation_records {
        let document = documents_by_id
            .get(generation.did_document_action_id)
            .copied()
            .ok_or(HistoricalGenerationObserverErrorV2::ReferencedDidDocumentMissing)?;
        if !did_lineage.contains_document_action(generation.did_document_action_id) {
            return Err(
                HistoricalGenerationObserverErrorV2::ReferencedDidDocumentOutsideQualifiedLineage,
            );
        }
        if document.action_seq >= generation.action_seq {
            return Err(HistoricalGenerationObserverErrorV2::ReferencedDidDocumentNotEarlier);
        }

        let admission_methods: Vec<DidVerificationMethodAdmissionViewV2<'a>> = document
            .verification_methods
            .iter()
            .map(|method| DidVerificationMethodAdmissionViewV2 {
                id: method.id,
                type_: method.type_,
                controller: method.controller,
                public_key_multibase: method.public_key_multibase,
                algorithm: method.algorithm,
            })
            .collect();
        let method = resolve_admitted_did_authentication_method_v2(
            DidDocumentCryptoAdmissionViewV2 {
                did: document.did,
                verification_methods: &admission_methods,
                authentication: document.authentication,
                key_agreement: document.key_agreement,
            },
            generation.verifier_key_id,
        )
        .map_err(HistoricalGenerationObserverErrorV2::DidMethod)?;

        let assembled = assemble_verifier_key_generation_material_v2(
            VerifierKeyGenerationLifecycleAssertionV2 {
                generation_action_id: generation.action_id,
                did_document_action_id: generation.did_document_action_id,
                previous_generation_action_id: generation.previous_generation_action_id,
                verifier_did: generation.verifier_did,
                verifier_key_id: generation.verifier_key_id,
                key_generation: generation.key_generation,
                generation_action_timestamp_micros: generation.action_timestamp_micros,
                valid_from_micros: generation.valid_from_micros,
                valid_until_micros: generation.valid_until_micros,
            },
            document.action_id,
            method,
        )
        .map_err(HistoricalGenerationObserverErrorV2::GenerationMaterial)?;
        assembled_generations.push(assembled);
    }

    let target_history: Vec<ObservedVerifierKeyGenerationRecordV2<'_>> = assembled_generations
        .iter()
        .filter(|assembled| assembled.verifier_key_id() == signed_record.verifier_key_id)
        .map(|assembled| ObservedVerifierKeyGenerationRecordV2 {
            action_id: assembled.generation_action_id(),
            previous_generation_action_id: assembled.previous_generation_action_id(),
            generation: assembled.as_generation(),
        })
        .collect();

    let resolved_generation =
        resolve_record_bound_verifier_key_generation_v2(&target_history, signed_record)
            .map_err(HistoricalGenerationObserverErrorV2::GenerationLineage)?;

    let selected_did_document_action_id = assembled_generations
        .iter()
        .find(|assembled| {
            assembled.generation_action_id() == resolved_generation.generation_action_id()
        })
        .map(|assembled| assembled.did_document_action_id().to_string())
        .ok_or(HistoricalGenerationObserverErrorV2::SelectedGenerationAssemblyMissing)?;

    Ok(QualifiedNormalizedHistoricalGenerationObservationV2 {
        coverage,
        did_lineage,
        resolved_generation,
        selected_did_document_action_id,
        assembled_generation_count: assembled_generations.len(),
        deactivations,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_agent_activity_coverage_policy::{
        qualify_agent_activity_coverage_v2, AgentActivityObservationV2,
        ObservedActivityActionV2, ObservedChainStatusV2, ObservedHighestActivityV2,
    };
    use mycelix_crypto::{AlgorithmId, TaggedPublicKey};
    use mycelix_kvector_verification_record_policy::KVectorProofVerificationOutcomeV2;
    use mycelix_kvector_verifier_key_generation_policy::{
        derive_kvector_verifier_key_generation_digest_v2, KVectorVerifierKeyGenerationV2,
    };

    const DID: &str = "did:mycelix:verifier";
    const KEY_ID: &str = "did:mycelix:verifier#key-1";

    fn multibase(fill: u8) -> String {
        TaggedPublicKey::new(AlgorithmId::Ed25519, vec![fill; 32])
            .unwrap()
            .to_multibase()
    }

    fn coverage(actions: &[ObservedActivityActionV2<'_>]) -> QualifiedAgentActivityCoverageV2 {
        let highest_ids = ["head-42"];
        qualify_agent_activity_coverage_v2(AgentActivityObservationV2 {
            status: ObservedChainStatusV2::Valid {
                action_seq: 42,
                action_id: "head-42",
            },
            valid_activity: actions,
            rejected_activity: &[],
            highest_observed: Some(ObservedHighestActivityV2 {
                action_seq: 42,
                action_ids: &highest_ids,
            }),
            warrant_count: 0,
        })
        .unwrap()
    }

    fn generation_digest(fill: u8) -> [u8; 32] {
        let key = vec![fill; 32];
        derive_kvector_verifier_key_generation_digest_v2(KVectorVerifierKeyGenerationV2 {
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            algorithm: AlgorithmId::Ed25519,
            public_key_bytes: &key,
            key_generation: 1,
            issued_at_micros: 900_000,
            valid_from_micros: 1_000_000,
            valid_until_micros: 2_000_000,
        })
        .unwrap()
    }

    fn signed_record<'a>(
        digest: &'a [u8],
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
            verifier_key_generation_sha256: digest,
            signature_scheme_id: "ed25519-v1",
            verification_policy_sha256: &POLICY,
            outcome: KVectorProofVerificationOutcomeV2::Accepted,
            verified_at_micros,
            valid_until_micros: verified_at_micros + 200_000,
        }
    }

    fn method<'a>(key: &'a str) -> Vec<FetchedDidVerificationMethodV2<'a>> {
        vec![FetchedDidVerificationMethodV2 {
            id: "#key-1",
            type_: AlgorithmId::Ed25519.did_verification_method_type(),
            controller: DID,
            public_key_multibase: key,
            algorithm: Some(AlgorithmId::Ed25519.as_u16()),
        }]
    }

    fn did_document<'a>(
        action_seq: u32,
        action_id: &'a str,
        previous_action_id: Option<&'a str>,
        version: u32,
        methods: &'a [FetchedDidVerificationMethodV2<'a>],
        authentication: &'a [&'a str],
    ) -> FetchedIdentitySecurityRecordV2<'a> {
        FetchedIdentitySecurityRecordV2::DidDocument(FetchedDidDocumentRecordV2 {
            action_seq,
            action_id,
            previous_action_id,
            version,
            did: DID,
            verification_methods: methods,
            authentication,
            key_agreement: &[],
        })
    }

    fn generation_record<'a>(
        action_seq: u32,
        action_id: &'a str,
        did_document_action_id: &'a str,
    ) -> FetchedIdentitySecurityRecordV2<'a> {
        FetchedIdentitySecurityRecordV2::VerifierKeyGeneration(
            FetchedVerifierKeyGenerationRecordV2 {
                action_seq,
                action_id,
                action_timestamp_micros: 900_000,
                verifier_did: DID,
                verifier_key_id: KEY_ID,
                did_document_action_id,
                key_generation: 1,
                valid_from_micros: 1_000_000,
                valid_until_micros: 2_000_000,
                previous_generation_action_id: None,
            },
        )
    }

    #[test]
    fn exact_lineage_and_generation_resolve_historical_key() {
        let key = multibase(0x42);
        let methods = method(&key);
        let auth = ["#key-1"];
        let records = [
            did_document(8, "did-1", None, 1, &methods, &auth),
            generation_record(20, "gen-1", "did-1"),
        ];
        let actions = [
            ObservedActivityActionV2 {
                action_seq: 8,
                action_id: "did-1",
            },
            ObservedActivityActionV2 {
                action_seq: 20,
                action_id: "gen-1",
            },
        ];
        let digest = generation_digest(0x42);

        let observed = assemble_normalized_historical_generation_observation_v2(
            coverage(&actions),
            &records,
            signed_record(&digest, 1_500_000),
        )
        .unwrap();

        assert!(observed.did_lineage().contains_document_action("did-1"));
        assert_eq!(observed.resolved_generation().generation_action_id(), "gen-1");
        assert_eq!(observed.selected_did_document_action_id(), "did-1");
        assert_eq!(observed.resolved_generation().public_key_bytes(), &[0x42; 32]);
    }

    #[test]
    fn second_independent_did_root_fails_before_generation_material() {
        let key = multibase(0x42);
        let methods = method(&key);
        let auth = ["#key-1"];
        let records = [
            did_document(8, "did-1", None, 1, &methods, &auth),
            did_document(12, "rogue-root", None, 1, &methods, &auth),
            generation_record(20, "gen-1", "rogue-root"),
        ];
        let actions = [
            ObservedActivityActionV2 {
                action_seq: 8,
                action_id: "did-1",
            },
            ObservedActivityActionV2 {
                action_seq: 12,
                action_id: "rogue-root",
            },
            ObservedActivityActionV2 {
                action_seq: 20,
                action_id: "gen-1",
            },
        ];
        let digest = generation_digest(0x42);

        assert_eq!(
            assemble_normalized_historical_generation_observation_v2(
                coverage(&actions),
                &records,
                signed_record(&digest, 1_500_000),
            )
            .unwrap_err(),
            HistoricalGenerationObserverErrorV2::DidDocumentLineage(
                DidDocumentLineageErrorV2::MultipleCreationRoots
            )
        );
    }

    #[test]
    fn branched_did_updates_fail_before_generation_material() {
        let key = multibase(0x42);
        let methods = method(&key);
        let auth = ["#key-1"];
        let records = [
            did_document(8, "did-1", None, 1, &methods, &auth),
            did_document(12, "did-2a", Some("did-1"), 2, &methods, &auth),
            did_document(13, "did-2b", Some("did-1"), 999, &methods, &auth),
            generation_record(20, "gen-1", "did-2a"),
        ];
        let actions = [
            ObservedActivityActionV2 {
                action_seq: 8,
                action_id: "did-1",
            },
            ObservedActivityActionV2 {
                action_seq: 12,
                action_id: "did-2a",
            },
            ObservedActivityActionV2 {
                action_seq: 13,
                action_id: "did-2b",
            },
            ObservedActivityActionV2 {
                action_seq: 20,
                action_id: "gen-1",
            },
        ];
        let digest = generation_digest(0x42);

        assert_eq!(
            assemble_normalized_historical_generation_observation_v2(
                coverage(&actions),
                &records,
                signed_record(&digest, 1_500_000),
            )
            .unwrap_err(),
            HistoricalGenerationObserverErrorV2::DidDocumentLineage(
                DidDocumentLineageErrorV2::BranchingHistory
            )
        );
    }

    #[test]
    fn fetched_record_substitution_fails_before_lineage() {
        let key = multibase(0x42);
        let methods = method(&key);
        let auth = ["#key-1"];
        let records = [did_document(
            8,
            "substituted-did",
            None,
            1,
            &methods,
            &auth,
        )];
        let actions = [ObservedActivityActionV2 {
            action_seq: 8,
            action_id: "qualified-did",
        }];
        let digest = generation_digest(0x42);

        assert_eq!(
            assemble_normalized_historical_generation_observation_v2(
                coverage(&actions),
                &records,
                signed_record(&digest, 1_500_000),
            )
            .unwrap_err(),
            HistoricalGenerationObserverErrorV2::FetchedRecordNotInQualifiedActivity
        );
    }

    #[test]
    fn missing_qualified_generation_fetch_fails_closed() {
        let key = multibase(0x42);
        let methods = method(&key);
        let auth = ["#key-1"];
        let records = [did_document(8, "did-1", None, 1, &methods, &auth)];
        let actions = [
            ObservedActivityActionV2 {
                action_seq: 8,
                action_id: "did-1",
            },
            ObservedActivityActionV2 {
                action_seq: 20,
                action_id: "gen-1",
            },
        ];
        let digest = generation_digest(0x42);

        assert_eq!(
            assemble_normalized_historical_generation_observation_v2(
                coverage(&actions),
                &records,
                signed_record(&digest, 1_500_000),
            )
            .unwrap_err(),
            HistoricalGenerationObserverErrorV2::QualifiedActivityRecordMissing
        );
    }

    #[test]
    fn later_deactivation_is_retained_without_retroactive_veto() {
        let key = multibase(0x42);
        let methods = method(&key);
        let auth = ["#key-1"];
        let records = [
            did_document(8, "did-1", None, 1, &methods, &auth),
            generation_record(20, "gen-1", "did-1"),
            FetchedIdentitySecurityRecordV2::DidDeactivation(
                FetchedDidDeactivationRecordV2 {
                    action_seq: 30,
                    action_id: "deactivate",
                    did: DID,
                },
            ),
        ];
        let actions = [
            ObservedActivityActionV2 {
                action_seq: 8,
                action_id: "did-1",
            },
            ObservedActivityActionV2 {
                action_seq: 20,
                action_id: "gen-1",
            },
            ObservedActivityActionV2 {
                action_seq: 30,
                action_id: "deactivate",
            },
        ];
        let digest = generation_digest(0x42);

        let observed = assemble_normalized_historical_generation_observation_v2(
            coverage(&actions),
            &records,
            signed_record(&digest, 1_500_000),
        )
        .unwrap();
        assert_eq!(observed.deactivations().len(), 1);
        assert_eq!(observed.deactivations()[0].action_id(), "deactivate");
        assert_eq!(observed.resolved_generation().generation_action_id(), "gen-1");
    }

    #[test]
    fn observer_success_fields_are_verifier_owned() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedNormalizedHistoricalGenerationObservationV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedNormalizedHistoricalGenerationObservationV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for public_field in [
            "pub coverage:",
            "pub did_lineage:",
            "pub resolved_generation:",
            "pub selected_did_document_action_id:",
            "pub assembled_generation_count:",
            "pub deactivations:",
        ] {
            assert!(!body.contains(public_field));
        }
    }
}
