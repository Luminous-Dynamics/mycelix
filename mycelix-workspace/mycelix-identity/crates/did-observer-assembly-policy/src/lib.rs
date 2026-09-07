// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure evidence-assembly theorem for the Identity V2 DID verifier observer.
//!
//! This crate composes three independent proof boundaries without performing any
//! Holochain/network reads itself:
//!
//! 1. #279 qualifies one viewpoint-scoped agent-activity observation and retains the
//!    exact matching action set;
//! 2. every fetched security-relevant DID record must match that qualified set exactly,
//!    and every fetched DID document must pass #272 cryptographic admission;
//! 3. the resulting exact CRUD/deactivation facts are passed to #267 for branch-aware
//!    verifier-key lineage resolution.
//!
//! The HDK adapter that feeds this theorem must query the security-relevant DID entry
//! types (DidDocument and DidDeactivation) together with `ActivityRequest::Full`, then
//! fetch every returned action by exact hash. Link indexes may corroborate discovery but
//! are not authority inputs to this theorem.
//!
//! Success remains observation-scoped. It is not signature authenticity, global DHT
//! consensus, verifier-policy currentness, or positive K-vector evidence.

#![forbid(unsafe_code)]

use std::collections::BTreeSet;

use mycelix_agent_activity_coverage_policy::{
    qualify_agent_activity_coverage_v2, AgentActivityCoverageErrorV2,
    AgentActivityObservationV2,
};
use mycelix_did_document_crypto_policy::{
    validate_did_document_crypto_v2, DidDocumentCryptoAdmissionErrorV2,
    DidDocumentCryptoAdmissionViewV2, DidVerificationMethodAdmissionViewV2,
};
use mycelix_did_verifier_lineage_policy::{
    resolve_observed_verifier_key_v2, DidVerifierLineageErrorV2, ObservedDidDeactivationV2,
    ObservedDidDocumentV2, ObservedVerifierKeyV2,
};
use mycelix_kvector_verifier_policy_body::KVectorVerifierPolicyBodyV2;
use mycelix_verifier_key_method_policy::{
    DidVerificationDocumentViewV2, DidVerificationMethodViewV2,
};

#[derive(Debug, Clone, Copy)]
pub struct FetchedDidVerificationMethodV2<'a> {
    pub id: &'a str,
    pub type_: &'a str,
    pub controller: &'a str,
    pub public_key_multibase: &'a str,
    pub algorithm: Option<u16>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FetchedDidDocumentActionKindV2<'a> {
    Create,
    Update { original_action_id: &'a str },
}

#[derive(Debug, Clone, Copy)]
pub struct FetchedDidDocumentRecordV2<'a> {
    pub action_seq: u32,
    pub action_id: &'a str,
    pub action_kind: FetchedDidDocumentActionKindV2<'a>,
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
pub enum FetchedDidSecurityRecordV2<'a> {
    Document(FetchedDidDocumentRecordV2<'a>),
    Deactivation(FetchedDidDeactivationRecordV2<'a>),
}

impl FetchedDidSecurityRecordV2<'_> {
    fn action_seq(&self) -> u32 {
        match self {
            Self::Document(record) => record.action_seq,
            Self::Deactivation(record) => record.action_seq,
        }
    }

    fn action_id(&self) -> &str {
        match self {
            Self::Document(record) => record.action_id,
            Self::Deactivation(record) => record.action_id,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualifiedDidObserverAssemblyV2 {
    coverage_head_action_seq: u32,
    coverage_head_action_id: String,
    observed_verifier_key: ObservedVerifierKeyV2,
}

impl QualifiedDidObserverAssemblyV2 {
    pub fn coverage_head_action_seq(&self) -> u32 {
        self.coverage_head_action_seq
    }

    pub fn coverage_head_action_id(&self) -> &str {
        &self.coverage_head_action_id
    }

    /// Observation-scoped verifier key selected only after exact activity/record
    /// membership, document crypto admission, and branch-aware lineage all pass.
    pub fn observed_verifier_key(&self) -> &ObservedVerifierKeyV2 {
        &self.observed_verifier_key
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DidObserverAssemblyErrorV2 {
    AgentActivity(AgentActivityCoverageErrorV2),
    DuplicateFetchedRecord,
    FetchedRecordNotInQualifiedActivity,
    QualifiedActivityRecordMissing,
    DidDocumentCrypto(DidDocumentCryptoAdmissionErrorV2),
    DidLineage(DidVerifierLineageErrorV2),
}

struct PreparedDidDocumentV2<'a> {
    action_id: &'a str,
    previous_action_id: Option<&'a str>,
    version: u32,
    did: &'a str,
    verifier_methods: Vec<DidVerificationMethodViewV2<'a>>,
    authentication: &'a [&'a str],
}

/// Compose one exact observer evidence set into observation-scoped verifier-key evidence.
///
/// The supplied `records` must be a bijection with the exact matching action set retained
/// by #279. Therefore:
///
/// - a fetched record whose `(sequence, hash)` was not qualified fails;
/// - a duplicate fetched record fails;
/// - any qualified action that was not fetched/typed fails;
/// - a deactivation action cannot be omitted after it appeared in qualified activity;
/// - every document is revalidated through #272 from the same normalized fields later
///   used to construct the #267 verifier-document view;
/// - CRUD update parentage comes from the fetched action's exact `original_action_id`,
///   not document timestamps, versions, or mutable index links.
pub fn assemble_observed_verifier_key_v2<'a>(
    activity_observation: AgentActivityObservationV2<'a>,
    records: &'a [FetchedDidSecurityRecordV2<'a>],
    policy: KVectorVerifierPolicyBodyV2<'a>,
) -> Result<QualifiedDidObserverAssemblyV2, DidObserverAssemblyErrorV2> {
    let qualified_activity = qualify_agent_activity_coverage_v2(activity_observation)
        .map_err(DidObserverAssemblyErrorV2::AgentActivity)?;

    let mut remaining: BTreeSet<(u32, String)> = qualified_activity
        .matching_valid_actions()
        .iter()
        .map(|action| (action.action_seq(), action.action_id().to_string()))
        .collect();
    let mut seen: BTreeSet<(u32, &str)> = BTreeSet::new();

    let mut prepared_documents: Vec<PreparedDidDocumentV2<'a>> = Vec::new();
    let mut deactivations: Vec<ObservedDidDeactivationV2<'a>> = Vec::new();

    for record in records {
        let seq = record.action_seq();
        let action_id = record.action_id();

        if !qualified_activity.contains_action(seq, action_id) {
            return Err(DidObserverAssemblyErrorV2::FetchedRecordNotInQualifiedActivity);
        }
        if !seen.insert((seq, action_id)) {
            return Err(DidObserverAssemblyErrorV2::DuplicateFetchedRecord);
        }
        remaining.remove(&(seq, action_id.to_string()));

        match record {
            FetchedDidSecurityRecordV2::Document(document) => {
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

                validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                    did: document.did,
                    verification_methods: &admission_methods,
                    authentication: document.authentication,
                    key_agreement: document.key_agreement,
                })
                .map_err(DidObserverAssemblyErrorV2::DidDocumentCrypto)?;

                let verifier_methods: Vec<DidVerificationMethodViewV2<'a>> = document
                    .verification_methods
                    .iter()
                    .map(|method| DidVerificationMethodViewV2 {
                        id: method.id,
                        type_: method.type_,
                        controller: method.controller,
                        public_key_multibase: method.public_key_multibase,
                        algorithm: method.algorithm,
                    })
                    .collect();

                let previous_action_id = match document.action_kind {
                    FetchedDidDocumentActionKindV2::Create => None,
                    FetchedDidDocumentActionKindV2::Update { original_action_id } => {
                        Some(original_action_id)
                    }
                };

                prepared_documents.push(PreparedDidDocumentV2 {
                    action_id: document.action_id,
                    previous_action_id,
                    version: document.version,
                    did: document.did,
                    verifier_methods,
                    authentication: document.authentication,
                });
            }
            FetchedDidSecurityRecordV2::Deactivation(deactivation) => {
                deactivations.push(ObservedDidDeactivationV2 {
                    action_id: deactivation.action_id,
                    did: deactivation.did,
                });
            }
        }
    }

    if !remaining.is_empty() {
        return Err(DidObserverAssemblyErrorV2::QualifiedActivityRecordMissing);
    }

    let history: Vec<ObservedDidDocumentV2<'_>> = prepared_documents
        .iter()
        .map(|document| ObservedDidDocumentV2 {
            action_id: document.action_id,
            previous_action_id: document.previous_action_id,
            version: document.version,
            document: DidVerificationDocumentViewV2 {
                did: document.did,
                verification_methods: &document.verifier_methods,
                authentication: document.authentication,
            },
        })
        .collect();

    let observed_verifier_key = resolve_observed_verifier_key_v2(&history, &deactivations, policy)
        .map_err(DidObserverAssemblyErrorV2::DidLineage)?;

    Ok(QualifiedDidObserverAssemblyV2 {
        coverage_head_action_seq: qualified_activity.valid_head_action_seq(),
        coverage_head_action_id: qualified_activity.valid_head_action_id().to_string(),
        observed_verifier_key,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_agent_activity_coverage_policy::{
        ObservedActivityActionV2, ObservedChainStatusV2, ObservedHighestActivityV2,
    };
    use mycelix_did_verifier_lineage_policy::DidVerifierLineageErrorV2;

    const DID: &str = "did:mycelix:verifier";
    const KEY_ID: &str = "did:mycelix:verifier#key-1";
    const ED_KEY: &str = "z6MkhaXgBZDvotDkL5257faiztiGiC2QtKLGpbnnEGta2doK";

    static METHODS: [FetchedDidVerificationMethodV2<'static>; 1] =
        [FetchedDidVerificationMethodV2 {
            id: "#key-1",
            type_: "Ed25519VerificationKey2020",
            controller: DID,
            public_key_multibase: ED_KEY,
            algorithm: Some(0xed01),
        }];
    static AUTH: [&str; 1] = ["#key-1"];
    static NO_KA: [&str; 0] = [];

    fn policy() -> KVectorVerifierPolicyBodyV2<'static> {
        KVectorVerifierPolicyBodyV2 {
            policy_id: "policy:kvector-prod-v2",
            policy_version: "2.0.0",
            backend_id: "winterfell-v2",
            circuit_id: "mycelix-kvector-range-v2",
            circuit_version: "2.0.0",
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            signature_scheme_id: "ed25519-v1",
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
            max_record_lifetime_micros: 2_000_000,
        }
    }

    fn activity<'a>(
        actions: &'a [ObservedActivityActionV2<'a>],
        highest_ids: &'a [&'a str],
    ) -> AgentActivityObservationV2<'a> {
        AgentActivityObservationV2 {
            status: ObservedChainStatusV2::Valid {
                action_seq: 42,
                action_id: "head-42",
            },
            valid_activity: actions,
            rejected_activity: &[],
            highest_observed: Some(ObservedHighestActivityV2 {
                action_seq: 42,
                action_ids: highest_ids,
            }),
            warrant_count: 0,
        }
    }

    fn create_record(action_seq: u32, action_id: &'static str) -> FetchedDidSecurityRecordV2<'static> {
        FetchedDidSecurityRecordV2::Document(FetchedDidDocumentRecordV2 {
            action_seq,
            action_id,
            action_kind: FetchedDidDocumentActionKindV2::Create,
            version: 1,
            did: DID,
            verification_methods: &METHODS,
            authentication: &AUTH,
            key_agreement: &NO_KA,
        })
    }

    fn update_record(
        action_seq: u32,
        action_id: &'static str,
        original_action_id: &'static str,
        version: u32,
    ) -> FetchedDidSecurityRecordV2<'static> {
        FetchedDidSecurityRecordV2::Document(FetchedDidDocumentRecordV2 {
            action_seq,
            action_id,
            action_kind: FetchedDidDocumentActionKindV2::Update { original_action_id },
            version,
            did: DID,
            verification_methods: &METHODS,
            authentication: &AUTH,
            key_agreement: &NO_KA,
        })
    }

    #[test]
    fn exact_qualified_activity_and_records_compose_to_observed_key() {
        let actions = [
            ObservedActivityActionV2 {
                action_seq: 8,
                action_id: "did-create",
            },
            ObservedActivityActionV2 {
                action_seq: 21,
                action_id: "did-update",
            },
        ];
        let highest = ["head-42"];
        let records = [
            create_record(8, "did-create"),
            update_record(21, "did-update", "did-create", 2),
        ];

        let qualified = assemble_observed_verifier_key_v2(
            activity(&actions, &highest),
            &records,
            policy(),
        )
        .expect("exact observation should compose");

        assert_eq!(qualified.coverage_head_action_seq(), 42);
        assert_eq!(qualified.coverage_head_action_id(), "head-42");
        assert_eq!(
            qualified.observed_verifier_key().document_action_id,
            "did-update"
        );
        assert_eq!(qualified.observed_verifier_key().key.canonical_key_id, KEY_ID);
    }

    #[test]
    fn substituted_fetched_action_fails_before_lineage() {
        let actions = [ObservedActivityActionV2 {
            action_seq: 8,
            action_id: "qualified-create",
        }];
        let highest = ["head-42"];
        let records = [create_record(8, "substituted-create")];

        assert_eq!(
            assemble_observed_verifier_key_v2(activity(&actions, &highest), &records, policy()),
            Err(DidObserverAssemblyErrorV2::FetchedRecordNotInQualifiedActivity)
        );
    }

    #[test]
    fn missing_exact_fetch_fails_closed() {
        let actions = [
            ObservedActivityActionV2 {
                action_seq: 8,
                action_id: "did-create",
            },
            ObservedActivityActionV2 {
                action_seq: 21,
                action_id: "did-update",
            },
        ];
        let highest = ["head-42"];
        let records = [create_record(8, "did-create")];

        assert_eq!(
            assemble_observed_verifier_key_v2(activity(&actions, &highest), &records, policy()),
            Err(DidObserverAssemblyErrorV2::QualifiedActivityRecordMissing)
        );
    }

    #[test]
    fn deactivation_in_qualified_activity_cannot_be_omitted() {
        let actions = [
            ObservedActivityActionV2 {
                action_seq: 8,
                action_id: "did-create",
            },
            ObservedActivityActionV2 {
                action_seq: 30,
                action_id: "did-deactivate",
            },
        ];
        let highest = ["head-42"];
        let deactivation = FetchedDidSecurityRecordV2::Deactivation(
            FetchedDidDeactivationRecordV2 {
                action_seq: 30,
                action_id: "did-deactivate",
                did: DID,
            },
        );
        let records = [create_record(8, "did-create"), deactivation];

        assert_eq!(
            assemble_observed_verifier_key_v2(activity(&actions, &highest), &records, policy()),
            Err(DidObserverAssemblyErrorV2::DidLineage(
                DidVerifierLineageErrorV2::DidDeactivated
            ))
        );
    }

    #[test]
    fn malformed_document_fails_crypto_admission_before_lineage() {
        static BAD_METHODS: [FetchedDidVerificationMethodV2<'static>; 1] =
            [FetchedDidVerificationMethodV2 {
                id: "#key-1",
                type_: "MlDsa65VerificationKey2024",
                controller: DID,
                public_key_multibase: ED_KEY,
                algorithm: Some(0xed01),
            }];

        let actions = [ObservedActivityActionV2 {
            action_seq: 8,
            action_id: "did-create",
        }];
        let highest = ["head-42"];
        let records = [FetchedDidSecurityRecordV2::Document(
            FetchedDidDocumentRecordV2 {
                action_seq: 8,
                action_id: "did-create",
                action_kind: FetchedDidDocumentActionKindV2::Create,
                version: 1,
                did: DID,
                verification_methods: &BAD_METHODS,
                authentication: &AUTH,
                key_agreement: &NO_KA,
            },
        )];

        assert!(matches!(
            assemble_observed_verifier_key_v2(activity(&actions, &highest), &records, policy()),
            Err(DidObserverAssemblyErrorV2::DidDocumentCrypto(_))
        ));
    }

    #[test]
    fn fetched_update_parentage_is_lineage_authority() {
        let actions = [
            ObservedActivityActionV2 {
                action_seq: 8,
                action_id: "did-create",
            },
            ObservedActivityActionV2 {
                action_seq: 21,
                action_id: "did-update",
            },
        ];
        let highest = ["head-42"];
        let records = [
            create_record(8, "did-create"),
            update_record(21, "did-update", "missing-parent", 2),
        ];

        assert_eq!(
            assemble_observed_verifier_key_v2(activity(&actions, &highest), &records, policy()),
            Err(DidObserverAssemblyErrorV2::DidLineage(
                DidVerifierLineageErrorV2::UpdateParentMissing
            ))
        );
    }
}
