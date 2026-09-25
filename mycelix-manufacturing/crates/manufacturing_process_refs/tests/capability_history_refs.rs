use mycelix_manufacturing_process_refs::{
    DigestAlgorithmV1, DigestRefV1, ExternalSubjectRefV1, ResourceOfferEvidenceClassV1,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
struct CapabilityHistorySubjectRefV1(ExternalSubjectRefV1);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
struct CapabilityAggregateCommitmentRefV1(ExternalSubjectRefV1);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
enum CapabilityEvidenceAttachmentV1 {
    PublicHistory(CapabilityHistorySubjectRefV1),
    PrivateAggregate(CapabilityAggregateCommitmentRefV1),
}

impl CapabilityEvidenceAttachmentV1 {
    fn validate(&self) -> Result<(), String> {
        match self {
            Self::PublicHistory(reference) => reference.0.validate(),
            Self::PrivateAggregate(reference) => reference.0.validate(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct ResourceCapabilityEvidenceBundleV1 {
    resource_offer_evidence_class: ResourceOfferEvidenceClassV1,
    #[serde(default)]
    attachments: Vec<CapabilityEvidenceAttachmentV1>,
}

impl ResourceCapabilityEvidenceBundleV1 {
    fn validate(&self) -> Result<(), String> {
        let mut seen = BTreeSet::new();
        for attachment in &self.attachments {
            attachment.validate()?;
            if !seen.insert(attachment.clone()) {
                return Err("duplicate capability-history/aggregate attachment".into());
            }
        }
        Ok(())
    }
}

fn digest(fill: char) -> DigestRefV1 {
    DigestRefV1 {
        algorithm: DigestAlgorithmV1::Blake3,
        hex: std::iter::repeat_n(fill, 64).collect(),
    }
}

fn subject(namespace: &str, id: &str, fill: char) -> ExternalSubjectRefV1 {
    ExternalSubjectRefV1 {
        namespace: namespace.into(),
        subject_id: id.into(),
        semantic_version: "1".into(),
        content_digest: digest(fill),
    }
}

#[test]
fn public_history_and_private_aggregate_remain_distinct() {
    let public = CapabilityEvidenceAttachmentV1::PublicHistory(CapabilityHistorySubjectRefV1(
        subject("symthaea.mfg-capability-history", "hist-001", 'a'),
    ));
    let private = CapabilityEvidenceAttachmentV1::PrivateAggregate(
        CapabilityAggregateCommitmentRefV1(subject(
            "symthaea.mfg-capability-aggregate",
            "commit-001",
            'b',
        )),
    );
    assert_ne!(public, private);

    let public_json = serde_json::to_string(&public).unwrap();
    let private_json = serde_json::to_string(&private).unwrap();
    assert_ne!(public_json, private_json);
    assert_eq!(
        serde_json::from_str::<CapabilityEvidenceAttachmentV1>(&public_json).unwrap(),
        public
    );
    assert_eq!(
        serde_json::from_str::<CapabilityEvidenceAttachmentV1>(&private_json).unwrap(),
        private
    );
}

#[test]
fn malformed_nested_history_ref_fails_closed() {
    let malformed = CapabilityEvidenceAttachmentV1::PublicHistory(CapabilityHistorySubjectRefV1(
        ExternalSubjectRefV1 {
            namespace: "symthaea.mfg-capability-history".into(),
            subject_id: "hist-bad".into(),
            semantic_version: "1".into(),
            content_digest: DigestRefV1 {
                algorithm: DigestAlgorithmV1::Sha256,
                hex: "ABC".into(),
            },
        },
    ));
    assert!(malformed.validate().is_err());
}

#[test]
fn duplicate_attachments_reject() {
    let attachment = CapabilityEvidenceAttachmentV1::PrivateAggregate(
        CapabilityAggregateCommitmentRefV1(subject(
            "symthaea.mfg-capability-aggregate",
            "commit-001",
            'b',
        )),
    );
    let bundle = ResourceCapabilityEvidenceBundleV1 {
        resource_offer_evidence_class: ResourceOfferEvidenceClassV1::QualifiedUnderProfile,
        attachments: vec![attachment.clone(), attachment],
    };
    assert!(bundle.validate().is_err());
}

#[test]
fn capability_history_attachment_does_not_upgrade_offer_evidence_class() {
    let bundle = ResourceCapabilityEvidenceBundleV1 {
        resource_offer_evidence_class: ResourceOfferEvidenceClassV1::Declared,
        attachments: vec![CapabilityEvidenceAttachmentV1::PublicHistory(
            CapabilityHistorySubjectRefV1(subject(
                "symthaea.mfg-capability-history",
                "hist-strong-looking",
                'c',
            )),
        )],
    };
    assert!(bundle.validate().is_ok());
    assert_eq!(
        bundle.resource_offer_evidence_class,
        ResourceOfferEvidenceClassV1::Declared
    );
}

#[test]
fn private_aggregate_ref_contains_identity_not_raw_production_payload() {
    let attachment = CapabilityEvidenceAttachmentV1::PrivateAggregate(
        CapabilityAggregateCommitmentRefV1(subject(
            "symthaea.mfg-capability-aggregate",
            "aggregate-q3",
            'd',
        )),
    );
    let json = serde_json::to_string(&attachment).unwrap();
    assert!(json.contains("aggregate-q3"));
    assert!(!json.contains("raw_measurements"));
    assert!(!json.contains("customer_id"));
    assert!(!json.contains("price"));
    assert!(!json.contains("queue_depth"));
}
