use serde::{Deserialize, Serialize};

use crate::{
    GitObjectIdV1, QualificationReceiptDigestV1, QualificationResultV1, Sha256DigestV1,
};

/// Schema ID embedded inside the custom in-toto predicate JSON.
///
/// The outer in-toto predicate type URI remains an explicit authentication-policy field
/// and is independently checked by verifier backends.
pub const QUALIFICATION_ATTESTATION_PREDICATE_SCHEMA_V1: &str =
    "mycelix-qualification-attestation-predicate-v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationAttestationPredicateV1 {
    pub predicate_version: u32,
    pub predicate_schema: String,
    pub receipt_digest: QualificationReceiptDigestV1,
    pub qualification_profile: String,
    pub subject: GitObjectIdV1,
    pub coherence_result_digest: Sha256DigestV1,
    pub result: QualificationResultV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum QualificationAttestationPredicateErrorV1 {
    UnsupportedPredicateVersion { actual: u32 },
    PredicateSchemaMismatch { actual: String },
    EmptyQualificationProfile,
}

impl QualificationAttestationPredicateV1 {
    pub fn validate(&self) -> Result<(), QualificationAttestationPredicateErrorV1> {
        if self.predicate_version != 1 {
            return Err(
                QualificationAttestationPredicateErrorV1::UnsupportedPredicateVersion {
                    actual: self.predicate_version,
                },
            );
        }
        if self.predicate_schema != QUALIFICATION_ATTESTATION_PREDICATE_SCHEMA_V1 {
            return Err(
                QualificationAttestationPredicateErrorV1::PredicateSchemaMismatch {
                    actual: self.predicate_schema.clone(),
                },
            );
        }
        if self.qualification_profile.trim().is_empty() {
            return Err(QualificationAttestationPredicateErrorV1::EmptyQualificationProfile);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{QualificationReceiptCanonicalizationV1, Sha256DigestV1};

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    #[test]
    fn exact_v1_predicate_is_valid() {
        let predicate = QualificationAttestationPredicateV1 {
            predicate_version: 1,
            predicate_schema: QUALIFICATION_ATTESTATION_PREDICATE_SCHEMA_V1.into(),
            receipt_digest: QualificationReceiptDigestV1 {
                canonicalization: QualificationReceiptCanonicalizationV1::BinaryV1,
                sha256: digest(1),
            },
            qualification_profile: "myc-zkp-range-001aq".into(),
            subject: GitObjectIdV1::sha1([0xaa; 20]),
            coherence_result_digest: digest(2),
            result: QualificationResultV1::Pass,
        };
        assert_eq!(predicate.validate(), Ok(()));
    }

    #[test]
    fn schema_or_version_substitution_fails() {
        let mut predicate = QualificationAttestationPredicateV1 {
            predicate_version: 1,
            predicate_schema: QUALIFICATION_ATTESTATION_PREDICATE_SCHEMA_V1.into(),
            receipt_digest: QualificationReceiptDigestV1 {
                canonicalization: QualificationReceiptCanonicalizationV1::BinaryV1,
                sha256: digest(1),
            },
            qualification_profile: "myc-zkp-range-001aq".into(),
            subject: GitObjectIdV1::sha1([0xaa; 20]),
            coherence_result_digest: digest(2),
            result: QualificationResultV1::Pass,
        };
        predicate.predicate_version = 2;
        assert!(matches!(
            predicate.validate(),
            Err(QualificationAttestationPredicateErrorV1::UnsupportedPredicateVersion {
                actual: 2
            })
        ));

        predicate.predicate_version = 1;
        predicate.predicate_schema = "other-schema".into();
        assert!(matches!(
            predicate.validate(),
            Err(QualificationAttestationPredicateErrorV1::PredicateSchemaMismatch { .. })
        ));
    }
}
