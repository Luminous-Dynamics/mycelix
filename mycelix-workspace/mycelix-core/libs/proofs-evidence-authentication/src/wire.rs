use serde::Deserialize;

use crate::{
    AuthenticationFreshnessPolicyV1, AuthenticationPolicyErrorV1, GitObjectIdV1,
    QualificationAttestationPredicateErrorV1, QualificationAttestationPredicateV1,
    QualificationReceiptDigestV1, QualificationResultV1, ReceiptAuthenticationPolicyV1,
    Sha256DigestV1, SourceRevisionPolicyV1, TransparencyPolicyV1, VerifierProfileV1,
    WorkflowRevisionPolicyV1,
};

/// Maximum total JSON document size accepted by an untrusted v1 parser.
pub const MAX_UNTRUSTED_WIRE_JSON_BYTES_V1: usize = 256 * 1024;
/// Maximum byte length accepted for one untrusted v1 textual identity field.
pub const MAX_UNTRUSTED_WIRE_STRING_BYTES_V1: usize = 1024;
/// Maximum number of explicitly allowed signer-workflow revisions in one policy.
pub const MAX_UNTRUSTED_WORKFLOW_REVISIONS_V1: usize = 64;

/// Parse/validation failures at the untrusted JSON boundary.
///
/// This layer establishes only strict structural acceptance. It performs no
/// cryptographic verification and cannot mint an authenticated capability.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UntrustedWireParseErrorV1 {
    InputTooLarge {
        maximum: usize,
        actual: usize,
    },
    InvalidJson,
    FieldTooLong {
        field: &'static str,
        maximum: usize,
        actual: usize,
    },
    TooManyWorkflowRevisions {
        maximum: usize,
        actual: usize,
    },
    InvalidPolicy(AuthenticationPolicyErrorV1),
    InvalidPredicate(QualificationAttestationPredicateErrorV1),
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StrictVerifierProfileV1 {
    profile_id: String,
    backend_family: String,
    backend_version: String,
    verification_profile: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StrictSourceRevisionPolicyV1 {
    exact_commit: GitObjectIdV1,
    exact_git_ref: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StrictReceiptAuthenticationPolicyV1 {
    profile_id: String,
    verifier_profile: StrictVerifierProfileV1,
    expected_attestation_subject_name: String,
    expected_predicate_type: String,
    expected_predicate_schema: String,
    trusted_root_profile: String,
    trusted_root_digest: Sha256DigestV1,
    oidc_issuer: String,
    source_repository: String,
    source_repository_owner: String,
    signer_workflow: String,
    signer_workflow_revision: WorkflowRevisionPolicyV1,
    source_revision: StrictSourceRevisionPolicyV1,
    transparency_policy: TransparencyPolicyV1,
    freshness_policy: AuthenticationFreshnessPolicyV1,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StrictQualificationAttestationPredicateV1 {
    predicate_version: u32,
    predicate_schema: String,
    receipt_digest: QualificationReceiptDigestV1,
    qualification_profile: String,
    subject: GitObjectIdV1,
    coherence_result_digest: Sha256DigestV1,
    result: QualificationResultV1,
}

/// Strictly parse untrusted JSON into a validated authentication policy.
///
/// Callers that participate in an authority-bearing verifier path should use
/// this function rather than direct convenience deserialization of the public
/// policy type.
pub fn parse_untrusted_authentication_policy_json_v1(
    bytes: &[u8],
) -> Result<ReceiptAuthenticationPolicyV1, UntrustedWireParseErrorV1> {
    check_document_bound(bytes)?;
    let raw: StrictReceiptAuthenticationPolicyV1 =
        serde_json::from_slice(bytes).map_err(|_| UntrustedWireParseErrorV1::InvalidJson)?;

    for (field, value) in [
        ("profile_id", raw.profile_id.as_str()),
        ("verifier_profile.profile_id", raw.verifier_profile.profile_id.as_str()),
        (
            "verifier_profile.backend_family",
            raw.verifier_profile.backend_family.as_str(),
        ),
        (
            "verifier_profile.backend_version",
            raw.verifier_profile.backend_version.as_str(),
        ),
        (
            "verifier_profile.verification_profile",
            raw.verifier_profile.verification_profile.as_str(),
        ),
        (
            "expected_attestation_subject_name",
            raw.expected_attestation_subject_name.as_str(),
        ),
        ("expected_predicate_type", raw.expected_predicate_type.as_str()),
        (
            "expected_predicate_schema",
            raw.expected_predicate_schema.as_str(),
        ),
        ("trusted_root_profile", raw.trusted_root_profile.as_str()),
        ("oidc_issuer", raw.oidc_issuer.as_str()),
        ("source_repository", raw.source_repository.as_str()),
        (
            "source_repository_owner",
            raw.source_repository_owner.as_str(),
        ),
        ("signer_workflow", raw.signer_workflow.as_str()),
    ] {
        check_text_bound(field, value)?;
    }

    if let Some(value) = raw.source_revision.exact_git_ref.as_deref() {
        check_text_bound("source_revision.exact_git_ref", value)?;
    }

    if let WorkflowRevisionPolicyV1::Allowed(revisions) = &raw.signer_workflow_revision {
        if revisions.len() > MAX_UNTRUSTED_WORKFLOW_REVISIONS_V1 {
            return Err(UntrustedWireParseErrorV1::TooManyWorkflowRevisions {
                maximum: MAX_UNTRUSTED_WORKFLOW_REVISIONS_V1,
                actual: revisions.len(),
            });
        }
    }

    let policy = ReceiptAuthenticationPolicyV1 {
        profile_id: raw.profile_id,
        verifier_profile: VerifierProfileV1 {
            profile_id: raw.verifier_profile.profile_id,
            backend_family: raw.verifier_profile.backend_family,
            backend_version: raw.verifier_profile.backend_version,
            verification_profile: raw.verifier_profile.verification_profile,
        },
        expected_attestation_subject_name: raw.expected_attestation_subject_name,
        expected_predicate_type: raw.expected_predicate_type,
        expected_predicate_schema: raw.expected_predicate_schema,
        trusted_root_profile: raw.trusted_root_profile,
        trusted_root_digest: raw.trusted_root_digest,
        oidc_issuer: raw.oidc_issuer,
        source_repository: raw.source_repository,
        source_repository_owner: raw.source_repository_owner,
        signer_workflow: raw.signer_workflow,
        signer_workflow_revision: raw.signer_workflow_revision,
        source_revision: SourceRevisionPolicyV1 {
            exact_commit: raw.source_revision.exact_commit,
            exact_git_ref: raw.source_revision.exact_git_ref,
        },
        transparency_policy: raw.transparency_policy,
        freshness_policy: raw.freshness_policy,
    };

    policy
        .validate()
        .map_err(UntrustedWireParseErrorV1::InvalidPolicy)?;
    Ok(policy)
}

/// Strictly parse untrusted JSON into a validated Mycelix qualification predicate.
///
/// Successful parsing establishes no signature authenticity, signer identity,
/// transparency evidence, freshness, production admission, or application authority.
pub fn parse_untrusted_qualification_predicate_json_v1(
    bytes: &[u8],
) -> Result<QualificationAttestationPredicateV1, UntrustedWireParseErrorV1> {
    check_document_bound(bytes)?;
    let raw: StrictQualificationAttestationPredicateV1 =
        serde_json::from_slice(bytes).map_err(|_| UntrustedWireParseErrorV1::InvalidJson)?;

    check_text_bound("predicate_schema", &raw.predicate_schema)?;
    check_text_bound("qualification_profile", &raw.qualification_profile)?;

    let predicate = QualificationAttestationPredicateV1 {
        predicate_version: raw.predicate_version,
        predicate_schema: raw.predicate_schema,
        receipt_digest: raw.receipt_digest,
        qualification_profile: raw.qualification_profile,
        subject: raw.subject,
        coherence_result_digest: raw.coherence_result_digest,
        result: raw.result,
    };

    predicate
        .validate()
        .map_err(UntrustedWireParseErrorV1::InvalidPredicate)?;
    Ok(predicate)
}

fn check_document_bound(bytes: &[u8]) -> Result<(), UntrustedWireParseErrorV1> {
    let actual = bytes.len();
    if actual > MAX_UNTRUSTED_WIRE_JSON_BYTES_V1 {
        return Err(UntrustedWireParseErrorV1::InputTooLarge {
            maximum: MAX_UNTRUSTED_WIRE_JSON_BYTES_V1,
            actual,
        });
    }
    Ok(())
}

fn check_text_bound(field: &'static str, value: &str) -> Result<(), UntrustedWireParseErrorV1> {
    let actual = value.len();
    if actual > MAX_UNTRUSTED_WIRE_STRING_BYTES_V1 {
        return Err(UntrustedWireParseErrorV1::FieldTooLong {
            field,
            maximum: MAX_UNTRUSTED_WIRE_STRING_BYTES_V1,
            actual,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        QualificationReceiptCanonicalizationV1,
        QUALIFICATION_ATTESTATION_PREDICATE_SCHEMA_V1,
    };

    fn digest(byte: u8) -> Sha256DigestV1 {
        Sha256DigestV1::from_bytes([byte; 32])
    }

    fn policy() -> ReceiptAuthenticationPolicyV1 {
        ReceiptAuthenticationPolicyV1 {
            profile_id: "github-public-v1".into(),
            verifier_profile: VerifierProfileV1 {
                profile_id: "github-attestation-verifier-v1".into(),
                backend_family: "github-artifact-attestation".into(),
                backend_version: "v1".into(),
                verification_profile: "public-sigstore-qualification-v1".into(),
            },
            expected_attestation_subject_name: "mycelix-qualification-receipt".into(),
            expected_predicate_type: "https://mycelix.org/attestations/qualification/v1".into(),
            expected_predicate_schema: QUALIFICATION_ATTESTATION_PREDICATE_SCHEMA_V1.into(),
            trusted_root_profile: "sigstore-public-good-v1".into(),
            trusted_root_digest: digest(9),
            oidc_issuer: "https://token.actions.githubusercontent.com".into(),
            source_repository: "Luminous-Dynamics/mycelix".into(),
            source_repository_owner: "Luminous-Dynamics".into(),
            signer_workflow: ".github/workflows/proofs.yml".into(),
            signer_workflow_revision: WorkflowRevisionPolicyV1::Exact(GitObjectIdV1::sha1([
                0xbb; 20
            ])),
            source_revision: SourceRevisionPolicyV1 {
                exact_commit: GitObjectIdV1::sha1([0xcc; 20]),
                exact_git_ref: Some("refs/heads/main".into()),
            },
            transparency_policy: TransparencyPolicyV1::PublicTransparencyRequired,
            freshness_policy: AuthenticationFreshnessPolicyV1::CurrentAtVerification,
        }
    }

    fn predicate() -> QualificationAttestationPredicateV1 {
        QualificationAttestationPredicateV1 {
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
        }
    }

    #[test]
    fn exact_policy_and_predicate_parse_strictly() {
        let policy_json = serde_json::to_vec(&policy()).unwrap();
        assert_eq!(
            parse_untrusted_authentication_policy_json_v1(&policy_json).unwrap(),
            policy()
        );

        let predicate_json = serde_json::to_vec(&predicate()).unwrap();
        assert_eq!(
            parse_untrusted_qualification_predicate_json_v1(&predicate_json).unwrap(),
            predicate()
        );
    }

    #[test]
    fn oversized_document_fails_before_deserialization() {
        let oversized = vec![b' '; MAX_UNTRUSTED_WIRE_JSON_BYTES_V1 + 1];
        assert_eq!(
            parse_untrusted_authentication_policy_json_v1(&oversized),
            Err(UntrustedWireParseErrorV1::InputTooLarge {
                maximum: MAX_UNTRUSTED_WIRE_JSON_BYTES_V1,
                actual: MAX_UNTRUSTED_WIRE_JSON_BYTES_V1 + 1,
            })
        );
    }

    #[test]
    fn unknown_top_level_and_nested_policy_fields_fail() {
        let mut value = serde_json::to_value(policy()).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("production_authority".into(), serde_json::Value::Bool(true));
        assert_eq!(
            parse_untrusted_authentication_policy_json_v1(&serde_json::to_vec(&value).unwrap()),
            Err(UntrustedWireParseErrorV1::InvalidJson)
        );

        let mut value = serde_json::to_value(policy()).unwrap();
        value["verifier_profile"]
            .as_object_mut()
            .unwrap()
            .insert("trust_me".into(), serde_json::Value::Bool(true));
        assert_eq!(
            parse_untrusted_authentication_policy_json_v1(&serde_json::to_vec(&value).unwrap()),
            Err(UntrustedWireParseErrorV1::InvalidJson)
        );

        let mut value = serde_json::to_value(policy()).unwrap();
        value["source_revision"]
            .as_object_mut()
            .unwrap()
            .insert("branch_is_trusted".into(), serde_json::Value::Bool(true));
        assert_eq!(
            parse_untrusted_authentication_policy_json_v1(&serde_json::to_vec(&value).unwrap()),
            Err(UntrustedWireParseErrorV1::InvalidJson)
        );
    }

    #[test]
    fn duplicate_policy_fields_fail() {
        let json = serde_json::to_string(&policy()).unwrap();
        let duplicate = format!(r#"{{"profile_id":"duplicate",{}"#, &json[1..]);
        assert_eq!(
            parse_untrusted_authentication_policy_json_v1(duplicate.as_bytes()),
            Err(UntrustedWireParseErrorV1::InvalidJson)
        );
    }

    #[test]
    fn unknown_predicate_claim_and_wrong_schema_fail() {
        let mut value = serde_json::to_value(predicate()).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("production_authority".into(), serde_json::Value::Bool(true));
        assert_eq!(
            parse_untrusted_qualification_predicate_json_v1(&serde_json::to_vec(&value).unwrap()),
            Err(UntrustedWireParseErrorV1::InvalidJson)
        );

        let mut wrong = predicate();
        wrong.predicate_schema = "other-schema".into();
        assert!(matches!(
            parse_untrusted_qualification_predicate_json_v1(&serde_json::to_vec(&wrong).unwrap()),
            Err(UntrustedWireParseErrorV1::InvalidPredicate(_))
        ));
    }

    #[test]
    fn overlong_policy_and_excess_revision_list_fail_before_authority() {
        let mut overlong = policy();
        overlong.source_repository = "x".repeat(MAX_UNTRUSTED_WIRE_STRING_BYTES_V1 + 1);
        assert!(matches!(
            parse_untrusted_authentication_policy_json_v1(&serde_json::to_vec(&overlong).unwrap()),
            Err(UntrustedWireParseErrorV1::FieldTooLong {
                field: "source_repository",
                ..
            })
        ));

        let mut too_many = policy();
        too_many.signer_workflow_revision = WorkflowRevisionPolicyV1::Allowed(
            (0..=MAX_UNTRUSTED_WORKFLOW_REVISIONS_V1)
                .map(|index| {
                    let mut bytes = [0_u8; 20];
                    bytes[0] = index as u8;
                    GitObjectIdV1::sha1(bytes)
                })
                .collect(),
        );
        assert!(matches!(
            parse_untrusted_authentication_policy_json_v1(&serde_json::to_vec(&too_many).unwrap()),
            Err(UntrustedWireParseErrorV1::TooManyWorkflowRevisions { .. })
        ));
    }

    #[test]
    fn malformed_typed_identity_fails_during_strict_parse() {
        let mut value = serde_json::to_value(policy()).unwrap();
        value["source_revision"]["exact_commit"] = serde_json::Value::String("sha1:nothex".into());
        assert_eq!(
            parse_untrusted_authentication_policy_json_v1(&serde_json::to_vec(&value).unwrap()),
            Err(UntrustedWireParseErrorV1::InvalidJson)
        );
    }
}
