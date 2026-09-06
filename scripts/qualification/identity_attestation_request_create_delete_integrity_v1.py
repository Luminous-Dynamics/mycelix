#!/usr/bin/env python3
"""Materialize fail-closed AttestationRequest create/delete DHT hardening.

Qualification-only: this edits the CI checkout, never committed product source.
Every replacement requires exactly one expected before-image so stale or
ambiguous security transformations fail closed.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LIB = ROOT / "mycelix-workspace/mycelix-identity/zomes/trust_credential/integrity/src/lib.rs"


def replace_once(text: str, before: str, after: str, label: str) -> str:
    count = text.count(before)
    if count != 1:
        raise SystemExit(
            f"ERROR: {label} before-image count is {count}, expected exactly 1; "
            "refuse stale/ambiguous integrity transformation"
        )
    return text.replace(before, after, 1)


text = LIB.read_text()

old_register_delete = '''        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete their entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
'''
new_register_delete = '''        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete their entries".into(),
                ));
            }

            // Attestation requests have a typed lifecycle. Generic deletion would
            // create a second, untyped cancellation language and bypass the
            // canonical Pending -> Cancelled transition policy.
            let request_entry_type = EntryType::App(AppEntryDef::try_from(
                UnitEntryTypes::AttestationRequest,
            )?);
            if original.action().entry_type() == Some(&request_entry_type) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Attestation requests cannot be deleted; use the typed lifecycle".into(),
                ));
            }

            Ok(ValidateCallbackResult::Valid)
        }
'''
text = replace_once(
    text,
    old_register_delete,
    new_register_delete,
    "typed AttestationRequest delete guard",
)

old_create_request = '''/// Validate attestation request creation
fn validate_create_request(
    action: Create,
    req: AttestationRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the request to its committer -- request_attestation already
    // derives `requester_did` from agent_info() coordinator-side with zero
    // user input, so this never rejects a legitimate request; it's the
    // real DHT-level enforcement a modified coordinator could otherwise
    // bypass (P0 author-binding gap).
    let expected_requester = format!("did:mycelix:{}", action.author);
    if req.requester_did != expected_requester {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation request requester_did must be the committing agent (forgery)".to_string(),
        ));
    }

    // Requester must be a valid DID
    if !req.requester_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester must be a valid DID".into(),
        ));
    }

    // Subject must be a valid DID
    if !req.subject_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Subject must be a valid DID".into(),
        ));
    }

    // Cannot request attestation from yourself
    if req.requester_did == req.subject_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot request attestation from yourself".into(),
        ));
    }

    // New requests must be pending
    if req.status != AttestationStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "New requests must have Pending status".into(),
        ));
    }

    // Min trust score must be valid if specified
    if let Some(score) = req.min_trust_score {
        if !(0.0..=1.0).contains(&score) {
            return Ok(ValidateCallbackResult::Invalid(
                "Minimum trust score must be in [0, 1]".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}
'''
new_create_request = '''/// Pure structural admission theorem for new attestation request roots.
///
/// This deliberately owns only entry shape/actionability. Requester author
/// binding remains the outer HDI authority check in `validate_create_request`.
fn validate_attestation_request_creation_shape(
    req: &AttestationRequest,
) -> Result<(), &'static str> {
    if req.id.is_empty() || req.id.len() > 256 {
        return Err("Attestation request ID must be 1-256 characters");
    }
    if req.requester_did.is_empty()
        || req.requester_did.len() > 256
        || !req.requester_did.starts_with("did:")
    {
        return Err("Requester DID must be a valid DID of 1-256 characters");
    }
    if req.subject_did.is_empty()
        || req.subject_did.len() > 256
        || !req.subject_did.starts_with("did:")
    {
        return Err("Subject DID must be a valid DID of 1-256 characters");
    }
    if req.requester_did == req.subject_did {
        return Err("Cannot request attestation from yourself");
    }
    if req.components.is_empty() {
        return Err("At least one K-Vector component is required");
    }
    if req.purpose.is_empty() || req.purpose.len() > 2048 {
        return Err("Attestation request purpose must be 1-2048 characters");
    }
    if req.status != AttestationStatus::Pending {
        return Err("New requests must have Pending status");
    }
    if let Some(score) = req.min_trust_score {
        if !score.is_finite() {
            return Err("Minimum trust score must be finite");
        }
        if !(0.0..=1.0).contains(&score) {
            return Err("Minimum trust score must be in [0, 1]");
        }
    }
    if req.expires_at <= req.created_at {
        return Err("Attestation request expiration must be after creation time");
    }
    Ok(())
}

/// Validate attestation request creation.
fn validate_create_request(
    action: Create,
    req: AttestationRequest,
) -> ExternResult<ValidateCallbackResult> {
    // DHT-authoritative requester binding: a modified coordinator cannot claim
    // another agent as the immutable request author.
    let expected_requester = format!("did:mycelix:{}", action.author);
    if req.requester_did != expected_requester {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation request requester_did must be the committing agent (forgery)".to_string(),
        ));
    }

    match validate_attestation_request_creation_shape(&req) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason.into())),
    }
}
'''
text = replace_once(
    text,
    old_create_request,
    new_create_request,
    "AttestationRequest creation theorem",
)

old_request_tests = '''    #[test]
    fn create_request_requester_forgery_rejected() {
        let req = valid_request(format!("did:mycelix:{}", me()));
        let result = validate_create_request(test_action(other_agent()), req).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_presentation(subject_did: String) -> TrustPresentation {
'''
new_request_tests = '''    #[test]
    fn create_request_requester_forgery_rejected() {
        let req = valid_request(format!("did:mycelix:{}", me()));
        let result = validate_create_request(test_action(other_agent()), req).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn request_creation_shape_accepts_supported_boundary() {
        let mut req = valid_request(format!("did:mycelix:{}", me()));
        req.id = "r".repeat(256);
        req.purpose = "p".repeat(2048);
        assert_eq!(validate_attestation_request_creation_shape(&req), Ok(()));
    }

    #[test]
    fn request_creation_shape_rejects_unaddressable_id() {
        let mut req = valid_request(format!("did:mycelix:{}", me()));
        req.id = "r".repeat(257);
        assert!(validate_attestation_request_creation_shape(&req).is_err());
    }

    #[test]
    fn request_creation_shape_rejects_empty_components() {
        let mut req = valid_request(format!("did:mycelix:{}", me()));
        req.components.clear();
        assert!(validate_attestation_request_creation_shape(&req).is_err());
    }

    #[test]
    fn request_creation_shape_rejects_bad_purpose_bounds() {
        let mut empty = valid_request(format!("did:mycelix:{}", me()));
        empty.purpose.clear();
        assert!(validate_attestation_request_creation_shape(&empty).is_err());

        let mut long = valid_request(format!("did:mycelix:{}", me()));
        long.purpose = "p".repeat(2049);
        assert!(validate_attestation_request_creation_shape(&long).is_err());
    }

    #[test]
    fn request_creation_shape_rejects_bad_did_bounds() {
        let mut bad_subject = valid_request(format!("did:mycelix:{}", me()));
        bad_subject.subject_did = "did:".to_string() + &"s".repeat(253);
        assert_eq!(bad_subject.subject_did.len(), 257);
        assert!(validate_attestation_request_creation_shape(&bad_subject).is_err());

        let mut empty_requester = valid_request(format!("did:mycelix:{}", me()));
        empty_requester.requester_did.clear();
        assert!(validate_attestation_request_creation_shape(&empty_requester).is_err());
    }

    #[test]
    fn request_creation_shape_rejects_nonfinite_and_out_of_range_score() {
        for score in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -0.01, 1.01] {
            let mut req = valid_request(format!("did:mycelix:{}", me()));
            req.min_trust_score = Some(score);
            assert!(
                validate_attestation_request_creation_shape(&req).is_err(),
                "score {score:?} must be rejected"
            );
        }
    }

    #[test]
    fn request_creation_shape_requires_positive_validity_interval() {
        for expires_at in [Timestamp::from_micros(0), Timestamp::from_micros(-1)] {
            let mut req = valid_request(format!("did:mycelix:{}", me()));
            req.created_at = Timestamp::from_micros(0);
            req.expires_at = expires_at;
            assert!(validate_attestation_request_creation_shape(&req).is_err());
        }
    }

    #[test]
    fn request_creation_shape_requires_pending_root() {
        let mut req = valid_request(format!("did:mycelix:{}", me()));
        req.status = AttestationStatus::Fulfilled;
        assert!(validate_attestation_request_creation_shape(&req).is_err());
    }

    fn valid_presentation(subject_did: String) -> TrustPresentation {
'''
text = replace_once(
    text,
    old_request_tests,
    new_request_tests,
    "AttestationRequest creation regression tests",
)

required = [
    "fn validate_attestation_request_creation_shape(",
    "req.id.is_empty() || req.id.len() > 256",
    "req.components.is_empty()",
    "req.purpose.is_empty() || req.purpose.len() > 2048",
    "!score.is_finite()",
    "req.expires_at <= req.created_at",
    "UnitEntryTypes::AttestationRequest",
    "Attestation requests cannot be deleted; use the typed lifecycle",
    "request_creation_shape_accepts_supported_boundary",
    "request_creation_shape_rejects_unaddressable_id",
    "request_creation_shape_rejects_nonfinite_and_out_of_range_score",
    "request_creation_shape_requires_positive_validity_interval",
]
for needle in required:
    if needle not in text:
        raise SystemExit(f"ERROR: generated request create/delete contract missing: {needle!r}")

create_start = text.index("fn validate_attestation_request_creation_shape(")
create_end = text.index("/// Validate attestation request creation.", create_start)
create_body = text[create_start:create_end]
for forbidden in [
    "HashSet",
    "duplicate component",
    "duplicate_component",
]:
    if forbidden in create_body:
        raise SystemExit(
            f"ERROR: unqualified component-set semantics leaked into this tranche: {forbidden!r}"
        )

delete_start = text.index("FlatOp::RegisterDelete(OpDelete { action })")
delete_end = text.index("    }\n}\n\n/// Validate trust credential creation", delete_start)
delete_body = text[delete_start:delete_end]
if "original.action().entry_type() == Some(&request_entry_type)" not in delete_body:
    raise SystemExit("ERROR: request deletion is not gated by the canonical entry type")
if "Only the original entry author can delete their entries" not in delete_body:
    raise SystemExit("ERROR: ordinary same-author delete rule was weakened")

LIB.write_text(text)
print("Materialized AttestationRequest create/delete integrity V1 against exact before-images.")
