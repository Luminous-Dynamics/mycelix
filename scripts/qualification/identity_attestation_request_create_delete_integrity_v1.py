#!/usr/bin/env python3
"""Materialize fail-closed AttestationRequest DHT admission/deletion hardening.

Qualification-only: edits the CI checkout, never committed product source.
Every replacement requires exactly one expected before-image so stale or
ambiguous security transformations fail closed.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/mycelix-identity/zomes/trust_credential/integrity/Cargo.toml"
LIB = ROOT / "mycelix-workspace/mycelix-identity/zomes/trust_credential/integrity/src/lib.rs"


def replace_once(text: str, before: str, after: str, label: str) -> str:
    count = text.count(before)
    if count != 1:
        raise SystemExit(
            f"ERROR: {label} before-image count is {count}, expected exactly 1; "
            "refuse stale/ambiguous integrity transformation"
        )
    return text.replace(before, after, 1)


manifest = MANIFEST.read_text()
old_deps = '''serde_json = { workspace = true }
thiserror = { workspace = true }
'''
new_deps = '''serde_json = { workspace = true }
thiserror = { workspace = true }
mycelix-attestation-request-root-policy = { path = "../../../crates/attestation-request-root-policy" }
'''
manifest = replace_once(manifest, old_deps, new_deps, "root-policy dependency")
MANIFEST.write_text(manifest)

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
            // create a second, untyped cancellation language.
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
text = replace_once(text, old_register_delete, new_register_delete, "typed request delete guard")

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
new_create_request = '''fn request_status_v1(
    status: &AttestationStatus,
) -> mycelix_attestation_request_root_policy::AttestationRequestStatusV1 {
    use mycelix_attestation_request_root_policy::AttestationRequestStatusV1 as V1;
    match status {
        AttestationStatus::Pending => V1::Pending,
        AttestationStatus::Fulfilled => V1::Fulfilled,
        AttestationStatus::Declined => V1::Declined,
        AttestationStatus::Expired => V1::Expired,
        AttestationStatus::Cancelled => V1::Cancelled,
    }
}

/// Validate attestation request creation.
fn validate_create_request(
    action: Create,
    req: AttestationRequest,
) -> ExternResult<ValidateCallbackResult> {
    // DHT-authoritative requester binding remains local to HDI.
    let expected_requester = format!("did:mycelix:{}", action.author);
    if req.requester_did != expected_requester {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation request requester_did must be the committing agent (forgery)".to_string(),
        ));
    }

    let root = mycelix_attestation_request_root_policy::AttestationRequestRootAssertionV1 {
        id: &req.id,
        requester_did: &req.requester_did,
        subject_did: &req.subject_did,
        component_count: req.components.len(),
        purpose: &req.purpose,
        min_trust_score: req.min_trust_score,
        status: request_status_v1(&req.status),
        created_at_micros: req.created_at.as_micros(),
        expires_at_micros: req.expires_at.as_micros(),
    };

    match mycelix_attestation_request_root_policy::validate_attestation_request_root_assertion_v1(root) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid attestation request root: {reason:?}"
        ))),
    }
}
'''
text = replace_once(text, old_create_request, new_create_request, "root-policy DHT adapter")

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
    fn create_request_delegates_supported_boundary_to_root_policy() {
        let mut req = valid_request(format!("did:mycelix:{}", me()));
        req.id = "r".repeat(256);
        req.purpose = "p".repeat(2048);
        let result = validate_create_request(test_action(me()), req).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn create_request_delegates_invalid_root_shapes_to_root_policy() {
        let cases = [f32::NAN, f32::INFINITY, -0.01, 1.01];
        for score in cases {
            let mut req = valid_request(format!("did:mycelix:{}", me()));
            req.min_trust_score = Some(score);
            let result = validate_create_request(test_action(me()), req).unwrap();
            assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
        }

        let mut req = valid_request(format!("did:mycelix:{}", me()));
        req.id = "r".repeat(257);
        let result = validate_create_request(test_action(me()), req).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));

        let mut req = valid_request(format!("did:mycelix:{}", me()));
        req.expires_at = req.created_at;
        let result = validate_create_request(test_action(me()), req).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_presentation(subject_did: String) -> TrustPresentation {
'''
text = replace_once(text, old_request_tests, new_request_tests, "root-policy adapter regressions")

required = [
    "AttestationRequestRootAssertionV1",
    "validate_attestation_request_root_assertion_v1",
    "request_status_v1(",
    "created_at_micros: req.created_at.as_micros()",
    "expires_at_micros: req.expires_at.as_micros()",
    "UnitEntryTypes::AttestationRequest",
    "Attestation requests cannot be deleted; use the typed lifecycle",
    "create_request_delegates_supported_boundary_to_root_policy",
    "create_request_delegates_invalid_root_shapes_to_root_policy",
]
for needle in required:
    if needle not in text:
        raise SystemExit(f"ERROR: generated DHT adapter missing: {needle!r}")

for forbidden in [
    "fn validate_attestation_request_creation_shape(",
    "req.id.is_empty() || req.id.len() > 256",
    "req.purpose.is_empty() || req.purpose.len() > 2048",
]:
    if forbidden in text:
        raise SystemExit(f"ERROR: private request-root theorem survived DHT adapter: {forbidden!r}")

LIB.write_text(text)
print("Materialized #185-backed AttestationRequest DHT adapter against exact before-images.")
