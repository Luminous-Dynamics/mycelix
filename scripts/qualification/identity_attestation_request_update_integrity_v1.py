#!/usr/bin/env python3
"""Materialize the AttestationRequest DHT adapter to the canonical lifecycle policy.

Qualification-only: edits the CI checkout, never committed product source. Every
replacement requires one exact before-image so drift/ambiguity fails closed.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
INTEGRITY = ROOT / "mycelix-workspace/mycelix-identity/zomes/trust_credential/integrity"
MANIFEST = INTEGRITY / "Cargo.toml"
LIB = INTEGRITY / "src/lib.rs"


def replace_once(text: str, before: str, after: str, label: str) -> str:
    count = text.count(before)
    if count != 1:
        raise SystemExit(
            f"ERROR: {label} before-image count is {count}, expected exactly 1; "
            "refuse stale/ambiguous integrity transformation"
        )
    return text.replace(before, after, 1)


manifest = MANIFEST.read_text()
manifest = replace_once(
    manifest,
    "holochain_serialized_bytes = { workspace = true }\nserde = { workspace = true }\n",
    "holochain_serialized_bytes = { workspace = true }\n"
    "mycelix-attestation-request-policy = { path = \"../../../crates/attestation-request-policy\" }\n"
    "serde = { workspace = true }\n",
    "integrity lifecycle policy dependency",
)

text = LIB.read_text()
text = replace_once(
    text,
    "use hdi::prelude::*;\n",
    "use hdi::prelude::*;\n"
    "use mycelix_attestation_request_policy::{\n"
    "    AttestationRequestActorV1, AttestationRequestAssertionEqualityV1,\n"
    "    AttestationRequestStatusV1, AttestationRequestTransitionV1,\n"
    "    classify_attestation_request_actor_v1, validate_attestation_request_transition_v1,\n"
    "};\n",
    "shared lifecycle policy imports",
)

old_register_update = '''        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can update their entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
'''
new_register_update = '''        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };

            // Preserve the existing same-author rule for every ordinary update.
            let original_action = must_get_action(action.original_action_address.clone())?;
            if *original_action.action().author() == action.author {
                return Ok(ValidateCallbackResult::Valid);
            }

            // AttestationRequest is the only cross-author candidate exception:
            // request creation is requester-authored while legitimate Fulfilled/
            // Declined responses are subject-authored. This op grants only the
            // possibility to update. StoreEntry validation remains the exact
            // actor/status/time authority gate.
            let original_record =
                must_get_valid_record(action.original_action_address.clone())?;
            if let Some(request) = original_record
                .entry()
                .to_app_option::<AttestationRequest>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                let committer_did = format!("did:mycelix:{}", action.author);
                if classify_attestation_request_actor_v1(
                    &committer_did,
                    &request.requester_did,
                    &request.subject_did,
                ) == AttestationRequestActorV1::Subject
                {
                    return Ok(ValidateCallbackResult::Valid);
                }
            }

            Ok(ValidateCallbackResult::Invalid(
                "Update author is not permitted for the original entry".into(),
            ))
        }
'''
text = replace_once(
    text,
    old_register_update,
    new_register_update,
    "RegisterUpdate request subject exception",
)

old_update_request = '''/// Validate attestation request update
fn validate_update_request(
    action: Update,
    req: AttestationRequest,
) -> ExternResult<ValidateCallbackResult> {
    if !req.requester_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester must be a valid DID".into(),
        ));
    }

    // Fetch original to enforce state transitions
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: AttestationRequest = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original attestation request not found".into()
        )))?;

    // Bind the update to the original subject -- fulfill_attestation/
    // decline_attestation (the only paths that update this entry) already
    // check the caller is the subject coordinator-side, but that trusts the
    // coordinator; this is the real DHT-level enforcement a modified
    // coordinator can't bypass (P0 author-binding gap).
    let committer_did = format!("did:mycelix:{}", action.author);
    if committer_did != original.subject_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation request update must be committed by its subject".to_string(),
        ));
    }

    // Immutable fields
    if req.id != original.id {
        return Ok(ValidateCallbackResult::Invalid(
            "Attestation request ID cannot be changed".into(),
        ));
    }
    if req.requester_did != original.requester_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester DID cannot be changed".into(),
        ));
    }
    if req.subject_did != original.subject_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Subject DID cannot be changed".into(),
        ));
    }

    // State machine: Pending → Fulfilled/Declined/Expired/Cancelled
    // Terminal states (Fulfilled, Declined, Expired, Cancelled) cannot transition further
    let valid = match (&original.status, &req.status) {
        (AttestationStatus::Pending, AttestationStatus::Fulfilled)
        | (AttestationStatus::Pending, AttestationStatus::Declined)
        | (AttestationStatus::Pending, AttestationStatus::Expired)
        | (AttestationStatus::Pending, AttestationStatus::Cancelled) => true,
        (a, b) if a == b => true,
        _ => false,
    };

    if !valid {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid attestation request status transition".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}
'''
new_update_request = '''fn attestation_request_status_v1(status: &AttestationStatus) -> AttestationRequestStatusV1 {
    match status {
        AttestationStatus::Pending => AttestationRequestStatusV1::Pending,
        AttestationStatus::Fulfilled => AttestationRequestStatusV1::Fulfilled,
        AttestationStatus::Declined => AttestationRequestStatusV1::Declined,
        AttestationStatus::Expired => AttestationRequestStatusV1::Expired,
        AttestationStatus::Cancelled => AttestationRequestStatusV1::Cancelled,
    }
}

fn attestation_request_assertion_equality_v1(
    original: &AttestationRequest,
    updated: &AttestationRequest,
) -> AttestationRequestAssertionEqualityV1 {
    AttestationRequestAssertionEqualityV1 {
        id: updated.id == original.id,
        requester_did: updated.requester_did == original.requester_did,
        subject_did: updated.subject_did == original.subject_did,
        components: updated.components == original.components,
        min_trust_score: updated.min_trust_score == original.min_trust_score,
        min_tier: updated.min_tier == original.min_tier,
        purpose: updated.purpose == original.purpose,
        expires_at: updated.expires_at == original.expires_at,
        created_at: updated.created_at == original.created_at,
    }
}

/// Validate AttestationRequest updates by adapting HDI-local data into the
/// dependency-free lifecycle theorem shared with future read-side resolution.
fn validate_update_request(
    action: Update,
    req: AttestationRequest,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: AttestationRequest = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original attestation request not found".into()
        )))?;

    let committer_did = format!("did:mycelix:{}", action.author);
    let actor = classify_attestation_request_actor_v1(
        &committer_did,
        &original.requester_did,
        &original.subject_did,
    );

    let transition = AttestationRequestTransitionV1 {
        original_status: attestation_request_status_v1(&original.status),
        updated_status: attestation_request_status_v1(&req.status),
        actor,
        assertion_equality: attestation_request_assertion_equality_v1(&original, &req),
        update_timestamp_micros: action.timestamp.as_micros(),
        expires_at_micros: original.expires_at.as_micros(),
    };

    match validate_attestation_request_transition_v1(transition) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid attestation request transition: {error:?}"
        ))),
    }
}
'''
text = replace_once(
    text,
    old_update_request,
    new_update_request,
    "AttestationRequest lifecycle policy adapter",
)

marker = '''    proptest! {
'''
adapter_tests = '''    fn base_attestation_request_for_policy_adapter() -> AttestationRequest {
        AttestationRequest {
            id: "req-policy-1".to_string(),
            requester_did: "did:mycelix:requester".to_string(),
            subject_did: "did:mycelix:subject".to_string(),
            components: vec![KVectorComponent::Reputation],
            min_trust_score: Some(0.5),
            min_tier: Some(TrustTier::Standard),
            purpose: "governance eligibility".to_string(),
            expires_at: Timestamp::from_micros(1_000),
            status: AttestationStatus::Pending,
            created_at: Timestamp::from_micros(100),
        }
    }

    #[test]
    fn attestation_status_mapping_covers_every_variant() {
        let cases = [
            (AttestationStatus::Pending, AttestationRequestStatusV1::Pending),
            (AttestationStatus::Fulfilled, AttestationRequestStatusV1::Fulfilled),
            (AttestationStatus::Declined, AttestationRequestStatusV1::Declined),
            (AttestationStatus::Expired, AttestationRequestStatusV1::Expired),
            (AttestationStatus::Cancelled, AttestationRequestStatusV1::Cancelled),
        ];
        for (local, canonical) in cases {
            assert_eq!(attestation_request_status_v1(&local), canonical);
        }
    }

    #[test]
    fn request_assertion_equality_adapter_covers_every_frozen_field() {
        let original = base_attestation_request_for_policy_adapter();
        let mut variants = Vec::new();

        let mut v = original.clone();
        v.id = "changed".into();
        variants.push(v);
        let mut v = original.clone();
        v.requester_did = "did:mycelix:changed-requester".into();
        variants.push(v);
        let mut v = original.clone();
        v.subject_did = "did:mycelix:changed-subject".into();
        variants.push(v);
        let mut v = original.clone();
        v.components = vec![KVectorComponent::Activity];
        variants.push(v);
        let mut v = original.clone();
        v.min_trust_score = Some(0.6);
        variants.push(v);
        let mut v = original.clone();
        v.min_tier = Some(TrustTier::Guardian);
        variants.push(v);
        let mut v = original.clone();
        v.purpose = "changed purpose".into();
        variants.push(v);
        let mut v = original.clone();
        v.expires_at = Timestamp::from_micros(2_000);
        variants.push(v);
        let mut v = original.clone();
        v.created_at = Timestamp::from_micros(200);
        variants.push(v);

        for variant in variants {
            let equality = attestation_request_assertion_equality_v1(&original, &variant);
            let flags = [
                equality.id,
                equality.requester_did,
                equality.subject_did,
                equality.components,
                equality.min_trust_score,
                equality.min_tier,
                equality.purpose,
                equality.expires_at,
                equality.created_at,
            ];
            assert_eq!(flags.into_iter().filter(|equal| !*equal).count(), 1);
        }
    }

'''
text = replace_once(text, marker, adapter_tests + marker, "policy adapter regression tests")

for required in [
    "mycelix_attestation_request_policy::{",
    "classify_attestation_request_actor_v1(",
    "AttestationRequestAssertionEqualityV1 {",
    "AttestationRequestTransitionV1 {",
    "validate_attestation_request_transition_v1(transition)",
    "update_timestamp_micros: action.timestamp.as_micros()",
    "expires_at_micros: original.expires_at.as_micros()",
    "attestation_status_mapping_covers_every_variant",
    "request_assertion_equality_adapter_covers_every_frozen_field",
]:
    if required not in text:
        raise SystemExit(f"ERROR: lifecycle adapter after-image missing: {required}")

register_start = text.index("        FlatOp::RegisterUpdate(update) => {")
register_end = text.index("        FlatOp::RegisterDelete", register_start)
register_body = text[register_start:register_end]
for required in [
    "*original_action.action().author() == action.author",
    "classify_attestation_request_actor_v1(",
    "== AttestationRequestActorV1::Subject",
]:
    if required not in register_body:
        raise SystemExit(f"ERROR: RegisterUpdate authority adapter missing: {required}")

update_start = text.index("fn validate_update_request(")
update_end = text.index("#[cfg(test)]", update_start)
update_body = text[update_start:update_end]
for forbidden in [
    "enum AttestationRequestUpdateActor",
    "fn validate_attestation_request_transition(",
    "let valid = match",
    "(a, b) if a == b",
    "AttestationStatus::Pending, AttestationStatus::Fulfilled",
]:
    if forbidden in update_body:
        raise SystemExit(f"ERROR: private request lifecycle rule remains: {forbidden}")

MANIFEST.write_text(manifest)
LIB.write_text(text)
print("Materialized AttestationRequest DHT adapter to canonical lifecycle policy V1.")
