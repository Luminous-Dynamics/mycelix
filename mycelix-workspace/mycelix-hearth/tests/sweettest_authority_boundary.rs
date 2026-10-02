// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Real Holochain 0.7 app-interface authority-boundary qualification.
//!
//! This test deliberately avoids SweetConductor::call() for the exercised calls.
//! It opens an authenticated app websocket and submits AppRequest::CallZome with
//! conductor-signed ZomeCallParamsSigned values, so the request crosses the
//! actual authenticated app-interface authorization boundary.
//!
//! Evidence semantics:
//! - authorized_reached_zome: a valid signed author call returns ZomeCallResponse::Ok.
//! - unauthorized_at_authorization_boundary: a signed call carrying an invalid capability
//!   secret returns ZomeCallResponse::Unauthorized. This records authorization failure at
//!   the app-interface boundary without inferring function-body execution from ZomeCalled.
//! - signature_binding: mutating signed parameter bytes without re-signing returns
//!   ZomeCallResponse::Unauthorized.
//! - authorized_deserialization_rejection: a valid signed call with a payload that cannot
//!   deserialize to the target function input returns a zome-call error before function-body
//!   execution.
//! - authorized_semantic_rejection: a valid signed author call returns the guest error
//!   emitted by update_member_role, which intentionally rejects all calls.
//!
//! This is runtime evidence of selected Holochain 0.7 app-interface authorization
//! dimensions, not a claim that every authorization dimension has been qualified.
//!
//! Important response-semantics rule: AppResponse::ZomeCalled means the zome-call response
//! was successfully formed/deserialized at the app interface. It is not itself proof that
//! the zome function body ran; authorization failures can use the same outer response.

use holochain::conductor::api::{AppRequest, AppResponse, ZomeCallParamsSigned};
use holochain::prelude::*;
use holochain::sweettest::{SweetConductor, SweetDnaFile};
use holochain::test_utils::{new_zome_call_params, retry_fn_until_timeout};

use std::path::PathBuf;

fn hearth_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop();
    path.push("dna");
    path.push("mycelix_hearth.dna");
    path
}

async fn signed_call<P: serde::Serialize + std::fmt::Debug>(
    conductor: &SweetConductor,
    cell_id: &CellId,
    function: &str,
    payload: P,
    cap_secret: Option<CapSecret>,
) -> ZomeCallParamsSigned {
    let mut params =
        new_zome_call_params(cell_id, function, payload, "hearth_kinship").unwrap();
    params.cap_secret = cap_secret;

    ZomeCallParamsSigned::try_from_params(&conductor.keystore(), params)
        .await
        .unwrap()
}

async fn signed_call_as_agent<P: serde::Serialize + std::fmt::Debug>(
    conductor: &SweetConductor,
    cell_id: &CellId,
    provenance: AgentPubKey,
    function: &str,
    payload: P,
    cap_secret: CapSecret,
) -> ZomeCallParamsSigned {
    let mut params =
        new_zome_call_params(cell_id, function, payload, "hearth_kinship").unwrap();
    params.provenance = provenance;
    params.cap_secret = Some(cap_secret);

    ZomeCallParamsSigned::try_from_params(&conductor.keystore(), params)
        .await
        .unwrap()
}

async fn signed_call_with_nonce<P: serde::Serialize + std::fmt::Debug>(
    conductor: &SweetConductor,
    cell_id: &CellId,
    function: &str,
    payload: P,
    nonce_bytes: [u8; 32],
) -> ZomeCallParamsSigned {
    let mut params =
        new_zome_call_params(cell_id, function, payload, "hearth_kinship").unwrap();

    // Use explicit ordered nonces so the fixture proves stale/lower-nonce rejection,
    // rather than relying on the incidental ordering of freshly generated nonces.
    params.nonce = Nonce256Bits::try_from(nonce_bytes.as_slice())
        .expect("32-byte nonce must construct a Nonce256Bits value");
    params.cap_secret = None;

    ZomeCallParamsSigned::try_from_params(&conductor.keystore(), params)
        .await
        .unwrap()
}

async fn signed_expired_call(
    conductor: &SweetConductor,
    cell_id: &CellId,
    function: &str,
) -> ZomeCallParamsSigned {
    let mut params =
        new_zome_call_params(cell_id, function, (), "hearth_kinship").unwrap();

    // Keep the normal generated nonce/provenance/signature, but make the signed
    // invocation's expiry unambiguously earlier than the conductor's current time.
    params.expires_at = Timestamp::now()
        .saturating_sub(&std::time::Duration::from_secs(60));
    params.cap_secret = None;

    ZomeCallParamsSigned::try_from_params(&conductor.keystore(), params)
        .await
        .unwrap()
}

async fn submit_call(
    conductor: &SweetConductor,
    signed: ZomeCallParamsSigned,
) -> AppResponse {
    let (app_tx, _app_rx) = conductor.app_ws_client::<AppResponse>("test-app".into()).await;
    app_tx
        .request(AppRequest::CallZome(Box::new(signed)))
        .await
        .unwrap()
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_authenticated_app_call_reaches_zome() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    // Use a read-only coordinator query with no application-level civic gate.
    // This isolates the Holochain app-interface authentication/authorization boundary
    // from Hearth's higher-level eligibility policy.
    let signed = signed_call(
        &conductor,
        alice.cell_id(),
        "get_my_hearths",
        (),
        None,
    )
    .await;

    let response = submit_call(&conductor, signed).await;

    match response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Ok(output) => {
                let _: Vec<Record> = output.decode().expect("authorized zome result must decode");
            }
            other => panic!("authorized call reached conductor but returned {other:?}"),
        },
        other => panic!("expected ZomeCalled response, got {other:?}"),
    }
}


#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_signed_payload_mutation_is_rejected_by_signature_binding() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    // Start with a valid author-signed invocation, then mutate only the serialized
    // ZomeCallParams bytes after signing. The signature is intentionally left intact.
    // This isolates signature binding: the conductor must reject the tampered request
    // rather than dispatching the modified payload to the zome.
    let signed = signed_call(
        &conductor,
        alice.cell_id(),
        "get_my_hearths",
        (),
        None,
    )
    .await;

    let mut tampered = signed.clone();
    let mut params: ZomeCallParams = tampered
        .bytes
        .decode()
        .expect("valid signed call must decode before tampering");
    params.payload = ExternIO::encode(serde_json::json!({
        "tampered": true
    }))
    .expect("tampered payload must serialize");
    tampered.bytes = ExternIO::encode(params).expect("tampered call must serialize");

    let response = submit_call(&conductor, tampered).await;

    match response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Unauthorized(..) => {}
            other => panic!(
                "signature-bound payload mutation should be unauthorized, got {other:?}"
            ),
        },
        other => panic!("expected ZomeCalled response for signature rejection, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_authorized_malformed_payload_is_rejected_before_function_body() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    // get_my_hearths takes (). A JSON object is intentionally not deserializable
    // to that input type. The call is still signed by the cell author, so capability
    // authorization is not the rejection under test.
    let signed = signed_call(
        &conductor,
        alice.cell_id(),
        "get_my_hearths",
        serde_json::json!({"unexpected": true}),
        None,
    )
    .await;

    let response = submit_call(&conductor, signed).await;

    match response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Err(error) => {
                let message = format!("{error:?}");
                assert!(
                    !message.contains("Role changes are disabled"),
                    "malformed payload must not reach the semantic role-mutation path: {message}"
                );
            }
            other => panic!("malformed payload should return a zome-call error, got {other:?}"),
        },
        other => panic!("expected ZomeCalled response, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_capability_is_rejected_at_authorization_boundary() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    // Same application call as the authorized case, but with an invalid capability.
    // Unauthorized is the documented authorization-failure response; do not infer
    // function-body execution merely because the outer response is ZomeCalled.
    let invalid_secret = CapSecret::from([0xA5; CAP_SECRET_BYTES]);
    let signed = signed_call(
        &conductor,
        alice.cell_id(),
        "get_my_hearths",
        (),
        Some(invalid_secret),
    )
    .await;

    let response = submit_call(&conductor, signed).await;

    match response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Unauthorized(..) => {}
            other => panic!("invalid capability should be unauthorized, got {other:?}"),
        },
        other => panic!("expected ZomeCalled response, got {other:?}"),
    }
}


#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_expired_signed_call_is_rejected_at_authorization_boundary() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    let signed = signed_expired_call(&conductor, alice.cell_id(), "get_my_hearths").await;
    let response = submit_call(&conductor, signed).await;

    match response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Unauthorized(..) => {}
            other => panic!("expired invocation should be rejected at authorization boundary, got {other:?}"),
        },
        other => panic!("expected ZomeCalled response for expiry rejection, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_replayed_signed_call_is_rejected_by_nonce_boundary() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    // Construct one signed invocation and submit the exact same signed bytes twice.
    // Holochain 0.7 requires zome-call nonces to be unique; the second submission
    // therefore exercises replay protection without guessing or manufacturing a
    // Nonce256Bits value.
    let signed = signed_call(
        &conductor,
        alice.cell_id(),
        "get_my_hearths",
        (),
        None,
    )
    .await;

    let first_response = submit_call(&conductor, signed.clone()).await;
    match first_response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Ok(output) => {
                let _: Vec<Record> = output
                    .decode()
                    .expect("first authorized replay-fixture call must decode");
            }
            other => panic!("first submission must be authorized, got {other:?}"),
        },
        other => panic!("expected ZomeCalled response for first submission, got {other:?}"),
    }

    let second_response = submit_call(&conductor, signed).await;
    match second_response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Unauthorized(..) => {}
            other => panic!("replayed signed call should be rejected by nonce authorization, got {other:?}"),
        },
        other => panic!("expected ZomeCalled response for replay rejection, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_stale_lower_nonce_is_rejected_by_nonce_boundary() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    // Submit a deliberately higher nonce first, then a lower nonce from the same
    // provenance. This isolates the "older call after a higher nonce was seen"
    // rule without mutating a signed request after signing.
    let high = signed_call_with_nonce(
        &conductor,
        alice.cell_id(),
        "get_my_hearths",
        (),
        [0x01; 32],
    )
    .await;

    let low = signed_call_with_nonce(
        &conductor,
        alice.cell_id(),
        "get_my_hearths",
        (),
        [0x00; 32],
    )
    .await;

    let high_response = submit_call(&conductor, high).await;
    match high_response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Ok(output) => {
                let _: Vec<Record> = output
                    .decode()
                    .expect("higher-nonce call must decode");
            }
            other => panic!("higher-nonce call must be authorized first, got {other:?}"),
        },
        other => panic!("expected ZomeCalled response for higher nonce, got {other:?}"),
    }

    let low_response = submit_call(&conductor, low).await;
    match low_response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Unauthorized(..) => {}
            other => panic!(
                "lower nonce after a higher nonce was witnessed should be unauthorized, got {other:?}"
            ),
        },
        other => panic!("expected ZomeCalled response for stale nonce rejection, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_authorized_unknown_function_is_rejected_at_dispatch_boundary() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    // The caller is a valid cell author and the zome name is valid. Only the
    // function identity is invalid, isolating dispatch lookup from signature,
    // capability, nonce, and application-level semantic validation.
    let signed = signed_call(
        &conductor,
        alice.cell_id(),
        "function_that_does_not_exist",
        (),
        None,
    )
    .await;

    let response = submit_call(&conductor, signed).await;

    match response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Err(error) => {
                let message = format!("{error:?}");
                assert!(
                    !message.contains("Role changes are disabled"),
                    "unknown function must not reach Hearth semantic validation: {message}"
                );
            }
            other => panic!("unknown function should be rejected at dispatch, got {other:?}"),
        },
        other => panic!("expected ZomeCalled response for dispatch rejection, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_authorized_call_reaches_zome_and_is_semantically_rejected() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    // This function deliberately rejects every invocation inside the zome. The
    // valid author signature and absent capability secret prove the request crosses
    // Holochain authorization before the rejection is produced by Hearth itself.
    let payload = serde_json::json!({
        "membership_hash": ActionHash::from_raw_36(vec![0u8; 36]),
        "new_role": "Elder"
    });
    let signed = signed_call(
        &conductor,
        alice.cell_id(),
        "update_member_role",
        payload,
        None,
    )
    .await;

    let response = submit_call(&conductor, signed).await;

    match response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Ok(_) => {
                panic!("disabled role mutation unexpectedly succeeded")
            }
            ZomeCallResponse::Err(error) => {
                let message = format!("{error:?}");
                assert!(
                    message.contains(
                        "Role changes are disabled until governance-authorized membership revisions are implemented"
                    ),
                    "expected the Hearth guest rejection, got {message}"
                );
            }
            other => panic!("expected semantic zome rejection, got {other:?}"),
        },
        other => panic!("expected ZomeCalled response, got {other:?}"),
    }
}


#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_assigned_capability_binds_signer_and_revocation() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    // Generate two non-author signing identities. Neither is the cell author,
    // so neither receives the implicit author grant.
    let bob = conductor
        .keystore()
        .new_sign_keypair_random()
        .await
        .unwrap();
    let eve = conductor
        .keystore()
        .new_sign_keypair_random()
        .await
        .unwrap();

    let cap_secret = CapSecret::from([0xC3; CAP_SECRET_BYTES]);
    let mut functions = std::collections::HashSet::new();
    functions.insert(("hearth_kinship".into(), "get_my_hearths".into()));

    let mut assignees = std::collections::BTreeSet::new();
    assignees.insert(bob.clone());

    // Exercise the public 0.7 Admin API for both grant creation and revocation.
    // This keeps AUTH-10 at the same conductor/admin boundary rather than mixing
    // a direct test helper for creation with an AdminRequest only for revocation.
    let (admin_tx, _admin_rx) = conductor.admin_ws_client::<AdminResponse>().await;
    let grant_response = admin_tx
        .request(AdminRequest::GrantZomeCallCapability(Box::new(
            GrantZomeCallCapabilityPayload {
                cell_id: alice.cell_id().clone(),
                cap_grant: GrantZomeCallCapabilityGrant {
                    tag: "HEARTH-AUTH-ASSIGNED-1".into(),
                    constraint: GrantConstraint::Assigned {
                        secret: cap_secret,
                        assignees,
                    },
                    grant: ZomeCallGrant {
                        functions: GrantedFunctions::Listed(functions),
                    },
                },
            },
        )))
        .await
        .unwrap();

    let grant_action_hash = match grant_response {
        AdminResponse::ZomeCallCapabilityGranted(action_hash) => action_hash,
        other => panic!("assigned capability grant failed: {other:?}"),
    };

    retry_fn_until_timeout(
        || async {
            conductor
                .all_ops_integrated(alice.cell_id().dna_hash())
                .await
                .unwrap()
        },
        None,
        None,
    )
    .await
    .unwrap();

    // Correct signer + correct secret: the assigned capability authorizes the call.
    let bob_call = signed_call_as_agent(
        &conductor,
        alice.cell_id(),
        bob.clone(),
        "get_my_hearths",
        (),
        cap_secret,
    )
    .await;
    match submit_call(&conductor, bob_call).await {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Ok(output) => {
                let _: Vec<Record> = output.decode().expect("assigned call must decode");
            }
            other => panic!("assigned signer with correct secret must succeed, got {other:?}"),
        },
        other => panic!("expected ZomeCalled for assigned positive case, got {other:?}"),
    }

    // Same secret, wrong signer: the secret alone must not confer access.
    let eve_call = signed_call_as_agent(
        &conductor,
        alice.cell_id(),
        eve,
        "get_my_hearths",
        (),
        cap_secret,
    )
    .await;
    match submit_call(&conductor, eve_call).await {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Unauthorized(..) => {}
            other => panic!("unassigned signer must be unauthorized, got {other:?}"),
        },
        other => panic!("expected ZomeCalled for assigned negative case, got {other:?}"),
    }

    // Revoke the exact grant, then prove the previously authorized signer loses access.
    let revoke_response = admin_tx
        .request(AdminRequest::RevokeZomeCallCapability {
            action_hash: grant_action_hash,
            cell_id: alice.cell_id().clone(),
        })
        .await
        .unwrap();
    assert!(
        matches!(revoke_response, AdminResponse::ZomeCallCapabilityRevoked),
        "capability revocation failed: {revoke_response:?}"
    );

    retry_fn_until_timeout(
        || async {
            conductor
                .all_ops_integrated(alice.cell_id().dna_hash())
                .await
                .unwrap()
        },
        None,
        None,
    )
    .await
    .unwrap();

    let bob_after_revoke = signed_call_as_agent(
        &conductor,
        alice.cell_id(),
        bob,
        "get_my_hearths",
        (),
        cap_secret,
    )
    .await;
    match submit_call(&conductor, bob_after_revoke).await {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Unauthorized(..) => {}
            other => panic!("revoked assigned capability must be unauthorized, got {other:?}"),
        },
        other => panic!("expected ZomeCalled after assigned capability revocation, got {other:?}"),
    }
}

#[test]
fn test_authority_case_manifest_is_structurally_valid() {
    let manifest: serde_json::Value = serde_json::from_str(include_str!(
        "hearth-07-authority-boundary-cases.json"
    ))
    .expect("authority case manifest must be valid JSON");

    assert_eq!(
        manifest["schema_version"],
        "HEARTH-AUTH-0.7-CASESET-1"
    );

    let cases = manifest["cases"]
        .as_array()
        .expect("authority case manifest must contain a cases array");

    assert_eq!(cases.len(), 10, "manifest must enumerate all current authority cases");

    let mut ids = cases
        .iter()
        .map(|case| {
            case["case_id"]
                .as_str()
                .expect("every authority case needs a case_id")
                .to_owned()
        })
        .collect::<Vec<_>>();
    ids.sort();

    assert_eq!(
        ids,
        vec![
            "AUTH-01",
            "AUTH-02",
            "AUTH-03",
            "AUTH-04",
            "AUTH-05",
            "AUTH-06",
            "AUTH-07",
            "AUTH-08",
            "AUTH-09",
            "AUTH-10",
        ]
    );

    for case in cases {
        assert!(case["test"].is_string(), "every case needs a test name");
        assert!(case["boundary"].is_string(), "every case needs a boundary");
        assert!(case["expected"].is_string(), "every case needs an expected result");
        assert!(
            case["semantic_execution"].is_boolean()
                || case["semantic_execution"].as_str().is_some(),
            "semantic_execution must be boolean or a documented phase marker"
        );
    }
}
