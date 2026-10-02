// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Real Holochain 0.7 app-interface authority-boundary qualification.
//!
//! This test deliberately avoids SweetConductor::call() for the exercised calls.
//! It opens an authenticated app websocket and submits AppRequest::CallZome with
//! conductor-signed ZomeCallParamsSigned values, so the request crosses the
//! actual authentication/authorization boundary before reaching the zome.
//!
//! Evidence semantics:
//! - authorized_reached_zome: a valid signed author call returns ZomeCallResponse::Ok.
//! - unauthorized_pre_zome: a signed call carrying an invalid capability secret returns
//!   ZomeCallResponse::Unauthorized; Holochain rejects it before zome dispatch.
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
use holochain::test_utils::new_zome_call_params;

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
