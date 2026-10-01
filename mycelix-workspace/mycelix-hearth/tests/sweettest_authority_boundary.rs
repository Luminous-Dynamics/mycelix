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
//!   ZomeCallResponse::Unauthorized; the zome is not dispatched.
//!
//! This is runtime evidence of the Holochain 0.7 app-interface boundary, not a claim
//! that every possible authorization dimension has been qualified.

use holochain::conductor::api::{AppRequest, AppResponse, ZomeCallParamsSigned};
use holochain::prelude::*;
use holochain::sweettest::{SweetConductor, SweetDnaFile};
use holochain::test_utils::new_zome_call_params;

use std::path::PathBuf;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum HearthType {
    Nuclear,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CreateHearthInput {
    name: String,
    description: String,
    hearth_type: HearthType,
    max_members: Option<u32>,
}

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

    let input = CreateHearthInput {
        name: "Authority Boundary Fixture".into(),
        description: "Holochain 0.7 authenticated app-interface qualification".into(),
        hearth_type: HearthType::Nuclear,
        max_members: Some(4),
    };

    // The cell author is implicitly authorized when no capability secret is supplied.
    let signed = signed_call(
        &conductor,
        alice.cell_id(),
        "create_hearth",
        input,
        None,
    )
    .await;

    let response = submit_call(&conductor, signed).await;

    match response {
        AppResponse::ZomeCalled(result) => match *result {
            ZomeCallResponse::Ok(output) => {
                let _: Record = output.decode().expect("authorized zome result must decode");
            }
            other => panic!("authorized call reached conductor but returned {other:?}"),
        },
        other => panic!("expected ZomeCalled response, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_capability_is_rejected_before_zome_dispatch() {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    let input = CreateHearthInput {
        name: "Should Never Execute".into(),
        description: "Invalid capability boundary case".into(),
        hearth_type: HearthType::Nuclear,
        max_members: Some(4),
    };

    let invalid_secret = CapSecret::from([0xA5; CAP_SECRET_BYTES]);
    let signed = signed_call(
        &conductor,
        alice.cell_id(),
        "create_hearth",
        input,
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
