use hdk::prelude::*;
use holochain::conductor::api::{
    error::ExternalApiWireError, AppInterfaceApi, AppRequest, AppResponse, ZomeCallParamsSigned,
};
use holochain::sweettest::{SweetAgents, SweetConductor, SweetDnaFile, SweetInlineZomes};
use holochain_keystore::MetaLairClient;
use holochain_nonce::Nonce256Bits;
use holochain_serialized_bytes::prelude::SerializedBytes;
use holochain_types::prelude::{CellId, ExternIO, ZomeCallParams};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc,
};

const CANON1_CORPUS_PATH: &str = "../docs/integral/d6s-canon-1-golden-vectors.json";
const CANON1_PROBE_CASE: &str = "reordered-properties";

#[derive(Debug, Clone, Serialize, Deserialize, SerializedBytes)]
struct ProbeInput {
    canonical_bytes: Vec<u8>,
    expected_commitment: String,
    semantic_valid: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, SerializedBytes, PartialEq, Eq)]
enum ProbeResult {
    Accepted,
    SemanticRejected,
    D6sCommitmentMismatch,
}

#[derive(Debug, Clone, Serialize, Deserialize, SerializedBytes)]
struct GrantMaterial {
    action_hash: ActionHash,
    secret: CapSecret,
}

fn commitment(domain: &[u8], bytes: &[u8]) -> String {
    let mut preimage = Vec::with_capacity(domain.len() + bytes.len());
    preimage.extend_from_slice(domain);
    preimage.extend_from_slice(bytes);
    Sha256::digest(preimage)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

#[derive(Debug, Deserialize)]
struct Canon1Vector {
    name: String,
    canonical: String,
    commitment: String,
}

#[derive(Debug, Deserialize)]
struct Canon1Corpus {
    hash_domain: String,
    cases: Vec<Canon1Vector>,
}

fn canon1_corpus_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(CANON1_CORPUS_PATH)
}

fn load_probe_vector() -> (Vec<u8>, String, Vec<u8>) {
    let corpus_bytes = std::fs::read(canon1_corpus_path()).expect("D6S-CANON-1 corpus must exist");
    let corpus: Canon1Corpus =
        serde_json::from_slice(&corpus_bytes).expect("D6S-CANON-1 corpus must parse");
    let vector = corpus
        .cases
        .iter()
        .find(|case| case.name == CANON1_PROBE_CASE)
        .expect("D6S-CANON-1 probe vector must exist");
    let commitment = commitment(corpus.hash_domain.as_bytes(), vector.canonical.as_bytes());
    assert_eq!(
        commitment, vector.commitment,
        "D6S-CANON-1 probe vector commitment must match the corpus"
    );
    (
        vector.canonical.as_bytes().to_vec(),
        vector.commitment.clone(),
        corpus.hash_domain.into_bytes(),
    )
}

fn assert_frozen_d6s_identity() {
    let corpus = std::fs::read(canon1_corpus_path()).expect("D6S-CANON-1 corpus must exist");
    let actual = Sha256::digest(&corpus)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<String>();
    let manifest = std::fs::read_to_string(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../docs/integral/d6s-canon-1-manifest.json"),
    )
    .expect("D6S-CANON-1 manifest must exist");
    let manifest: serde_json::Value =
        serde_json::from_str(&manifest).expect("D6S-CANON-1 manifest must parse");
    assert_eq!(
        actual,
        manifest["corpus_sha256"]
            .as_str()
            .expect("manifest corpus_sha256 must be a string")
    );
    let (canonical, expected_commitment, domain) = load_probe_vector();
    assert_eq!(commitment(&domain, &canonical), expected_commitment);
}

fn authority_probe_zome(reached: Arc<AtomicUsize>, d6s_domain: Vec<u8>) -> SweetInlineZomes {
    SweetInlineZomes::new(Vec::new(), 0)
        .function("probe", move |_api, input: ProbeInput| {
            reached.fetch_add(1, Ordering::SeqCst);
            let actual = commitment(&d6s_domain, &input.canonical_bytes);
            if actual != input.expected_commitment {
                return Ok(ProbeResult::D6sCommitmentMismatch);
            }
            if input.semantic_valid {
                Ok(ProbeResult::Accepted)
            } else {
                Ok(ProbeResult::SemanticRejected)
            }
        })
        .function("grant_probe", move |api, agent: AgentPubKey| {
            let secret = CapSecret::from([0xA5; CAP_SECRET_BYTES]);
            let mut functions = HashSet::new();
            functions.insert((SweetInlineZomes::COORDINATOR.into(), "probe".into()));
            let action_hash = api.create(CreateInput::new(
                EntryDefLocation::CapGrant,
                EntryVisibility::Private,
                Entry::CapGrant(CapGrantEntry {
                    tag: "d6u-probe".into(),
                    access: (secret, agent).into(),
                    functions: GrantedFunctions::Listed(functions),
                }),
                ChainTopOrdering::default(),
            ))?;
            Ok(GrantMaterial {
                action_hash,
                secret,
            })
        })
        .function("revoke_probe", move |api, action_hash: ActionHash| {
            Ok(api.delete(DeleteInput::new(action_hash, ChainTopOrdering::default()))?)
        })
}

#[allow(clippy::too_many_arguments)]
fn params<P: Serialize + std::fmt::Debug>(
    cell_id: &CellId,
    provenance: &AgentPubKey,
    zome_name: ZomeName,
    fn_name: FunctionName,
    cap_secret: Option<CapSecret>,
    input: P,
    nonce: Nonce256Bits,
    expires_at: Timestamp,
) -> ZomeCallParams {
    ZomeCallParams {
        cell_id: cell_id.clone(),
        zome_name,
        cap_secret,
        fn_name,
        payload: ExternIO::encode(input).expect("probe input must encode"),
        provenance: provenance.clone(),
        nonce,
        expires_at,
    }
}

async fn call(
    api: &AppInterfaceApi,
    installed_app_id: &str,
    keystore: &MetaLairClient,
    params: ZomeCallParams,
) -> AppResponse {
    let signed = ZomeCallParamsSigned::try_from_params(keystore, params)
        .await
        .expect("test keystore must sign invocation");
    api.handle_request(
        installed_app_id.to_string(),
        Ok(AppRequest::CallZome(Box::new(signed))),
    )
    .await
    .expect("app interface request must complete")
}

fn decode_zome<T: DeserializeOwned + std::fmt::Debug>(response: AppResponse) -> T {
    match response {
        AppResponse::ZomeCalled(io) => io.decode().expect("zome response payload must decode"),
        other => panic!("unexpected AppResponse: {other:?}"),
    }
}

fn expect_probe_result(response: AppResponse, expected: ProbeResult) {
    let actual: ProbeResult = decode_zome(response);
    assert_eq!(actual, expected);
}

fn expect_ok(response: AppResponse) {
    match response {
        AppResponse::ZomeCalled(_) => {}
        other => panic!("expected successful zome response, got {other:?}"),
    }
}

fn record_case(id: &str, outcome: &str, zome_reached: bool) {
    println!("D6U_CASE\t{id}\t{outcome}\tzome-reached={zome_reached}\tPASS");
}

fn expect_authentication_failed(response: AppResponse) {
    match response {
        AppResponse::Error(ExternalApiWireError::ZomeCallAuthenticationFailed(_)) => {}
        other => panic!("expected authentication failure, got {other:?}"),
    }
}

fn expect_unauthorized_reason(response: AppResponse, reason: &str) -> String {
    match response {
        AppResponse::Error(ExternalApiWireError::ZomeCallUnauthorized(message)) => {
            assert!(
                message.contains(reason),
                "expected authorization reason {reason}, got {message}"
            );
            message.to_string()
        }
        other => panic!("expected ZomeCallUnauthorized wire error, got {other:?}"),
    }
}

fn expect_internal_error(response: AppResponse) -> String {
    match response {
        AppResponse::Error(ExternalApiWireError::InternalError(message)) => message.to_string(),
        other => panic!("expected conductor internal error, got {other:?}"),
    }
}

fn expect_internal_error_contains(response: AppResponse, reason: &str) -> String {
    match response {
        AppResponse::Error(ExternalApiWireError::InternalError(message)) => {
            assert!(
                message.contains(reason),
                "expected internal error reason {reason}, got {message}"
            );
            message.to_string()
        }
        other => panic!("expected conductor internal error, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn d6u_runtime_authority_boundary() {
    assert_frozen_d6s_identity();

    let reached = Arc::new(AtomicUsize::new(0));
    let (frozen_canonical, frozen_commitment, d6s_domain) = load_probe_vector();
    let zome = authority_probe_zome(reached.clone(), d6s_domain);
    let (dna, _, _) = SweetDnaFile::unique_from_inline_zomes(zome).await;

    let mut conductor = SweetConductor::standard().await;
    let alice = SweetAgents::one(conductor.keystore()).await;
    let bob = SweetAgents::one(conductor.keystore()).await;
    let charlie = SweetAgents::one(conductor.keystore()).await;

    let alice_app = conductor
        .setup_app_for_agent("d6u-alice", alice.clone(), [&dna])
        .await
        .expect("Alice app must install");
    let bob_app = conductor
        .setup_app_for_agent("d6u-bob", bob.clone(), [&dna])
        .await
        .expect("Bob app must install");

    let alice_cell = alice_app.cells()[0].cell_id().clone();
    let bob_cell = bob_app.cells()[0].cell_id().clone();
    let app_api = AppInterfaceApi::new(conductor.clone());

    let base = ProbeInput {
        canonical_bytes: frozen_canonical,
        expected_commitment: frozen_commitment,
        semantic_valid: true,
    };

    let (n, exp) = holochain_nonce::fresh_nonce(Timestamp::now()).unwrap();
    expect_probe_result(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &alice,
                SweetInlineZomes::COORDINATOR.into(),
                "probe".into(),
                None,
                base.clone(),
                n,
                exp,
            ),
        )
        .await,
        ProbeResult::Accepted,
    );
    assert_eq!(reached.load(Ordering::SeqCst), 1);
    record_case("canonical-payload-accepted", "accepted", true);

    let semantic_invalid = ProbeInput {
        semantic_valid: false,
        ..base.clone()
    };
    let (n, exp) = holochain_nonce::fresh_nonce(Timestamp::now()).unwrap();
    expect_probe_result(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &alice,
                SweetInlineZomes::COORDINATOR.into(),
                "probe".into(),
                None,
                semantic_invalid,
                n,
                exp,
            ),
        )
        .await,
        ProbeResult::SemanticRejected,
    );
    assert_eq!(reached.load(Ordering::SeqCst), 2);
    record_case("authorized-semantic-rejection", "semantic-rejected", true);

    let mutated = ProbeInput {
        canonical_bytes: br#"{"a":1,"b":3}"#.to_vec(),
        ..base.clone()
    };
    let (n, exp) = holochain_nonce::fresh_nonce(Timestamp::now()).unwrap();
    expect_probe_result(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &alice,
                SweetInlineZomes::COORDINATOR.into(),
                "probe".into(),
                None,
                mutated,
                n,
                exp,
            ),
        )
        .await,
        ProbeResult::D6sCommitmentMismatch,
    );
    assert_eq!(reached.load(Ordering::SeqCst), 3);
    println!(
        "D6U_APPLICATION_CHECK\tprobe-local-d6s-commitment-mutation\tresult=d6s-commitment-mismatch;zome-reached=true\tPASS"
    );

    let alice_params = params(
        &alice_cell,
        &alice,
        SweetInlineZomes::COORDINATOR.into(),
        "probe".into(),
        None,
        base.clone(),
        Nonce256Bits::from([0x31; 32]),
        holochain_nonce::fresh_nonce(Timestamp::now()).unwrap().1,
    );
    let alice_signed =
        ZomeCallParamsSigned::try_from_params(&conductor.keystore(), alice_params.clone())
            .await
            .expect("Alice signature fixture must be constructible");
    let bob_signed = ZomeCallParamsSigned::try_from_params(
        &conductor.keystore(),
        ZomeCallParams {
            provenance: bob.clone(),
            ..alice_params
        },
    )
    .await
    .expect("Bob signature fixture must be constructible");

    let invalid_signature =
        ZomeCallParamsSigned::new(alice_signed.bytes.into(), bob_signed.signature);
    let before = reached.load(Ordering::SeqCst);
    let invalid_response = app_api
        .handle_request(
            "d6u-alice".into(),
            Ok(AppRequest::CallZome(Box::new(invalid_signature))),
        )
        .await
        .expect("invalid signature request must complete");
    expect_authentication_failed(invalid_response);
    assert_eq!(reached.load(Ordering::SeqCst), before);
    record_case("wire-signature-invalid", "authentication-failed", false);

    let (n, exp) = holochain_nonce::fresh_nonce(Timestamp::now()).unwrap();
    expect_probe_result(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &alice,
                SweetInlineZomes::COORDINATOR.into(),
                "probe".into(),
                None,
                base.clone(),
                n,
                exp,
            ),
        )
        .await,
        ProbeResult::Accepted,
    );
    assert_eq!(reached.load(Ordering::SeqCst), before + 1);
    record_case("author-grant", "accepted", true);

    let grant_response = call(
        &app_api,
        "d6u-alice",
        &conductor.keystore(),
        params(
            &alice_cell,
            &alice,
            SweetInlineZomes::COORDINATOR.into(),
            "grant_probe".into(),
            None,
            bob.clone(),
            Nonce256Bits::from([0x40; 32]),
            holochain_nonce::fresh_nonce(Timestamp::now()).unwrap().1,
        ),
    )
    .await;

    let grant_material: GrantMaterial = decode_zome(grant_response);

    let before = reached.load(Ordering::SeqCst);
    let (n, exp) = holochain_nonce::fresh_nonce(Timestamp::now()).unwrap();
    expect_probe_result(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &bob,
                SweetInlineZomes::COORDINATOR.into(),
                "probe".into(),
                Some(grant_material.secret),
                base.clone(),
                n,
                exp,
            ),
        )
        .await,
        ProbeResult::Accepted,
    );
    assert_eq!(reached.load(Ordering::SeqCst), before + 1);
    record_case("valid-capability", "accepted", true);

    let wrong_cap = CapSecret::from([0x5A; CAP_SECRET_BYTES]);
    let (n, exp) = holochain_nonce::fresh_nonce(Timestamp::now()).unwrap();
    let before = reached.load(Ordering::SeqCst);
    expect_unauthorized_reason(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &bob,
                SweetInlineZomes::COORDINATOR.into(),
                "probe".into(),
                Some(wrong_cap),
                base.clone(),
                n,
                exp,
            ),
        )
        .await,
        "BadCapGrant",
    );
    assert_eq!(reached.load(Ordering::SeqCst), before);
    record_case("wrong-capability", "authorization-failed", false);

    let (n, exp) = holochain_nonce::fresh_nonce(Timestamp::now()).unwrap();
    let before = reached.load(Ordering::SeqCst);
    expect_unauthorized_reason(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &charlie,
                SweetInlineZomes::COORDINATOR.into(),
                "probe".into(),
                Some(grant_material.secret),
                base.clone(),
                n,
                exp,
            ),
        )
        .await,
        "BadCapGrant",
    );
    assert_eq!(reached.load(Ordering::SeqCst), before);
    record_case("provenance-mismatch", "authorization-failed", false);

    expect_ok(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &alice,
                SweetInlineZomes::COORDINATOR.into(),
                "revoke_probe".into(),
                None,
                grant_material.action_hash.clone(),
                Nonce256Bits::from([0x50; 32]),
                holochain_nonce::fresh_nonce(Timestamp::now()).unwrap().1,
            ),
        )
        .await,
    );

    let (n, exp) = holochain_nonce::fresh_nonce(Timestamp::now()).unwrap();
    let before = reached.load(Ordering::SeqCst);
    expect_unauthorized_reason(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &bob,
                SweetInlineZomes::COORDINATOR.into(),
                "probe".into(),
                Some(grant_material.secret),
                base.clone(),
                n,
                exp,
            ),
        )
        .await,
        "BadCapGrant",
    );
    assert_eq!(reached.load(Ordering::SeqCst), before);
    record_case("revoked-capability", "authorization-failed", false);

    let replay_nonce = Nonce256Bits::from([0xff; 32]);
    let replay_params = params(
        &bob_cell,
        &bob,
        SweetInlineZomes::COORDINATOR.into(),
        "probe".into(),
        None,
        base.clone(),
        replay_nonce,
        holochain_nonce::fresh_nonce(Timestamp::now()).unwrap().1,
    );
    let replay_signed = ZomeCallParamsSigned::try_from_params(&conductor.keystore(), replay_params)
        .await
        .expect("replay fixture must sign");

    expect_probe_result(
        app_api
            .handle_request(
                "d6u-bob".into(),
                Ok(AppRequest::CallZome(Box::new(replay_signed.clone()))),
            )
            .await
            .unwrap(),
        ProbeResult::Accepted,
    );

    let before = reached.load(Ordering::SeqCst);
    expect_unauthorized_reason(
        app_api
            .handle_request(
                "d6u-bob".into(),
                Ok(AppRequest::CallZome(Box::new(replay_signed))),
            )
            .await
            .unwrap(),
        "Duplicate",
    );
    assert_eq!(reached.load(Ordering::SeqCst), before);
    record_case("nonce-replay", "authorization-failed", false);

    let future_expiry =
        (Timestamp::now() + std::time::Duration::from_secs(60 * 60)).expect("future expiry");
    let future = params(
        &bob_cell,
        &bob,
        SweetInlineZomes::COORDINATOR.into(),
        "probe".into(),
        None,
        base.clone(),
        Nonce256Bits::from([0x61; 32]),
        future_expiry,
    );
    let before = reached.load(Ordering::SeqCst);
    let future_message = expect_unauthorized_reason(
        call(&app_api, "d6u-bob", &conductor.keystore(), future).await,
        "Future",
    );
    assert_eq!(reached.load(Ordering::SeqCst), before);
    println!("D6U_RUNTIME_WITNESS\tfuture-expiry-rejection\t{future_message}");
    println!("D6U_SUBSTRATE_CHECK\tfuture-expiry-rejection\t{future_message}\tPASS");
    let expired = params(
        &alice_cell,
        &alice,
        SweetInlineZomes::COORDINATOR.into(),
        "probe".into(),
        None,
        base.clone(),
        Nonce256Bits::from([0x70; 32]),
        Timestamp::from_micros(0),
    );
    let before = reached.load(Ordering::SeqCst);
    expect_unauthorized_reason(
        call(&app_api, "d6u-alice", &conductor.keystore(), expired).await,
        "Expired",
    );
    assert_eq!(reached.load(Ordering::SeqCst), before);
    record_case("expired-invocation", "authorization-failed", false);

    let wrong_zome = params(
        &alice_cell,
        &alice,
        "wrong-zome".into(),
        "probe".into(),
        None,
        base.clone(),
        Nonce256Bits::from([0x71; 32]),
        holochain_nonce::fresh_nonce(Timestamp::now()).unwrap().1,
    );
    let before = reached.load(Ordering::SeqCst);
    let wrong_zome_message = expect_internal_error_contains(
        call(&app_api, "d6u-alice", &conductor.keystore(), wrong_zome).await,
        "Zome not found: Zome 'wrong-zome' not found",
    );
    assert_eq!(reached.load(Ordering::SeqCst), before);
    println!("D6U_RUNTIME_WITNESS\twrong-zome-routing\t{wrong_zome_message}");
    println!("D6U_SUBSTRATE_CHECK\twrong-zome-routing\t{wrong_zome_message}\tPASS");
    record_case("wrong-zome", "routing-failed", false);

    let wrong_function = params(
        &alice_cell,
        &alice,
        SweetInlineZomes::COORDINATOR.into(),
        "no_such_function".into(),
        None,
        base.clone(),
        Nonce256Bits::from([0x72; 32]),
        holochain_nonce::fresh_nonce(Timestamp::now()).unwrap().1,
    );
    let before = reached.load(Ordering::SeqCst);
    let wrong_function_response =
        call(&app_api, "d6u-alice", &conductor.keystore(), wrong_function).await;
    let wrong_function_message = expect_internal_error_contains(
        wrong_function_response,
        "Attempted to call a zome function that doesn't exist: Zome: coordinator Fn no_such_function",
    );
    assert_eq!(reached.load(Ordering::SeqCst), before);
    println!("D6U_RUNTIME_WITNESS\twrong-function-routing\t{wrong_function_message}");
    println!("D6U_SUBSTRATE_CHECK\twrong-function-routing\t{wrong_function_message}\tPASS");
    record_case("wrong-function", "routing-failed", false);

    let missing_cell = CellId::new(alice_cell.dna_hash().clone(), charlie.clone());
    let wrong_cell = params(
        &missing_cell,
        &alice,
        SweetInlineZomes::COORDINATOR.into(),
        "probe".into(),
        None,
        base.clone(),
        Nonce256Bits::from([0x73; 32]),
        holochain_nonce::fresh_nonce(Timestamp::now()).unwrap().1,
    );
    let before = reached.load(Ordering::SeqCst);
    let wrong_cell_message =
        expect_internal_error(call(&app_api, "d6u-bob", &conductor.keystore(), wrong_cell).await);
    assert_eq!(reached.load(Ordering::SeqCst), before);
    assert!(
        wrong_cell_message.contains("CellMissing("),
        "wrong-cell runtime witness must identify a missing cell: {wrong_cell_message}"
    );
    println!("D6U_RUNTIME_WITNESS\twrong-cell-routing\t{wrong_cell_message}");
    println!("D6U_SUBSTRATE_CHECK\twrong-cell-routing\t{wrong_cell_message}\tPASS");
    record_case("wrong-cell", "routing-failed", false);

    let before = reached.load(Ordering::SeqCst);
    conductor
        .raw_handle()
        .holochain_p2p()
        .block(Block::new(
            BlockTarget::Cell(alice_cell.clone(), CellBlockReason::BadCrypto),
            InclusiveTimestampInterval::try_new(Timestamp::now(), Timestamp::max())
                .expect("block interval must be valid"),
        ))
        .await
        .expect("system block must be committed");

    expect_unauthorized_reason(
        call(
            &app_api,
            "d6u-alice",
            &conductor.keystore(),
            params(
                &alice_cell,
                &alice,
                SweetInlineZomes::COORDINATOR.into(),
                "probe".into(),
                None,
                base,
                Nonce256Bits::from([0x74; 32]),
                holochain_nonce::fresh_nonce(Timestamp::now()).unwrap().1,
            ),
        )
        .await,
        "BlockedProvenance",
    );
    assert_eq!(reached.load(Ordering::SeqCst), before);
    record_case("blocked-provenance", "authorization-failed", false);

    assert_eq!(reached.load(Ordering::SeqCst), 6);
}

#[test]
fn d6u_fixture_commitment_matches_d6s_canon_1() {
    assert_frozen_d6s_identity();
}
