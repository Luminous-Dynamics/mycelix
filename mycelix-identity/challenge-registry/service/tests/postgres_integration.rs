use mycelix_verifier_challenge_registry::{
    ChallengeContext, ChallengeSecret, PostgresChallengeRegistry,
};
use std::time::Duration;
use tokio::sync::Barrier;
use uuid::Uuid;

fn context() -> ChallengeContext {
    ChallengeContext::new(
        "did:mycelix:postgres-integration-verifier",
        "urn:mycelix:postgres-integration",
        Uuid::new_v4(),
        [0x31; 32],
        [0x42; 32],
    )
    .expect("valid integration context")
}

fn database_url() -> String {
    std::env::var("DATABASE_URL")
        .expect("DATABASE_URL must point at the disposable PostgreSQL service")
}

#[tokio::test]
async fn sqlx_adapter_registers_consumes_once_and_rejects_replay() {
    let registry = PostgresChallengeRegistry::connect(&database_url())
        .await
        .expect("connect authoritative PostgreSQL primary");
    let context = context();
    let secret = registry
        .issue(&context, Duration::from_secs(300))
        .await
        .expect("register fresh challenge");

    assert!(registry
        .consume_and_commit(&secret, &context)
        .await
        .expect("commit consume"));
    assert!(!registry
        .consume_and_commit(&secret, &context)
        .await
        .expect("replay must be a clean rejection"));
    registry.close().await;
}

#[tokio::test]
async fn independent_pools_racing_to_consume_have_exactly_one_winner() {
    let url = database_url();
    let first = PostgresChallengeRegistry::connect_with_limit(&url, 4)
        .await
        .expect("first independent pool");
    let second = PostgresChallengeRegistry::connect_with_limit(&url, 4)
        .await
        .expect("second independent pool");
    let context = context();
    let issued = first
        .issue(&context, Duration::from_secs(300))
        .await
        .expect("register race challenge");
    let wire = *issued.as_wire_bytes();
    let first_secret = ChallengeSecret::from_wire_bytes(&wire).expect("first presentation copy");
    let second_secret = ChallengeSecret::from_wire_bytes(&wire).expect("second presentation copy");

    let barrier = std::sync::Arc::new(Barrier::new(2));
    let first_task = {
        let first = first.clone();
        let barrier = barrier.clone();
        let context = context.clone();
        tokio::spawn(async move {
            barrier.wait().await;
            first.consume_and_commit(&first_secret, &context).await
        })
    };
    let second_task = {
        let second = second.clone();
        let barrier = barrier.clone();
        tokio::spawn(async move {
            barrier.wait().await;
            second.consume_and_commit(&second_secret, &context).await
        })
    };

    let result_a = first_task.await.expect("first consumer task");
    let result_b = second_task.await.expect("second consumer task");
    let results = [result_a, result_b];
    let accepted = results
        .iter()
        .filter(|result| result.as_ref().is_ok_and(|accepted| *accepted))
        .count();
    let rejected = results
        .iter()
        .filter(|result| result.as_ref().is_ok_and(|accepted| !*accepted))
        .count();
    assert_eq!(accepted, 1, "exactly one pool must commit the consume");
    assert_eq!(rejected, 1, "the losing pool must observe a replay rejection");

    first.close().await;
    second.close().await;
}
