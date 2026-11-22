use std::{
    path::PathBuf,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use async_nats::jetstream::{
    self,
    stream::{Config as StreamConfig, DiscardPolicy, RetentionPolicy, StorageType},
};
use myc_dir_fabric_nats_adapter::{
    postgres::PostgresEffectCommitVerifier, JetStreamAdapter, JetStreamAdapterConfig,
    OwnerDispositionVerifier, VerifiedOwnerDisposition,
};
use myc_dir_fabric_nats_reference::{
    AckMode, DurableOwnerDisposition, NatsCommandBinding, NatsCommandBindingInput,
};
use rustls_tokio_postgres::{config_from_ca_cert, MakeRustlsConnect};
use sha2::{Digest as ShaDigest, Sha256};
use tokio_postgres::Client;

struct SyntheticOwnerReceiptVerifier;

#[async_trait::async_trait]
impl OwnerDispositionVerifier for SyntheticOwnerReceiptVerifier {
    async fn verify_durable_disposition(
        &self,
        binding: &myc_dir_fabric_nats_reference::NatsCommandBinding,
        disposition: &DurableOwnerDisposition,
    ) -> Result<(), String> {
        let plan =
            myc_dir_fabric_nats_reference::plan_acknowledgement(binding, disposition, AckMode::Ack);
        match plan {
            myc_dir_fabric_nats_reference::AckPlan::Acknowledge { .. } => Ok(()),
            myc_dir_fabric_nats_reference::AckPlan::DoNotAcknowledge => {
                Err("synthetic receipt does not match exact test command".into())
            }
        }
    }
}

async fn connect_postgres_for_test() -> Client {
    let dsn = std::env::var("POSTGRES_DSN")
        .expect("POSTGRES_DSN must point at the disposable TLS PostgreSQL primary");
    let ca_file = PathBuf::from(
        std::env::var("POSTGRES_CA_FILE")
            .expect("POSTGRES_CA_FILE must point at the test CA certificate"),
    );
    let tls = MakeRustlsConnect::new(
        config_from_ca_cert(ca_file).expect("load test PostgreSQL CA"),
    );
    let (client, connection) = tokio_postgres::connect(&dsn, tls)
        .await
        .expect("connect to PostgreSQL primary with CA-verified TLS");
    tokio::spawn(async move {
        if let Err(error) = connection.await {
            eprintln!("test PostgreSQL connection ended: {error}");
        }
    });
    client
}

async fn create_committed_postgres_effect(
    suffix: &str,
    payload: &[u8],
    stream_name: &str,
    subject: &str,
    consumer_name: &str,
) -> NatsCommandBindingInput {
    let realm = format!("myc-realm-{suffix}");
    let resource = format!("myc-resource-{suffix}");
    let action = "balance-adjust".to_owned();
    let owner = "postgres-effect-owner".to_owned();
    let object = format!("myc-account-{suffix}");
    let currency = "USD".to_owned();
    let operation = format!("myc-operation-{suffix}");
    let worker = "worker-a".to_owned();
    let claim_proof = [0x55_u8; 32];
    let idempotency = [0x44_u8; 32];
    let request = [0x11_u8; 32];
    let decision = [0x22_u8; 32];
    let payload_digest: [u8; 32] = Sha256::digest(payload).into();

    let mut client = connect_postgres_for_test().await;
    let transaction = client
        .transaction()
        .await
        .expect("start PostgreSQL effect transaction");
    transaction
        .batch_execute("SET LOCAL synchronous_commit = on")
        .await
        .expect("enable synchronous commit for effect transaction");

    transaction
        .execute(
            "INSERT INTO myc_effect_owner.effect_fence_state \
             (realm_id, resource_id, action_id, effect_owner_id, generation) \
             VALUES ($1, $2, $3, $4, 1)",
            &[&realm, &resource, &action, &owner],
        )
        .await
        .expect("create test owner fence");
    transaction
        .execute(
            "INSERT INTO myc_effect_owner.protected_balance \
             (realm_id, resource_id, action_id, effect_owner_id, object_id, currency_code, balance_minor) \
             VALUES ($1, $2, $3, $4, $5, $6, 1000)",
            &[&realm, &resource, &action, &owner, &object, &currency],
        )
        .await
        .expect("create test protected balance");

    transaction
        .query_one(
            "SELECT myc_effect_owner.install_worker_claim( \
               $1, $2, $3, $4, $5, $6, 1, $7, \
               'postgres-db-clock-v1', \
               clock_timestamp() - interval '1 minute', \
               clock_timestamp() + interval '1 hour')",
            &[
                &realm,
                &resource,
                &action,
                &owner,
                &operation,
                &worker,
                &&claim_proof[..],
            ],
        )
        .await
        .expect("install initial worker claim");

    let effect_row = transaction
        .query_one(
            "SELECT replayed, effect_id::text, accepted_generation \
             FROM myc_effect_owner.apply_claim_fenced_balance_delta( \
               $1, $2, $3, $4, $5, $6, 1, $7, $8, $9, $10, $11, $12, \
               $13, 1, $14, 'postgres-db-clock-v1', \
               clock_timestamp() - interval '1 minute', \
               clock_timestamp() + interval '1 hour')",
            &[
                &realm,
                &resource,
                &action,
                &owner,
                &object,
                &currency,
                &&idempotency[..],
                &operation,
                &&request[..],
                &&decision[..],
                &&payload_digest[..],
                &100_i64,
                &worker,
                &&claim_proof[..],
            ],
        )
        .await
        .expect("execute protected effect and append the receipt");
    let replayed: bool = effect_row.get(0);
    let effect_id: String = effect_row.get(1);
    let accepted_generation: i64 = effect_row.get(2);
    assert!(!replayed, "first physical effect attempt must not be a replay");
    assert!(!effect_id.is_empty());
    assert_eq!(accepted_generation, 1);
    transaction
        .commit()
        .await
        .expect("commit effect and append-only receipt");

    NatsCommandBindingInput {
        account_id: "default".into(),
        realm_id: realm,
        resource_id: resource,
        action_id: action,
        object_id: object,
        currency_code: currency,
        stream_name: stream_name.into(),
        subject: subject.into(),
        consumer_name: consumer_name.into(),
        nats_message_id: format!("pg-effect-publication-{suffix}"),
        payload_digest,
        request_digest: request,
        decision_receipt_digest: decision,
        logical_operation_id: operation,
        idempotency_digest: idempotency,
        effect_owner_id: owner,
        owner_epoch: 1,
        resource_generation: 1,
    }
}

#[tokio::test]
#[ignore = "requires disposable NATS JetStream and CA-verified PostgreSQL services"]
async fn postgres_effect_commit_is_the_evidence_that_permits_jetstream_acksync() {
    let (adapter, stream_name, suffix) = provision_adapter().await;
    let subject = adapter.config().subject.clone();
    let consumer_name = adapter.config().durable_consumer.clone();
    let payload = format!("postgres-effect-command-{suffix}").into_bytes();

    let input = create_committed_postgres_effect(
        &suffix,
        &payload,
        &stream_name,
        &subject,
        &consumer_name,
    )
    .await;
    let binding = NatsCommandBinding::new(input.clone())
        .expect("construct exact database-scoped command binding");

    let verifier = PostgresEffectCommitVerifier::connect_with_ca(
        &std::env::var("POSTGRES_DSN").expect("POSTGRES_DSN is configured"),
        std::env::var("POSTGRES_CA_FILE").expect("POSTGRES_CA_FILE is configured"),
    )
    .await
    .expect("construct CA-verified PostgreSQL effect verifier");

    let wrong_scope = NatsCommandBinding::new(NatsCommandBindingInput {
        resource_id: format!("wrong-resource-{suffix}"),
        ..input.clone()
    })
    .expect("construct wrong-scope negative control");
    assert!(
        verifier.verify_effect_commit(&wrong_scope).await.is_err(),
        "receipt for a different resource scope must not authorize ACK"
    );

    let wrong_request = NatsCommandBinding::new(NatsCommandBindingInput {
        request_digest: [0x91; 32],
        ..input.clone()
    })
    .expect("construct wrong-request negative control");
    assert!(
        verifier.verify_effect_commit(&wrong_request).await.is_err(),
        "receipt with a different request digest must not authorize ACK"
    );

    let wrong_decision = NatsCommandBinding::new(NatsCommandBindingInput {
        decision_receipt_digest: [0x92; 32],
        ..input.clone()
    })
    .expect("construct wrong-decision negative control");
    assert!(
        verifier.verify_effect_commit(&wrong_decision).await.is_err(),
        "receipt with a different decision digest must not authorize ACK"
    );

    let wrong_payload = NatsCommandBinding::new(NatsCommandBindingInput {
        payload_digest: [0x93; 32],
        ..input.clone()
    })
    .expect("construct wrong-payload negative control");
    assert!(
        verifier.verify_effect_commit(&wrong_payload).await.is_err(),
        "receipt with a different payload digest must not authorize ACK"
    );

    let wrong_generation = NatsCommandBinding::new(NatsCommandBindingInput {
        resource_generation: 2,
        ..input.clone()
    })
    .expect("construct wrong-generation negative control");
    assert!(
        verifier.verify_effect_commit(&wrong_generation).await.is_err(),
        "receipt with a different accepted generation must not authorize ACK"
    );

    let verified = verifier
        .verify_effect_commit(&binding)
        .await
        .expect("load and verify the exact append-only PostgreSQL effect row");

    let publish = adapter
        .publish(input, &payload)
        .await
        .expect("publish command for the committed PostgreSQL effect");
    let delivery = adapter
        .fetch_one()
        .await
        .expect("fetch NATS delivery")
        .expect("published command must be delivered");
    assert_eq!(delivery.observation.stream_sequence, publish.sequence);

    let ack = delivery
        .acknowledge(&verified, AckMode::AckSync)
        .await
        .expect("only the PostgreSQL-verified receipt permits AckSync");
    assert!(ack.broker_confirmed);
    assert_eq!(
        ack.assessment,
        myc_dir_fabric_nats_reference::BrokerAckAssessment::Confirmed
    );
    assert_eq!(ack.stream, stream_name);
}

fn unique_suffix() -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock must be after UNIX epoch")
        .as_nanos();
    format!("{:x}", nanos)
}

async fn provision_adapter() -> (JetStreamAdapter, String, String) {
    let url =
        std::env::var("NATS_URL").expect("NATS_URL must point at a disposable JetStream server");
    let suffix = unique_suffix();
    let stream_name = format!("MYC_TEST_{suffix}");
    let subject = format!("mycelix.test.{suffix}");
    let durable = format!("myc_workers_{suffix}");

    let client = async_nats::connect(&url)
        .await
        .expect("connect to disposable NATS server");
    let context = jetstream::new(client);
    context
        .get_or_create_stream(StreamConfig {
            name: stream_name.clone(),
            subjects: vec![subject.clone()],
            retention: RetentionPolicy::Limits,
            storage: StorageType::File,
            num_replicas: 1,
            discard: DiscardPolicy::New,
            max_messages: 10_000,
            max_bytes: 32 * 1024 * 1024,
            duplicate_window: Duration::from_secs(30),
            max_age: Duration::from_secs(7 * 24 * 60 * 60),
            ..Default::default()
        })
        .await
        .expect("create test stream");

    let adapter = JetStreamAdapter::connect(
        &url,
        JetStreamAdapterConfig {
            account_id: "default".into(),
            stream_name: stream_name.clone(),
            subject,
            durable_consumer: durable,
            duplicate_window: Duration::from_secs(30),
            expected_stream_replicas: 1,
            expected_retention: RetentionPolicy::Limits,
            max_messages: 10_000,
            max_bytes: 32 * 1024 * 1024,
            max_age: Duration::from_secs(7 * 24 * 60 * 60),
            expected_domain: None,
            expected_account_hash: None,
            ack_wait: Duration::from_millis(500),
            max_ack_pending: 32,
            max_deliver: 5,
            fetch_timeout: Duration::from_secs(4),
        },
    )
    .await
    .expect("construct the Mycelix adapter");

    adapter
        .verify_server_configuration()
        .await
        .expect("pin existing stream and explicit durable-consumer configuration");

    (adapter, stream_name, suffix)
}

fn command_input(
    stream_name: &str,
    subject: &str,
    consumer_name: &str,
    message_id: &str,
    operation_id: &str,
) -> NatsCommandBindingInput {
    NatsCommandBindingInput {
        account_id: "default".into(),
        realm_id: "realm-test".into(),
        resource_id: "resource-test".into(),
        action_id: "balance-adjust".into(),
        object_id: "account-test".into(),
        currency_code: "USD".into(),
        stream_name: stream_name.into(),
        subject: subject.into(),
        consumer_name: consumer_name.into(),
        nats_message_id: message_id.into(),
        // The adapter computes the payload digest from the actual bytes.
        payload_digest: [0; 32],
        request_digest: [1; 32],
        decision_receipt_digest: [2; 32],
        logical_operation_id: operation_id.into(),
        idempotency_digest: [3; 32],
        effect_owner_id: "postgres-owner-a".into(),
        owner_epoch: 7,
        resource_generation: 19,
    }
}

async fn connect_postgres_for_test() -> Client {
    let dsn = std::env::var("POSTGRES_DSN")
        .expect("POSTGRES_DSN must point at the disposable TLS PostgreSQL primary");
    let ca_file = PathBuf::from(
        std::env::var("POSTGRES_CA_FILE")
            .expect("POSTGRES_CA_FILE must point at the test CA certificate"),
    );
    let tls = MakeRustlsConnect::new(
        config_from_ca_cert(ca_file).expect("load test PostgreSQL CA"),
    );
    let (client, connection) = tokio_postgres::connect(&dsn, tls)
        .await
        .expect("connect to PostgreSQL primary with CA-verified TLS");
    tokio::spawn(async move {
        if let Err(error) = connection.await {
            eprintln!("test PostgreSQL connection ended: {error}");
        }
    });
    client
}

async fn create_committed_postgres_effect(
    suffix: &str,
    payload: &[u8],
) -> NatsCommandBindingInput {
    let realm = format!("myc-realm-{suffix}");
    let resource = format!("myc-resource-{suffix}");
    let action = "balance-adjust".to_owned();
    let owner = "postgres-effect-owner".to_owned();
    let object = format!("myc-account-{suffix}");
    let currency = "USD".to_owned();
    let operation = format!("myc-operation-{suffix}");
    let worker = "worker-a".to_owned();
    let claim_proof = [0x55_u8; 32];
    let idempotency = [0x44_u8; 32];
    let request = [0x11_u8; 32];
    let decision = [0x22_u8; 32];
    let payload_digest: [u8; 32] = Sha256::digest(payload).into();
    let mut client = connect_postgres_for_test().await;
    let transaction = client
        .transaction()
        .await
        .expect("start PostgreSQL effect transaction");
    transaction
        .batch_execute("SET LOCAL synchronous_commit = on")
        .await
        .expect("enable synchronous commit for effect transaction");

    transaction
        .execute(
            "INSERT INTO myc_effect_owner.effect_fence_state \
             (realm_id, resource_id, action_id, effect_owner_id, generation) \
             VALUES ($1, $2, $3, $4, 1)",
            &[&realm, &resource, &action, &owner],
        )
        .await
        .expect("create test owner fence");
    transaction
        .execute(
            "INSERT INTO myc_effect_owner.protected_balance \
             (realm_id, resource_id, action_id, effect_owner_id, object_id, currency_code, balance_minor) \
             VALUES ($1, $2, $3, $4, $5, $6, 1000)",
            &[&realm, &resource, &action, &owner, &object, &currency],
        )
        .await
        .expect("create test protected balance");

    transaction
        .query_one(
            "SELECT myc_effect_owner.install_worker_claim( \
               $1, $2, $3, $4, $5, $6, 1, $7, \
               'postgres-db-clock-v1', \
               clock_timestamp() - interval '1 minute', \
               clock_timestamp() + interval '1 hour')",
            &[
                &realm,
                &resource,
                &action,
                &owner,
                &operation,
                &worker,
                &&claim_proof[..],
            ],
        )
        .await
        .expect("install initial worker claim");

    let effect_row = transaction
        .query_one(
            "SELECT replayed, effect_id::text, accepted_generation \
             FROM myc_effect_owner.apply_claim_fenced_balance_delta( \
               $1, $2, $3, $4, $5, $6, 1, $7, $8, $9, $10, $11, 100, \
               $12, 1, $13, 'postgres-db-clock-v1', \
               clock_timestamp() - interval '1 minute', \
               clock_timestamp() + interval '1 hour')",
            &[
                &realm,
                &resource,
                &action,
                &owner,
                &object,
                &currency,
                &&idempotency[..],
                &operation,
                &&request[..],
                &&decision[..],
                &&payload_digest[..],
                &worker,
                &&claim_proof[..],
            ],
        )
        .await
        .expect("execute protected effect and append the receipt");
    let replayed: bool = effect_row.get(0);
    let effect_id: String = effect_row.get(1);
    let accepted_generation: i64 = effect_row.get(2);
    assert!(!replayed, "first physical effect attempt must not be a replay");
    assert!(!effect_id.is_empty());
    assert_eq!(accepted_generation, 1);
    transaction
        .commit()
        .await
        .expect("durably commit effect and append-only receipt");

    NatsCommandBindingInput {
        account_id: "default".into(),
        realm_id: realm,
        resource_id: resource,
        action_id: action,
        object_id: object,
        currency_code: currency,
        stream_name: String::new(),
        subject: String::new(),
        consumer_name: String::new(),
        nats_message_id: String::new(),
        payload_digest,
        request_digest: request,
        decision_receipt_digest: decision,
        logical_operation_id: operation,
        idempotency_digest: idempotency,
        effect_owner_id: owner,
        owner_epoch: 1,
        resource_generation: 1,
    }
}

#[tokio::test]
#[ignore = "requires disposable NATS JetStream and CA-verified PostgreSQL services"]
async fn postgres_effect_commit_is_the_evidence_that_permits_jetstream_acksync() {
    let (adapter, stream_name, suffix) = provision_adapter().await;
    let subject = adapter.config().subject.clone();
    let consumer_name = adapter.config().durable_consumer.clone();
    let payload = format!("postgres-effect-command-{suffix}").into_bytes();

    let mut input = create_committed_postgres_effect(&suffix, &payload).await;
    input.stream_name = stream_name.clone();
    input.subject = subject.clone();
    input.consumer_name = consumer_name;
    input.nats_message_id = format!("postgres-effect-publication-{suffix}");

    let verifier = PostgresEffectCommitVerifier::connect_with_ca(
        &std::env::var("POSTGRES_DSN").expect("POSTGRES_DSN is configured"),
        std::env::var("POSTGRES_CA_FILE").expect("POSTGRES_CA_FILE is configured"),
    )
    .await
    .expect("construct CA-verified PostgreSQL effect verifier");

    let wrong_scope = NatsCommandBinding::new(NatsCommandBindingInput {
        resource_id: format!("wrong-resource-{suffix}"),
        ..input.clone()
    })
    .expect("construct wrong-scope negative control");
    assert!(
        verifier.verify_effect_commit(&wrong_scope).await.is_err(),
        "receipt for a different resource scope must not authorize ACK"
    );

    let publish = adapter
        .publish(input.clone(), &payload)
        .await
        .expect("publish command for a committed PostgreSQL effect");
    let delivery = adapter
        .fetch_one()
        .await
        .expect("fetch NATS delivery")
        .expect("published command must be delivered");
    assert_eq!(delivery.observation.stream_sequence, publish.sequence);

    let verified = verifier
        .verify_effect_commit(&delivery.binding)
        .await
        .expect("load and verify the exact append-only PostgreSQL effect row");
    let ack = delivery
        .acknowledge(&verified, AckMode::AckSync)
        .await
        .expect("only the PostgreSQL-verified receipt permits AckSync");
    assert!(ack.broker_confirmed);
    assert_eq!(
        ack.assessment,
        myc_dir_fabric_nats_reference::BrokerAckAssessment::Confirmed
    );
    assert_eq!(ack.stream, stream_name);
}

#[tokio::test]
#[ignore = "requires the pinned disposable NATS JetStream service; CI invokes ignored tests explicitly"]
async fn publish_dedup_pull_and_double_ack_are_transport_only() {
    let (adapter, stream_name, suffix) = provision_adapter().await;
    let subject = adapter.config().subject.clone();
    let consumer_name = adapter.config().durable_consumer.clone();
    let payload = format!("test-command-{suffix}").into_bytes();
    let operation = format!("logical-operation-{suffix}");
    let input = command_input(
        &stream_name,
        &subject,
        &consumer_name,
        &format!("publication-{suffix}"),
        &operation,
    );

    let first = adapter
        .publish(input.clone(), &payload)
        .await
        .expect("first publish should be stored");
    assert_eq!(first.stream, stream_name);
    assert!(first.sequence > 0);
    assert!(!first.duplicate);

    let second = adapter
        .publish(input, &payload)
        .await
        .expect("same Nats-Msg-Id inside duplicate window should return PubAck");
    assert!(second.duplicate);
    assert_eq!(second.sequence, first.sequence);

    let delivery = adapter
        .fetch_one()
        .await
        .expect("pull should succeed")
        .expect("one message must be available");
    assert_eq!(delivery.observation.stream_sequence, first.sequence);
    assert_eq!(delivery.observation.delivery_count, 1);
    assert_eq!(delivery.payload.as_ref(), payload.as_slice());

    let verified = VerifiedOwnerDisposition::verify(
        &delivery.binding,
        DurableOwnerDisposition::EffectCommitted {
            logical_operation_id: operation,
            effect_owner_id: "postgres-owner-a".into(),
            idempotency_digest: [3; 32],
            effect_receipt_digest: [9; 32],
        },
        &SyntheticOwnerReceiptVerifier,
    )
    .await
    .expect("test owner verifier should accept the matching durable receipt");
    let ack = delivery
        .acknowledge(&verified, AckMode::AckSync)
        .await
        .expect("verified durable owner disposition should permit AckSync");
    assert!(ack.broker_confirmed);
    assert_eq!(
        ack.assessment,
        myc_dir_fabric_nats_reference::BrokerAckAssessment::Confirmed
    );
    assert_eq!(ack.stream, stream_name);
}

#[tokio::test]
#[ignore = "requires the pinned disposable NATS JetStream service; CI invokes ignored tests explicitly"]
async fn unacked_pull_is_redelivered_with_same_stream_sequence_and_higher_delivery_count() {
    let (adapter, stream_name, suffix) = provision_adapter().await;
    let subject = adapter.config().subject.clone();
    let consumer_name = adapter.config().durable_consumer.clone();
    let payload = format!("redelivery-command-{suffix}").into_bytes();
    let operation = format!("redelivery-operation-{suffix}");
    let input = command_input(
        &stream_name,
        &subject,
        &consumer_name,
        &format!("redelivery-publication-{suffix}"),
        &operation,
    );

    let publish = adapter
        .publish(input, &payload)
        .await
        .expect("publish redelivery fixture");
    let first = adapter
        .fetch_one()
        .await
        .expect("first pull succeeds")
        .expect("first delivery exists");
    let first_observation = first.observation.clone();
    assert_eq!(first_observation.stream_sequence, publish.sequence);
    assert_eq!(first_observation.delivery_count, 1);

    // Drop without ACK. This intentionally exercises AckWait, not Progress.
    drop(first);
    tokio::time::sleep(Duration::from_millis(1100)).await;

    let redelivery = adapter
        .fetch_one()
        .await
        .expect("redelivery pull succeeds")
        .expect("the unacked message must be redelivered");
    assert_eq!(
        redelivery.observation.stream_sequence,
        first_observation.stream_sequence
    );
    assert!(
        redelivery.observation.delivery_count > first_observation.delivery_count,
        "delivery count should advance after AckWait redelivery"
    );

    // A durable handoff is a valid transport disposition, but it is not an
    // effect-success receipt. The independent reconciler now owns the outcome.
    let verified = VerifiedOwnerDisposition::verify(
        &redelivery.binding,
        DurableOwnerDisposition::ReconciliationHandoffDurable {
            logical_operation_id: operation,
            effect_owner_id: "postgres-owner-a".into(),
            idempotency_digest: [3; 32],
            attempt_id: format!("attempt-{suffix}"),
            handoff_receipt_digest: [10; 32],
        },
        &SyntheticOwnerReceiptVerifier,
    )
    .await
    .expect("test owner verifier should accept the matching reconciliation handoff");
    let ack = redelivery
        .acknowledge(&verified, AckMode::AckSync)
        .await
        .expect("verified durable reconciliation handoff should permit AckSync");
    assert!(ack.broker_confirmed);
    assert_eq!(
        ack.assessment,
        myc_dir_fabric_nats_reference::BrokerAckAssessment::Confirmed
    );
}
