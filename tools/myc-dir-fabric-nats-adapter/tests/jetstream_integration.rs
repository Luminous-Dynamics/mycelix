use std::time::{Duration, SystemTime, UNIX_EPOCH};

use async_nats::jetstream::{
    self,
    stream::{Config as StreamConfig, DiscardPolicy, RetentionPolicy, StorageType},
};
use myc_dir_fabric_nats_adapter::{
    JetStreamAdapter, JetStreamAdapterConfig, OwnerDispositionVerifier,
    VerifiedOwnerDisposition,
};
use myc_dir_fabric_nats_reference::{
    AckMode, DurableOwnerDisposition, NatsCommandBindingInput,
};

struct SyntheticOwnerReceiptVerifier;

impl OwnerDispositionVerifier for SyntheticOwnerReceiptVerifier {
    fn verify_durable_disposition(
        &self,
        binding: &myc_dir_fabric_nats_reference::NatsCommandBinding,
        disposition: &DurableOwnerDisposition,
    ) -> Result<(), String> {
        let plan = myc_dir_fabric_nats_reference::plan_acknowledgement(
            binding,
            disposition,
            AckMode::Ack,
        );
        match plan {
            myc_dir_fabric_nats_reference::AckPlan::Acknowledge { .. } => Ok(()),
            myc_dir_fabric_nats_reference::AckPlan::DoNotAcknowledge => {
                Err("synthetic receipt does not match exact test command".into())
            }
        }
    }
}

fn unique_suffix() -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock must be after UNIX epoch")
        .as_nanos();
    format!("{:x}", nanos)
}

async fn provision_adapter() -> (JetStreamAdapter, String, String) {
    let url = std::env::var("NATS_URL")
        .expect("NATS_URL must point at a disposable JetStream server");
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
    ).expect("test owner verifier should accept the matching durable receipt");
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
    assert_eq!(redelivery.observation.stream_sequence, first_observation.stream_sequence);
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
    ).expect("test owner verifier should accept the matching reconciliation handoff");
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
