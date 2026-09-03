use mycelix_infrastructure_types::*;
use proptest::prelude::*;

fn key(name: &str) -> ResourceKeyV1 {
    ResourceKeyV1::new(name).unwrap()
}

fn resources(storage: u128, bandwidth: u128) -> ResourceVectorV1 {
    ResourceVectorV1::new([
        (key("storage/bytes"), storage),
        (key("bandwidth/bytes"), bandwidth),
    ])
    .unwrap()
}

fn payload(tag: &str, byte: u8) -> PayloadCommitmentV1 {
    PayloadCommitmentV1::from_canonical_bytes(tag, &[byte]).unwrap()
}

proptest! {
    #[test]
    fn offering_cannot_exceed_capability(
        cap_storage in 1u128..1_000_000,
        cap_bandwidth in 1u128..1_000_000,
        extra in 1u128..1_000_000,
    ) {
        let capability = CapabilityEnvelopeV1::new(
            PartyIdV1([1; 32]),
            TimeWindowV1::new(10, 1000).unwrap(),
            resources(cap_storage, cap_bandwidth),
            payload("storage/capability-v1", 1),
        ).unwrap();

        let attempted = OfferingEnvelopeV1::new(
            &capability,
            TimeWindowV1::new(20, 900).unwrap(),
            resources(cap_storage.saturating_add(extra), cap_bandwidth),
            payload("storage/offering-v1", 2),
        );

        prop_assert_eq!(attempted.unwrap_err(), InfrastructureErrorV1::ResourceExceedsParent);
    }

    #[test]
    fn lease_cannot_outlive_offering(start in 100u64..500, end in 501u64..900) {
        let capability = CapabilityEnvelopeV1::new(
            PartyIdV1([1; 32]),
            TimeWindowV1::new(0, 1000).unwrap(),
            resources(1000, 1000),
            payload("storage/capability-v1", 1),
        ).unwrap();
        let offering = OfferingEnvelopeV1::new(
            &capability,
            TimeWindowV1::new(start, end).unwrap(),
            resources(900, 900),
            payload("storage/offering-v1", 2),
        ).unwrap();

        let attempted = LeaseEnvelopeV1::new(
            &offering,
            PartyIdV1([2; 32]),
            TimeWindowV1::new(start - 1, end).unwrap(),
            resources(100, 100),
            payload("storage/lease-v1", 3),
        );

        prop_assert_eq!(attempted.unwrap_err(), InfrastructureErrorV1::WindowOutsideParent);
    }

    #[test]
    fn ids_are_deterministic(storage in 1u128..1_000_000, bandwidth in 1u128..1_000_000) {
        let make = || CapabilityEnvelopeV1::new(
            PartyIdV1([7; 32]),
            TimeWindowV1::new(100, 200).unwrap(),
            resources(storage, bandwidth),
            payload("storage/capability-v1", 9),
        ).unwrap();
        prop_assert_eq!(make().id, make().id);
    }
}

#[test]
fn receipt_is_bound_to_lease_and_parties() {
    let capability = CapabilityEnvelopeV1::new(
        PartyIdV1([1; 32]),
        TimeWindowV1::new(0, 1000).unwrap(),
        resources(1000, 1000),
        payload("storage/capability-v1", 1),
    ).unwrap();
    let offering = OfferingEnvelopeV1::new(
        &capability,
        TimeWindowV1::new(100, 900).unwrap(),
        resources(900, 900),
        payload("storage/offering-v1", 2),
    ).unwrap();
    let lease = LeaseEnvelopeV1::new(
        &offering,
        PartyIdV1([2; 32]),
        TimeWindowV1::new(200, 800).unwrap(),
        resources(500, 500),
        payload("storage/lease-v1", 3),
    ).unwrap();
    let receipt = ReceiptEnvelopeV1::new(
        &lease,
        TimeWindowV1::new(250, 700).unwrap(),
        UnixMillisV1(710),
        resources(400, 300),
        payload("storage/receipt-v1", 4),
    ).unwrap();

    assert_eq!(receipt.recompute_id().unwrap(), receipt.id);
    receipt.validate_against(&lease).unwrap();
}

#[test]
fn mutation_changes_recomputed_id() {
    let mut capability = CapabilityEnvelopeV1::new(
        PartyIdV1([1; 32]),
        TimeWindowV1::new(0, 1000).unwrap(),
        resources(1000, 1000),
        payload("storage/capability-v1", 1),
    ).unwrap();
    let original = capability.id;
    capability.validity = TimeWindowV1::new(0, 999).unwrap();
    assert_ne!(capability.recompute_id().unwrap(), original);
}

#[test]
fn stored_id_must_match_recomputed_id() {
    let mut capability = CapabilityEnvelopeV1::new(
        PartyIdV1([1; 32]),
        TimeWindowV1::new(0, 1000).unwrap(),
        resources(1000, 1000),
        payload("storage/capability-v1", 1),
    ).unwrap();
    capability.id = StableIdV1([9; 32]);
    assert_eq!(capability.validate().unwrap_err(), InfrastructureErrorV1::IdMismatch);
}

#[test]
fn unsupported_schema_version_is_rejected() {
    let mut capability = CapabilityEnvelopeV1::new(
        PartyIdV1([1; 32]),
        TimeWindowV1::new(0, 1000).unwrap(),
        resources(1000, 1000),
        payload("storage/capability-v1", 1),
    ).unwrap();
    capability.schema_version = 2;
    assert_eq!(capability.validate().unwrap_err(), InfrastructureErrorV1::UnsupportedSchemaVersion);
}

#[test]
fn receipt_cannot_overclaim_lease_resources() {
    let capability = CapabilityEnvelopeV1::new(
        PartyIdV1([1; 32]),
        TimeWindowV1::new(0, 1000).unwrap(),
        resources(1000, 1000),
        payload("storage/capability-v1", 1),
    ).unwrap();
    let offering = OfferingEnvelopeV1::new(
        &capability,
        TimeWindowV1::new(100, 900).unwrap(),
        resources(900, 900),
        payload("storage/offering-v1", 2),
    ).unwrap();
    let lease = LeaseEnvelopeV1::new(
        &offering,
        PartyIdV1([2; 32]),
        TimeWindowV1::new(200, 800).unwrap(),
        resources(500, 500),
        payload("storage/lease-v1", 3),
    ).unwrap();

    let err = ReceiptEnvelopeV1::new(
        &lease,
        TimeWindowV1::new(250, 700).unwrap(),
        UnixMillisV1(710),
        resources(501, 500),
        payload("storage/receipt-v1", 4),
    ).unwrap_err();

    assert_eq!(err, InfrastructureErrorV1::ResourceExceedsParent);
}

#[test]
fn receipt_cannot_be_observed_before_service_window_ends() {
    let capability = CapabilityEnvelopeV1::new(
        PartyIdV1([1; 32]),
        TimeWindowV1::new(0, 1000).unwrap(),
        resources(1000, 1000),
        payload("storage/capability-v1", 1),
    ).unwrap();
    let offering = OfferingEnvelopeV1::new(
        &capability,
        TimeWindowV1::new(100, 900).unwrap(),
        resources(900, 900),
        payload("storage/offering-v1", 2),
    ).unwrap();
    let lease = LeaseEnvelopeV1::new(
        &offering,
        PartyIdV1([2; 32]),
        TimeWindowV1::new(200, 800).unwrap(),
        resources(500, 500),
        payload("storage/lease-v1", 3),
    ).unwrap();

    let err = ReceiptEnvelopeV1::new(
        &lease,
        TimeWindowV1::new(250, 700).unwrap(),
        UnixMillisV1(699),
        resources(400, 300),
        payload("storage/receipt-v1", 4),
    ).unwrap_err();

    assert_eq!(err, InfrastructureErrorV1::ReceiptBeforeServiceEnd);
}
