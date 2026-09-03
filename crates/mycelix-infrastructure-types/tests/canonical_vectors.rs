use mycelix_infrastructure_types::*;

#[test]
fn resource_order_does_not_change_id() {
    let storage = ResourceKeyV1::new("storage/bytes").unwrap();
    let bandwidth = ResourceKeyV1::new("bandwidth/bytes").unwrap();
    let a = ResourceVectorV1::new([
        (storage.clone(), 10),
        (bandwidth.clone(), 20),
    ]).unwrap();
    let b = ResourceVectorV1::new([
        (bandwidth, 20),
        (storage, 10),
    ]).unwrap();
    let payload = PayloadCommitmentV1::from_canonical_bytes("storage/capability-v1", b"same").unwrap();

    let left = CapabilityEnvelopeV1::new(
        PartyIdV1([3; 32]),
        TimeWindowV1::new(1, 2).unwrap(),
        a,
        payload.clone(),
    ).unwrap();
    let right = CapabilityEnvelopeV1::new(
        PartyIdV1([3; 32]),
        TimeWindowV1::new(1, 2).unwrap(),
        b,
        payload,
    ).unwrap();

    assert_eq!(left.id, right.id);
}

#[test]
fn domain_separation_changes_id() {
    let fields: [&[u8]; 1] = [b"same"];
    let capability = StableIdV1::derive("capability", 1, &fields).unwrap();
    let offering = StableIdV1::derive("offering", 1, &fields).unwrap();
    assert_ne!(capability, offering);
}

#[test]
fn payload_schema_is_part_of_commitment() {
    let a = PayloadCommitmentV1::from_canonical_bytes("storage/capability-v1", b"same").unwrap();
    let b = PayloadCommitmentV1::from_canonical_bytes("compute/capability-v1", b"same").unwrap();
    assert_ne!(a.digest, b.digest);
}

#[test]
fn duplicate_resource_dimensions_are_rejected() {
    let storage = ResourceKeyV1::new("storage/bytes").unwrap();
    let err = ResourceVectorV1::new([(storage.clone(), 10), (storage, 20)]).unwrap_err();
    assert_eq!(err, InfrastructureErrorV1::DuplicateResourceKey);
}
