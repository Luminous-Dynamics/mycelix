use mycelix_infrastructure_types::{PartyIdV1, StableIdV1};
use mycelix_reader_enrollment::{
    ReaderEnrollmentErrorV1, ReaderEnrollmentRegistryV1, ReaderEnrollmentV1,
    XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1, XeniaHybridReaderCredentialV1,
    reconstruct_reader_enrollment_registry_v1,
};

fn ml(byte: u8) -> Vec<u8> {
    vec![byte; XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1]
}

fn credential(ed: u8, pq: u8) -> XeniaHybridReaderCredentialV1 {
    XeniaHybridReaderCredentialV1::new([ed; 32], ml(pq)).unwrap()
}

fn party(byte: u8) -> PartyIdV1 {
    PartyIdV1([byte; 32])
}

fn group(byte: u8) -> StableIdV1 {
    StableIdV1([byte; 32])
}

#[test]
fn group_order_and_duplicates_do_not_change_enrollment_commitment() {
    let first = ReaderEnrollmentV1::new(
        credential(1, 2),
        party(7),
        [group(9), group(8), group(9)],
    )
    .unwrap();
    let second =
        ReaderEnrollmentV1::new(credential(1, 2), party(7), [group(8), group(9)]).unwrap();

    assert_eq!(first.groups(), &[group(8), group(9)]);
    assert_eq!(first.id(), second.id());
}

#[test]
fn enrollment_reconstruction_recomputes_claimed_commitment() {
    let original = ReaderEnrollmentV1::new(credential(1, 2), party(7), [group(8)]).unwrap();
    let restored = ReaderEnrollmentV1::from_stable_id(
        credential(1, 2),
        party(7),
        [group(8)],
        original.id(),
    )
    .unwrap();
    assert_eq!(restored, original);

    assert!(matches!(
        ReaderEnrollmentV1::from_stable_id(
            credential(1, 2),
            party(7),
            [group(8)],
            StableIdV1([0xEE; 32]),
        ),
        Err(ReaderEnrollmentErrorV1::EnrollmentCommitmentMismatch)
    ));
}

#[test]
fn registry_reconstruction_recomputes_complete_snapshot_commitment() {
    let a = ReaderEnrollmentV1::new(credential(1, 2), party(7), [group(8)]).unwrap();
    let b = ReaderEnrollmentV1::new(credential(3, 4), party(9), [group(10)]).unwrap();
    let expected = ReaderEnrollmentRegistryV1::from_enrollments([a.clone(), b.clone()]).unwrap();

    let restored =
        reconstruct_reader_enrollment_registry_v1([b.clone(), a.clone()], expected.id()).unwrap();
    assert_eq!(restored, expected);

    assert!(matches!(
        reconstruct_reader_enrollment_registry_v1([a, b], StableIdV1([0xEE; 32])),
        Err(ReaderEnrollmentErrorV1::RegistryCommitmentMismatch)
    ));
}

#[test]
fn registry_commitment_changes_when_enrollment_authority_changes() {
    let first = ReaderEnrollmentRegistryV1::from_enrollments([
        ReaderEnrollmentV1::new(credential(1, 2), party(7), [group(8)]).unwrap(),
    ])
    .unwrap();
    let changed_groups = ReaderEnrollmentRegistryV1::from_enrollments([
        ReaderEnrollmentV1::new(credential(1, 2), party(7), [group(9)]).unwrap(),
    ])
    .unwrap();
    let changed_principal = ReaderEnrollmentRegistryV1::from_enrollments([
        ReaderEnrollmentV1::new(credential(1, 2), party(6), [group(8)]).unwrap(),
    ])
    .unwrap();

    assert_ne!(first.id(), changed_groups.id());
    assert_ne!(first.id(), changed_principal.id());
}

#[test]
fn malformed_lookup_does_not_degrade_to_partial_key_matching() {
    let registry = ReaderEnrollmentRegistryV1::from_enrollments([
        ReaderEnrollmentV1::new(credential(1, 2), party(7), []).unwrap(),
    ])
    .unwrap();

    assert!(matches!(
        registry.lookup_keys(&[1; 32], &[2; 32]),
        Err(ReaderEnrollmentErrorV1::InvalidMlDsa65PublicKeyLength { .. })
    ));
}
