use std::collections::BTreeSet;

use mycelix_content_core::*;
use mycelix_infrastructure_types::{PartyIdV1, UnixMillisV1};

fn j(value: &str) -> JurisdictionV1 {
    JurisdictionV1::new(value).unwrap()
}

fn prefs() -> PlacementPreferencesV1 {
    PlacementPreferencesV1::new(Some(50), 10, 20, 5, 10).unwrap()
}

#[test]
fn forbidden_jurisdiction_cannot_be_optimized_back_in() {
    let requirements = PlacementRequirementsV1::new(
        3,
        FailureDomainPolicyV1::new([(FailureDomainKindV1::Operator, 3)]).unwrap(),
        BTreeSet::new(),
        BTreeSet::from([j("US")]),
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    )
    .unwrap();

    assert!(!requirements.jurisdiction_allowed(&j("US")));
    assert!(requirements.jurisdiction_allowed(&j("ZA")));
}

#[test]
fn failure_domain_minimum_cannot_exceed_replicas() {
    let err = PlacementRequirementsV1::new(
        2,
        FailureDomainPolicyV1::new([(FailureDomainKindV1::Site, 3)]).unwrap(),
        BTreeSet::new(),
        BTreeSet::new(),
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    )
    .unwrap_err();
    assert_eq!(err, ContentErrorV1::FailureDomainExceedsReplicas);
}

#[test]
fn same_jurisdiction_cannot_be_allowed_and_forbidden() {
    let err = PlacementRequirementsV1::new(
        1,
        FailureDomainPolicyV1::empty(),
        BTreeSet::from([j("ZA")]),
        BTreeSet::from([j("ZA")]),
        EncryptionRequirementV1::NotRequired,
        RetentionRequirementV1::BestEffort,
    )
    .unwrap_err();
    assert_eq!(err, ContentErrorV1::JurisdictionConflict);
}

#[test]
fn private_backup_requires_client_side_encryption() {
    let requirements = PlacementRequirementsV1::new(
        2,
        FailureDomainPolicyV1::new([(FailureDomainKindV1::Site, 2)]).unwrap(),
        BTreeSet::new(),
        BTreeSet::new(),
        EncryptionRequirementV1::ProviderAtRest,
        RetentionRequirementV1::Indefinite,
    )
    .unwrap();

    let err = StorageIntentV1::new(
        ObjectIdV1([7; 32]),
        PartyIdV1([1; 32]),
        StorageClassV1::PrivateBackup,
        requirements,
        prefs(),
        UnixMillisV1(100),
    )
    .unwrap_err();
    assert_eq!(err, ContentErrorV1::PrivateBackupRequiresClientEncryption);
}

#[test]
fn hard_policy_mutation_invalidates_intent_id() {
    let requirements = PlacementRequirementsV1::new(
        3,
        FailureDomainPolicyV1::new([(FailureDomainKindV1::Operator, 3)]).unwrap(),
        BTreeSet::from([j("ZA")]),
        BTreeSet::new(),
        EncryptionRequirementV1::ClientSide,
        RetentionRequirementV1::MinimumSeconds(3600),
    )
    .unwrap();
    let mut intent = StorageIntentV1::new(
        ObjectIdV1([7; 32]),
        PartyIdV1([1; 32]),
        StorageClassV1::Durable,
        requirements,
        prefs(),
        UnixMillisV1(100),
    )
    .unwrap();
    let original = intent.id;
    intent.requirements.forbidden_jurisdictions.insert(j("US"));
    assert_ne!(intent.recompute_id().unwrap(), original);
    assert_eq!(intent.validate().unwrap_err(), ContentErrorV1::IdMismatch);
}

#[test]
fn jurisdiction_must_start_with_uppercase_letter() {
    assert_eq!(
        JurisdictionV1::new("1A").unwrap_err(),
        ContentErrorV1::InvalidJurisdiction,
    );
}

#[test]
fn failure_domain_policy_is_order_independent() {
    let left = FailureDomainPolicyV1::new([
        (FailureDomainKindV1::Operator, 3),
        (FailureDomainKindV1::Site, 2),
    ])
    .unwrap();
    let right = FailureDomainPolicyV1::new([
        (FailureDomainKindV1::Site, 2),
        (FailureDomainKindV1::Operator, 3),
    ])
    .unwrap();
    assert_eq!(left, right);
}
