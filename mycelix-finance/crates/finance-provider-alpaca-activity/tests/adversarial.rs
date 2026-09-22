use finance_provider_alpaca_activity::{
    map_activity_identity_v1, AlpacaActivityIdentityEvidenceV1,
    AlpacaActivityIdentityMappingV1, AlpacaIdentityMappingErrorV1,
    ALPACA_ACTIVITY_SCHEMA_PROFILE_V1,
};

const ACCOUNT: &str = "11111111-1111-4111-8111-111111111111";
const REF: &str = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaa1";
const PREVIOUS: &str = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaa2";

fn base() -> AlpacaActivityIdentityEvidenceV1 {
    AlpacaActivityIdentityEvidenceV1 {
        deployment_profile: "trading-paper".into(),
        provider_schema_profile: ALPACA_ACTIVITY_SCHEMA_PROFILE_V1.into(),
        account_id: ACCOUNT.into(),
        activity_type: "TRD".into(),
        ref_id: Some(REF.into()),
        previous_id: None,
        execution_type: Some("fill".into()),
    }
}

#[test]
fn malformed_account_uuid_fails_closed() {
    let mut input = base();
    input.account_id = "not-an-account-uuid".into();
    assert_eq!(
        map_activity_identity_v1(&input),
        Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
            field: "account_id"
        })
    );
}

#[test]
fn malformed_previous_uuid_fails_closed() {
    let mut input = base();
    input.execution_type = Some("trade_correct".into());
    input.previous_id = Some("not-a-previous-uuid".into());
    assert_eq!(
        map_activity_identity_v1(&input),
        Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
            field: "previous_id"
        })
    );
}

#[test]
fn missing_or_empty_execution_type_is_never_a_fill() {
    let mut missing = base();
    missing.execution_type = None;
    assert_eq!(
        map_activity_identity_v1(&missing),
        Err(AlpacaIdentityMappingErrorV1::ExecutionIdentityAmbiguous)
    );

    let mut empty = base();
    empty.execution_type = Some(String::new());
    assert_eq!(
        map_activity_identity_v1(&empty),
        Err(AlpacaIdentityMappingErrorV1::ExecutionIdentityAmbiguous)
    );
}

#[test]
fn every_reviewed_deployment_profile_is_exact_and_scope_distinct() {
    let mut admitted = Vec::new();
    for profile in [
        "trading-paper",
        "trading-live",
        "broker-sandbox",
        "broker-production",
    ] {
        let mut input = base();
        input.deployment_profile = profile.into();
        let mapped = map_activity_identity_v1(&input).unwrap();
        assert!(matches!(mapped, AlpacaActivityIdentityMappingV1::Fill(_)));
        admitted.push(mapped);
    }

    for left in 0..admitted.len() {
        for right in (left + 1)..admitted.len() {
            assert_ne!(admitted[left], admitted[right]);
        }
    }

    for unreviewed in ["paper", "live", "sandbox", "production", "TRADING-PAPER"] {
        let mut input = base();
        input.deployment_profile = unreviewed.into();
        assert_eq!(
            map_activity_identity_v1(&input),
            Err(AlpacaIdentityMappingErrorV1::ProviderProfileMismatch)
        );
    }
}

#[test]
fn uuid_length_and_separator_boundaries_fail_closed() {
    for bad in [
        "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaa",
        "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaaa",
        "aaaaaaaa_aaaa-4aaa-8aaa-aaaaaaaaaaaa",
    ] {
        let mut input = base();
        input.ref_id = Some(bad.into());
        assert!(matches!(
            map_activity_identity_v1(&input),
            Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
                field: "ref_id"
            })
        ));
    }
}

#[test]
fn correction_and_bust_are_not_fill_variants() {
    for execution_type in ["trade_correct", "trade_bust"] {
        let mut input = base();
        input.execution_type = Some(execution_type.into());
        input.previous_id = Some(PREVIOUS.into());
        let mapped = map_activity_identity_v1(&input).unwrap();
        assert!(!matches!(mapped, AlpacaActivityIdentityMappingV1::Fill(_)));
    }
}

#[test]
fn bust_self_reference_fails_closed() {
    let mut input = base();
    input.execution_type = Some("trade_bust".into());
    input.previous_id = input.ref_id.clone();
    assert_eq!(
        map_activity_identity_v1(&input),
        Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
            field: "previous_id"
        })
    );
}

#[test]
fn unknown_future_execution_type_is_ambiguous_even_without_ref() {
    let mut input = base();
    input.execution_type = Some("provider_future_extension".into());
    input.ref_id = None;
    assert_eq!(
        map_activity_identity_v1(&input),
        Err(AlpacaIdentityMappingErrorV1::ExecutionIdentityAmbiguous)
    );
}
