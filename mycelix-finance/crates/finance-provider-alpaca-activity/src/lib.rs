#![forbid(unsafe_code)]

//! Pure Alpaca Activity SSE provider-identity admission.
//!
//! This crate deliberately owns no HTTP/SSE client, replay cursor, order
//! correlation, raw evidence commitment, credentials, clocks, persistence,
//! order effects, or FIN-MKT economic normalization.

use serde::{Deserialize, Serialize};

pub const ALPACA_ACTIVITY_SCHEMA_PROFILE_V1: &str = "alpaca-us-activity-sse-v2beta1";
const MAX_PROFILE_TOKEN_BYTES: usize = 64;
const MAX_ACTIVITY_TOKEN_BYTES: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum AlpacaDeploymentProfileV1 {
    TradingPaper,
    TradingLive,
    BrokerSandbox,
    BrokerProduction,
}

impl AlpacaDeploymentProfileV1 {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::TradingPaper => "trading-paper",
            Self::TradingLive => "trading-live",
            Self::BrokerSandbox => "broker-sandbox",
            Self::BrokerProduction => "broker-production",
        }
    }
}

/// Bounded identity-only projection supplied to the pure mapper.
///
/// Transport/publication identity (`event_id`), order correlation
/// (`details.order_id`), and raw/evidence commitments are deliberately absent.
/// A full provider payload therefore cannot be deserialized directly into this
/// type without first passing through an outer parser/projection boundary.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AlpacaActivityIdentityEvidenceV1 {
    pub deployment_profile: String,
    pub provider_schema_profile: String,
    pub account_id: String,
    pub activity_type: String,
    #[serde(default)]
    pub ref_id: Option<String>,
    #[serde(default)]
    pub previous_id: Option<String>,
    #[serde(default)]
    pub execution_type: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum AlpacaIdentityMappingErrorV1 {
    NotMarketExecution,
    ExecutionIdentityUnavailable,
    ExecutionIdentityAmbiguous,
    AdjustmentLineageUnavailable,
    ProviderProfileMismatch,
    MalformedIdentityEvidence { field: &'static str },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct AlpacaIdentityScopeV1 {
    provider_schema_profile: &'static str,
    deployment_profile: AlpacaDeploymentProfileV1,
    account_id: String,
}

impl AlpacaIdentityScopeV1 {
    pub const fn provider_schema_profile(&self) -> &'static str {
        self.provider_schema_profile
    }

    pub const fn deployment_profile(&self) -> AlpacaDeploymentProfileV1 {
        self.deployment_profile
    }

    pub fn account_id(&self) -> &str {
        &self.account_id
    }
}

/// A fill execution identity admitted by [`map_activity_identity_v1`].
///
/// Positive fields are private and this type intentionally does not implement
/// `Deserialize`; callers must rerun admission from bounded identity evidence.
///
/// ```compile_fail
/// use finance_provider_alpaca_activity::AdmittedAlpacaFillExecutionRefV1;
/// let _: AdmittedAlpacaFillExecutionRefV1 = serde_json::from_str("{}").unwrap();
/// ```
#[must_use]
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct AdmittedAlpacaFillExecutionRefV1 {
    scope: AlpacaIdentityScopeV1,
    provider_execution_ref: String,
}

impl AdmittedAlpacaFillExecutionRefV1 {
    pub fn scope(&self) -> &AlpacaIdentityScopeV1 {
        &self.scope
    }

    pub fn provider_execution_ref(&self) -> &str {
        &self.provider_execution_ref
    }
}

#[must_use]
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct AdmittedAlpacaCorrectionActivityRefV1 {
    scope: AlpacaIdentityScopeV1,
    provider_activity_ref: String,
    previous_activity_ref: String,
}

impl AdmittedAlpacaCorrectionActivityRefV1 {
    pub fn scope(&self) -> &AlpacaIdentityScopeV1 {
        &self.scope
    }

    pub fn provider_activity_ref(&self) -> &str {
        &self.provider_activity_ref
    }

    pub fn previous_activity_ref(&self) -> &str {
        &self.previous_activity_ref
    }
}

#[must_use]
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct AdmittedAlpacaBustActivityRefV1 {
    scope: AlpacaIdentityScopeV1,
    provider_activity_ref: String,
    previous_activity_ref: String,
}

impl AdmittedAlpacaBustActivityRefV1 {
    pub fn scope(&self) -> &AlpacaIdentityScopeV1 {
        &self.scope
    }

    pub fn provider_activity_ref(&self) -> &str {
        &self.provider_activity_ref
    }

    pub fn previous_activity_ref(&self) -> &str {
        &self.previous_activity_ref
    }
}

#[must_use]
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum AlpacaActivityIdentityMappingV1 {
    Fill(AdmittedAlpacaFillExecutionRefV1),
    Correction(AdmittedAlpacaCorrectionActivityRefV1),
    Bust(AdmittedAlpacaBustActivityRefV1),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TradeExecutionRoleV1 {
    Fill,
    Correction,
    Bust,
}

pub fn map_activity_identity_v1(
    evidence: &AlpacaActivityIdentityEvidenceV1,
) -> Result<AlpacaActivityIdentityMappingV1, AlpacaIdentityMappingErrorV1> {
    if evidence.provider_schema_profile.len() > MAX_PROFILE_TOKEN_BYTES {
        return Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
            field: "provider_schema_profile",
        });
    }
    if evidence.provider_schema_profile != ALPACA_ACTIVITY_SCHEMA_PROFILE_V1 {
        return Err(AlpacaIdentityMappingErrorV1::ProviderProfileMismatch);
    }

    let deployment_profile = parse_deployment_profile(&evidence.deployment_profile)?;
    let account_id = canonical_uuid(&evidence.account_id, "account_id")?;

    if evidence.activity_type.len() > MAX_ACTIVITY_TOKEN_BYTES {
        return Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
            field: "activity_type",
        });
    }
    if evidence.activity_type != "TRD" {
        return Err(AlpacaIdentityMappingErrorV1::NotMarketExecution);
    }

    let role = parse_execution_role(evidence.execution_type.as_deref())?;

    let ref_id = evidence
        .ref_id
        .as_deref()
        .ok_or(AlpacaIdentityMappingErrorV1::ExecutionIdentityUnavailable)
        .and_then(|value| canonical_uuid(value, "ref_id"))?;

    let scope = AlpacaIdentityScopeV1 {
        provider_schema_profile: ALPACA_ACTIVITY_SCHEMA_PROFILE_V1,
        deployment_profile,
        account_id,
    };

    match role {
        TradeExecutionRoleV1::Fill => {
            if evidence.previous_id.is_some() {
                return Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
                    field: "previous_id",
                });
            }
            Ok(AlpacaActivityIdentityMappingV1::Fill(
                AdmittedAlpacaFillExecutionRefV1 {
                    scope,
                    provider_execution_ref: ref_id,
                },
            ))
        }
        TradeExecutionRoleV1::Correction => {
            let previous_activity_ref = required_previous_id(evidence, &ref_id)?;
            Ok(AlpacaActivityIdentityMappingV1::Correction(
                AdmittedAlpacaCorrectionActivityRefV1 {
                    scope,
                    provider_activity_ref: ref_id,
                    previous_activity_ref,
                },
            ))
        }
        TradeExecutionRoleV1::Bust => {
            let previous_activity_ref = required_previous_id(evidence, &ref_id)?;
            Ok(AlpacaActivityIdentityMappingV1::Bust(
                AdmittedAlpacaBustActivityRefV1 {
                    scope,
                    provider_activity_ref: ref_id,
                    previous_activity_ref,
                },
            ))
        }
    }
}

fn parse_execution_role(
    value: Option<&str>,
) -> Result<TradeExecutionRoleV1, AlpacaIdentityMappingErrorV1> {
    let value = value.ok_or(AlpacaIdentityMappingErrorV1::ExecutionIdentityAmbiguous)?;
    if value.len() > MAX_ACTIVITY_TOKEN_BYTES {
        return Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
            field: "execution_type",
        });
    }
    match value {
        "fill" => Ok(TradeExecutionRoleV1::Fill),
        "trade_correct" => Ok(TradeExecutionRoleV1::Correction),
        "trade_bust" => Ok(TradeExecutionRoleV1::Bust),
        _ => Err(AlpacaIdentityMappingErrorV1::ExecutionIdentityAmbiguous),
    }
}

fn required_previous_id(
    evidence: &AlpacaActivityIdentityEvidenceV1,
    ref_id: &str,
) -> Result<String, AlpacaIdentityMappingErrorV1> {
    let previous = evidence
        .previous_id
        .as_deref()
        .ok_or(AlpacaIdentityMappingErrorV1::AdjustmentLineageUnavailable)
        .and_then(|value| canonical_uuid(value, "previous_id"))?;

    if previous == ref_id {
        return Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
            field: "previous_id",
        });
    }

    Ok(previous)
}

fn parse_deployment_profile(
    value: &str,
) -> Result<AlpacaDeploymentProfileV1, AlpacaIdentityMappingErrorV1> {
    if value.len() > MAX_PROFILE_TOKEN_BYTES {
        return Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
            field: "deployment_profile",
        });
    }
    match value {
        "trading-paper" => Ok(AlpacaDeploymentProfileV1::TradingPaper),
        "trading-live" => Ok(AlpacaDeploymentProfileV1::TradingLive),
        "broker-sandbox" => Ok(AlpacaDeploymentProfileV1::BrokerSandbox),
        "broker-production" => Ok(AlpacaDeploymentProfileV1::BrokerProduction),
        _ => Err(AlpacaIdentityMappingErrorV1::ProviderProfileMismatch),
    }
}

fn canonical_uuid(
    value: &str,
    field: &'static str,
) -> Result<String, AlpacaIdentityMappingErrorV1> {
    let bytes = value.as_bytes();
    if bytes.len() != 36 {
        return Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence { field });
    }

    for (index, byte) in bytes.iter().copied().enumerate() {
        let is_hyphen = matches!(index, 8 | 13 | 18 | 23);
        if is_hyphen {
            if byte != b'-' {
                return Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence { field });
            }
        } else if !byte.is_ascii_hexdigit() {
            return Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence { field });
        }
    }

    Ok(value.to_ascii_lowercase())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json, Value};

    const ACCOUNT_A: &str = "11111111-1111-4111-8111-111111111111";
    const REF_A: &str = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaa1";

    fn evidence() -> AlpacaActivityIdentityEvidenceV1 {
        AlpacaActivityIdentityEvidenceV1 {
            deployment_profile: "trading-paper".into(),
            provider_schema_profile: ALPACA_ACTIVITY_SCHEMA_PROFILE_V1.into(),
            account_id: ACCOUNT_A.into(),
            activity_type: "TRD".into(),
            ref_id: Some(REF_A.into()),
            previous_id: None,
            execution_type: Some("fill".into()),
        }
    }

    fn outcome_name(
        result: &Result<AlpacaActivityIdentityMappingV1, AlpacaIdentityMappingErrorV1>,
    ) -> &'static str {
        match result {
            Ok(AlpacaActivityIdentityMappingV1::Fill(_)) => "StableFillExecutionIdentityAdmitted",
            Ok(AlpacaActivityIdentityMappingV1::Correction(_)) => {
                "StableCorrectionActivityIdentityAdmitted"
            }
            Ok(AlpacaActivityIdentityMappingV1::Bust(_)) => {
                "StableBustActivityIdentityAdmitted"
            }
            Err(AlpacaIdentityMappingErrorV1::NotMarketExecution) => "NotMarketExecution",
            Err(AlpacaIdentityMappingErrorV1::ExecutionIdentityUnavailable) => {
                "ExecutionIdentityUnavailable"
            }
            Err(AlpacaIdentityMappingErrorV1::ExecutionIdentityAmbiguous) => {
                "ExecutionIdentityAmbiguous"
            }
            Err(AlpacaIdentityMappingErrorV1::AdjustmentLineageUnavailable) => {
                "AdjustmentLineageUnavailable"
            }
            Err(AlpacaIdentityMappingErrorV1::ProviderProfileMismatch) => {
                "ProviderProfileMismatch"
            }
            Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence { .. }) => {
                "MalformedIdentityEvidence"
            }
        }
    }

    #[test]
    fn exact_frozen_identity_corpus_is_reconstructed() {
        let corpus: Value = serde_json::from_str(include_str!(
            "../../../docs/providers/fixtures/FIN_ALPACA_EXECID_001_V0_1.json"
        ))
        .expect("frozen corpus must be valid JSON");

        let cases = corpus["cases"].as_array().expect("cases array");
        let mut mapped = 0usize;

        for case in cases {
            if case["layer"].as_str() != Some("identity-mapper") {
                continue;
            }
            mapped += 1;
            let event = &case["event"];
            let details = &event["details"];
            let evidence = AlpacaActivityIdentityEvidenceV1 {
                deployment_profile: case["deployment_profile"]
                    .as_str()
                    .expect("deployment profile")
                    .to_owned(),
                provider_schema_profile: ALPACA_ACTIVITY_SCHEMA_PROFILE_V1.to_owned(),
                account_id: event["account_id"].as_str().expect("account_id").to_owned(),
                activity_type: event["activity_type"]
                    .as_str()
                    .expect("activity_type")
                    .to_owned(),
                ref_id: event["ref_id"].as_str().map(str::to_owned),
                previous_id: event["previous_id"].as_str().map(str::to_owned),
                execution_type: details["execution_type"].as_str().map(str::to_owned),
            };

            let result = map_activity_identity_v1(&evidence);
            let expected = case["expect"]["outcome"].as_str().expect("expected outcome");
            assert_eq!(outcome_name(&result), expected, "case {}", case["id"]);

            match result {
                Ok(AlpacaActivityIdentityMappingV1::Fill(fill)) => {
                    if let Some(expected_ref) = case["expect"]["provider_execution_ref"].as_str() {
                        assert_eq!(fill.provider_execution_ref(), expected_ref);
                    }
                }
                Ok(AlpacaActivityIdentityMappingV1::Correction(correction)) => {
                    assert_eq!(
                        correction.provider_activity_ref(),
                        case["expect"]["provider_activity_ref"].as_str().unwrap()
                    );
                    assert_eq!(
                        correction.previous_activity_ref(),
                        case["expect"]["previous_activity_ref"].as_str().unwrap()
                    );
                }
                Ok(AlpacaActivityIdentityMappingV1::Bust(bust)) => {
                    assert_eq!(
                        bust.provider_activity_ref(),
                        case["expect"]["provider_activity_ref"].as_str().unwrap()
                    );
                    assert_eq!(
                        bust.previous_activity_ref(),
                        case["expect"]["previous_activity_ref"].as_str().unwrap()
                    );
                }
                Err(_) => {}
            }
        }

        assert_eq!(mapped, 14, "stream-control case must stay outside mapper");
    }

    #[test]
    fn transport_order_and_raw_evidence_fields_are_not_identity_input() {
        let with_transport_fields = json!({
            "deployment_profile": "trading-paper",
            "provider_schema_profile": ALPACA_ACTIVITY_SCHEMA_PROFILE_V1,
            "account_id": ACCOUNT_A,
            "activity_type": "TRD",
            "ref_id": REF_A,
            "execution_type": "fill",
            "event_id": "01K9AAAA000000000000000001",
            "order_id": "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbb1",
            "source_evidence_commitment": "deadbeef"
        });
        assert!(serde_json::from_value::<AlpacaActivityIdentityEvidenceV1>(with_transport_fields).is_err());
    }

    #[test]
    fn deployment_profile_is_part_of_identity_scope() {
        let paper = map_activity_identity_v1(&evidence()).unwrap();
        let mut broker = evidence();
        broker.deployment_profile = "broker-sandbox".into();
        let broker = map_activity_identity_v1(&broker).unwrap();
        assert_ne!(paper, broker);
    }

    #[test]
    fn account_is_part_of_identity_scope() {
        let account_a = map_activity_identity_v1(&evidence()).unwrap();
        let mut other = evidence();
        other.account_id = "22222222-2222-4222-8222-222222222222".into();
        let account_b = map_activity_identity_v1(&other).unwrap();
        assert_ne!(account_a, account_b);
    }

    #[test]
    fn uuid_text_is_canonicalized_without_changing_identity() {
        let lower = map_activity_identity_v1(&evidence()).unwrap();
        let mut upper = evidence();
        upper.account_id = ACCOUNT_A.to_ascii_uppercase();
        upper.ref_id = Some(REF_A.to_ascii_uppercase());
        let upper = map_activity_identity_v1(&upper).unwrap();
        assert_eq!(lower, upper);
    }

    #[test]
    fn missing_ref_never_falls_back_to_any_other_field() {
        let mut missing = evidence();
        missing.ref_id = None;
        assert_eq!(
            map_activity_identity_v1(&missing),
            Err(AlpacaIdentityMappingErrorV1::ExecutionIdentityUnavailable)
        );
    }

    #[test]
    fn unknown_execution_type_precedes_missing_ref_fallback_logic() {
        let mut unknown = evidence();
        unknown.execution_type = Some("future_unknown".into());
        unknown.ref_id = None;
        assert_eq!(
            map_activity_identity_v1(&unknown),
            Err(AlpacaIdentityMappingErrorV1::ExecutionIdentityAmbiguous)
        );
    }

    #[test]
    fn adjustment_requires_distinct_exact_predecessor() {
        let mut correction = evidence();
        correction.execution_type = Some("trade_correct".into());
        correction.previous_id = None;
        assert_eq!(
            map_activity_identity_v1(&correction),
            Err(AlpacaIdentityMappingErrorV1::AdjustmentLineageUnavailable)
        );

        correction.previous_id = correction.ref_id.clone();
        assert_eq!(
            map_activity_identity_v1(&correction),
            Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
                field: "previous_id"
            })
        );
    }

    #[test]
    fn ordinary_fill_rejects_unexpected_adjustment_lineage() {
        let mut fill = evidence();
        fill.previous_id = Some("aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaa2".into());
        assert_eq!(
            map_activity_identity_v1(&fill),
            Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
                field: "previous_id"
            })
        );
    }

    #[test]
    fn unknown_schema_or_deployment_fails_closed() {
        let mut wrong_schema = evidence();
        wrong_schema.provider_schema_profile = "future-ga".into();
        assert_eq!(
            map_activity_identity_v1(&wrong_schema),
            Err(AlpacaIdentityMappingErrorV1::ProviderProfileMismatch)
        );

        let mut wrong_deployment = evidence();
        wrong_deployment.deployment_profile = "mystery".into();
        assert_eq!(
            map_activity_identity_v1(&wrong_deployment),
            Err(AlpacaIdentityMappingErrorV1::ProviderProfileMismatch)
        );
    }

    #[test]
    fn oversized_profile_and_activity_tokens_fail_closed() {
        let mut oversized_profile = evidence();
        oversized_profile.deployment_profile = "x".repeat(MAX_PROFILE_TOKEN_BYTES + 1);
        assert_eq!(
            map_activity_identity_v1(&oversized_profile),
            Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
                field: "deployment_profile"
            })
        );

        let mut oversized_execution = evidence();
        oversized_execution.execution_type = Some("x".repeat(MAX_ACTIVITY_TOKEN_BYTES + 1));
        assert_eq!(
            map_activity_identity_v1(&oversized_execution),
            Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
                field: "execution_type"
            })
        );
    }

    #[test]
    fn bounded_malformed_ids_fail_without_panicking() {
        for bad in [
            "",
            "not-a-uuid",
            "00000000-0000-0000-0000-00000000000z",
            "x",
        ] {
            let mut malformed = evidence();
            malformed.ref_id = Some(bad.to_string());
            assert!(matches!(
                map_activity_identity_v1(&malformed),
                Err(AlpacaIdentityMappingErrorV1::MalformedIdentityEvidence {
                    field: "ref_id"
                })
            ));
        }
    }
}
