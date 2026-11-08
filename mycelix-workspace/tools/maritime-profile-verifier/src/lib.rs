//! Effects-disabled checks for caller-supplied Maritime Commons profile evidence.
//!
//! This crate validates consistency and pin binding. It is not a credential
//! verifier, registry client, jurisdictional law engine, or provider adapter.

use chrono::{DateTime, NaiveDateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Disposition {
    Pass,
    Reject,
    Conflict,
    Unknown,
    NotEvaluable,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct Check {
    pub disposition: Disposition,
    pub code: String,
    pub detail: String,
}

impl Check {
    pub fn new(disposition: Disposition, code: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            disposition,
            code: code.into(),
            detail: detail.into(),
        }
    }

    fn pass(code: &str, detail: &str) -> Self {
        Self::new(Disposition::Pass, code, detail)
    }

    fn reject(code: &str, detail: &str) -> Self {
        Self::new(Disposition::Reject, code, detail)
    }

    fn conflict(code: &str, detail: &str) -> Self {
        Self::new(Disposition::Conflict, code, detail)
    }

    fn unknown(code: &str, detail: &str) -> Self {
        Self::new(Disposition::Unknown, code, detail)
    }

    fn not_evaluable(code: &str, detail: &str) -> Self {
        Self::new(Disposition::NotEvaluable, code, detail)
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ValidationReport {
    pub disposition: Disposition,
    pub checks: Vec<Check>,
}

impl ValidationReport {
    pub fn from_checks(checks: Vec<Check>) -> Self {
        let disposition = checks
            .iter()
            .map(|check| check.disposition)
            .max_by_key(|value| disposition_rank(*value))
            .unwrap_or(Disposition::NotEvaluable);

        Self {
            disposition,
            checks,
        }
    }

    pub fn is_pass(&self) -> bool {
        self.disposition == Disposition::Pass
    }
}

fn disposition_rank(value: Disposition) -> u8 {
    match value {
        Disposition::Pass => 0,
        Disposition::NotEvaluable => 1,
        Disposition::Unknown => 2,
        Disposition::Reject => 3,
        Disposition::Conflict => 4,
    }
}

fn sha256_digest_is_well_formed(value: &str) -> bool {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return false;
    };

    hex.len() == 64
        && hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn parse_rfc3339(value: &str) -> Option<DateTime<chrono::FixedOffset>> {
    DateTime::parse_from_rfc3339(value).ok()
}

fn parse_local_lexeme(value: &str) -> Option<NaiveDateTime> {
    const FORMATS: [&str; 2] = ["%Y-%m-%dT%H:%M:%S%.f", "%Y-%m-%d %H:%M:%S%.f"];

    FORMATS
        .iter()
        .find_map(|format| NaiveDateTime::parse_from_str(value, format).ok())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArtifactMaturity {
    Production,
    PreRelease,
    Draft,
    Deprecated,
    Withdrawn,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ArtifactPin {
    pub authority: String,
    pub family: String,
    pub identifier: String,
    pub version: String,
    pub maturity: ArtifactMaturity,
    pub sha256: String,
}

pub fn evaluate_artifact_pin(
    expected: &ArtifactPin,
    observed: Option<&ArtifactPin>,
) -> ValidationReport {
    let Some(observed) = observed else {
        return ValidationReport::from_checks(vec![Check::unknown(
            "artifact_pin_missing",
            "No artifact identity was supplied; the standard family name alone is insufficient.",
        )]);
    };

    let mut checks = Vec::new();

    if !sha256_digest_is_well_formed(&expected.sha256)
        || !sha256_digest_is_well_formed(&observed.sha256)
    {
        checks.push(Check::reject(
            "artifact_digest_malformed",
            "Both expected and observed artifact digests must use sha256 followed by 64 lowercase hexadecimal characters.",
        ));
    }

    if [
        (&expected.authority, &observed.authority),
        (&expected.family, &observed.family),
        (&expected.identifier, &observed.identifier),
        (&expected.version, &observed.version),
        (&expected.sha256, &observed.sha256),
    ]
    .iter()
    .any(|(left, right)| left != right)
    {
        checks.push(Check::conflict(
            "artifact_identity_mismatch",
            "Authority, family, identifier, version, and content digest must match the pinned artifact exactly.",
        ));
    }

    if expected.maturity != observed.maturity {
        checks.push(Check::conflict(
            "artifact_maturity_mismatch",
            "Production, pre-release, draft, deprecated, and withdrawn artifacts are distinct profile identities.",
        ));
    }

    if observed.maturity == ArtifactMaturity::Unknown {
        checks.push(Check::unknown(
            "artifact_maturity_unknown",
            "The observed artifact maturity is not established.",
        ));
    }

    if checks.is_empty() {
        checks.push(Check::pass(
            "artifact_pin_matches",
            "The supplied artifact identity, digest, version, and maturity match the expected pin.",
        ));
    }

    ValidationReport::from_checks(checks)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RecognitionDisposition {
    Recognized,
    NotRecognized,
    Unknown,
    NotAssessed,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ExternalIdentifier {
    pub scheme: String,
    pub namespace: String,
    pub scheme_version: String,
    pub issuer: String,
    pub subject_type: String,
    pub value: String,
    pub jurisdiction: Option<String>,
    pub purpose: String,
    pub valid_from: Option<String>,
    pub valid_until: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct IdentifierContext {
    pub expected_subject_type: String,
    pub expected_jurisdiction: Option<String>,
    pub expected_purpose: String,
    pub as_of: String,
    pub issuer_recognition: RecognitionDisposition,
}

pub fn evaluate_identifier(
    identifier: &ExternalIdentifier,
    context: &IdentifierContext,
) -> ValidationReport {
    let mut checks = Vec::new();

    let required = [
        ("scheme", identifier.scheme.as_str()),
        ("namespace", identifier.namespace.as_str()),
        ("scheme_version", identifier.scheme_version.as_str()),
        ("issuer", identifier.issuer.as_str()),
        ("subject_type", identifier.subject_type.as_str()),
        ("value", identifier.value.as_str()),
        ("purpose", identifier.purpose.as_str()),
    ];

    if required.iter().all(|(_, value)| non_empty(value)) {
        checks.push(Check::pass(
            "identifier_qualified",
            "The identifier has non-empty scheme, namespace, version, issuer, subject, value, and purpose fields.",
        ));
    } else {
        checks.push(Check::reject(
            "identifier_scope_incomplete",
            "A bare string, display name, or identifier without scheme and issuer scope is not a qualified identifier.",
        ));
    }

    if identifier.subject_type == context.expected_subject_type {
        checks.push(Check::pass(
            "identifier_subject_matches",
            "The identifier subject type matches the requested subject.",
        ));
    } else {
        checks.push(Check::reject(
            "identifier_subject_mismatch",
            "The identifier is not bound to the required subject type.",
        ));
    }

    match context.expected_jurisdiction.as_deref() {
        Some(expected) if identifier.jurisdiction.as_deref() == Some(expected) => checks.push(
            Check::pass(
                "identifier_jurisdiction_matches",
                "The identifier jurisdiction matches the requested jurisdiction.",
            ),
        ),
        Some(_) => checks.push(Check::reject(
            "identifier_jurisdiction_mismatch",
            "The identifier does not carry the required jurisdiction scope.",
        )),
        None => checks.push(Check::not_evaluable(
            "identifier_jurisdiction_not_requested",
            "No jurisdiction constraint was supplied; this check makes no jurisdictional claim.",
        )),
    }

    if identifier.purpose == context.expected_purpose {
        checks.push(Check::pass(
            "identifier_purpose_matches",
            "The identifier purpose matches the requested purpose.",
        ));
    } else {
        checks.push(Check::reject(
            "identifier_purpose_mismatch",
            "An identifier valid for one purpose must not be reused for another purpose implicitly.",
        ));
    }

    match parse_rfc3339(&context.as_of) {
        None => checks.push(Check::reject(
            "identifier_evaluation_time_invalid",
            "The evaluation time must be an RFC 3339 timestamp with an explicit offset.",
        )),
        Some(as_of) => {
            let valid_from = match identifier.valid_from.as_deref() {
                Some(value) => parse_rfc3339(value),
                None => None,
            };
            let valid_until = match identifier.valid_until.as_deref() {
                Some(value) => parse_rfc3339(value),
                None => None,
            };

            if identifier.valid_from.is_some() && valid_from.is_none() {
                checks.push(Check::reject(
                    "identifier_valid_from_invalid",
                    "The identifier lower validity bound is not a valid RFC 3339 timestamp.",
                ));
            } else if valid_from.is_some_and(|start| as_of < start) {
                checks.push(Check::reject(
                    "identifier_not_yet_valid",
                    "The identifier is not valid at the requested evaluation time.",
                ));
            }

            if identifier.valid_until.is_some() && valid_until.is_none() {
                checks.push(Check::reject(
                    "identifier_valid_until_invalid",
                    "The identifier upper validity bound is not a valid RFC 3339 timestamp.",
                ));
            } else if valid_until.is_some_and(|end| as_of >= end) {
                checks.push(Check::reject(
                    "identifier_expired",
                    "The identifier validity interval uses an exclusive upper bound and has expired.",
                ));
            }

            if (identifier.valid_from.is_none() || valid_from.is_some())
                && (identifier.valid_until.is_none() || valid_until.is_some())
                && valid_from.is_none_or(|start| as_of >= start)
                && valid_until.is_none_or(|end| as_of < end)
            {
                checks.push(Check::pass(
                    "identifier_validity_interval",
                    "The identifier is within its declared validity interval at the supplied evaluation time.",
                ));
            }
        }
    }

    checks.push(recognition_check(context.issuer_recognition));

    ValidationReport::from_checks(checks)
}

fn recognition_check(recognition: RecognitionDisposition) -> Check {
    match recognition {
        RecognitionDisposition::Recognized => Check::pass(
            "issuer_recognition_supplied",
            "The caller supplied an independently evaluated recognized-issuer disposition.",
        ),
        RecognitionDisposition::NotRecognized => Check::reject(
            "issuer_not_recognized",
            "The issuer is explicitly not recognized for this context.",
        ),
        RecognitionDisposition::Unknown => Check::unknown(
            "issuer_recognition_unknown",
            "The issuer recognition state cannot be established from the supplied evidence.",
        ),
        RecognitionDisposition::NotAssessed => Check::not_evaluable(
            "issuer_recognition_not_assessed",
            "This crate cannot infer issuer recognition from an issuer string or identifier value.",
        ),
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct CurrencyMetadata {
    pub artifact_id: String,
    pub release: String,
    pub sha256: String,
    pub currency_code: String,
    pub minor_unit_exponent: u8,
    pub effective_from: String,
    pub effective_until: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct MoneyAmount {
    pub currency_code: String,
    pub amount_minor: u128,
    pub metadata_artifact_id: String,
    pub metadata_release: String,
    pub metadata_sha256: String,
    pub minor_unit_exponent: u8,
    pub effective_at: String,
}

pub fn evaluate_money(
    amount: &MoneyAmount,
    metadata: Option<&CurrencyMetadata>,
) -> ValidationReport {
    let mut checks = Vec::new();

    let code_valid = amount.currency_code.len() == 3
        && amount
            .currency_code
            .bytes()
            .all(|byte| byte.is_ascii_uppercase());

    if code_valid {
        checks.push(Check::pass(
            "currency_code_shape",
            "The currency code has three uppercase ASCII letters.",
        ));
    } else {
        checks.push(Check::reject(
            "currency_code_shape_invalid",
            "The currency code must use the three-letter uppercase form declared by the profile.",
        ));
    }

    if amount.metadata_sha256 == "sha256:".to_owned() || !sha256_digest_is_well_formed(&amount.metadata_sha256) {
        checks.push(Check::reject(
            "currency_metadata_digest_invalid",
            "The amount must carry a sha256 digest for the exact currency metadata artifact.",
        ));
    }

    let Some(metadata) = metadata else {
        checks.push(Check::unknown(
            "currency_metadata_missing",
            "No pinned currency metadata snapshot was supplied; the amount's major-unit meaning cannot be established.",
        ));
        return ValidationReport::from_checks(checks);
    };

    if !sha256_digest_is_well_formed(&metadata.sha256) {
        checks.push(Check::reject(
            "currency_metadata_artifact_digest_invalid",
            "The supplied currency metadata artifact digest is malformed.",
        ));
    }

    if amount.metadata_artifact_id != metadata.artifact_id
        || amount.metadata_release != metadata.release
        || amount.metadata_sha256 != metadata.sha256
    {
        checks.push(Check::conflict(
            "currency_metadata_pin_mismatch",
            "Artifact ID, release, and digest must match the exact caller-pinned currency metadata snapshot.",
        ));
    } else {
        checks.push(Check::pass(
            "currency_metadata_pin_matches",
            "The amount references the exact pinned currency metadata artifact.",
        ));
    }

    if amount.currency_code != metadata.currency_code
        || amount.minor_unit_exponent != metadata.minor_unit_exponent
    {
        checks.push(Check::conflict(
            "currency_scale_mismatch",
            "Currency code and minor-unit exponent disagree with the pinned metadata snapshot.",
        ));
    } else {
        checks.push(Check::pass(
            "currency_scale_matches",
            "Currency code and minor-unit exponent match the pinned metadata snapshot.",
        ));
    }

    match parse_rfc3339(&amount.effective_at) {
        None => checks.push(Check::reject(
            "money_effective_time_invalid",
            "The monetary effective time must be an RFC 3339 timestamp with an explicit offset.",
        )),
        Some(effective_at) => {
            let start = parse_rfc3339(&metadata.effective_from);
            let end = metadata
                .effective_until
                .as_deref()
                .map(parse_rfc3339)
                .transpose_option();

            if start.is_none() || (metadata.effective_until.is_some() && end.is_none()) {
                checks.push(Check::unknown(
                    "currency_metadata_effectivity_unknown",
                    "The metadata effective interval is incomplete or invalid.",
                ));
            } else {
                let start = start.expect("checked above");
                if effective_at < start || end.is_some_and(|until| effective_at >= until) {
                    checks.push(Check::reject(
                        "currency_metadata_not_effective",
                        "The pinned currency metadata is not effective at the amount's declared transaction time.",
                    ));
                } else {
                    checks.push(Check::pass(
                        "currency_metadata_effective",
                        "The pinned currency metadata is effective at the declared transaction time.",
                    ));
                }
            }
        }
    }

    ValidationReport::from_checks(checks)
}

// Converts Option<Option<T>> to Option<T> only when the outer Option is Some.
// A malformed supplied end date remains None and is handled as an unknown interval.
trait TransposeOption<T> {
    fn transpose_option(self) -> Option<T>;
}

impl<T> TransposeOption<T> for Option<Option<T>> {
    fn transpose_option(self) -> Option<T> {
        self.flatten()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LocalTimeResolution {
    Unique,
    FoldEarlier,
    FoldLater,
    GapRejected,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct TemporalEvidence {
    pub source_local_lexeme: String,
    pub source_timestamp: String,
    pub time_zone_id: String,
    pub tzdb_version: String,
    pub resolution: LocalTimeResolution,
    pub resolver_evidence_ref: Option<String>,
    pub resolved_utc: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct TemporalContext {
    pub expected_tzdb_version: String,
}

pub fn evaluate_temporal(
    evidence: &TemporalEvidence,
    context: &TemporalContext,
) -> ValidationReport {
    let mut checks = Vec::new();

    if non_empty(&evidence.time_zone_id) {
        checks.push(Check::pass(
            "time_zone_identifier_present",
            "An IANA-style time-zone identifier was supplied.",
        ));
    } else {
        checks.push(Check::unknown(
            "time_zone_identifier_missing",
            "A numeric UTC offset alone cannot identify the source civil-time rule set.",
        ));
    }

    if non_empty(&context.expected_tzdb_version)
        && evidence.tzdb_version == context.expected_tzdb_version
    {
        checks.push(Check::pass(
            "tzdb_version_matches",
            "The temporal evidence names the exact expected time-zone database version.",
        ));
    } else if !non_empty(&evidence.tzdb_version) || !non_empty(&context.expected_tzdb_version) {
        checks.push(Check::unknown(
            "tzdb_version_missing",
            "The source or expected tzdb version is missing.",
        ));
    } else {
        checks.push(Check::conflict(
            "tzdb_version_mismatch",
            "Civil-time interpretation must not silently substitute a different tzdb release.",
        ));
    }

    let source = parse_rfc3339(&evidence.source_timestamp);
    let local = parse_local_lexeme(&evidence.source_local_lexeme);

    match (source, local) {
        (Some(source), Some(local)) if source.naive_local() == local => checks.push(Check::pass(
            "source_local_offset_consistent",
            "The preserved local-time lexeme agrees with the local component of the source timestamp and explicit offset.",
        )),
        (Some(_), Some(_)) => checks.push(Check::reject(
            "source_local_offset_mismatch",
            "The local-time lexeme and source timestamp describe different local wall times.",
        )),
        _ => checks.push(Check::reject(
            "source_temporal_value_invalid",
            "The local lexeme or offset-bearing source timestamp cannot be parsed.",
        )),
    }

    match evidence.resolution {
        LocalTimeResolution::GapRejected => checks.push(Check::reject(
            "nonexistent_local_time",
            "The resolver identified a local civil time inside a time-zone gap.",
        )),
        LocalTimeResolution::Unknown => checks.push(Check::unknown(
            "local_time_resolution_unknown",
            "The local time's unique/fold/gap disposition is not established.",
        )),
        LocalTimeResolution::Unique
        | LocalTimeResolution::FoldEarlier
        | LocalTimeResolution::FoldLater => {
            if evidence
                .resolver_evidence_ref
                .as_deref()
                .is_some_and(non_empty)
            {
                checks.push(Check::pass(
                    "local_time_resolution_evidence_linked",
                    "A resolver evidence reference is linked to the declared resolution disposition.",
                ));
            } else {
                checks.push(Check::unknown(
                    "local_time_resolution_evidence_missing",
                    "A unique or fold resolution requires an externally supplied resolver evidence reference.",
                ));
            }
        }
    }

    match (
        source,
        evidence.resolved_utc.as_deref().and_then(parse_rfc3339),
    ) {
        (Some(source), Some(resolved)) => {
            if resolved.offset().local_minus_utc() != 0 {
                checks.push(Check::reject(
                    "resolved_timestamp_not_utc",
                    "The resolved timestamp must be expressed with a zero UTC offset.",
                ));
            } else if source.with_timezone(&Utc) == resolved.with_timezone(&Utc) {
                checks.push(Check::pass(
                    "resolved_instant_matches_source",
                    "The resolved UTC instant is identical to the instant represented by the preserved source timestamp.",
                ));
            } else {
                checks.push(Check::reject(
                    "resolved_instant_mismatch",
                    "The resolved UTC instant does not match the preserved source timestamp.",
                ));
            }
        }
        (Some(_), None) => checks.push(Check::unknown(
            "resolved_utc_missing_or_invalid",
            "No valid resolved UTC instant was supplied.",
        )),
        _ => checks.push(Check::not_evaluable(
            "source_instant_unavailable",
            "The source instant must parse before a resolved instant can be compared.",
        )),
    }

    ValidationReport::from_checks(checks)
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct UnitRegistrySnapshot {
    pub authority: String,
    pub registry_name: String,
    pub version: String,
    pub sha256: String,
    pub units: BTreeMap<String, String>,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct TypedQuantity {
    pub value_decimal: String,
    pub unit_code: String,
    pub registry_name: String,
    pub registry_version: String,
    pub registry_sha256: String,
}

fn is_decimal_lexeme(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.is_empty() {
        return false;
    }

    let start = usize::from(bytes[0] == b'-');
    if start == bytes.len() {
        return false;
    }

    let mut seen_decimal = false;
    let mut digits_before = 0usize;
    let mut digits_after = 0usize;

    for byte in &bytes[start..] {
        match byte {
            b'0'..=b'9' if seen_decimal => digits_after += 1,
            b'0'..=b'9' => digits_before += 1,
            b'.' if !seen_decimal => seen_decimal = true,
            _ => return false,
        }
    }

    digits_before > 0 && (!seen_decimal || digits_after > 0)
}

pub fn evaluate_quantity(
    quantity: &TypedQuantity,
    registry: Option<&UnitRegistrySnapshot>,
    expected_dimension: &str,
) -> ValidationReport {
    let mut checks = Vec::new();

    if is_decimal_lexeme(&quantity.value_decimal) {
        checks.push(Check::pass(
            "quantity_decimal_exact",
            "The quantity uses a plain decimal lexeme and does not require binary floating-point parsing.",
        ));
    } else {
        checks.push(Check::reject(
            "quantity_decimal_invalid",
            "The quantity must be a finite decimal string without exponent notation, NaN, or infinity.",
        ));
    }

    let Some(registry) = registry else {
        checks.push(Check::unknown(
            "unit_registry_missing",
            "No exact unit-registry snapshot was supplied.",
        ));
        return ValidationReport::from_checks(checks);
    };

    if !sha256_digest_is_well_formed(&registry.sha256)
        || !sha256_digest_is_well_formed(&quantity.registry_sha256)
    {
        checks.push(Check::reject(
            "unit_registry_digest_invalid",
            "Both the quantity and registry must carry well-formed SHA-256 digests.",
        ));
    }

    if quantity.registry_name != registry.registry_name
        || quantity.registry_version != registry.version
        || quantity.registry_sha256 != registry.sha256
    {
        checks.push(Check::conflict(
            "unit_registry_pin_mismatch",
            "Unit registry name, version, and digest must match the exact pinned registry snapshot.",
        ));
    } else {
        checks.push(Check::pass(
            "unit_registry_pin_matches",
            "The quantity references the exact pinned unit registry snapshot.",
        ));
    }

    match registry.units.get(&quantity.unit_code) {
        None => checks.push(Check::unknown(
            "unit_code_not_resolved",
            "The supplied pinned registry does not provide a dimension for this unit code.",
        )),
        Some(dimension) if dimension != expected_dimension => checks.push(Check::reject(
            "unit_dimension_mismatch",
            "The unit's declared physical dimension does not match the required dimension.",
        )),
        Some(_) => checks.push(Check::pass(
            "unit_dimension_matches",
            "The unit code resolves under the pinned registry to the required physical dimension.",
        )),
    }

    ValidationReport::from_checks(checks)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OverlayMaturity {
    Research,
    Reviewed,
    Approved,
    Superseded,
    Withdrawn,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClaimScope {
    TechnicalFixture,
    AuthorityDependent,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct JurisdictionOverlay {
    pub jurisdiction_id: String,
    pub maturity: OverlayMaturity,
    pub profile_sha256: String,
    pub authority_ref: Option<String>,
    pub approval_evidence_ref: Option<String>,
}

pub fn evaluate_jurisdiction_overlay(
    overlay: Option<&JurisdictionOverlay>,
    expected_jurisdiction: &str,
    scope: ClaimScope,
    authority_recognition: RecognitionDisposition,
) -> ValidationReport {
    let Some(overlay) = overlay else {
        return ValidationReport::from_checks(vec![Check::unknown(
            "jurisdiction_overlay_missing",
            "No jurisdiction overlay was supplied.",
        )]);
    };

    let mut checks = Vec::new();

    if overlay.jurisdiction_id == expected_jurisdiction {
        checks.push(Check::pass(
            "jurisdiction_overlay_scope_matches",
            "The overlay is explicitly scoped to the requested jurisdiction identifier.",
        ));
    } else {
        checks.push(Check::reject(
            "jurisdiction_overlay_scope_mismatch",
            "An overlay for another jurisdiction cannot authorize this jurisdiction's claim.",
        ));
    }

    if !sha256_digest_is_well_formed(&overlay.profile_sha256) {
        checks.push(Check::reject(
            "jurisdiction_overlay_digest_invalid",
            "The overlay must carry a well-formed digest for its exact profile artifact.",
        ));
    }

    match overlay.maturity {
        OverlayMaturity::Superseded | OverlayMaturity::Withdrawn => checks.push(Check::reject(
            "jurisdiction_overlay_not_current",
            "Superseded or withdrawn overlays cannot support a current decision.",
        )),
        OverlayMaturity::Unknown => checks.push(Check::unknown(
            "jurisdiction_overlay_maturity_unknown",
            "The overlay's maturity state is not established.",
        )),
        OverlayMaturity::Research | OverlayMaturity::Reviewed => match scope {
            ClaimScope::TechnicalFixture => checks.push(Check::pass(
                "research_overlay_usable_for_fixture",
                "Research/reviewed overlays may label technical compatibility fixtures only.",
            )),
            ClaimScope::AuthorityDependent => checks.push(Check::not_evaluable(
                "research_overlay_cannot_authorize",
                "A research or reviewed profile is not an approved legal or competent-authority decision.",
            )),
        },
        OverlayMaturity::Approved => {
            let authority_refs_present = overlay
                .authority_ref
                .as_deref()
                .is_some_and(non_empty)
                && overlay
                    .approval_evidence_ref
                    .as_deref()
                    .is_some_and(non_empty);

            if !authority_refs_present {
                checks.push(Check::unknown(
                    "overlay_approval_evidence_missing",
                    "An approved status requires explicit authority and approval-evidence references.",
                ));
            } else if scope == ClaimScope::TechnicalFixture {
                checks.push(Check::pass(
                    "approved_overlay_fixture_scope",
                    "The approved overlay is usable as a technical fixture; this does not independently validate the approval.",
                ));
            } else {
                checks.push(recognition_check(authority_recognition));
            }
        }
    }

    ValidationReport::from_checks(checks)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(character: char) -> String {
        format!("sha256:{}", character.to_string().repeat(64))
    }

    fn valid_identifier() -> ExternalIdentifier {
        ExternalIdentifier {
            scheme: "national-vessel-registry".into(),
            namespace: "example-country".into(),
            scheme_version: "2026-1".into(),
            issuer: "registry-authority".into(),
            subject_type: "vessel".into(),
            value: "REG-001".into(),
            jurisdiction: Some("XX".into()),
            purpose: "maritime-service-access".into(),
            valid_from: Some("2026-01-01T00:00:00Z".into()),
            valid_until: Some("2027-01-01T00:00:00Z".into()),
        }
    }

    fn valid_identifier_context() -> IdentifierContext {
        IdentifierContext {
            expected_subject_type: "vessel".into(),
            expected_jurisdiction: Some("XX".into()),
            expected_purpose: "maritime-service-access".into(),
            as_of: "2026-11-08T10:00:00Z".into(),
            issuer_recognition: RecognitionDisposition::Recognized,
        }
    }

    #[test]
    fn identifier_requires_subject_and_purpose_scope() {
        let mut identifier = valid_identifier();
        identifier.subject_type = "organization".into();
        let report = evaluate_identifier(&identifier, &valid_identifier_context());
        assert_eq!(report.disposition, Disposition::Reject);
        assert!(report.checks.iter().any(|check| check.code == "identifier_subject_mismatch"));
    }

    #[test]
    fn unassessed_issuer_is_not_implicitly_recognized() {
        let mut context = valid_identifier_context();
        context.issuer_recognition = RecognitionDisposition::NotAssessed;
        let report = evaluate_identifier(&valid_identifier(), &context);
        assert_eq!(report.disposition, Disposition::NotEvaluable);
    }

    #[test]
    fn identifier_validity_uses_exclusive_upper_bound() {
        let mut identifier = valid_identifier();
        identifier.valid_until = Some("2026-11-08T10:00:00Z".into());
        let report = evaluate_identifier(&identifier, &valid_identifier_context());
        assert_eq!(report.disposition, Disposition::Reject);
        assert!(report.checks.iter().any(|check| check.code == "identifier_expired"));
    }

    fn metadata(code: &str, exponent: u8) -> CurrencyMetadata {
        CurrencyMetadata {
            artifact_id: "iso4217-list".into(),
            release: "fixture-2026-11".into(),
            sha256: digest('a'),
            currency_code: code.into(),
            minor_unit_exponent: exponent,
            effective_from: "2026-01-01T00:00:00Z".into(),
            effective_until: None,
        }
    }

    fn money(code: &str, exponent: u8) -> MoneyAmount {
        MoneyAmount {
            currency_code: code.into(),
            amount_minor: 1234,
            metadata_artifact_id: "iso4217-list".into(),
            metadata_release: "fixture-2026-11".into(),
            metadata_sha256: digest('a'),
            minor_unit_exponent: exponent,
            effective_at: "2026-11-08T10:00:00Z".into(),
        }
    }

    #[test]
    fn money_accepts_zero_two_and_three_exponent_fixtures_without_hardcoding_codes() {
        for (code, exponent) in [("AAA", 0), ("BBB", 2), ("CCC", 3)] {
            let report = evaluate_money(&money(code, exponent), Some(&metadata(code, exponent)));
            assert_eq!(report.disposition, Disposition::Pass, "{code}/{exponent}");
        }
    }

    #[test]
    fn money_rejects_scale_that_disagrees_with_pinned_metadata() {
        let report = evaluate_money(&money("BBB", 3), Some(&metadata("BBB", 2)));
        assert_eq!(report.disposition, Disposition::Conflict);
    }

    #[test]
    fn money_unknown_without_metadata_snapshot() {
        let report = evaluate_money(&money("BBB", 2), None);
        assert_eq!(report.disposition, Disposition::Unknown);
    }

    #[test]
    fn money_metadata_effectivity_is_checked() {
        let mut snapshot = metadata("BBB", 2);
        snapshot.effective_from = "2027-01-01T00:00:00Z".into();
        let report = evaluate_money(&money("BBB", 2), Some(&snapshot));
        assert_eq!(report.disposition, Disposition::Reject);
    }

    fn valid_temporal() -> TemporalEvidence {
        TemporalEvidence {
            source_local_lexeme: "2026-11-08T08:00:00".into(),
            source_timestamp: "2026-11-08T08:00:00+05:45".into(),
            time_zone_id: "Asia/Kathmandu".into(),
            tzdb_version: "2026f".into(),
            resolution: LocalTimeResolution::Unique,
            resolver_evidence_ref: Some("resolver-receipt:001".into()),
            resolved_utc: Some("2026-11-08T02:15:00Z".into()),
        }
    }

    #[test]
    fn temporal_accepts_quarter_hour_offset_when_instants_match() {
        let report = evaluate_temporal(
            &valid_temporal(),
            &TemporalContext {
                expected_tzdb_version: "2026f".into(),
            },
        );
        assert_eq!(report.disposition, Disposition::Pass);
    }

    #[test]
    fn temporal_rejects_silent_tzdb_substitution() {
        let report = evaluate_temporal(
            &valid_temporal(),
            &TemporalContext {
                expected_tzdb_version: "2026e".into(),
            },
        );
        assert_eq!(report.disposition, Disposition::Conflict);
    }

    #[test]
    fn temporal_keeps_unresolved_fold_unknown() {
        let mut evidence = valid_temporal();
        evidence.resolution = LocalTimeResolution::Unknown;
        let report = evaluate_temporal(
            &evidence,
            &TemporalContext {
                expected_tzdb_version: "2026f".into(),
            },
        );
        assert_eq!(report.disposition, Disposition::Unknown);
    }

    #[test]
    fn temporal_rejects_nonexistent_local_time() {
        let mut evidence = valid_temporal();
        evidence.resolution = LocalTimeResolution::GapRejected;
        let report = evaluate_temporal(
            &evidence,
            &TemporalContext {
                expected_tzdb_version: "2026f".into(),
            },
        );
        assert_eq!(report.disposition, Disposition::Reject);
    }

    fn registry() -> UnitRegistrySnapshot {
        UnitRegistrySnapshot {
            authority: "unit-registry-authority".into(),
            registry_name: "UCUM".into(),
            version: "2.2".into(),
            sha256: digest('b'),
            units: BTreeMap::from([
                ("m".into(), "length".into()),
                ("W".into(), "power".into()),
                ("Wh".into(), "energy".into()),
            ]),
        }
    }

    fn quantity(unit: &str) -> TypedQuantity {
        TypedQuantity {
            value_decimal: "12.750".into(),
            unit_code: unit.into(),
            registry_name: "UCUM".into(),
            registry_version: "2.2".into(),
            registry_sha256: digest('b'),
        }
    }

    #[test]
    fn quantity_checks_physical_dimension() {
        let report = evaluate_quantity(&quantity("W"), Some(&registry()), "energy");
        assert_eq!(report.disposition, Disposition::Reject);
    }

    #[test]
    fn quantity_does_not_float_parse_or_accept_exponent_notation() {
        let mut value = quantity("m");
        value.value_decimal = "1.2e3".into();
        let report = evaluate_quantity(&value, Some(&registry()), "length");
        assert_eq!(report.disposition, Disposition::Reject);
    }

    #[test]
    fn quantity_unknown_unit_is_not_a_pass() {
        let report = evaluate_quantity(&quantity("unknown-unit"), Some(&registry()), "length");
        assert_eq!(report.disposition, Disposition::Unknown);
    }

    fn overlay(maturity: OverlayMaturity) -> JurisdictionOverlay {
        JurisdictionOverlay {
            jurisdiction_id: "ZA".into(),
            maturity,
            profile_sha256: digest('c'),
            authority_ref: Some("authority:example".into()),
            approval_evidence_ref: Some("approval:record-001".into()),
        }
    }

    #[test]
    fn research_overlay_is_fixture_only_not_authority() {
        let research = overlay(OverlayMaturity::Research);
        let fixture = evaluate_jurisdiction_overlay(
            Some(&research),
            "ZA",
            ClaimScope::TechnicalFixture,
            RecognitionDisposition::NotAssessed,
        );
        let authority = evaluate_jurisdiction_overlay(
            Some(&research),
            "ZA",
            ClaimScope::AuthorityDependent,
            RecognitionDisposition::NotAssessed,
        );
        assert_eq!(fixture.disposition, Disposition::Pass);
        assert_eq!(authority.disposition, Disposition::NotEvaluable);
    }

    #[test]
    fn approved_overlay_still_requires_external_authority_recognition() {
        let approved = overlay(OverlayMaturity::Approved);
        let report = evaluate_jurisdiction_overlay(
            Some(&approved),
            "ZA",
            ClaimScope::AuthorityDependent,
            RecognitionDisposition::Unknown,
        );
        assert_eq!(report.disposition, Disposition::Unknown);
    }

    #[test]
    fn prerelease_and_production_artifacts_are_not_interchangeable() {
        let expected = ArtifactPin {
            authority: "UNECE".into(),
            family: "UN-LOCODE".into(),
            identifier: "UNLOCODE".into(),
            version: "2025-1".into(),
            maturity: ArtifactMaturity::Production,
            sha256: digest('d'),
        };
        let observed = ArtifactPin {
            maturity: ArtifactMaturity::PreRelease,
            ..expected.clone()
        };
        let report = evaluate_artifact_pin(&expected, Some(&observed));
        assert_eq!(report.disposition, Disposition::Conflict);
    }

    #[test]
    fn empty_report_is_not_evaluable() {
        let report = ValidationReport::from_checks(Vec::new());
        assert_eq!(report.disposition, Disposition::NotEvaluable);
    }
}
