// SPDX-License-Identifier: AGPL-3.0-or-later
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::{env, fs, process};

const PROFILE_FIXTURE: &str = include_str!("../fixtures/fictional-source-profile-v0.1.json");
const PACKAGE_VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum ProfileStatus {
    Draft,
    Approved,
    Revoked,
    Expired,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum TermsReviewStatus {
    Verified,
    Unverified,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum ReviewDecision {
    Approved,
    Rejected,
    NotReviewed,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
enum Operation {
    ManualAccess,
    AutomatedRetrieval,
    TemporaryProcessing,
    Cache,
    FullTextIndexing,
    EmbeddingGeneration,
    Summarization,
    ModelTraining,
    ModelFineTuning,
    BenchmarkEvaluation,
    QuoteDisplay,
    Redistribution,
    MetadataProcessing,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
enum Purpose {
    ManualLegalResearch,
    AutomatedLegalResearch,
    BenchmarkEvaluation,
    ModelTraining,
    ModelServing,
    InternalAudit,
    PublicDisplay,
    Redistribution,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
enum ActorClass {
    HumanResearcher,
    InternalReviewer,
    AutomatedService,
    ProductionService,
    PublicUser,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum PermissionDecision {
    Allowed,
    Denied,
    Unknown,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum AdmissionDecision {
    Admitted,
    Denied,
    Unknown,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SourceUseProfile {
    schema_version: String,
    profile_id: String,
    profile_status: ProfileStatus,
    source: SourceIdentity,
    terms: TermsEvidence,
    reviewer: Option<ReviewerEvidence>,
    operations: Vec<OperationGrant>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SourceIdentity {
    publisher_id: String,
    canonical_ref: String,
    component_id: String,
    version_id: String,
    content_sha256: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct TermsEvidence {
    terms_ref: String,
    terms_version: String,
    terms_sha256: Option<String>,
    review_status: TermsReviewStatus,
    verified_at_utc: Option<String>,
    valid_from_utc: Option<String>,
    valid_until_utc: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct ReviewerEvidence {
    reviewer_ref: String,
    reviewer_basis: String,
    decision: ReviewDecision,
    reviewed_at_utc: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct OperationGrant {
    grant_id: String,
    operation: Operation,
    decision: PermissionDecision,
    purposes: Vec<Purpose>,
    actor_classes: Vec<ActorClass>,
    territories: Vec<String>,
    valid_from_utc: Option<String>,
    valid_until_utc: Option<String>,
    max_retention_seconds: u64,
    evidence_refs: Vec<String>,
    rationale: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SourceUseRequest {
    schema_version: String,
    source_component_id: String,
    source_version_id: String,
    operation: Operation,
    purpose: Purpose,
    actor_class: ActorClass,
    territory: String,
    as_of_utc: String,
    requested_retention_seconds: u64,
    observed_terms_sha256: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
struct ValidationReceipt {
    status: &'static str,
    profile_id: String,
    profile_sha256: String,
    profile_status: ProfileStatus,
    operation_grant_count: usize,
    claim_ceiling: &'static str,
}

#[derive(Clone, Debug, Serialize)]
struct AdmissionReceipt {
    receipt_version: &'static str,
    evaluator_version: &'static str,
    profile_id: String,
    profile_sha256: String,
    request_sha256: String,
    permission_decision: Option<PermissionDecision>,
    admission_decision: AdmissionDecision,
    reason_code: String,
    permission_grant_id: Option<String>,
    expected_terms_sha256: Option<String>,
    observed_terms_sha256: Option<String>,
    request: SourceUseRequest,
    claim_ceiling: &'static str,
}

struct DecisionResult {
    permission_decision: Option<PermissionDecision>,
    admission_decision: AdmissionDecision,
    reason_code: &'static str,
    grant_id: Option<String>,
}

fn main() {
    match run() {
        Ok(true) => {}
        Ok(false) => process::exit(1),
        Err(error) => {
            eprintln!("legal-source-permission-core: {error}");
            process::exit(2);
        }
    }
}

fn run() -> Result<bool, String> {
    let mut args = env::args().skip(1);
    let command = args.next().ok_or_else(usage)?;
    match command.as_str() {
        "validate" => {
            let profile_path = args.next().ok_or_else(usage)?;
            if args.next().is_some() {
                return Err(usage());
            }
            let raw = fs::read(&profile_path)
                .map_err(|error| format!("cannot read {profile_path}: {error}"))?;
            let profile: SourceUseProfile = serde_json::from_slice(&raw)
                .map_err(|error| format!("{profile_path} is invalid JSON: {error}"))?;
            validate_profile(&profile)?;
            let receipt = ValidationReceipt {
                status: "VALID",
                profile_id: profile.profile_id.clone(),
                profile_sha256: sha256_hex(&raw),
                profile_status: profile.profile_status,
                operation_grant_count: profile.operations.len(),
                claim_ceiling: "Structural profile validation only; not a legal opinion or proof of lawful use.",
            };
            println!(
                "{}",
                serde_json::to_string_pretty(&receipt)
                    .map_err(|error| format!("cannot serialize validation receipt: {error}"))?
            );
            Ok(true)
        }
        "check" => {
            let profile_path = args.next().ok_or_else(usage)?;
            let request_path = args.next().ok_or_else(usage)?;
            if args.next().is_some() {
                return Err(usage());
            }
            let profile_raw = fs::read(&profile_path)
                .map_err(|error| format!("cannot read {profile_path}: {error}"))?;
            let request_raw = fs::read(&request_path)
                .map_err(|error| format!("cannot read {request_path}: {error}"))?;
            let profile: SourceUseProfile = serde_json::from_slice(&profile_raw)
                .map_err(|error| format!("{profile_path} is invalid JSON: {error}"))?;
            let request: SourceUseRequest = serde_json::from_slice(&request_raw)
                .map_err(|error| format!("{request_path} is invalid JSON: {error}"))?;
            validate_profile(&profile)?;
            validate_request(&request)?;
            let decision = evaluate(&profile, &request);
            let receipt = AdmissionReceipt {
                receipt_version: "0.1.0",
                evaluator_version: PACKAGE_VERSION,
                profile_id: profile.profile_id.clone(),
                profile_sha256: sha256_hex(&profile_raw),
                request_sha256: sha256_hex(&request_raw),
                permission_decision: decision.permission_decision,
                admission_decision: decision.admission_decision,
                reason_code: decision.reason_code.to_string(),
                permission_grant_id: decision.grant_id,
                expected_terms_sha256: profile.terms.terms_sha256.clone(),
                observed_terms_sha256: request.observed_terms_sha256.clone(),
                request,
                claim_ceiling: "Admission-control result only; not a copyright, ownership, license-validity, legal-exception, or lawful-use determination.",
            };
            println!(
                "{}",
                serde_json::to_string_pretty(&receipt)
                    .map_err(|error| format!("cannot serialize admission receipt: {error}"))?
            );
            // A valid denied/unknown decision is a successfully computed result, not a CLI error.
            Ok(true)
        }
        _ => Err(usage()),
    }
}

fn usage() -> String {
    [
        "usage: legal-source-permission-core validate <profile.json>",
        "       legal-source-permission-core check <profile.json> <request.json>",
    ]
    .join("\n")
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn parse_date(value: &str) -> Option<(i32, u32, u32)> {
    if value.len() != 10
        || value.as_bytes().get(4) != Some(&b'-')
        || value.as_bytes().get(7) != Some(&b'-')
    {
        return None;
    }
    let year: i32 = value.get(0..4)?.parse().ok()?;
    let month: u32 = value.get(5..7)?.parse().ok()?;
    let day: u32 = value.get(8..10)?.parse().ok()?;
    if year < 1 || !(1..=12).contains(&month) {
        return None;
    }
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let month_days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return None,
    };
    if day == 0 || day > month_days {
        return None;
    }
    Some((year, month, day))
}

fn valid_utc_timestamp(value: &str) -> bool {
    if value.len() != 20
        || value.as_bytes().get(10) != Some(&b'T')
        || value.as_bytes().get(13) != Some(&b':')
        || value.as_bytes().get(16) != Some(&b':')
        || value.as_bytes().get(19) != Some(&b'Z')
    {
        return false;
    }
    if parse_date(&value[..10]).is_none() {
        return false;
    }
    let hour = value.get(11..13).and_then(|v| v.parse::<u32>().ok());
    let minute = value.get(14..16).and_then(|v| v.parse::<u32>().ok());
    let second = value.get(17..19).and_then(|v| v.parse::<u32>().ok());
    matches!(hour, Some(0..=23))
        && matches!(minute, Some(0..=59))
        && matches!(second, Some(0..=59))
}

fn validate_interval(
    from: Option<&str>,
    until: Option<&str>,
    context: &str,
    errors: &mut Vec<String>,
) {
    if let Some(value) = from {
        if !valid_utc_timestamp(value) {
            errors.push(format!("{context} has invalid valid_from_utc timestamp {value}"));
        }
    }
    if let Some(value) = until {
        if !valid_utc_timestamp(value) {
            errors.push(format!("{context} has invalid valid_until_utc timestamp {value}"));
        }
    }
    if let (Some(from), Some(until)) = (from, until) {
        if valid_utc_timestamp(from) && valid_utc_timestamp(until) && from >= until {
            errors.push(format!("{context} must use a non-empty half-open validity interval"));
        }
    }
}

fn validate_profile(profile: &SourceUseProfile) -> Result<(), String> {
    let mut errors = Vec::new();
    if profile.schema_version != "0.1.0" {
        errors.push(format!(
            "unsupported profile schema version {}",
            profile.schema_version
        ));
    }
    if profile.profile_id.trim().is_empty() {
        errors.push("profile_id must not be empty".to_string());
    }
    for (name, value) in [
        ("source.publisher_id", profile.source.publisher_id.as_str()),
        ("source.canonical_ref", profile.source.canonical_ref.as_str()),
        ("source.component_id", profile.source.component_id.as_str()),
        ("source.version_id", profile.source.version_id.as_str()),
        ("terms.terms_ref", profile.terms.terms_ref.as_str()),
        ("terms.terms_version", profile.terms.terms_version.as_str()),
    ] {
        if value.trim().is_empty() {
            errors.push(format!("{name} must not be empty"));
        }
    }
    if let Some(digest) = profile.source.content_sha256.as_deref() {
        if !is_sha256(digest) {
            errors.push("source.content_sha256 must be a lowercase SHA-256 digest or null".to_string());
        }
    }
    if let Some(digest) = profile.terms.terms_sha256.as_deref() {
        if !is_sha256(digest) {
            errors.push("terms.terms_sha256 must be a lowercase SHA-256 digest or null".to_string());
        }
    }
    validate_interval(
        profile.terms.valid_from_utc.as_deref(),
        profile.terms.valid_until_utc.as_deref(),
        "terms",
        &mut errors,
    );
    if let Some(verified_at) = profile.terms.verified_at_utc.as_deref() {
        if !valid_utc_timestamp(verified_at) {
            errors.push(format!("invalid terms.verified_at_utc timestamp {verified_at}"));
        }
    }
    if let Some(reviewer) = &profile.reviewer {
        if reviewer.reviewer_ref.trim().is_empty() || reviewer.reviewer_basis.trim().is_empty() {
            errors.push("reviewer_ref and reviewer_basis must not be empty".to_string());
        }
        if !valid_utc_timestamp(&reviewer.reviewed_at_utc) {
            errors.push("reviewer.reviewed_at_utc must be a normalized UTC timestamp".to_string());
        }
    }
    if profile.profile_status == ProfileStatus::Approved {
        match &profile.reviewer {
            Some(reviewer) if reviewer.decision == ReviewDecision::Approved => {}
            _ => errors.push("an approved profile requires an approved reviewer record".to_string()),
        }
        if profile.terms.review_status != TermsReviewStatus::Verified {
            errors.push("an approved profile requires verified terms".to_string());
        }
        if profile.terms.terms_sha256.is_none() {
            errors.push("an approved profile requires an exact terms SHA-256 digest".to_string());
        }
        if profile.terms.verified_at_utc.is_none() {
            errors.push("an approved profile requires terms.verified_at_utc".to_string());
        }
        if profile.terms.valid_from_utc.is_none() {
            errors.push("an approved profile requires a terms validity start".to_string());
        }
    }
    if profile.operations.is_empty() {
        errors.push("at least one operation grant is required".to_string());
    }

    let mut grant_ids = BTreeSet::new();
    for grant in &profile.operations {
        let context = format!("operation grant {}", grant.grant_id);
        if grant.grant_id.trim().is_empty() || !grant_ids.insert(grant.grant_id.as_str()) {
            errors.push(format!("empty or duplicate grant_id: {}", grant.grant_id));
        }
        if grant.purposes.is_empty()
            || grant.actor_classes.is_empty()
            || grant.territories.is_empty()
            || grant.evidence_refs.is_empty()
        {
            errors.push(format!(
                "{context} requires purposes, actor_classes, territories, and evidence_refs"
            ));
        }
        if grant.rationale.trim().is_empty() {
            errors.push(format!("{context} requires a nonempty rationale"));
        }
        if grant.territories.iter().any(|territory| {
            territory.len() != 2 || !territory.bytes().all(|b| b.is_ascii_uppercase())
        }) {
            errors.push(format!("{context} territories must be two uppercase ASCII letters"));
        }
        let purpose_set: BTreeSet<Purpose> = grant.purposes.iter().copied().collect();
        if purpose_set.len() != grant.purposes.len() {
            errors.push(format!("{context} repeats a purpose"));
        }
        let actor_set: BTreeSet<ActorClass> = grant.actor_classes.iter().copied().collect();
        if actor_set.len() != grant.actor_classes.len() {
            errors.push(format!("{context} repeats an actor class"));
        }
        let territory_set: BTreeSet<&str> = grant.territories.iter().map(String::as_str).collect();
        if territory_set.len() != grant.territories.len() {
            errors.push(format!("{context} repeats a territory"));
        }
        let evidence_set: BTreeSet<&str> = grant.evidence_refs.iter().map(String::as_str).collect();
        if evidence_set.len() != grant.evidence_refs.len()
            || grant.evidence_refs.iter().any(|item| item.trim().is_empty())
        {
            errors.push(format!("{context} has duplicate or empty evidence references"));
        }
        validate_interval(
            grant.valid_from_utc.as_deref(),
            grant.valid_until_utc.as_deref(),
            &context,
            &mut errors,
        );
        if profile.profile_status == ProfileStatus::Approved && grant.decision == PermissionDecision::Allowed
        {
            if grant.evidence_refs.is_empty() {
                errors.push(format!("{context} cannot allow an operation without evidence references"));
            }
        }
    }

    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "profile validation failed:\n - {}",
            errors.join("\n - ")
        ))
    }
}

fn validate_request(request: &SourceUseRequest) -> Result<(), String> {
    let mut errors = Vec::new();
    if request.schema_version != "0.1.0" {
        errors.push(format!(
            "unsupported request schema version {}",
            request.schema_version
        ));
    }
    for (name, value) in [
        ("source_component_id", request.source_component_id.as_str()),
        ("source_version_id", request.source_version_id.as_str()),
    ] {
        if value.trim().is_empty() {
            errors.push(format!("{name} must not be empty"));
        }
    }
    if request.territory.len() != 2
        || !request.territory.bytes().all(|byte| byte.is_ascii_uppercase())
    {
        errors.push("territory must be two uppercase ASCII letters".to_string());
    }
    if !valid_utc_timestamp(&request.as_of_utc) {
        errors.push("as_of_utc must be a normalized UTC timestamp".to_string());
    }
    if let Some(digest) = request.observed_terms_sha256.as_deref() {
        if !is_sha256(digest) {
            errors.push("observed_terms_sha256 must be a lowercase SHA-256 digest or null".to_string());
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "request validation failed:\n - {}",
            errors.join("\n - ")
        ))
    }
}

fn interval_contains(from: Option<&str>, until: Option<&str>, as_of: &str) -> bool {
    from.is_none_or(|from| as_of >= from) && until.is_none_or(|until| as_of < until)
}

fn unknown(reason: &'static str) -> DecisionResult {
    DecisionResult {
        permission_decision: Some(PermissionDecision::Unknown),
        admission_decision: AdmissionDecision::Unknown,
        reason_code: reason,
        grant_id: None,
    }
}

fn denied(reason: &'static str, grant_id: Option<String>) -> DecisionResult {
    DecisionResult {
        permission_decision: Some(PermissionDecision::Denied),
        admission_decision: AdmissionDecision::Denied,
        reason_code: reason,
        grant_id,
    }
}

fn evaluate(profile: &SourceUseProfile, request: &SourceUseRequest) -> DecisionResult {
    if profile.profile_status == ProfileStatus::Revoked {
        return denied("profile_revoked", None);
    }
    if profile.profile_status != ProfileStatus::Approved {
        return unknown("profile_not_approved_or_expired");
    }
    let reviewer_is_approved = profile
        .reviewer
        .as_ref()
        .is_some_and(|reviewer| reviewer.decision == ReviewDecision::Approved);
    if !reviewer_is_approved {
        return unknown("reviewer_approval_not_established");
    }
    if profile.terms.review_status != TermsReviewStatus::Verified {
        return unknown("terms_not_verified");
    }
    let Some(expected_terms_sha256) = profile.terms.terms_sha256.as_deref() else {
        return unknown("terms_digest_missing");
    };
    let Some(observed_terms_sha256) = request.observed_terms_sha256.as_deref() else {
        return unknown("observed_terms_digest_missing");
    };
    if expected_terms_sha256 != observed_terms_sha256 {
        return unknown("terms_digest_mismatch_requires_rereview");
    }
    if !interval_contains(
        profile.terms.valid_from_utc.as_deref(),
        profile.terms.valid_until_utc.as_deref(),
        &request.as_of_utc,
    ) {
        return unknown("terms_not_effective_at_request_time");
    }
    if profile.source.component_id != request.source_component_id
        || profile.source.version_id != request.source_version_id
    {
        return unknown("source_component_or_version_mismatch");
    }

    let operation_grants: Vec<&OperationGrant> = profile
        .operations
        .iter()
        .filter(|grant| grant.operation == request.operation)
        .collect();
    if operation_grants.is_empty() {
        return unknown("no_permission_record_for_operation");
    }

    let scope_grants: Vec<&OperationGrant> = operation_grants
        .into_iter()
        .filter(|grant| {
            grant.purposes.contains(&request.purpose)
                && grant.actor_classes.contains(&request.actor_class)
                && grant.territories.iter().any(|territory| territory == &request.territory)
        })
        .collect();
    if scope_grants.is_empty() {
        return denied("request_outside_declared_scope", None);
    }

    let time_grants: Vec<&OperationGrant> = scope_grants
        .into_iter()
        .filter(|grant| {
            interval_contains(
                grant.valid_from_utc.as_deref(),
                grant.valid_until_utc.as_deref(),
                &request.as_of_utc,
            )
        })
        .collect();
    if time_grants.is_empty() {
        return denied("grant_not_effective_at_request_time", None);
    }

    // Denial takes precedence over an overlapping allow. A mixture of allowed
    // and unknown grants for the same request is ambiguous and therefore unknown.
    if let Some(grant) = time_grants
        .iter()
        .find(|grant| grant.decision == PermissionDecision::Denied)
    {
        return denied("operation_explicitly_denied", Some(grant.grant_id.clone()));
    }
    if time_grants.iter().any(|grant| grant.decision == PermissionDecision::Unknown) {
        return unknown("overlapping_grants_include_unknown_permission");
    }

    let retention_limit = time_grants
        .iter()
        .map(|grant| grant.max_retention_seconds)
        .min()
        .unwrap_or(0);
    let selected_grant = time_grants
        .iter()
        .min_by_key(|grant| grant.max_retention_seconds)
        .expect("time_grants is nonempty");
    if request.requested_retention_seconds > retention_limit {
        return DecisionResult {
            permission_decision: Some(PermissionDecision::Allowed),
            admission_decision: AdmissionDecision::Denied,
            reason_code: "requested_retention_exceeds_grant_limit",
            grant_id: Some(selected_grant.grant_id.clone()),
        };
    }

    DecisionResult {
        permission_decision: Some(PermissionDecision::Allowed),
        admission_decision: AdmissionDecision::Admitted,
        reason_code: "operation_explicitly_permitted_within_scope",
        grant_id: Some(selected_grant.grant_id.clone()),
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> SourceUseProfile {
        let profile: SourceUseProfile =
            serde_json::from_str(PROFILE_FIXTURE).expect("synthetic profile parses");
        validate_profile(&profile).expect("synthetic profile validates");
        profile
    }

    fn request(operation: Operation, purpose: Purpose, actor_class: ActorClass) -> SourceUseRequest {
        let profile = profile();
        SourceUseRequest {
            schema_version: "0.1.0".to_string(),
            source_component_id: profile.source.component_id,
            source_version_id: profile.source.version_id,
            operation,
            purpose,
            actor_class,
            territory: "ZA".to_string(),
            as_of_utc: "2026-10-19T08:00:00Z".to_string(),
            requested_retention_seconds: 0,
            observed_terms_sha256: profile.terms.terms_sha256,
        }
    }

    fn admission(request: &SourceUseRequest) -> DecisionResult {
        evaluate(&profile(), request)
    }

    #[test]
    fn fictional_profile_validates_with_thirteen_operation_grants() {
        let profile = profile();
        assert_eq!(profile.operations.len(), 13);
        assert_eq!(profile.profile_status, ProfileStatus::Approved);
    }

    #[test]
    fn explicitly_permitted_benchmark_evaluation_is_admitted() {
        let request = request(
            Operation::BenchmarkEvaluation,
            Purpose::BenchmarkEvaluation,
            ActorClass::InternalReviewer,
        );
        let result = admission(&request);
        assert_eq!(result.permission_decision, Some(PermissionDecision::Allowed));
        assert_eq!(result.admission_decision, AdmissionDecision::Admitted);
        assert_eq!(result.reason_code, "operation_explicitly_permitted_within_scope");
    }

    #[test]
    fn explicitly_denied_cache_is_not_admitted() {
        let request = request(Operation::Cache, Purpose::BenchmarkEvaluation, ActorClass::AutomatedService);
        let result = admission(&request);
        assert_eq!(result.admission_decision, AdmissionDecision::Denied);
        assert_eq!(result.reason_code, "operation_explicitly_denied");
    }

    #[test]
    fn unknown_indexing_permission_is_not_an_implicit_grant() {
        let request = request(
            Operation::FullTextIndexing,
            Purpose::BenchmarkEvaluation,
            ActorClass::InternalReviewer,
        );
        let result = admission(&request);
        assert_eq!(result.permission_decision, Some(PermissionDecision::Unknown));
        assert_eq!(result.admission_decision, AdmissionDecision::Unknown);
    }

    #[test]
    fn terms_digest_mismatch_fails_closed() {
        let mut request = request(
            Operation::BenchmarkEvaluation,
            Purpose::BenchmarkEvaluation,
            ActorClass::InternalReviewer,
        );
        request.observed_terms_sha256 = Some("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".to_string());
        assert_eq!(admission(&request).reason_code, "terms_digest_mismatch_requires_rereview");
    }

    #[test]
    fn missing_observed_terms_digest_fails_closed() {
        let mut request = request(
            Operation::BenchmarkEvaluation,
            Purpose::BenchmarkEvaluation,
            ActorClass::InternalReviewer,
        );
        request.observed_terms_sha256 = None;
        assert_eq!(admission(&request).admission_decision, AdmissionDecision::Unknown);
    }

    #[test]
    fn unknown_source_version_does_not_inherit_another_versions_grant() {
        let mut request = request(
            Operation::BenchmarkEvaluation,
            Purpose::BenchmarkEvaluation,
            ActorClass::InternalReviewer,
        );
        request.source_version_id = "snapshot-2027-10".to_string();
        assert_eq!(admission(&request).reason_code, "source_component_or_version_mismatch");
        assert_eq!(admission(&request).admission_decision, AdmissionDecision::Unknown);
    }

    #[test]
    fn purpose_actor_and_territory_are_all_scope_bound() {
        let mut wrong_purpose = request(
            Operation::BenchmarkEvaluation,
            Purpose::PublicDisplay,
            ActorClass::InternalReviewer,
        );
        assert_eq!(admission(&wrong_purpose).admission_decision, AdmissionDecision::Denied);
        wrong_purpose = request(
            Operation::BenchmarkEvaluation,
            Purpose::BenchmarkEvaluation,
            ActorClass::PublicUser,
        );
        assert_eq!(admission(&wrong_purpose).admission_decision, AdmissionDecision::Denied);
        wrong_purpose = request(
            Operation::BenchmarkEvaluation,
            Purpose::BenchmarkEvaluation,
            ActorClass::InternalReviewer,
        );
        wrong_purpose.territory = "US".to_string();
        assert_eq!(admission(&wrong_purpose).admission_decision, AdmissionDecision::Denied);
    }

    #[test]
    fn requested_retention_must_not_exceed_grant_limit() {
        let mut request = request(
            Operation::AutomatedRetrieval,
            Purpose::AutomatedLegalResearch,
            ActorClass::AutomatedService,
        );
        request.requested_retention_seconds = 31;
        let result = admission(&request);
        assert_eq!(result.permission_decision, Some(PermissionDecision::Allowed));
        assert_eq!(result.admission_decision, AdmissionDecision::Denied);
        assert_eq!(result.reason_code, "requested_retention_exceeds_grant_limit");
    }

    #[test]
    fn draft_or_expired_profiles_are_unknown_and_revoked_profile_is_denied() {
        let request = request(
            Operation::BenchmarkEvaluation,
            Purpose::BenchmarkEvaluation,
            ActorClass::InternalReviewer,
        );
        let mut draft = profile();
        draft.profile_status = ProfileStatus::Draft;
        assert_eq!(evaluate(&draft, &request).admission_decision, AdmissionDecision::Unknown);
        let mut expired = profile();
        expired.profile_status = ProfileStatus::Expired;
        assert_eq!(evaluate(&expired, &request).admission_decision, AdmissionDecision::Unknown);
        let mut revoked = profile();
        revoked.profile_status = ProfileStatus::Revoked;
        assert_eq!(evaluate(&revoked, &request).admission_decision, AdmissionDecision::Denied);
    }

    #[test]
    fn expired_terms_do_not_authorize_use() {
        let mut profile = profile();
        profile.terms.valid_until_utc = Some("2026-10-18T00:00:00Z".to_string());
        let request = request(
            Operation::BenchmarkEvaluation,
            Purpose::BenchmarkEvaluation,
            ActorClass::InternalReviewer,
        );
        assert_eq!(evaluate(&profile, &request).admission_decision, AdmissionDecision::Unknown);
        assert_eq!(
            evaluate(&profile, &request).reason_code,
            "terms_not_effective_at_request_time"
        );
    }

    #[test]
    fn invalid_calendar_and_non_utc_timestamps_are_rejected() {
        assert!(!valid_utc_timestamp("2025-02-29T10:00:00Z"));
        assert!(valid_utc_timestamp("2024-02-29T10:00:00Z"));
        assert!(!valid_utc_timestamp("2026-10-19T10:00:00+02:00"));
        assert!(!valid_utc_timestamp("2026-14-19T10:00:00Z"));
    }

    #[test]
    fn validation_rejects_duplicate_operation_grant_ids() {
        let mut profile = profile();
        let duplicate = profile.operations[0].clone();
        profile.operations.push(duplicate);
        assert!(validate_profile(&profile).is_err());
    }

    #[test]
    fn sha256_matches_standard_known_vector() {
        assert_eq!(
            sha256_hex(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
}
