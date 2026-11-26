//! Qualified negative evidence claims under open-world discovery.
//!
//! A retrieval miss is not proof of absence. This module distinguishes a fully
//! searched, authorized, closed source universe from withheld, unknown, unqueried,
//! undelivered, unanswered, incomplete, and unverified states. It emits aggregate
//! counts only; raw source identifiers and contents never appear in reports.
//!
//! PASS is bounded to the declared query, recipient, purpose, source universe,
//! candidate pool, and history cutoff. It does not prove that the declared universe
//! contains every possible source or authenticate upstream receipt producers.

use std::collections::BTreeSet;

use sha2::{Digest, Sha256};

const UNIVERSE_DOMAIN: &[u8] = b"MYCELIX-SCOPED-ABSENCE-UNIVERSE-V0.1\0";
const SCOPE_DOMAIN: &[u8] = b"MYCELIX-SCOPED-ABSENCE-SCOPE-V0.1\0";

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn push_len(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_str(out: &mut Vec<u8>, value: &str) {
    push_len(out, value.len());
    out.extend_from_slice(value.as_bytes());
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

fn unique_nonempty(values: &[String]) -> bool {
    let mut seen = BTreeSet::new();
    values
        .iter()
        .all(|value| nonempty(value) && seen.insert(value.as_str()))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AbsenceClaimKind {
    NoEvidenceInDeclaredScope,
    NoVerifiedEvidenceInDeclaredScope,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourcePermissionState {
    Shareable,
    Scoped,
    WithheldByRights,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourceQueryDisposition {
    Searched,
    NoResponse,
    NotDelivered,
    NotQueried,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourceMatchDisposition {
    NoMatch,
    CandidateFound,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CandidateVerificationState {
    NotApplicable,
    Verified,
    Rejected,
    Unknown,
}

/// One entry for each source in the declared search universe.
///
/// source_reference_digest is an opaque digest, not a source name or URI.
/// disposition_receipt_digest commits to the source's permission/query/match
/// disposition. query_receipt_digest is mandatory whenever a query was attempted.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ScopedSourceDisposition {
    pub source_reference_digest: String,
    pub permission: SourcePermissionState,
    pub query_disposition: SourceQueryDisposition,
    pub match_disposition: SourceMatchDisposition,
    pub verification: CandidateVerificationState,
    pub disposition_receipt_digest: String,
    pub query_receipt_digest: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ScopedAbsenceScope {
    pub request_id: String,
    pub recipient_id: String,
    pub purpose_id: String,
    pub query_digest: String,
    pub subject_scope_digest: String,
    pub frozen_history_cutoff_ms: u64,
    /// Digest of the exact, enumerated source-reference set.
    pub source_universe_digest: String,
    pub declared_source_count: u64,
    /// Digest of the eligible candidate pool searched under this scope.
    pub candidate_pool_digest: String,
    pub source_universe_complete: bool,
    pub candidate_pool_complete: bool,
    pub source_universe_receipt_digest: Option<String>,
    pub candidate_pool_receipt_digest: Option<String>,
    pub inquiry_plan_digest: String,
    pub inquiry_execution_receipt_digest: Option<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScopedAbsenceStatus {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScopedAbsenceConclusion {
    NoEvidenceFoundInClosedScope,
    NoVerifiedEvidenceFoundInClosedScope,
    VerifiedEvidenceFound,
    RejectedCandidateEvidenceFound,
    CandidateVerificationUnknown,
    RestrictedOrUnknownAccess,
    NoResponseReceived,
    QueryNotDelivered,
    SourceNotQueried,
    SearchCoverageIncomplete,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ScopedAbsenceReport {
    pub status: ScopedAbsenceStatus,
    pub claim: AbsenceClaimKind,
    pub conclusion: ScopedAbsenceConclusion,
    /// Digest of the frozen request/scope/universe contract.
    pub scope_digest: Option<String>,
    /// Aggregate counts only; no source IDs or content are emitted.
    pub declared_source_count: u64,
    pub accounted_source_count: usize,
    pub searched_source_count: usize,
    pub no_match_count: usize,
    pub candidate_count: usize,
    pub verified_candidate_count: usize,
    pub rejected_candidate_count: usize,
    pub unknown_candidate_verification_count: usize,
    pub withheld_source_count: usize,
    pub unknown_permission_count: usize,
    pub no_response_count: usize,
    pub undelivered_count: usize,
    pub unqueried_count: usize,
    pub unknown_match_count: usize,
    pub reasons: Vec<String>,
}

/// Computes the universe digest over sorted opaque source-reference digests.
/// The record order is not semantically meaningful.
pub fn compute_source_universe_digest(
    source_reference_digests: &[String],
) -> Result<String, String> {
    if !unique_nonempty(source_reference_digests)
        || source_reference_digests
            .iter()
            .any(|digest| !valid_digest(digest))
    {
        return Err("source reference digests must be unique lowercase SHA-256 hex".into());
    }
    let mut sorted = source_reference_digests.to_vec();
    sorted.sort();
    let mut bytes = Vec::new();
    bytes.extend_from_slice(UNIVERSE_DOMAIN);
    push_len(&mut bytes, sorted.len());
    for digest in sorted {
        push_str(&mut bytes, &digest);
    }
    Ok(sha256_hex(&bytes))
}

/// Computes a digest that binds the request, recipient, purpose, query, subject,
/// cutoff, source universe, candidate pool, completeness receipts and inquiry plan.
pub fn compute_scoped_absence_scope_digest(scope: &ScopedAbsenceScope) -> String {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(SCOPE_DOMAIN);
    for value in [
        scope.request_id.as_str(),
        scope.recipient_id.as_str(),
        scope.purpose_id.as_str(),
        scope.query_digest.as_str(),
        scope.subject_scope_digest.as_str(),
        scope.source_universe_digest.as_str(),
        scope.candidate_pool_digest.as_str(),
        scope.inquiry_plan_digest.as_str(),
    ] {
        push_str(&mut bytes, value);
    }
    push_u64(&mut bytes, scope.frozen_history_cutoff_ms);
    push_u64(&mut bytes, scope.declared_source_count);
    bytes.push(u8::from(scope.source_universe_complete));
    bytes.push(u8::from(scope.candidate_pool_complete));
    for receipt in [
        scope.source_universe_receipt_digest.as_deref(),
        scope.candidate_pool_receipt_digest.as_deref(),
        scope.inquiry_execution_receipt_digest.as_deref(),
    ] {
        match receipt {
            Some(value) => {
                bytes.push(1);
                push_str(&mut bytes, value);
            }
            None => bytes.push(0),
        }
    }
    sha256_hex(&bytes)
}

fn empty_report(
    claim: AbsenceClaimKind,
    status: ScopedAbsenceStatus,
    conclusion: ScopedAbsenceConclusion,
    scope_digest: Option<String>,
    declared_source_count: u64,
    accounted_source_count: usize,
    reasons: Vec<String>,
) -> ScopedAbsenceReport {
    ScopedAbsenceReport {
        status,
        claim,
        conclusion,
        scope_digest,
        declared_source_count,
        accounted_source_count,
        searched_source_count: 0,
        no_match_count: 0,
        candidate_count: 0,
        verified_candidate_count: 0,
        rejected_candidate_count: 0,
        unknown_candidate_verification_count: 0,
        withheld_source_count: 0,
        unknown_permission_count: 0,
        no_response_count: 0,
        undelivered_count: 0,
        unqueried_count: 0,
        unknown_match_count: 0,
        reasons,
    }
}

fn validate_scope(scope: &ScopedAbsenceScope) -> Vec<String> {
    let mut errors = Vec::new();
    for (label, value) in [
        ("request", scope.request_id.as_str()),
        ("recipient", scope.recipient_id.as_str()),
        ("purpose", scope.purpose_id.as_str()),
        ("query digest", scope.query_digest.as_str()),
        ("subject/scope digest", scope.subject_scope_digest.as_str()),
        ("source-universe digest", scope.source_universe_digest.as_str()),
        ("candidate-pool digest", scope.candidate_pool_digest.as_str()),
        ("inquiry-plan digest", scope.inquiry_plan_digest.as_str()),
    ] {
        if !nonempty(value) {
            errors.push(format!("{label} must be non-empty"));
        }
    }
    for (label, value) in [
        ("query", scope.query_digest.as_str()),
        ("subject/scope", scope.subject_scope_digest.as_str()),
        ("source universe", scope.source_universe_digest.as_str()),
        ("candidate pool", scope.candidate_pool_digest.as_str()),
        ("inquiry plan", scope.inquiry_plan_digest.as_str()),
    ] {
        if !valid_digest(value) {
            errors.push(format!("{label} digest is malformed"));
        }
    }
    for (label, receipt) in [
        ("source-universe", scope.source_universe_receipt_digest.as_deref()),
        ("candidate-pool", scope.candidate_pool_receipt_digest.as_deref()),
        ("inquiry-execution", scope.inquiry_execution_receipt_digest.as_deref()),
    ] {
        if receipt.is_some_and(|value| !valid_digest(value)) {
            errors.push(format!("{label} receipt digest is malformed"));
        }
    }
    errors
}

fn validate_source_record(record: &ScopedSourceDisposition) -> Result<(), String> {
    if !valid_digest(&record.source_reference_digest)
        || !valid_digest(&record.disposition_receipt_digest)
    {
        return Err("source identity or disposition receipt digest is malformed".into());
    }
    if record.query_receipt_digest.as_deref().is_some_and(|value| !valid_digest(value)) {
        return Err("query receipt digest is malformed".into());
    }

    match record.permission {
        SourcePermissionState::WithheldByRights | SourcePermissionState::Unknown => {
            if record.query_disposition == SourceQueryDisposition::Searched
                || record.match_disposition != SourceMatchDisposition::Unknown
                || record.verification != CandidateVerificationState::Unknown
            {
                return Err("withheld or unknown-permission source carries query or evidence content state".into());
            }
        }
        SourcePermissionState::Shareable | SourcePermissionState::Scoped => {
            match record.query_disposition {
                SourceQueryDisposition::Searched => {
                    if record.query_receipt_digest.is_none() {
                        return Err("completed source search lacks a query receipt".into());
                    }
                    match record.match_disposition {
                        SourceMatchDisposition::NoMatch => {
                            if record.verification != CandidateVerificationState::NotApplicable {
                                return Err("no-match source must use NotApplicable verification".into());
                            }
                        }
                        SourceMatchDisposition::CandidateFound => {
                            if record.verification == CandidateVerificationState::NotApplicable {
                                return Err("candidate evidence requires a verification disposition".into());
                            }
                        }
                        SourceMatchDisposition::Unknown => {
                            if record.verification != CandidateVerificationState::Unknown {
                                return Err("unknown match must have unknown verification".into());
                            }
                        }
                    }
                }
                SourceQueryDisposition::NoResponse
                | SourceQueryDisposition::NotDelivered
                | SourceQueryDisposition::NotQueried => {
                    if record.match_disposition != SourceMatchDisposition::Unknown
                        || record.verification != CandidateVerificationState::Unknown
                    {
                        return Err("uncompleted search carries a match or verification conclusion".into());
                    }
                    if record.query_disposition != SourceQueryDisposition::NotQueried
                        && record.query_receipt_digest.is_none()
                    {
                        return Err("attempted query lacks a delivery/attempt receipt".into());
                    }
                }
            }
        }
    }
    Ok(())
}

fn conclusion_from_counts(
    counts: &ScopedAbsenceReport,
) -> ScopedAbsenceConclusion {
    if counts.verified_candidate_count > 0 {
        ScopedAbsenceConclusion::VerifiedEvidenceFound
    } else if counts.unknown_candidate_verification_count > 0
        || counts.unknown_match_count > 0
    {
        ScopedAbsenceConclusion::CandidateVerificationUnknown
    } else if counts.candidate_count > 0 {
        ScopedAbsenceConclusion::RejectedCandidateEvidenceFound
    } else if counts.withheld_source_count > 0 || counts.unknown_permission_count > 0 {
        ScopedAbsenceConclusion::RestrictedOrUnknownAccess
    } else if counts.no_response_count > 0 {
        ScopedAbsenceConclusion::NoResponseReceived
    } else if counts.undelivered_count > 0 {
        ScopedAbsenceConclusion::QueryNotDelivered
    } else if counts.unqueried_count > 0 {
        ScopedAbsenceConclusion::SourceNotQueried
    } else if counts.searched_source_count != counts.declared_source_count as usize
        || counts.accounted_source_count != counts.declared_source_count as usize
    {
        ScopedAbsenceConclusion::SearchCoverageIncomplete
    } else if counts.candidate_count == 0 && counts.no_match_count == counts.declared_source_count as usize {
        ScopedAbsenceConclusion::NoEvidenceFoundInClosedScope
    } else {
        ScopedAbsenceConclusion::SearchCoverageIncomplete
    }
}

/// Qualifies a single scoped absence claim.
///
/// PASS requires a closed, fully accounted source universe and completed search
/// of every authorized source. Withheld/unknown sources, unqueried holders,
/// undelivered requests, nonresponses and incomplete indexes never become
/// evidence of absence. A no-verified-evidence claim may pass when all discovered
/// candidates were explicitly rejected; a no-evidence claim may not.
pub fn qualify_scoped_absence_claim(
    scope: &ScopedAbsenceScope,
    sources: &[ScopedSourceDisposition],
    claim: AbsenceClaimKind,
) -> ScopedAbsenceReport {
    let scope_digest = Some(compute_scoped_absence_scope_digest(scope));
    let scope_errors = validate_scope(scope);
    if !scope_errors.is_empty() {
        return empty_report(
            claim,
            ScopedAbsenceStatus::Fail,
            ScopedAbsenceConclusion::SearchCoverageIncomplete,
            scope_digest,
            scope.declared_source_count,
            sources.len(),
            scope_errors,
        );
    }

    let mut errors = Vec::new();
    let mut seen = BTreeSet::new();
    for source in sources {
        if !seen.insert(source.source_reference_digest.as_str()) {
            errors.push("source universe contains duplicate source references".into());
        }
        if let Err(error) = validate_source_record(source) {
            errors.push(error);
        }
    }
    if !errors.is_empty() {
        return empty_report(
            claim,
            ScopedAbsenceStatus::Fail,
            ScopedAbsenceConclusion::SearchCoverageIncomplete,
            scope_digest,
            scope.declared_source_count,
            sources.len(),
            errors,
        );
    }

    let source_ids = sources
        .iter()
        .map(|source| source.source_reference_digest.clone())
        .collect::<Vec<_>>();
    let actual_universe_digest = match compute_source_universe_digest(&source_ids) {
        Ok(digest) => digest,
        Err(error) => {
            return empty_report(
                claim,
                ScopedAbsenceStatus::Fail,
                ScopedAbsenceConclusion::SearchCoverageIncomplete,
                scope_digest,
                scope.declared_source_count,
                sources.len(),
                vec![error],
            );
        }
    };
    if sources.len() == scope.declared_source_count as usize
        && actual_universe_digest != scope.source_universe_digest
    {
        return empty_report(
            claim,
            ScopedAbsenceStatus::Fail,
            ScopedAbsenceConclusion::SearchCoverageIncomplete,
            scope_digest,
            scope.declared_source_count,
            sources.len(),
            vec!["source-record identities do not match the declared source-universe digest".into()],
        );
    }

    let mut report = empty_report(
        claim,
        ScopedAbsenceStatus::Indeterminate,
        ScopedAbsenceConclusion::SearchCoverageIncomplete,
        scope_digest,
        scope.declared_source_count,
        sources.len(),
        Vec::new(),
    );

    for source in sources {
        match source.permission {
            SourcePermissionState::WithheldByRights => report.withheld_source_count += 1,
            SourcePermissionState::Unknown => report.unknown_permission_count += 1,
            SourcePermissionState::Shareable | SourcePermissionState::Scoped => {}
        }
        match source.query_disposition {
            SourceQueryDisposition::Searched => {
                report.searched_source_count += 1;
                match source.match_disposition {
                    SourceMatchDisposition::NoMatch => report.no_match_count += 1,
                    SourceMatchDisposition::CandidateFound => {
                        report.candidate_count += 1;
                        match source.verification {
                            CandidateVerificationState::Verified => report.verified_candidate_count += 1,
                            CandidateVerificationState::Rejected => report.rejected_candidate_count += 1,
                            CandidateVerificationState::Unknown => report.unknown_candidate_verification_count += 1,
                            CandidateVerificationState::NotApplicable => {}
                        }
                    }
                    SourceMatchDisposition::Unknown => report.unknown_match_count += 1,
                }
            }
            SourceQueryDisposition::NoResponse => report.no_response_count += 1,
            SourceQueryDisposition::NotDelivered => report.undelivered_count += 1,
            SourceQueryDisposition::NotQueried => report.unqueried_count += 1,
        }
    }

    report.conclusion = conclusion_from_counts(&report);

    let closed_universe = scope.source_universe_complete
        && scope.candidate_pool_complete
        && scope.source_universe_receipt_digest.is_some()
        && scope.candidate_pool_receipt_digest.is_some()
        && scope.inquiry_execution_receipt_digest.is_some()
        && scope.declared_source_count > 0
        && sources.len() == scope.declared_source_count as usize
        && actual_universe_digest == scope.source_universe_digest;

    if !closed_universe {
        report.status = ScopedAbsenceStatus::Indeterminate;
        report.reasons.push(
            "source universe or candidate pool is not attested complete for the frozen scope".into(),
        );
        return report;
    }
    if report.withheld_source_count > 0 || report.unknown_permission_count > 0 {
        report.status = ScopedAbsenceStatus::Indeterminate;
        report.reasons.push(
            "restricted or unknown-permission sources prevent an absence claim".into(),
        );
        return report;
    }
    if report.no_response_count > 0
        || report.undelivered_count > 0
        || report.unqueried_count > 0
        || report.unknown_match_count > 0
        || report.unknown_candidate_verification_count > 0
        || report.searched_source_count != report.declared_source_count as usize
    {
        report.status = ScopedAbsenceStatus::Indeterminate;
        report.reasons.push(
            "one or more sources were not successfully searched and classified".into(),
        );
        return report;
    }

    if report.verified_candidate_count > 0 {
        report.status = ScopedAbsenceStatus::Fail;
        report.reasons.push("verified evidence was found in the declared search scope".into());
        return report;
    }

    match claim {
        AbsenceClaimKind::NoEvidenceInDeclaredScope if report.candidate_count > 0 => {
            report.status = ScopedAbsenceStatus::Fail;
            report.reasons.push("candidate evidence was found, so a no-evidence claim is false".into());
        }
        AbsenceClaimKind::NoEvidenceInDeclaredScope => {
            report.status = ScopedAbsenceStatus::Pass;
            report.conclusion = ScopedAbsenceConclusion::NoEvidenceFoundInClosedScope;
            report.reasons.push("all declared sources were authorized and searched; every search completed with no match".into());
        }
        AbsenceClaimKind::NoVerifiedEvidenceInDeclaredScope => {
            report.status = ScopedAbsenceStatus::Pass;
            report.conclusion = ScopedAbsenceConclusion::NoVerifiedEvidenceFoundInClosedScope;
            report.reasons.push("all declared sources were authorized and searched; no verified candidate evidence was found".into());
        }
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn source(
        id: char,
        permission: SourcePermissionState,
        query: SourceQueryDisposition,
        matching: SourceMatchDisposition,
        verification: CandidateVerificationState,
    ) -> ScopedSourceDisposition {
        ScopedSourceDisposition {
            source_reference_digest: digest(id),
            permission,
            query_disposition: query,
            match_disposition: matching,
            verification,
            disposition_receipt_digest: digest('e'),
            query_receipt_digest: if query == SourceQueryDisposition::NotQueried {
                None
            } else {
                Some(digest('f'))
            },
        }
    }

    fn scope(sources: &[ScopedSourceDisposition]) -> ScopedAbsenceScope {
        let ids = sources
            .iter()
            .map(|source| source.source_reference_digest.clone())
            .collect::<Vec<_>>();
        ScopedAbsenceScope {
            request_id: "request-1".into(),
            recipient_id: "assistant-1".into(),
            purpose_id: "project-decision".into(),
            query_digest: digest('1'),
            subject_scope_digest: digest('2'),
            frozen_history_cutoff_ms: 1000,
            source_universe_digest: compute_source_universe_digest(&ids).unwrap(),
            declared_source_count: sources.len() as u64,
            candidate_pool_digest: digest('3'),
            source_universe_complete: true,
            candidate_pool_complete: true,
            source_universe_receipt_digest: Some(digest('4')),
            candidate_pool_receipt_digest: Some(digest('5')),
            inquiry_plan_digest: digest('6'),
            inquiry_execution_receipt_digest: Some(digest('7')),
        }
    }

    #[test]
    fn closed_authorized_search_with_no_matches_qualifies_no_evidence_claim() {
        let sources = vec![
            source('a', SourcePermissionState::Shareable, SourceQueryDisposition::Searched, SourceMatchDisposition::NoMatch, CandidateVerificationState::NotApplicable),
            source('b', SourcePermissionState::Scoped, SourceQueryDisposition::Searched, SourceMatchDisposition::NoMatch, CandidateVerificationState::NotApplicable),
        ];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Pass);
        assert_eq!(report.conclusion, ScopedAbsenceConclusion::NoEvidenceFoundInClosedScope);
        assert_eq!(report.no_match_count, 2);
    }

    #[test]
    fn verified_candidate_rejects_absence_claim() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::Searched,
            SourceMatchDisposition::CandidateFound,
            CandidateVerificationState::Verified,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Fail);
        assert_eq!(report.conclusion, ScopedAbsenceConclusion::VerifiedEvidenceFound);
    }

    #[test]
    fn rejected_candidate_allows_only_no_verified_evidence_claim() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::Searched,
            SourceMatchDisposition::CandidateFound,
            CandidateVerificationState::Rejected,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoVerifiedEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Pass);
        assert_eq!(report.conclusion, ScopedAbsenceConclusion::NoVerifiedEvidenceFoundInClosedScope);
    }

    #[test]
    fn rejected_candidate_does_not_allow_no_evidence_claim() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::Searched,
            SourceMatchDisposition::CandidateFound,
            CandidateVerificationState::Rejected,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Fail);
        assert_eq!(report.candidate_count, 1);
    }

    #[test]
    fn unknown_candidate_verification_is_indeterminate() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::Searched,
            SourceMatchDisposition::CandidateFound,
            CandidateVerificationState::Unknown,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoVerifiedEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Indeterminate);
    }

    #[test]
    fn withheld_source_never_becomes_evidence_of_absence() {
        let sources = vec![source(
            'a',
            SourcePermissionState::WithheldByRights,
            SourceQueryDisposition::NotQueried,
            SourceMatchDisposition::Unknown,
            CandidateVerificationState::Unknown,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Indeterminate);
        assert_eq!(report.withheld_source_count, 1);
    }

    #[test]
    fn unknown_permission_never_becomes_evidence_of_absence() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Unknown,
            SourceQueryDisposition::NotQueried,
            SourceMatchDisposition::Unknown,
            CandidateVerificationState::Unknown,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Indeterminate);
        assert_eq!(report.unknown_permission_count, 1);
    }

    #[test]
    fn delivered_query_without_response_is_not_a_completed_no_match() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::NoResponse,
            SourceMatchDisposition::Unknown,
            CandidateVerificationState::Unknown,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Indeterminate);
        assert_eq!(report.no_response_count, 1);
    }

    #[test]
    fn undelivered_query_is_not_a_completed_no_match() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::NotDelivered,
            SourceMatchDisposition::Unknown,
            CandidateVerificationState::Unknown,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Indeterminate);
        assert_eq!(report.undelivered_count, 1);
    }

    #[test]
    fn unqueried_source_is_not_a_completed_no_match() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::NotQueried,
            SourceMatchDisposition::Unknown,
            CandidateVerificationState::Unknown,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Indeterminate);
        assert_eq!(report.unqueried_count, 1);
    }

    #[test]
    fn incomplete_source_universe_is_indeterminate() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::Searched,
            SourceMatchDisposition::NoMatch,
            CandidateVerificationState::NotApplicable,
        )];
        let mut query = scope(&sources);
        query.source_universe_complete = false;
        let report = qualify_scoped_absence_claim(
            &query, &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Indeterminate);
    }

    #[test]
    fn incomplete_candidate_pool_is_indeterminate() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::Searched,
            SourceMatchDisposition::NoMatch,
            CandidateVerificationState::NotApplicable,
        )];
        let mut query = scope(&sources);
        query.candidate_pool_complete = false;
        let report = qualify_scoped_absence_claim(
            &query, &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Indeterminate);
    }

    #[test]
    fn omitted_source_record_is_indeterminate() {
        let sources = vec![
            source('a', SourcePermissionState::Shareable, SourceQueryDisposition::Searched, SourceMatchDisposition::NoMatch, CandidateVerificationState::NotApplicable),
            source('b', SourcePermissionState::Shareable, SourceQueryDisposition::Searched, SourceMatchDisposition::NoMatch, CandidateVerificationState::NotApplicable),
        ];
        let query = scope(&sources);
        let report = qualify_scoped_absence_claim(
            &query, &sources[..1], AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Indeterminate);
    }

    #[test]
    fn duplicate_source_records_fail_integrity() {
        let item = source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::Searched,
            SourceMatchDisposition::NoMatch,
            CandidateVerificationState::NotApplicable,
        );
        let sources = vec![item.clone(), item];
        let mut query = scope(&sources);
        query.declared_source_count = 2;
        let report = qualify_scoped_absence_claim(
            &query, &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Fail);
    }

    #[test]
    fn source_universe_digest_substitution_fails_integrity() {
        let sources = vec![source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::Searched,
            SourceMatchDisposition::NoMatch,
            CandidateVerificationState::NotApplicable,
        )];
        let mut query = scope(&sources);
        query.source_universe_digest = digest('f');
        let report = qualify_scoped_absence_claim(
            &query, &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Fail);
    }

    #[test]
    fn no_match_cannot_carry_verified_candidate_state() {
        let mut item = source(
            'a',
            SourcePermissionState::Shareable,
            SourceQueryDisposition::Searched,
            SourceMatchDisposition::NoMatch,
            CandidateVerificationState::Verified,
        );
        item.query_receipt_digest = Some(digest('f'));
        let report = qualify_scoped_absence_claim(
            &scope(std::slice::from_ref(&item)),
            &[item],
            AbsenceClaimKind::NoEvidenceInDeclaredScope,
        );
        assert_eq!(report.status, ScopedAbsenceStatus::Fail);
    }

    #[test]
    fn reports_do_not_expose_source_reference_digests() {
        let sources = vec![source(
            'a',
            SourcePermissionState::WithheldByRights,
            SourceQueryDisposition::NotQueried,
            SourceMatchDisposition::Unknown,
            CandidateVerificationState::Unknown,
        )];
        let report = qualify_scoped_absence_claim(
            &scope(&sources), &sources, AbsenceClaimKind::NoEvidenceInDeclaredScope
        );
        let debug = format!("{report:?}");
        assert!(!debug.contains(&sources[0].source_reference_digest));
    }
}
