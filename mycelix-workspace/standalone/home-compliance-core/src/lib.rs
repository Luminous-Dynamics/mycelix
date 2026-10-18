//! A small, deterministic gate for evidence packages produced by a future
//! home-design and compliance compiler.
//!
//! This crate does not interpret building codes, geometry, engineering
//! calculations, or professional credentials. It checks whether a scoped set
//! of mandatory findings is complete enough for professional review. It
//! intentionally never returns a legal "compliant" or "approved" verdict.

#![forbid(unsafe_code)]

pub mod composition;
pub mod evolution;
pub mod reform;
pub mod regulatory;

use std::collections::{HashMap, HashSet};

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RulePackStatus {
    Draft,
    Active,
    Expired,
    Withdrawn,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RulePack {
    pub id: String,
    pub jurisdiction: String,
    pub edition: String,
    /// Exact declared check scope for this edition of the rule pack.
    ///
    /// Production must verify that this manifest is part of the exact hashed
    /// and authenticated pack artifact. This prototype checks scope agreement
    /// with the assessment but does not authenticate the pack or its author.
    pub required_checks: Vec<CheckDefinition>,
    /// A source locator, ideally an official publisher or authority.
    pub source_uri: String,
    /// SHA-256 of the normalized rule-pack artifact, not of a copyrighted
    /// standard. Production must ensure the digest covers this check manifest;
    /// this core validates syntax only, not the digest or artifact authenticity.
    pub pack_sha256: String,
    pub status: RulePackStatus,
    /// UTC calendar day number since 1970-01-01.
    pub effective_from_day: u32,
    /// Exclusive end day. None means no end date is asserted in this pack.
    pub effective_until_day_exclusive: Option<u32>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CheckDefinition {
    /// Stable identifier within the versioned rule pack.
    pub id: String,
    /// Only mandatory, in-scope checks block preparation for professional review.
    pub mandatory: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EvidenceRef {
    /// Stable artifact identifier, such as a drawing, calculation, photo, or test.
    pub artifact_id: String,
    /// SHA-256 digest of the exact artifact bytes.
    pub artifact_sha256: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Result of checking a reference against the actual bytes in a trusted
/// artifact store. A verifier must recompute SHA-256 over the retrieved bytes;
/// comparing the declared digest string to itself is not verification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceVerification {
    Verified,
    ArtifactNotFound,
    DigestMismatch,
    Unavailable,
}

/// Trust boundary between this dependency-free gate and an artifact store.
///
/// Implementations must resolve the stable artifact ID, read the exact immutable
/// object, recompute its digest, and return `Verified` only when the bytes match.
/// The gate cannot independently prove that an implementation is honest or
/// correctly connected to the intended store; production must secure this adapter.
pub trait EvidenceVerifier {
    fn verify(&self, evidence: &EvidenceRef) -> EvidenceVerification;
}

impl<F> EvidenceVerifier for F
where
    F: Fn(&EvidenceRef) -> EvidenceVerification,
{
    fn verify(&self, evidence: &EvidenceRef) -> EvidenceVerification {
        self(evidence)
    }
}

pub enum CheckOutcome {
    Pass,
    Fail,
    Unknown,
    NotApplicable,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Finding {
    pub check_id: String,
    pub rule_pack_id: String,
    pub rule_edition: String,
    pub outcome: CheckOutcome,
    /// Human-readable explanation, not a substitute for machine-verifiable evidence.
    pub rationale: String,
    pub evidence: Vec<EvidenceRef>,
    /// Required for NotApplicable; should explain the scope decision.
    pub not_applicable_justification: Option<String>,
    /// Required for NotApplicable. Production integration must validate this
    /// identity and the person's authority instead of trusting this string.
    pub not_applicable_approved_by: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssessmentInput {
    pub rule_pack: RulePack,
    pub assessed_on_day: u32,
    pub required_checks: Vec<CheckDefinition>,
    pub findings: Vec<Finding>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GateStatus {
    Blocked,
    /// Metadata and scope are coherent, but the referenced evidence bytes have
    /// not yet been checked by an artifact-store verifier.
    MetadataCompleteAwaitingEvidenceVerification,
    /// The metadata gate passed and every referenced evidence artifact was
    /// found and its actual bytes matched the declared SHA-256 digest.
    ReadyForProfessionalReview,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Blocker {
    EmptyScope,
    NoMandatoryChecks,
    InvalidRulePackField(&'static str),
    RulePackNotActive,
    RulePackNotYetEffective,
    RulePackExpired,
    InvalidRulePackValidityWindow,
    InvalidRulePackDigest,
    EmptyRulePackCheckManifest,
    DuplicateRulePackCheck(String),
    MissingRulePackCheck(String),
    UnexpectedAssessmentCheck(String),
    RulePackCheckMandatoryFlagMismatch(String),
    EmptyCheckId,
    DuplicateCheckDefinition(String),
    MissingMandatoryFinding(String),
    DuplicateFinding(String),
    UnknownFinding(String),
    FindingRulePackMismatch(String),
    MissingRationale(String),
    MissingEvidence(String),
    EvidenceArtifactNotFound {
        check_id: String,
        evidence_id: String,
    },
    EvidenceDigestMismatch {
        check_id: String,
        evidence_id: String,
    },
    EvidenceVerificationUnavailable {
        check_id: String,
        evidence_id: String,
    },
    InvalidEvidenceDigest {
        check_id: String,
        evidence_id: String,
    },
    MandatoryCheckFailed(String),
    MandatoryCheckUnknown(String),
    UnjustifiedNotApplicable(String),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GateReport {
    pub status: GateStatus,
    pub blockers: Vec<Blocker>,
    pub pass_count: usize,
    pub fail_count: usize,
    pub unknown_count: usize,
    pub not_applicable_count: usize,
}

fn nonblank(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

/// Evaluate metadata and check-scope consistency only.
///
/// An empty blocker list yields `MetadataCompleteAwaitingEvidenceVerification`,
/// never review readiness. This function does not read referenced artifact bytes
/// or independently confirm their hashes; use `evaluate_verified_review_gate`
/// with a trusted artifact-store adapter before allowing review-ready status.
pub fn evaluate_review_gate(input: &AssessmentInput) -> GateReport {
    let mut blockers = Vec::new();
    let pack = &input.rule_pack;

    for (field, value) in [
        ("id", pack.id.as_str()),
        ("jurisdiction", pack.jurisdiction.as_str()),
        ("edition", pack.edition.as_str()),
        ("source_uri", pack.source_uri.as_str()),
    ] {
        if !nonblank(value) {
            blockers.push(Blocker::InvalidRulePackField(field));
        }
    }

    if !is_sha256_hex(&pack.pack_sha256) {
        blockers.push(Blocker::InvalidRulePackDigest);
    }

    if pack.status != RulePackStatus::Active {
        blockers.push(Blocker::RulePackNotActive);
    }
    if input.assessed_on_day < pack.effective_from_day {
        blockers.push(Blocker::RulePackNotYetEffective);
    }
    if let Some(end_day) = pack.effective_until_day_exclusive {
        if end_day <= pack.effective_from_day {
            blockers.push(Blocker::InvalidRulePackValidityWindow);
        }
        if input.assessed_on_day >= end_day {
            blockers.push(Blocker::RulePackExpired);
        }
    }

    if pack.required_checks.is_empty() {
        blockers.push(Blocker::EmptyRulePackCheckManifest);
    }
    let mut manifest_ids: HashMap<&str, bool> = HashMap::new();
    for check in &pack.required_checks {
        if !nonblank(&check.id) {
            blockers.push(Blocker::EmptyCheckId);
            continue;
        }
        if manifest_ids
            .insert(check.id.as_str(), check.mandatory)
            .is_some()
        {
            blockers.push(Blocker::DuplicateRulePackCheck(check.id.clone()));
        }
    }

    if input.required_checks.is_empty() {
        blockers.push(Blocker::EmptyScope);
    }
    if !input.required_checks.iter().any(|check| check.mandatory) {
        blockers.push(Blocker::NoMandatoryChecks);
    }

    let mut definition_ids = HashSet::new();
    let mut allowed_ids = HashSet::new();
    for check in &input.required_checks {
        if !nonblank(&check.id) {
            blockers.push(Blocker::EmptyCheckId);
            continue;
        }
        allowed_ids.insert(check.id.as_str());
        if !definition_ids.insert(check.id.as_str()) {
            blockers.push(Blocker::DuplicateCheckDefinition(check.id.clone()));
        }
        match manifest_ids.get(check.id.as_str()) {
            None => blockers.push(Blocker::UnexpectedAssessmentCheck(check.id.clone())),
            Some(pack_mandatory) if *pack_mandatory != check.mandatory => {
                blockers.push(Blocker::RulePackCheckMandatoryFlagMismatch(
                    check.id.clone(),
                ));
            }
            Some(_) => {}
        }
    }
    for check_id in manifest_ids.keys() {
        if !input
            .required_checks
            .iter()
            .any(|check| check.id.as_str() == *check_id)
        {
            blockers.push(Blocker::MissingRulePackCheck((*check_id).to_owned()));
        }
    }

    let mut findings_by_id: HashMap<&str, Vec<&Finding>> = HashMap::new();
    let mut seen_findings = HashSet::new();

    for finding in &input.findings {
        if !allowed_ids.contains(finding.check_id.as_str()) {
            blockers.push(Blocker::UnknownFinding(finding.check_id.clone()));
        }

        if !seen_findings.insert(finding.check_id.as_str()) {
            blockers.push(Blocker::DuplicateFinding(finding.check_id.clone()));
        }
        findings_by_id
            .entry(finding.check_id.as_str())
            .or_default()
            .push(finding);

        if finding.rule_pack_id != pack.id || finding.rule_edition != pack.edition {
            blockers.push(Blocker::FindingRulePackMismatch(finding.check_id.clone()));
        }
        if !nonblank(&finding.rationale) {
            blockers.push(Blocker::MissingRationale(finding.check_id.clone()));
        }

        let evidence_required = finding.outcome != CheckOutcome::Unknown;
        if evidence_required && finding.evidence.is_empty() {
            blockers.push(Blocker::MissingEvidence(finding.check_id.clone()));
        }
        for evidence in &finding.evidence {
            if !nonblank(&evidence.artifact_id) || !is_sha256_hex(&evidence.artifact_sha256) {
                blockers.push(Blocker::InvalidEvidenceDigest {
                    check_id: finding.check_id.clone(),
                    evidence_id: evidence.artifact_id.clone(),
                });
            }
        }

        let is_mandatory = input
            .required_checks
            .iter()
            .any(|check| check.id == finding.check_id && check.mandatory);

        match finding.outcome {
            CheckOutcome::Pass => {}
            CheckOutcome::Fail if is_mandatory => {
                blockers.push(Blocker::MandatoryCheckFailed(finding.check_id.clone()));
            }
            CheckOutcome::Fail => {}
            CheckOutcome::Unknown if is_mandatory => {
                blockers.push(Blocker::MandatoryCheckUnknown(finding.check_id.clone()));
            }
            CheckOutcome::Unknown => {}
            CheckOutcome::NotApplicable => {
                let justified = finding
                    .not_applicable_justification
                    .as_deref()
                    .is_some_and(nonblank);
                let approved = finding
                    .not_applicable_approved_by
                    .as_deref()
                    .is_some_and(nonblank);
                if !justified || !approved {
                    blockers.push(Blocker::UnjustifiedNotApplicable(finding.check_id.clone()));
                }
            }
        }
    }

    for check in &input.required_checks {
        if check.mandatory && nonblank(&check.id) && !findings_by_id.contains_key(check.id.as_str())
        {
            blockers.push(Blocker::MissingMandatoryFinding(check.id.clone()));
        }
    }

    let pass_count = input
        .findings
        .iter()
        .filter(|finding| finding.outcome == CheckOutcome::Pass)
        .count();
    let fail_count = input
        .findings
        .iter()
        .filter(|finding| finding.outcome == CheckOutcome::Fail)
        .count();
    let unknown_count = input
        .findings
        .iter()
        .filter(|finding| finding.outcome == CheckOutcome::Unknown)
        .count();
    let not_applicable_count = input
        .findings
        .iter()
        .filter(|finding| finding.outcome == CheckOutcome::NotApplicable)
        .count();

    let status = if blockers.is_empty() {
        GateStatus::MetadataCompleteAwaitingEvidenceVerification
    } else {
        GateStatus::Blocked
    };

    GateReport {
        status,
        blockers,
        pass_count,
        fail_count,
        unknown_count,
        not_applicable_count,
    }
}

/// Verify evidence bytes through a trusted artifact-store adapter before
/// allowing the review-readiness status.
///
/// `evaluate_review_gate` is deliberately only a metadata/scope check; its
/// positive result is `MetadataCompleteAwaitingEvidenceVerification`. This
/// stronger entry point promotes the report to `ReadyForProfessionalReview`
/// only if every referenced evidence object is found and its real bytes pass
/// SHA-256 verification. It is still not legal compliance, engineering safety,
/// certification, approval, or permission to build.
pub fn evaluate_verified_review_gate<V: EvidenceVerifier + ?Sized>(
    input: &AssessmentInput,
    verifier: &V,
) -> GateReport {
    let mut report = evaluate_review_gate(input);

    for finding in &input.findings {
        for evidence in &finding.evidence {
            let blocker = match verifier.verify(evidence) {
                EvidenceVerification::Verified => None,
                EvidenceVerification::ArtifactNotFound => Some(Blocker::EvidenceArtifactNotFound {
                    check_id: finding.check_id.clone(),
                    evidence_id: evidence.artifact_id.clone(),
                }),
                EvidenceVerification::DigestMismatch => Some(Blocker::EvidenceDigestMismatch {
                    check_id: finding.check_id.clone(),
                    evidence_id: evidence.artifact_id.clone(),
                }),
                EvidenceVerification::Unavailable => {
                    Some(Blocker::EvidenceVerificationUnavailable {
                        check_id: finding.check_id.clone(),
                        evidence_id: evidence.artifact_id.clone(),
                    })
                }
            };
            if let Some(blocker) = blocker {
                report.blockers.push(blocker);
            }
        }
    }

    report.status = if report.blockers.is_empty() {
        GateStatus::ReadyForProfessionalReview
    } else {
        GateStatus::Blocked
    };
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    fn good_pack() -> RulePack {
        RulePack {
            id: "za-residential-demo".into(),
            jurisdiction: "ZA/demo-municipality".into(),
            edition: "0.1.0-test".into(),
            required_checks: vec![CheckDefinition {
                id: "structural-review".into(),
                mandatory: true,
            }],
            source_uri: "https://example.invalid/rule-pack".into(),
            pack_sha256: "a".repeat(64),
            status: RulePackStatus::Active,
            effective_from_day: 100,
            effective_until_day_exclusive: Some(200),
        }
    }

    fn evidence() -> EvidenceRef {
        EvidenceRef {
            artifact_id: "drawing:A-101".into(),
            artifact_sha256: "b".repeat(64),
        }
    }

    fn finding(check_id: &str, outcome: CheckOutcome) -> Finding {
        Finding {
            check_id: check_id.into(),
            rule_pack_id: "za-residential-demo".into(),
            rule_edition: "0.1.0-test".into(),
            outcome,
            rationale: "Synthetic test rationale".into(),
            evidence: if outcome == CheckOutcome::Unknown {
                Vec::new()
            } else {
                vec![evidence()]
            },
            not_applicable_justification: None,
            not_applicable_approved_by: None,
        }
    }

    fn input(findings: Vec<Finding>) -> AssessmentInput {
        AssessmentInput {
            rule_pack: good_pack(),
            assessed_on_day: 150,
            required_checks: vec![CheckDefinition {
                id: "structural-review".into(),
                mandatory: true,
            }],
            findings,
        }
    }

    #[test]
    fn complete_mandatory_pass_requires_byte_verification_before_review_ready() {
        let report = evaluate_review_gate(&input(vec![finding(
            "structural-review",
            CheckOutcome::Pass,
        )]));
        assert_eq!(
            report.status,
            GateStatus::MetadataCompleteAwaitingEvidenceVerification
        );
        assert!(report.blockers.is_empty());
    }

    #[test]
    fn mandatory_failure_blocks_review_readiness() {
        let report = evaluate_review_gate(&input(vec![finding(
            "structural-review",
            CheckOutcome::Fail,
        )]));
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::MandatoryCheckFailed("structural-review".into()))
        );
    }

    #[test]
    fn mandatory_unknown_is_not_a_pass() {
        let report = evaluate_review_gate(&input(vec![finding(
            "structural-review",
            CheckOutcome::Unknown,
        )]));
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::MandatoryCheckUnknown("structural-review".into()))
        );
    }

    #[test]
    fn missing_mandatory_finding_blocks() {
        let report = evaluate_review_gate(&input(Vec::new()));
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::MissingMandatoryFinding(
            "structural-review".into()
        )));
    }

    #[test]
    fn pass_without_evidence_blocks() {
        let mut item = finding("structural-review", CheckOutcome::Pass);
        item.evidence.clear();
        let report = evaluate_review_gate(&input(vec![item]));
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::MissingEvidence("structural-review".into()))
        );
    }

    #[test]
    fn malformed_evidence_digest_blocks() {
        let mut item = finding("structural-review", CheckOutcome::Pass);
        item.evidence[0].artifact_sha256 = "not-a-digest".into();
        let report = evaluate_review_gate(&input(vec![item]));
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.iter().any(|blocker| matches!(
            blocker,
            Blocker::InvalidEvidenceDigest { check_id, .. }
                if check_id == "structural-review"
        )));
    }

    #[test]
    fn malformed_rule_pack_digest_blocks() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.rule_pack.pack_sha256 = "not-a-digest".into();
        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::InvalidRulePackDigest));
    }

    #[test]
    fn duplicate_check_definitions_block() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.required_checks.push(CheckDefinition {
            id: "structural-review".into(),
            mandatory: true,
        });
        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::DuplicateCheckDefinition(
            "structural-review".into()
        )));
    }

    #[test]
    fn expired_rule_pack_blocks() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.assessed_on_day = 200;
        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::RulePackExpired));
    }

    #[test]
    fn future_rule_pack_blocks() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.assessed_on_day = 99;
        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::RulePackNotYetEffective));
    }

    #[test]
    fn draft_rule_pack_blocks() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.rule_pack.status = RulePackStatus::Draft;
        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::RulePackNotActive));
    }

    #[test]
    fn finding_from_different_rule_edition_blocks() {
        let mut item = finding("structural-review", CheckOutcome::Pass);
        item.rule_edition = "older-edition".into();
        let report = evaluate_review_gate(&input(vec![item]));
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::FindingRulePackMismatch(
            "structural-review".into()
        )));
    }

    #[test]
    fn unrecognized_finding_blocks() {
        let report = evaluate_review_gate(&input(vec![
            finding("structural-review", CheckOutcome::Pass),
            finding("not-in-pack", CheckOutcome::Pass),
        ]));
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::UnknownFinding("not-in-pack".into()))
        );
    }

    #[test]
    fn duplicate_findings_block() {
        let report = evaluate_review_gate(&input(vec![
            finding("structural-review", CheckOutcome::Pass),
            finding("structural-review", CheckOutcome::Pass),
        ]));
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::DuplicateFinding("structural-review".into()))
        );
    }

    #[test]
    fn not_applicable_requires_justification_and_approval_record() {
        let report = evaluate_review_gate(&input(vec![finding(
            "structural-review",
            CheckOutcome::NotApplicable,
        )]));
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::UnjustifiedNotApplicable(
            "structural-review".into()
        )));
    }

    #[test]
    fn justified_not_applicable_is_review_ready() {
        let mut item = finding("structural-review", CheckOutcome::NotApplicable);
        item.not_applicable_justification = Some("Approved scope exclusion in this fixture".into());
        item.not_applicable_approved_by = Some("synthetic-reviewer-1".into());
        let report = evaluate_review_gate(&input(vec![item]));
        assert_eq!(
            report.status,
            GateStatus::MetadataCompleteAwaitingEvidenceVerification
        );
    }

    #[test]
    fn assessment_cannot_omit_a_check_declared_by_the_rule_pack() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.rule_pack.required_checks.push(CheckDefinition {
            id: "fire-egress-review".into(),
            mandatory: true,
        });

        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::MissingRulePackCheck("fire-egress-review".into()))
        );
    }

    #[test]
    fn assessment_cannot_change_mandatory_status_from_rule_pack() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.required_checks[0].mandatory = false;

        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::RulePackCheckMandatoryFlagMismatch(
                    "structural-review".into()
                ))
        );
    }

    #[test]
    fn assessment_cannot_add_a_check_outside_the_rule_pack() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.required_checks.push(CheckDefinition {
            id: "comfort-recommendation".into(),
            mandatory: false,
        });
        assessment
            .findings
            .push(finding("comfort-recommendation", CheckOutcome::Unknown));

        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::UnexpectedAssessmentCheck(
                    "comfort-recommendation".into()
                ))
        );
    }

    #[test]
    fn duplicate_rule_pack_manifest_entries_block() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        let duplicate = assessment.rule_pack.required_checks[0].clone();
        assessment.rule_pack.required_checks.push(duplicate);

        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::DuplicateRulePackCheck("structural-review".into()))
        );
    }

    #[test]
    fn empty_rule_pack_manifest_blocks() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.rule_pack.required_checks.clear();

        let report = evaluate_review_gate(&assessment);
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::EmptyRulePackCheckManifest)
        );
    }

    #[test]
    fn verified_gate_promotes_only_after_artifact_bytes_are_verified() {
        let assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        let report = evaluate_verified_review_gate(&assessment, &|_: &EvidenceRef| {
            EvidenceVerification::Verified
        });
        assert_eq!(report.status, GateStatus::ReadyForProfessionalReview);
        assert!(report.blockers.is_empty());
    }

    #[test]
    fn missing_evidence_artifact_blocks_verified_gate() {
        let assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        let report = evaluate_verified_review_gate(&assessment, &|_: &EvidenceRef| {
            EvidenceVerification::ArtifactNotFound
        });
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::EvidenceArtifactNotFound {
                    check_id: "structural-review".into(),
                    evidence_id: "drawing:A-101".into(),
                })
        );
    }

    #[test]
    fn mismatched_evidence_bytes_block_verified_gate() {
        let assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        let report = evaluate_verified_review_gate(&assessment, &|_: &EvidenceRef| {
            EvidenceVerification::DigestMismatch
        });
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::EvidenceDigestMismatch {
            check_id: "structural-review".into(),
            evidence_id: "drawing:A-101".into(),
        }));
    }

    #[test]
    fn unavailable_evidence_store_never_counts_as_verified() {
        let assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        let report = evaluate_verified_review_gate(&assessment, &|_: &EvidenceRef| {
            EvidenceVerification::Unavailable
        });
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(
            report
                .blockers
                .contains(&Blocker::EvidenceVerificationUnavailable {
                    check_id: "structural-review".into(),
                    evidence_id: "drawing:A-101".into(),
                })
        );
    }

    #[test]
    fn verified_artifact_bytes_cannot_override_metadata_blockers() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        assessment.required_checks[0].mandatory = false;

        let report = evaluate_verified_review_gate(&assessment, &|_: &EvidenceRef| {
            EvidenceVerification::Verified
        });
        assert_eq!(report.status, GateStatus::Blocked);
        assert!(report.blockers.contains(&Blocker::NoMandatoryChecks));
        assert!(
            report
                .blockers
                .contains(&Blocker::RulePackCheckMandatoryFlagMismatch(
                    "structural-review".into()
                ))
        );
    }

    #[test]
    fn empty_scope_and_scope_without_mandatory_checks_block() {
        let mut empty = input(Vec::new());
        empty.required_checks.clear();
        let empty_report = evaluate_review_gate(&empty);
        assert_eq!(empty_report.status, GateStatus::Blocked);
        assert!(empty_report.blockers.contains(&Blocker::EmptyScope));
        assert!(empty_report.blockers.contains(&Blocker::NoMandatoryChecks));

        let mut optional_only = input(Vec::new());
        optional_only.required_checks[0].mandatory = false;
        let optional_report = evaluate_review_gate(&optional_only);
        assert_eq!(optional_report.status, GateStatus::Blocked);
        assert!(
            optional_report
                .blockers
                .contains(&Blocker::NoMandatoryChecks)
        );
    }

    #[test]
    fn optional_unknown_is_reported_but_does_not_mask_mandatory_gate() {
        let mut assessment = input(vec![finding("structural-review", CheckOutcome::Pass)]);
        let optional_check = CheckDefinition {
            id: "comfort-recommendation".into(),
            mandatory: false,
        };
        assessment.required_checks.push(optional_check.clone());
        assessment.rule_pack.required_checks.push(optional_check);
        assessment
            .findings
            .push(finding("comfort-recommendation", CheckOutcome::Unknown));
        let report = evaluate_review_gate(&assessment);
        assert_eq!(
            report.status,
            GateStatus::MetadataCompleteAwaitingEvidenceVerification
        );
        assert_eq!(report.unknown_count, 1);
    }
}
