//! Independent response-span audit contract for evidence-grounded assistance.
//!
//! An independent auditor annotates material claims in the rendered response
//! and binds each claim to verified evidence origins. This module checks the
//! annotation envelope and span/evidence consistency. It does not perform
//! natural-language claim extraction, judge factual correctness, or authenticate
//! the auditor; those remain separate evaluator and trust-boundary duties.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

use crate::EvidenceStatus;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuditedClaimDisposition {
    Supported,
    Contested,
    Unresolved,
    NonMaterial,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuditedEvidenceRecord {
    pub evidence_id: String,
    pub source_origin_id: String,
    pub source_digest: String,
    pub status: EvidenceStatus,
    pub verified: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MaterialClaimAnnotation {
    pub annotation_id: String,
    pub start_byte: usize,
    pub end_byte: usize,
    /// SHA-256 of the exact UTF-8 response span, to bind annotation to text.
    pub claim_span_digest: String,
    pub disposition: AuditedClaimDisposition,
    pub supporting_evidence_ids: Vec<String>,
    pub contradicting_evidence_ids: Vec<String>,
    pub rationale: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct IndependentResponseAudit {
    pub request_id: String,
    pub response_id: String,
    pub auditor_id: String,
    pub audit_protocol_digest: String,
    /// Auditor attests that every material claim in the response was annotated.
    pub coverage_complete: bool,
    pub response_text: String,
    pub claims: Vec<MaterialClaimAnnotation>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResponseAuditStatus {
    Pass,
    Fail,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ResponseAuditReport {
    pub status: ResponseAuditStatus,
    pub annotated_claims: usize,
    pub supported_claims: usize,
    pub contested_claims: usize,
    pub unresolved_claims: usize,
    pub non_material_claims: usize,
    pub errors: Vec<String>,
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn digest(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

fn valid_reference_ids(ids: &[String]) -> bool {
    let mut unique = BTreeSet::new();
    ids.iter()
        .all(|id| nonempty(id) && unique.insert(id.as_str()))
}

fn evidence_origins<'a>(
    ids: &[String],
    catalog: &'a BTreeMap<&str, &'a AuditedEvidenceRecord>,
    annotation_id: &str,
    errors: &mut Vec<String>,
) -> BTreeSet<&'a str> {
    let mut origins = BTreeSet::new();
    for id in ids {
        match catalog.get(id.as_str()) {
            Some(item) => {
                if item.status != EvidenceStatus::Current || !item.verified {
                    errors.push(format!(
                        "annotation {annotation_id} cites evidence {id} that is not current and independently verified"
                    ));
                }
                origins.insert(item.source_origin_id.as_str());
            }
            None => errors.push(format!(
                "annotation {annotation_id} cites unknown evidence {id}"
            )),
        }
    }
    origins
}

pub fn validate_independent_response_audit(
    audit: &IndependentResponseAudit,
    evidence: &[AuditedEvidenceRecord],
) -> ResponseAuditReport {
    let mut errors = Vec::new();
    let mut catalog = BTreeMap::new();

    if !nonempty(&audit.request_id)
        || !nonempty(&audit.response_id)
        || !nonempty(&audit.auditor_id)
    {
        errors.push("request, response and auditor identities must be non-empty".into());
    }
    if !is_sha256(&audit.audit_protocol_digest) {
        errors.push("audit protocol digest must be lowercase SHA-256 hex".into());
    }
    if !audit.coverage_complete {
        errors.push("auditor did not attest complete material-claim coverage".into());
    }
    if audit.response_text.trim().is_empty() {
        errors.push("response text must not be empty".into());
    }
    if audit.claims.is_empty() {
        errors.push("response audit must annotate at least one material or non-material claim span".into());
    }

    for item in evidence {
        if !nonempty(&item.evidence_id)
            || !nonempty(&item.source_origin_id)
            || !is_sha256(&item.source_digest)
        {
            errors.push(format!(
                "evidence {} has invalid identity, origin or digest",
                item.evidence_id
            ));
        }
        if catalog.insert(item.evidence_id.as_str(), item).is_some() {
            errors.push(format!("duplicate evidence id {}", item.evidence_id));
        }
    }

    let mut annotation_ids = BTreeSet::new();
    let mut spans = Vec::new();
    let mut supported_claims = 0usize;
    let mut contested_claims = 0usize;
    let mut unresolved_claims = 0usize;
    let mut non_material_claims = 0usize;

    for claim in &audit.claims {
        if !nonempty(&claim.annotation_id)
            || !annotation_ids.insert(claim.annotation_id.as_str())
        {
            errors.push("annotation ids must be non-empty and unique".into());
        }
        if claim.start_byte >= claim.end_byte
            || claim.end_byte > audit.response_text.len()
            || !audit.response_text.is_char_boundary(claim.start_byte)
            || !audit.response_text.is_char_boundary(claim.end_byte)
        {
            errors.push(format!(
                "annotation {} has an invalid UTF-8 response span",
                claim.annotation_id
            ));
            continue;
        }
        let span = &audit.response_text[claim.start_byte..claim.end_byte];
        if !nonempty(span) || !is_sha256(&claim.claim_span_digest) {
            errors.push(format!(
                "annotation {} has empty text or invalid span digest",
                claim.annotation_id
            ));
        } else if digest(span.as_bytes()) != claim.claim_span_digest {
            errors.push(format!(
                "annotation {} span digest does not match rendered response text",
                claim.annotation_id
            ));
        }
        spans.push((claim.start_byte, claim.end_byte, claim.annotation_id.as_str()));

        if !valid_reference_ids(&claim.supporting_evidence_ids)
            || !valid_reference_ids(&claim.contradicting_evidence_ids)
        {
            errors.push(format!(
                "annotation {} contains duplicate or empty evidence references",
                claim.annotation_id
            ));
        }
        let supporting_origins = evidence_origins(
            &claim.supporting_evidence_ids,
            &catalog,
            &claim.annotation_id,
            &mut errors,
        );
        let contradicting_origins = evidence_origins(
            &claim.contradicting_evidence_ids,
            &catalog,
            &claim.annotation_id,
            &mut errors,
        );

        match claim.disposition {
            AuditedClaimDisposition::Supported => {
                supported_claims += 1;
                if claim.supporting_evidence_ids.is_empty()
                    || !claim.contradicting_evidence_ids.is_empty()
                {
                    errors.push(format!(
                        "supported annotation {} requires supporting evidence and no unresolved contradiction",
                        claim.annotation_id
                    ));
                }
            }
            AuditedClaimDisposition::Contested => {
                contested_claims += 1;
                if claim.supporting_evidence_ids.is_empty()
                    || claim.contradicting_evidence_ids.is_empty()
                {
                    errors.push(format!(
                        "contested annotation {} requires support and counterevidence",
                        claim.annotation_id
                    ));
                }
                if !supporting_origins.is_disjoint(&contradicting_origins) {
                    errors.push(format!(
                        "contested annotation {} does not preserve independent source origins",
                        claim.annotation_id
                    ));
                }
            }
            AuditedClaimDisposition::Unresolved => {
                unresolved_claims += 1;
                if !claim.supporting_evidence_ids.is_empty()
                    || !claim.contradicting_evidence_ids.is_empty()
                    || claim.rationale.as_deref().is_none_or(|value| !nonempty(value))
                {
                    errors.push(format!(
                        "unresolved annotation {} requires a rationale and no evidence asserted as decisive",
                        claim.annotation_id
                    ));
                }
            }
            AuditedClaimDisposition::NonMaterial => {
                non_material_claims += 1;
                if !claim.supporting_evidence_ids.is_empty()
                    || !claim.contradicting_evidence_ids.is_empty()
                {
                    errors.push(format!(
                        "non-material annotation {} must not carry material evidence citations",
                        claim.annotation_id
                    ));
                }
            }
        }
    }

    spans.sort_by_key(|span| span.0);
    for pair in spans.windows(2) {
        if pair[1].0 < pair[0].1 {
            errors.push(format!(
                "response annotations {} and {} overlap",
                pair[0].2, pair[1].2
            ));
        }
    }

    ResponseAuditReport {
        status: if errors.is_empty() {
            ResponseAuditStatus::Pass
        } else {
            ResponseAuditStatus::Fail
        },
        annotated_claims: audit.claims.len(),
        supported_claims,
        contested_claims,
        unresolved_claims,
        non_material_claims,
        errors,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest_text(text: &str) -> String {
        digest(text.as_bytes())
    }

    fn evidence(id: &str, origin: &str, verified: bool) -> AuditedEvidenceRecord {
        AuditedEvidenceRecord {
            evidence_id: id.into(),
            source_origin_id: origin.into(),
            source_digest: "a".repeat(64),
            status: EvidenceStatus::Current,
            verified,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn annotation(
        id: &str,
        start: usize,
        end: usize,
        text: &str,
        disposition: AuditedClaimDisposition,
        supporting: &[&str],
        contradicting: &[&str],
        rationale: Option<&str>,
    ) -> MaterialClaimAnnotation {
        MaterialClaimAnnotation {
            annotation_id: id.into(),
            start_byte: start,
            end_byte: end,
            claim_span_digest: digest_text(text),
            disposition,
            supporting_evidence_ids: supporting.iter().map(|value| (*value).into()).collect(),
            contradicting_evidence_ids: contradicting.iter().map(|value| (*value).into()).collect(),
            rationale: rationale.map(str::to_owned),
        }
    }

    fn audit(text: &str, claims: Vec<MaterialClaimAnnotation>) -> IndependentResponseAudit {
        IndependentResponseAudit {
            request_id: "request-1".into(),
            response_id: "response-1".into(),
            auditor_id: "independent-auditor-1".into(),
            audit_protocol_digest: "b".repeat(64),
            coverage_complete: true,
            response_text: text.into(),
            claims,
        }
    }

    #[test]
    fn supported_claim_with_current_verified_evidence_passes() {
        let text = "The revised date is 12 June.";
        let claim = annotation(
            "c1", 0, text.len(), text,
            AuditedClaimDisposition::Supported, &["e1"], &[], None,
        );
        let report = validate_independent_response_audit(&audit(text, vec![claim]), &[evidence("e1", "origin-1", true)]);
        assert_eq!(report.status, ResponseAuditStatus::Pass, "{:?}", report.errors);
    }

    #[test]
    fn stale_evidence_cannot_support_a_rendered_claim() {
        let text = "The old date is 10 June.";
        let claim = annotation("c1",0,text.len(),text,AuditedClaimDisposition::Supported,&["e1"],&[],None);
        let mut item = evidence("e1", "origin-1", true);
        item.status = EvidenceStatus::Superseded;
        let report = validate_independent_response_audit(&audit(text, vec![claim]), &[item]);
        assert_eq!(report.status, ResponseAuditStatus::Fail);
    }

    #[test]
    fn unverified_evidence_cannot_support_a_rendered_claim() {
        let text = "The date is 12 June.";
        let claim = annotation("c1",0,text.len(),text,AuditedClaimDisposition::Supported,&["e1"],&[],None);
        let report = validate_independent_response_audit(&audit(text, vec![claim]), &[evidence("e1", "origin-1", false)]);
        assert_eq!(report.status, ResponseAuditStatus::Fail);
    }

    #[test]
    fn contested_claim_requires_independent_supporting_and_contradicting_origins() {
        let text = "Two sources disagree about the date.";
        let claim = annotation(
            "c1",0,text.len(),text,AuditedClaimDisposition::Contested,&["e1"],&["e2"],None,
        );
        let report = validate_independent_response_audit(
            &audit(text, vec![claim]),
            &[evidence("e1","origin-1",true),evidence("e2","origin-2",true)],
        );
        assert_eq!(report.status, ResponseAuditStatus::Pass, "{:?}", report.errors);
    }

    #[test]
    fn contested_claim_cannot_double_count_one_origin() {
        let text = "Two records disagree about the date.";
        let claim = annotation(
            "c1",0,text.len(),text,AuditedClaimDisposition::Contested,&["e1"],&["e2"],None,
        );
        let report = validate_independent_response_audit(
            &audit(text, vec![claim]),
            &[evidence("e1","origin-1",true),evidence("e2","origin-1",true)],
        );
        assert_eq!(report.status, ResponseAuditStatus::Fail);
    }

    #[test]
    fn unresolved_claim_requires_rationale_and_no_decisive_citation() {
        let text = "The date could not be established.";
        let claim = annotation(
            "c1",0,text.len(),text,AuditedClaimDisposition::Unresolved,&[],&[],Some("sources conflict and none is verified"),
        );
        let report = validate_independent_response_audit(&audit(text, vec![claim]), &[]);
        assert_eq!(report.status, ResponseAuditStatus::Pass, "{:?}", report.errors);
    }

    #[test]
    fn incomplete_claim_coverage_fails() {
        let text = "The date is 12 June.";
        let claim = annotation("c1",0,text.len(),text,AuditedClaimDisposition::Supported,&["e1"],&[],None);
        let mut record = audit(text, vec![claim]);
        record.coverage_complete = false;
        let report = validate_independent_response_audit(&record, &[evidence("e1","origin-1",true)]);
        assert_eq!(report.status, ResponseAuditStatus::Fail);
    }

    #[test]
    fn span_must_use_utf8_boundaries() {
        let text = "Café is open.";
        let mut claim = annotation("c1",3,8,"é is ",AuditedClaimDisposition::Unresolved,&[],&[],Some("example"));
        claim.start_byte = 4;
        let report = validate_independent_response_audit(&audit(text, vec![claim]), &[]);
        assert_eq!(report.status, ResponseAuditStatus::Fail);
    }

    #[test]
    fn overlapping_claim_spans_fail() {
        let text = "The revised date is 12 June.";
        let first = annotation("c1",0,11,"The revised",AuditedClaimDisposition::Unresolved,&[],&[],Some("context"));
        let second = annotation("c2",5,text.len(),"vised date is 12 June.",AuditedClaimDisposition::Unresolved,&[],&[],Some("context"));
        let report = validate_independent_response_audit(&audit(text, vec![first,second]), &[]);
        assert_eq!(report.status, ResponseAuditStatus::Fail);
    }

    #[test]
    fn unknown_evidence_reference_fails() {
        let text = "The date is 12 June.";
        let claim = annotation("c1",0,text.len(),text,AuditedClaimDisposition::Supported,&["missing"],&[],None);
        let report = validate_independent_response_audit(&audit(text, vec![claim]), &[]);
        assert_eq!(report.status, ResponseAuditStatus::Fail);
    }

    #[test]
    fn span_digest_binds_annotation_to_exact_rendered_text() {
        let text = "The date is 12 June.";
        let mut claim = annotation("c1",0,text.len(),text,AuditedClaimDisposition::Unresolved,&[],&[],Some("pending"));
        claim.claim_span_digest = "c".repeat(64);
        let report = validate_independent_response_audit(&audit(text, vec![claim]), &[]);
        assert_eq!(report.status, ResponseAuditStatus::Fail);
    }
}
