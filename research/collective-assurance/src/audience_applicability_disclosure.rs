//! Multi-granularity audience, applicability and disclosure qualification.
//!
//! This module keeps three questions independent:
//! * audience: may this user-agent pair receive the entry?
//! * applicability: is the lesson valid for this pair in this exact situation?
//! * disclosure: may this content be disclosed to this pair under this policy?
//!
//! It is a pure reference qualifier. It is not an identity authority, policy
//! engine, memory store, redaction system, receipt authenticator or runtime gate.

use std::collections::BTreeSet;

use sha2::{Digest, Sha256};

const PROJECTION_DOMAIN: &[u8] = b"MYCELIX-MEMORY-PROJECTION-RECEIPT-V0.1\0";

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty() && value == value.trim() && value != "*"
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

fn push_str(out: &mut Vec<u8>, value: &str) {
    push_len(out, value.len());
    out.extend_from_slice(value.as_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_selector(out: &mut Vec<u8>, selector: &ScopeSelector) {
    match selector {
        ScopeSelector::Pair { user_id, agent_id } => {
            out.push(1);
            push_str(out, user_id);
            push_str(out, agent_id);
        }
        ScopeSelector::User { user_id } => {
            out.push(2);
            push_str(out, user_id);
        }
        ScopeSelector::Agent { agent_id } => {
            out.push(3);
            push_str(out, agent_id);
        }
        ScopeSelector::Global { collective_id } => {
            out.push(4);
            push_str(out, collective_id);
        }
    }
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

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ScopeSelector {
    /// One exact user-agent pair.
    Pair { user_id: String, agent_id: String },
    /// One user across that user's agents.
    User { user_id: String },
    /// One globally identified agent across users.
    Agent { agent_id: String },
    /// All pairs in one explicitly named collective, never every tenant by default.
    Global { collective_id: String },
}

impl ScopeSelector {
    fn validate(&self) -> Result<(), String> {
        match self {
            Self::Pair { user_id, agent_id } => {
                if !nonempty(user_id) || !nonempty(agent_id) {
                    return Err("pair selector requires exact non-empty user and agent IDs".into());
                }
            }
            Self::User { user_id } => {
                if !nonempty(user_id) {
                    return Err("user selector requires an exact non-empty user ID".into());
                }
            }
            Self::Agent { agent_id } => {
                if !nonempty(agent_id) {
                    return Err("agent selector requires an exact non-empty agent ID".into());
                }
            }
            Self::Global { collective_id } => {
                if !nonempty(collective_id) {
                    return Err("global selector requires an exact collective ID".into());
                }
            }
        }
        Ok(())
    }

    fn matches(&self, context: &MemoryUseContext) -> bool {
        match self {
            Self::Pair { user_id, agent_id } => {
                user_id == &context.user_id && agent_id == &context.agent_id
            }
            Self::User { user_id } => user_id == &context.user_id,
            Self::Agent { agent_id } => agent_id == &context.agent_id,
            Self::Global { collective_id } => collective_id == &context.collective_id,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ApplicabilityRule {
    /// The user-agent population for which this lesson generalizes.
    pub selector: ScopeSelector,
    /// Exact task/situation commitment; no empty or wildcard situation.
    pub situation_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DisclosureRule {
    /// The recipient population to which disclosure is allowed by this profile.
    pub selector: ScopeSelector,
    /// Exact disclosure-policy revision.
    pub policy_digest: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourceStream {
    /// The entry is already qualified as reusable content by an upstream owner.
    Reusable,
    /// The source contains protected material; only a separately reviewed
    /// projection may be used by this reference profile.
    Protected,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProjectionReceipt {
    pub receipt_id: String,
    pub entry_id: String,
    pub entry_revision: u64,
    pub input_payload_digest: String,
    pub output_payload_digest: String,
    pub transform_digest: String,
    /// Exact audience clause under which this projection may be received.
    pub audience_scope: ScopeSelector,
    /// Exact applicability clause under which the lesson is valid.
    pub applicability_scope: ScopeSelector,
    /// Exact disclosure clause authorizing this recipient projection.
    pub disclosure_scope: ScopeSelector,
    pub situation_digest: String,
    pub policy_digest: String,
    /// Opaque commitment to an independently produced review record.
    pub independent_review_digest: String,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
    pub receipt_digest: String,
}

/// Recompute the domain-separated digest over every projection-receipt field.
///
/// This proves self-consistent field binding only. It does not authenticate the
/// issuer, prove that a redaction transform removed every secret, or establish
/// that the independent review was competent or authorized.
pub fn compute_projection_receipt_digest(
    receipt: &ProjectionReceipt,
) -> Result<String, String> {
    if !nonempty(&receipt.receipt_id)
        || !nonempty(&receipt.entry_id)
        || receipt.entry_revision == 0
        || !valid_digest(&receipt.input_payload_digest)
        || !valid_digest(&receipt.output_payload_digest)
        || !valid_digest(&receipt.transform_digest)
        || receipt.audience_scope.validate().is_err()
        || receipt.applicability_scope.validate().is_err()
        || receipt.disclosure_scope.validate().is_err()
        || !valid_digest(&receipt.situation_digest)
        || !valid_digest(&receipt.policy_digest)
        || !valid_digest(&receipt.independent_review_digest)
        || receipt.issued_at_ms >= receipt.expires_at_ms
    {
        return Err("projection receipt is invalid or incomplete".into());
    }

    let mut bytes = Vec::new();
    bytes.extend_from_slice(PROJECTION_DOMAIN);
    push_str(&mut bytes, &receipt.receipt_id);
    push_str(&mut bytes, &receipt.entry_id);
    push_u64(&mut bytes, receipt.entry_revision);
    push_str(&mut bytes, &receipt.input_payload_digest);
    push_str(&mut bytes, &receipt.output_payload_digest);
    push_str(&mut bytes, &receipt.transform_digest);
    push_selector(&mut bytes, &receipt.audience_scope);
    push_selector(&mut bytes, &receipt.applicability_scope);
    push_selector(&mut bytes, &receipt.disclosure_scope);
    push_str(&mut bytes, &receipt.situation_digest);
    push_str(&mut bytes, &receipt.policy_digest);
    push_str(&mut bytes, &receipt.independent_review_digest);
    push_u64(&mut bytes, receipt.issued_at_ms);
    push_u64(&mut bytes, receipt.expires_at_ms);
    Ok(sha256_hex(&bytes))
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CollectiveMemoryEntry {
    pub entry_id: String,
    pub revision: u64,
    /// Digest of the protected/source artifact from which this entry was made.
    pub source_payload_digest: String,
    /// Digest of the exact content this consumer would receive.
    pub payload_digest: String,
    /// Who may read the entry. An empty vector is missing evidence, not global.
    pub audience: Vec<ScopeSelector>,
    /// Where the lesson is valid. Separate from permission to read it.
    pub applicability: Vec<ApplicabilityRule>,
    /// Where its content may be disclosed. Separate from audience and utility.
    pub disclosure: Vec<DisclosureRule>,
    pub source_stream: SourceStream,
    pub projection_receipt: Option<ProjectionReceipt>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryUseContext {
    pub user_id: String,
    pub agent_id: String,
    pub collective_id: String,
    pub situation_digest: String,
    pub policy_digest: String,
    pub used_at_ms: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AudienceQualificationStatus {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AudienceQualificationReport {
    pub status: AudienceQualificationStatus,
    pub audience_match: bool,
    pub applicability_match: bool,
    pub disclosure_match: bool,
    pub projection_verified: bool,
    pub reasons: Vec<String>,
}

impl AudienceQualificationReport {
    fn new() -> Self {
        Self {
            status: AudienceQualificationStatus::Pass,
            audience_match: false,
            applicability_match: false,
            disclosure_match: false,
            projection_verified: false,
            reasons: Vec::new(),
        }
    }

    fn fail(&mut self, reason: impl Into<String>) {
        self.status = AudienceQualificationStatus::Fail;
        self.reasons.push(reason.into());
    }

    fn indeterminate(&mut self, reason: impl Into<String>) {
        if self.status != AudienceQualificationStatus::Fail {
            self.status = AudienceQualificationStatus::Indeterminate;
        }
        self.reasons.push(reason.into());
    }
}

fn validate_selector_set(
    selectors: &[ScopeSelector],
    label: &str,
    report: &mut AudienceQualificationReport,
) -> bool {
    if selectors.is_empty() {
        report.indeterminate(format!("{label} evidence is missing"));
        return false;
    }
    let mut seen = BTreeSet::new();
    for selector in selectors {
        if let Err(reason) = selector.validate() {
            report.fail(format!("{label}: {reason}"));
            return false;
        }
        if !seen.insert(selector) {
            report.fail(format!("{label} contains a duplicate selector"));
            return false;
        }
    }
    true
}

/// Qualify one proposed memory use against all three independent boundaries.
///
/// A PASS means only that the exact in-memory declarations and projection
/// receipt commitment satisfy this finite reference contract. Callers must
/// separately authenticate identities, policies, receipts and runtime use.
pub fn qualify_audience_applicability_disclosure(
    entry: &CollectiveMemoryEntry,
    context: &MemoryUseContext,
) -> AudienceQualificationReport {
    let mut report = AudienceQualificationReport::new();

    if !nonempty(&entry.entry_id)
        || entry.revision == 0
        || !valid_digest(&entry.source_payload_digest)
        || !valid_digest(&entry.payload_digest)
    {
        report.fail("entry identity, revision or payload commitment is invalid");
        return report;
    }
    if !nonempty(&context.user_id)
        || !nonempty(&context.agent_id)
        || !nonempty(&context.collective_id)
        || !valid_digest(&context.situation_digest)
        || !valid_digest(&context.policy_digest)
        || context.used_at_ms == 0
    {
        report.fail("use context is invalid or incomplete");
        return report;
    }

    // Validate every dimension before evaluating any one of them. A known
    // mismatch must remain FAIL even if another dimension is missing.
    let audience_complete =
        validate_selector_set(&entry.audience, "audience", &mut report);
    if report.status == AudienceQualificationStatus::Fail {
        return report;
    }

    let applicability_complete = !entry.applicability.is_empty();
    if !applicability_complete {
        report.indeterminate("generalization/applicability evidence is missing");
    }
    for rule in &entry.applicability {
        if rule.selector.validate().is_err() || !valid_digest(&rule.situation_digest) {
            report.fail("applicability rule is malformed");
            return report;
        }
    }

    let disclosure_complete = !entry.disclosure.is_empty();
    if !disclosure_complete {
        report.indeterminate("disclosure-boundary evidence is missing");
    }
    for rule in &entry.disclosure {
        if rule.selector.validate().is_err() || !valid_digest(&rule.policy_digest) {
            report.fail("disclosure rule is malformed");
            return report;
        }
    }

    let audience_matches = entry.audience.iter().any(|scope| scope.matches(context));
    report.audience_match = audience_matches;
    if audience_complete && !audience_matches {
        report.fail("requesting user-agent pair is outside the entry audience");
        return report;
    }

    let applicability_matches = entry.applicability.iter().any(|rule| {
        rule.selector.matches(context) && rule.situation_digest == context.situation_digest
    });
    report.applicability_match = applicability_matches;
    if applicability_complete && !applicability_matches {
        report.fail("entry is not applicable to this pair and exact situation");
        return report;
    }

    let matching_disclosure = entry.disclosure.iter().find(|rule| {
        rule.selector.matches(context)
            && rule.policy_digest == context.policy_digest
            && (entry.source_stream == SourceStream::Reusable
                || entry
                    .projection_receipt
                    .as_ref()
                    .is_some_and(|receipt| receipt.disclosure_scope == rule.selector))
    });
    report.disclosure_match = matching_disclosure.is_some();
    if disclosure_complete && matching_disclosure.is_none() {
        report.fail("content is outside the recipient or policy disclosure boundary");
        return report;
    }

    // Incomplete evidence is not a grant. Only return INDETERMINATE after
    // checking all dimensions for any already-known negative condition.
    if report.status == AudienceQualificationStatus::Indeterminate {
        return report;
    }

    if entry.source_stream == SourceStream::Reusable {
        if entry.projection_receipt.is_some() {
            report.fail("reusable entry unexpectedly carries a protected-stream projection receipt");
            return report;
        }
        report.status = AudienceQualificationStatus::Pass;
        return report;
    }

    let Some(receipt) = &entry.projection_receipt else {
        report.fail("protected source stream has no independently reviewed projection receipt");
        return report;
    };
    let computed = match compute_projection_receipt_digest(receipt) {
        Ok(value) => value,
        Err(reason) => {
            report.fail(reason);
            return report;
        }
    };
    let Some(disclosure_rule) = matching_disclosure else {
        // Defensive: the complete/matched checks above should make this
        // unreachable, but absence must never be converted into a grant.
        report.indeterminate("matching disclosure rule could not be resolved");
        return report;
    };
    if receipt.receipt_digest != computed
        || receipt.entry_id != entry.entry_id
        || receipt.entry_revision != entry.revision
        || receipt.input_payload_digest != entry.source_payload_digest
        || receipt.output_payload_digest != entry.payload_digest
        || !entry.audience.contains(&receipt.audience_scope)
        || !receipt.audience_scope.matches(context)
        || !entry.applicability.iter().any(|rule| {
            rule.selector == receipt.applicability_scope
                && rule.situation_digest == receipt.situation_digest
                && rule.selector.matches(context)
        })
        || receipt.disclosure_scope != disclosure_rule.selector
        || receipt.situation_digest != context.situation_digest
        || receipt.policy_digest != context.policy_digest
        || context.used_at_ms < receipt.issued_at_ms
        || context.used_at_ms >= receipt.expires_at_ms
    {
        report.fail("projection receipt does not bind the exact entry, output, audience, applicability, disclosure, situation, policy or use time");
        return report;
    }

    report.projection_verified = true;
    report.status = AudienceQualificationStatus::Pass;
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    const D1: &str = "1111111111111111111111111111111111111111111111111111111111111111";
    const D2: &str = "2222222222222222222222222222222222222222222222222222222222222222";
    const D3: &str = "3333333333333333333333333333333333333333333333333333333333333333";
    const D4: &str = "4444444444444444444444444444444444444444444444444444444444444444";
    const D5: &str = "5555555555555555555555555555555555555555555555555555555555555555";

    fn pair(user: &str, agent: &str) -> ScopeSelector {
        ScopeSelector::Pair {
            user_id: user.into(),
            agent_id: agent.into(),
        }
    }

    fn context() -> MemoryUseContext {
        MemoryUseContext {
            user_id: "alice".into(),
            agent_id: "planner".into(),
            collective_id: "collective-a".into(),
            situation_digest: D2.into(),
            policy_digest: D3.into(),
            used_at_ms: 50,
        }
    }

    fn base_entry() -> CollectiveMemoryEntry {
        CollectiveMemoryEntry {
            entry_id: "lesson-1".into(),
            revision: 1,
            source_payload_digest: D1.into(),
            payload_digest: D1.into(),
            audience: vec![pair("alice", "planner")],
            applicability: vec![ApplicabilityRule {
                selector: pair("alice", "planner"),
                situation_digest: D2.into(),
            }],
            disclosure: vec![DisclosureRule {
                selector: pair("alice", "planner"),
                policy_digest: D3.into(),
            }],
            source_stream: SourceStream::Reusable,
            projection_receipt: None,
        }
    }

    fn projected_entry() -> CollectiveMemoryEntry {
        let mut entry = base_entry();
        entry.source_stream = SourceStream::Protected;
        entry.payload_digest = D4.into();
        let mut receipt = ProjectionReceipt {
            receipt_id: "projection-1".into(),
            entry_id: entry.entry_id.clone(),
            entry_revision: entry.revision,
            input_payload_digest: entry.source_payload_digest.clone(),
            output_payload_digest: entry.payload_digest.clone(),
            transform_digest: D5.into(),
            audience_scope: entry.audience[0].clone(),
            applicability_scope: entry.applicability[0].selector.clone(),
            disclosure_scope: entry.disclosure[0].selector.clone(),
            situation_digest: D2.into(),
            policy_digest: D3.into(),
            independent_review_digest: D1.into(),
            issued_at_ms: 10,
            expires_at_ms: 100,
            receipt_digest: String::new(),
        };
        receipt.receipt_digest = compute_projection_receipt_digest(&receipt).unwrap();
        entry.projection_receipt = Some(receipt);
        entry
    }

    #[test]
    fn exact_pair_audience_passes_when_all_three_boundaries_match() {
        let report = qualify_audience_applicability_disclosure(&base_entry(), &context());
        assert_eq!(report.status, AudienceQualificationStatus::Pass);
        assert!(report.audience_match);
        assert!(report.applicability_match);
        assert!(report.disclosure_match);
    }

    #[test]
    fn user_scope_matches_other_agents_of_the_same_user() {
        let mut entry = base_entry();
        let selector = ScopeSelector::User { user_id: "alice".into() };
        entry.audience = vec![selector.clone()];
        entry.applicability[0].selector = selector.clone();
        entry.disclosure[0].selector = selector;
        let mut request = context();
        request.agent_id = "writer".into();
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &request).status,
            AudienceQualificationStatus::Pass
        );
    }

    #[test]
    fn agent_scope_matches_the_same_agent_for_another_user() {
        let mut entry = base_entry();
        let selector = ScopeSelector::Agent { agent_id: "planner".into() };
        entry.audience = vec![selector.clone()];
        entry.applicability[0].selector = selector.clone();
        entry.disclosure[0].selector = selector;
        let mut request = context();
        request.user_id = "bob".into();
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &request).status,
            AudienceQualificationStatus::Pass
        );
    }

    #[test]
    fn global_scope_is_bound_to_one_named_collective() {
        let mut entry = base_entry();
        let selector = ScopeSelector::Global { collective_id: "collective-a".into() };
        entry.audience = vec![selector.clone()];
        entry.applicability[0].selector = selector.clone();
        entry.disclosure[0].selector = selector;
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Pass
        );
        let mut other = context();
        other.collective_id = "collective-b".into();
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &other).status,
            AudienceQualificationStatus::Fail
        );
    }

    #[test]
    fn audience_does_not_imply_applicability() {
        let mut entry = base_entry();
        entry.applicability[0].situation_digest = D5.into();
        let report = qualify_audience_applicability_disclosure(&entry, &context());
        assert!(report.audience_match);
        assert!(!report.applicability_match);
        assert_eq!(report.status, AudienceQualificationStatus::Fail);
    }

    #[test]
    fn applicability_does_not_imply_disclosure() {
        let mut entry = base_entry();
        entry.disclosure[0].selector = pair("bob", "planner");
        let report = qualify_audience_applicability_disclosure(&entry, &context());
        assert!(report.audience_match);
        assert!(report.applicability_match);
        assert!(!report.disclosure_match);
        assert_eq!(report.status, AudienceQualificationStatus::Fail);
    }

    #[test]
    fn missing_audience_is_indeterminate_not_global() {
        let mut entry = base_entry();
        entry.audience.clear();
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Indeterminate
        );
    }

    #[test]
    fn missing_applicability_or_disclosure_is_indeterminate() {
        let mut entry = base_entry();
        entry.applicability.clear();
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Indeterminate
        );
        let mut entry = base_entry();
        entry.disclosure.clear();
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Indeterminate
        );
    }

    #[test]
    fn protected_stream_requires_a_projection_receipt() {
        let mut entry = base_entry();
        entry.source_stream = SourceStream::Protected;
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Fail
        );
    }

    #[test]
    fn exact_reviewed_projection_receipt_passes_structural_qualification() {
        let report = qualify_audience_applicability_disclosure(&projected_entry(), &context());
        assert_eq!(report.status, AudienceQualificationStatus::Pass);
        assert!(report.projection_verified);
    }

    #[test]
    fn projection_receipt_mutation_is_rejected() {
        let mut entry = projected_entry();
        entry.projection_receipt.as_mut().unwrap().policy_digest = D5.into();
        let report = qualify_audience_applicability_disclosure(&entry, &context());
        assert_eq!(report.status, AudienceQualificationStatus::Fail);
        assert!(!report.projection_verified);
    }

    #[test]
    fn projection_cannot_be_replayed_to_another_user_agent_pair() {
        let mut entry = projected_entry();
        let selector = ScopeSelector::User { user_id: "alice".into() };
        entry.audience = vec![selector.clone()];
        entry.applicability[0].selector = selector.clone();
        entry.disclosure[0].selector = selector;
        let mut request = context();
        request.agent_id = "writer".into();
        let report = qualify_audience_applicability_disclosure(&entry, &request);
        assert_eq!(report.status, AudienceQualificationStatus::Fail);
    }

    #[test]
    fn malformed_wildcard_and_duplicate_selectors_fail_closed() {
        let mut entry = base_entry();
        entry.audience = vec![ScopeSelector::User { user_id: "*".into() }];
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Fail
        );
        let mut entry = base_entry();
        entry.audience.push(entry.audience[0].clone());
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Fail
        );
    }

    fn selector_for_audience(index: usize) -> ScopeSelector {
        match index {
            0 => pair("alice", "planner"),
            1 => ScopeSelector::User { user_id: "alice".into() },
            2 => ScopeSelector::Agent { agent_id: "planner".into() },
            3 => ScopeSelector::Global { collective_id: "collective-a".into() },
            _ => unreachable!(),
        }
    }

    fn selector_for_applicability(index: usize) -> ScopeSelector {
        match index {
            0 => pair("alice", "planner"),
            1 => ScopeSelector::User { user_id: "alice".into() },
            2 => ScopeSelector::Agent { agent_id: "planner".into() },
            3 => pair("bob", "writer"),
            _ => unreachable!(),
        }
    }

    #[test]
    fn sixty_four_cell_panel_checks_audience_applicability_and_disclosure_independently() {
        // 4 audience granularities × 4 applicability shapes × 4 disclosure states.
        // Expected outcomes are derived from the independently stated three-way
        // conjunction, not from the implementation's report fields.
        for audience_index in 0..4 {
            for applicability_index in 0..4 {
                for disclosure_index in 0..4 {
                    let mut entry = base_entry();
                    entry.audience = vec![selector_for_audience(audience_index)];
                    entry.applicability[0].selector =
                        selector_for_applicability(applicability_index);
                    entry.disclosure[0].selector = pair("alice", "planner");

                    match disclosure_index {
                        0 => {
                            entry.source_stream = SourceStream::Reusable;
                            entry.projection_receipt = None;
                        }
                        1 | 2 => {
                            entry = projected_entry();
                            entry.audience = vec![selector_for_audience(audience_index)];
                            entry.applicability[0].selector =
                                selector_for_applicability(applicability_index);
                            entry.disclosure[0].selector = pair("alice", "planner");
                            entry.source_stream = SourceStream::Protected;
                            let mut receipt = entry.projection_receipt.take().unwrap();
                            receipt.audience_scope = if disclosure_index == 2 {
                                pair("bob", "writer")
                            } else {
                                entry.audience[0].clone()
                            };
                            receipt.applicability_scope =
                                entry.applicability[0].selector.clone();
                            receipt.disclosure_scope = entry.disclosure[0].selector.clone();
                            receipt.situation_digest = D2.into();
                            receipt.policy_digest = D3.into();
                            receipt.receipt_digest =
                                compute_projection_receipt_digest(&receipt).unwrap();
                            entry.projection_receipt = Some(receipt);
                        }
                        3 => {
                            entry.source_stream = SourceStream::Reusable;
                            entry.disclosure.clear();
                            entry.projection_receipt = None;
                        }
                        _ => unreachable!(),
                    }

                    let report =
                        qualify_audience_applicability_disclosure(&entry, &context());
                    let expected = if applicability_index == 3 || disclosure_index == 2 {
                        AudienceQualificationStatus::Fail
                    } else if disclosure_index == 3 {
                        AudienceQualificationStatus::Indeterminate
                    } else {
                        AudienceQualificationStatus::Pass
                    };
                    assert_eq!(
                        report.status, expected,
                        "unexpected result for audience={audience_index}, applicability={applicability_index}, disclosure={disclosure_index}: {:?}",
                        report.reasons
                    );
                }
            }
        }
    }

    #[test]
    fn protected_projection_must_bind_all_three_exact_scope_clauses() {
        let mut entry = projected_entry();
        let receipt = entry.projection_receipt.as_mut().unwrap();
        receipt.applicability_scope = pair("bob", "writer");
        receipt.receipt_digest = compute_projection_receipt_digest(receipt).unwrap();
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Fail
        );

        let mut entry = projected_entry();
        let receipt = entry.projection_receipt.as_mut().unwrap();
        receipt.audience_scope = ScopeSelector::User { user_id: "bob".into() };
        receipt.receipt_digest = compute_projection_receipt_digest(receipt).unwrap();
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Fail
        );
    }

    #[test]
    fn expired_projection_receipt_cannot_authorize_later_use() {
        let mut entry = projected_entry();
        entry.projection_receipt.as_mut().unwrap().expires_at_ms = 40;
        let receipt = entry.projection_receipt.as_mut().unwrap();
        let digest = compute_projection_receipt_digest(receipt).unwrap();
        receipt.receipt_digest = digest;
        assert_eq!(
            qualify_audience_applicability_disclosure(&entry, &context()).status,
            AudienceQualificationStatus::Fail
        );
    }
}
