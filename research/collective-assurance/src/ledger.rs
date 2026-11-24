//! Append-only, hash-chained evidence lineage for collective deliberation.
//!
//! This module records observed workflow events and validates causal order,
//! disclosure rights, evidence verification, participant visibility and claim
//!/decision revision references. Hash chaining makes edits detectable; it does
//! not authenticate the recorder or prove that an event happened as claimed.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

const LEDGER_DOMAIN: &[u8] = b"MYCELIX-COLLECTIVE-EVIDENCE-LEDGER-V0.1\0";
const GENESIS_DOMAIN: &[u8] = b"MYCELIX-COLLECTIVE-EVIDENCE-LEDGER-GENESIS-V0.1\0";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DisclosurePermission {
    Shareable,
    Scoped,
    WithheldByRights,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DisclosureOutcome {
    Granted,
    WithheldByRights,
    NoResponse,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VerificationOutcome {
    Verified,
    Rejected,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClaimDisposition {
    Supported,
    Disputed,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecisionDisposition {
    Decide,
    Defer,
    Abstain,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ClaimReference {
    pub claim_id: String,
    pub revision: u32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LedgerEventKind {
    InformationAssigned {
        information_id: String,
        source_origin_id: String,
        permission: DisclosurePermission,
        permitted_recipients: Vec<String>,
    },
    RequestCreated {
        request_id: String,
        information_id: String,
    },
    RequestDelivered {
        request_id: String,
    },
    Disclosure {
        request_id: String,
        outcome: DisclosureOutcome,
        evidence_id: Option<String>,
        payload_digest: Option<String>,
    },
    EvidenceVerified {
        evidence_id: String,
        outcome: VerificationOutcome,
        payload_digest: String,
    },
    InformationViewed {
        evidence_id: String,
    },
    ClaimRevised {
        claim_id: String,
        revision: u32,
        disposition: ClaimDisposition,
        supporting_evidence_ids: Vec<String>,
        contradicting_evidence_ids: Vec<String>,
    },
    DecisionRecorded {
        decision_id: String,
        revision: u32,
        disposition: DecisionDisposition,
        claims: Vec<ClaimReference>,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LedgerEvent {
    pub sequence: u64,
    pub event_id: String,
    pub timestamp_ms: u64,
    pub actor_id: String,
    pub recipient_id: Option<String>,
    pub parent_event_id: Option<String>,
    pub previous_digest: String,
    pub event_digest: String,
    pub kind: LedgerEventKind,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LedgerValidationReport {
    pub valid: bool,
    pub event_count: usize,
    pub root_digest: Option<String>,
    pub errors: Vec<String>,
}

#[derive(Clone, Debug)]
struct AssignmentState {
    event_id: String,
    holder_id: String,
    source_origin_id: String,
    permission: DisclosurePermission,
    permitted_recipients: BTreeSet<String>,
}

#[derive(Clone, Debug)]
struct RequestState {
    event_id: String,
    information_id: String,
    requester_id: String,
    holder_id: String,
    delivered_event_id: Option<String>,
    disclosed: bool,
}

#[derive(Clone, Debug)]
struct EvidenceState {
    disclosure_event_id: String,
    source_holder_id: String,
    source_origin_id: String,
    recipient_id: String,
    payload_digest: String,
    verified: bool,
    rejected: bool,
    viewers: BTreeSet<String>,
}

#[derive(Clone, Debug)]
struct ClaimState {
    event_id: String,
    revision: u32,
    disposition: ClaimDisposition,
}

#[derive(Clone, Debug)]
struct DecisionState {
    event_id: String,
    revision: u32,
}

fn push_len(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_bytes(out: &mut Vec<u8>, value: &[u8]) {
    push_len(out, value.len());
    out.extend_from_slice(value);
}

fn push_str(out: &mut Vec<u8>, value: &str) {
    push_bytes(out, value.as_bytes());
}

fn push_optional_str(out: &mut Vec<u8>, value: Option<&str>) {
    match value {
        Some(value) => {
            out.push(1);
            push_str(out, value);
        }
        None => out.push(0),
    }
}

fn push_string_list(out: &mut Vec<u8>, values: &[String]) {
    push_len(out, values.len());
    for value in values {
        push_str(out, value);
    }
}

fn encode_kind(out: &mut Vec<u8>, kind: &LedgerEventKind) {
    match kind {
        LedgerEventKind::InformationAssigned {
            information_id,
            source_origin_id,
            permission,
            permitted_recipients,
        } => {
            out.push(1);
            push_str(out, information_id);
            push_str(out, source_origin_id);
            out.push(match permission {
                DisclosurePermission::Shareable => 1,
                DisclosurePermission::Scoped => 2,
                DisclosurePermission::WithheldByRights => 3,
                DisclosurePermission::Unknown => 4,
            });
            push_string_list(out, permitted_recipients);
        }
        LedgerEventKind::RequestCreated {
            request_id,
            information_id,
        } => {
            out.push(2);
            push_str(out, request_id);
            push_str(out, information_id);
        }
        LedgerEventKind::RequestDelivered { request_id } => {
            out.push(3);
            push_str(out, request_id);
        }
        LedgerEventKind::Disclosure {
            request_id,
            outcome,
            evidence_id,
            payload_digest,
        } => {
            out.push(4);
            push_str(out, request_id);
            out.push(match outcome {
                DisclosureOutcome::Granted => 1,
                DisclosureOutcome::WithheldByRights => 2,
                DisclosureOutcome::NoResponse => 3,
            });
            push_optional_str(out, evidence_id.as_deref());
            push_optional_str(out, payload_digest.as_deref());
        }
        LedgerEventKind::EvidenceVerified {
            evidence_id,
            outcome,
            payload_digest,
        } => {
            out.push(5);
            push_str(out, evidence_id);
            out.push(match outcome {
                VerificationOutcome::Verified => 1,
                VerificationOutcome::Rejected => 2,
            });
            push_str(out, payload_digest);
        }
        LedgerEventKind::InformationViewed { evidence_id } => {
            out.push(6);
            push_str(out, evidence_id);
        }
        LedgerEventKind::ClaimRevised {
            claim_id,
            revision,
            disposition,
            supporting_evidence_ids,
            contradicting_evidence_ids,
        } => {
            out.push(7);
            push_str(out, claim_id);
            out.extend_from_slice(&revision.to_be_bytes());
            out.push(match disposition {
                ClaimDisposition::Supported => 1,
                ClaimDisposition::Disputed => 2,
                ClaimDisposition::Unknown => 3,
            });
            push_string_list(out, supporting_evidence_ids);
            push_string_list(out, contradicting_evidence_ids);
        }
        LedgerEventKind::DecisionRecorded {
            decision_id,
            revision,
            disposition,
            claims,
        } => {
            out.push(8);
            push_str(out, decision_id);
            out.extend_from_slice(&revision.to_be_bytes());
            out.push(match disposition {
                DecisionDisposition::Decide => 1,
                DecisionDisposition::Defer => 2,
                DecisionDisposition::Abstain => 3,
            });
            push_len(out, claims.len());
            for claim in claims {
                push_str(out, &claim.claim_id);
                out.extend_from_slice(&claim.revision.to_be_bytes());
            }
        }
    }
}

impl LedgerEvent {
    /// Construct an event and calculate its hash-chain digest.
    ///
    /// The first event must use [genesis_digest]; each later event must use
    /// the preceding event's digest. Event hashes are tamper-evident, not
    /// signatures: recorder identity requires a separate signed receipt.
    #[allow(clippy::too_many_arguments)]
    pub fn seal(
        sequence: u64,
        event_id: impl Into<String>,
        timestamp_ms: u64,
        actor_id: impl Into<String>,
        recipient_id: Option<String>,
        parent_event_id: Option<String>,
        previous_digest: impl Into<String>,
        kind: LedgerEventKind,
    ) -> Result<Self, String> {
        let mut event = Self {
            sequence,
            event_id: event_id.into(),
            timestamp_ms,
            actor_id: actor_id.into(),
            recipient_id,
            parent_event_id,
            previous_digest: previous_digest.into(),
            event_digest: String::new(),
            kind,
        };
        if !is_sha256(&event.previous_digest) {
            return Err("previous digest must be lowercase SHA-256 hex".into());
        }
        event.event_digest = event.calculate_digest();
        Ok(event)
    }

    pub fn calculate_digest(&self) -> String {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(LEDGER_DOMAIN);
        bytes.extend_from_slice(&self.sequence.to_be_bytes());
        push_str(&mut bytes, &self.event_id);
        bytes.extend_from_slice(&self.timestamp_ms.to_be_bytes());
        push_str(&mut bytes, &self.actor_id);
        push_optional_str(&mut bytes, self.recipient_id.as_deref());
        push_optional_str(&mut bytes, self.parent_event_id.as_deref());
        push_str(&mut bytes, &self.previous_digest);
        encode_kind(&mut bytes, &self.kind);
        hex_digest(&bytes)
    }
}

pub fn genesis_digest() -> String {
    hex_digest(GENESIS_DOMAIN)
}

fn hex_digest(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn is_nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn add_error(errors: &mut Vec<String>, sequence: u64, message: impl AsRef<str>) {
    errors.push(format!("event {sequence}: {}", message.as_ref()));
}

fn check_parent<'events>(
    event: &LedgerEvent,
    earlier_events: &BTreeMap<String, &'events LedgerEvent>,
    errors: &mut Vec<String>,
) -> Option<&'events LedgerEvent> {
    match event.parent_event_id.as_deref() {
        Some(parent_id) => match earlier_events.get(parent_id) {
            Some(parent) => Some(*parent),
            None => {
                add_error(errors, event.sequence, "parent event is missing or not earlier");
                None
            }
        },
        None => None,
    }
}

fn check_reference_list(values: &[String]) -> bool {
    let mut unique = BTreeSet::new();
    values
        .iter()
        .all(|value| is_nonempty(value) && unique.insert(value.as_str()))
}

pub fn validate_ledger(events: &[LedgerEvent]) -> LedgerValidationReport {
    let mut errors = Vec::new();
    if events.is_empty() {
        errors.push("ledger must contain at least one event".into());
        return LedgerValidationReport {
            valid: false,
            event_count: 0,
            root_digest: None,
            errors,
        };
    }
    let mut assignments_by_id: BTreeMap<String, AssignmentState> = BTreeMap::new();
    let mut requests: BTreeMap<String, RequestState> = BTreeMap::new();
    let mut evidence: BTreeMap<String, EvidenceState> = BTreeMap::new();
    let mut claims: BTreeMap<(String, u32), ClaimState> = BTreeMap::new();
    let mut latest_claims: BTreeMap<String, ClaimState> = BTreeMap::new();
    let mut decisions: BTreeMap<String, DecisionState> = BTreeMap::new();
    let mut seen_events: BTreeMap<String, &LedgerEvent> = BTreeMap::new();
    let mut previous_digest = genesis_digest();
    let mut previous_time = 0;

    for (index, event) in events.iter().enumerate() {
        let expected_sequence = index as u64 + 1;
        if event.sequence != expected_sequence {
            add_error(&mut errors, event.sequence, "sequence must be contiguous from 1");
        }
        if !is_nonempty(&event.event_id)
            || !is_nonempty(&event.actor_id)
            || seen_events.contains_key(&event.event_id)
        {
            add_error(
                &mut errors,
                event.sequence,
                "event id and actor must be non-empty and event ids unique",
            );
        }
        if event.timestamp_ms < previous_time {
            add_error(&mut errors, event.sequence, "timestamp moved backwards");
        }
        previous_time = event.timestamp_ms;

        if event.previous_digest != previous_digest {
            add_error(&mut errors, event.sequence, "previous digest does not match prior event");
        }
        if !is_sha256(&event.event_digest) || event.event_digest != event.calculate_digest() {
            add_error(&mut errors, event.sequence, "event digest is invalid");
        }
        previous_digest = event.event_digest.clone();

        let parent = check_parent(event, &seen_events, &mut errors);

        match &event.kind {
            LedgerEventKind::InformationAssigned {
                information_id,
                source_origin_id,
                permission,
                permitted_recipients,
            } => {
                if event.recipient_id.is_some() || event.parent_event_id.is_some() {
                    add_error(&mut errors, event.sequence, "assignment must not have a recipient or parent");
                }
                if !is_nonempty(information_id)
                    || !is_nonempty(source_origin_id)
                    || assignments_by_id.contains_key(information_id)
                {
                    add_error(&mut errors, event.sequence, "information id must be non-empty and assigned once");
                }
                if !check_reference_list(permitted_recipients) {
                    add_error(&mut errors, event.sequence, "permitted recipients must be unique and non-empty");
                }
                if (*permission == DisclosurePermission::Scoped) == permitted_recipients.is_empty() {
                    add_error(&mut errors, event.sequence, "only scoped permissions may declare a non-empty recipient list");
                }
                assignments_by_id.insert(
                    information_id.clone(),
                    AssignmentState {
                        event_id: event.event_id.clone(),
                        holder_id: event.actor_id.clone(),
                        source_origin_id: source_origin_id.clone(),
                        permission: *permission,
                        permitted_recipients: permitted_recipients.iter().cloned().collect(),
                    },
                );
            }
            LedgerEventKind::RequestCreated {
                request_id,
                information_id,
            } => {
                let Some(assignment) = assignments_by_id.get(information_id) else {
                    add_error(&mut errors, event.sequence, "request references unknown information");
                    continue;
                };
                if !is_nonempty(request_id)
                    || requests.contains_key(request_id)
                    || event.recipient_id.as_deref() != Some(assignment.holder_id.as_str())
                    || event.actor_id == assignment.holder_id
                {
                    add_error(&mut errors, event.sequence, "request id, requester or assigned holder is invalid");
                }
                if event.parent_event_id.as_deref() != Some(assignment.event_id.as_str()) {
                    add_error(&mut errors, event.sequence, "request must cite the information assignment event");
                }
                requests.insert(
                    request_id.clone(),
                    RequestState {
                        event_id: event.event_id.clone(),
                        information_id: information_id.clone(),
                        requester_id: event.actor_id.clone(),
                        holder_id: assignment.holder_id.clone(),
                        delivered_event_id: None,
                        disclosed: false,
                    },
                );
            }
            LedgerEventKind::RequestDelivered { request_id } => {
                let Some(request) = requests.get_mut(request_id) else {
                    add_error(&mut errors, event.sequence, "delivery references unknown request");
                    continue;
                };
                if event.parent_event_id.as_deref() != Some(request.event_id.as_str())
                    || event.recipient_id.as_deref() != Some(request.holder_id.as_str())
                    || request.delivered_event_id.is_some()
                {
                    add_error(&mut errors, event.sequence, "delivery must follow and target its request exactly once");
                }
                request.delivered_event_id = Some(event.event_id.clone());
            }
            LedgerEventKind::Disclosure {
                request_id,
                outcome,
                evidence_id,
                payload_digest,
            } => {
                let Some(request) = requests.get_mut(request_id) else {
                    add_error(&mut errors, event.sequence, "disclosure references unknown request");
                    continue;
                };
                let Some(assignment) = assignments_by_id.get(&request.information_id) else {
                    add_error(&mut errors, event.sequence, "disclosure lost its information assignment");
                    continue;
                };
                if event.parent_event_id.as_deref()
                    != request.delivered_event_id.as_deref()
                    || event.actor_id != request.holder_id
                    || event.recipient_id.as_deref() != Some(request.requester_id.as_str())
                    || request.delivered_event_id.is_none()
                    || request.disclosed
                {
                    add_error(&mut errors, event.sequence, "disclosure must follow delivery, come from the holder and target the requester");
                }
                request.disclosed = true;
                match outcome {
                    DisclosureOutcome::Granted => {
                        let allowed = match assignment.permission {
                            DisclosurePermission::Shareable => true,
                            DisclosurePermission::Scoped => assignment
                                .permitted_recipients
                                .contains(&request.requester_id),
                            DisclosurePermission::WithheldByRights | DisclosurePermission::Unknown => false,
                        };
                        if !allowed {
                            add_error(&mut errors, event.sequence, "rights do not authorize this disclosure");
                        }
                        let (Some(evidence_id), Some(payload_digest)) =
                            (evidence_id.as_ref(), payload_digest.as_ref())
                        else {
                            add_error(&mut errors, event.sequence, "granted disclosure requires evidence id and payload digest");
                            continue;
                        };
                        if !is_nonempty(evidence_id)
                            || !is_sha256(payload_digest)
                            || evidence.contains_key(evidence_id)
                        {
                            add_error(&mut errors, event.sequence, "evidence id must be unique and payload digest valid");
                            continue;
                        }
                        evidence.insert(
                            evidence_id.clone(),
                            EvidenceState {
                                disclosure_event_id: event.event_id.clone(),
                                source_holder_id: request.holder_id.clone(),
                                source_origin_id: assignment.source_origin_id.clone(),
                                recipient_id: request.requester_id.clone(),
                                payload_digest: payload_digest.clone(),
                                verified: false,
                                rejected: false,
                                viewers: BTreeSet::new(),
                            },
                        );
                    }
                    DisclosureOutcome::WithheldByRights | DisclosureOutcome::NoResponse => {
                        if evidence_id.is_some() || payload_digest.is_some() {
                            add_error(&mut errors, event.sequence, "withheld or unanswered disclosure must not expose evidence identifiers or payload digests");
                        }
                        if *outcome == DisclosureOutcome::WithheldByRights
                            && assignment.permission != DisclosurePermission::WithheldByRights
                        {
                            add_error(&mut errors, event.sequence, "rights withholding does not match declared permission");
                        }
                    }
                }
            }
            LedgerEventKind::EvidenceVerified {
                evidence_id,
                outcome,
                payload_digest,
            } => {
                let Some(item) = evidence.get_mut(evidence_id) else {
                    add_error(&mut errors, event.sequence, "verification references unknown evidence");
                    continue;
                };
                if event.parent_event_id.as_deref() != Some(item.disclosure_event_id.as_str())
                    || event.actor_id == item.source_holder_id
                    || event.actor_id == item.recipient_id
                    || !is_sha256(payload_digest)
                    || *payload_digest != item.payload_digest
                {
                    add_error(&mut errors, event.sequence, "verification must be independent and match the disclosed payload");
                }
                if item.verified || item.rejected {
                    add_error(&mut errors, event.sequence, "evidence may be verified only once");
                }
                item.verified = *outcome == VerificationOutcome::Verified;
                item.rejected = *outcome == VerificationOutcome::Rejected;
            }
            LedgerEventKind::InformationViewed { evidence_id } => {
                let Some(item) = evidence.get_mut(evidence_id) else {
                    add_error(&mut errors, event.sequence, "view references unknown evidence");
                    continue;
                };
                if event.parent_event_id.as_deref() != Some(item.disclosure_event_id.as_str())
                    || event.actor_id != item.recipient_id
                    || event.recipient_id.is_some()
                {
                    add_error(&mut errors, event.sequence, "view must be recorded by the authorized recipient and cite its disclosure");
                }
                if !item.viewers.insert(event.actor_id.clone()) {
                    add_error(&mut errors, event.sequence, "duplicate evidence view");
                }
            }
            LedgerEventKind::ClaimRevised {
                claim_id,
                revision,
                disposition,
                supporting_evidence_ids,
                contradicting_evidence_ids,
            } => {
                if !is_nonempty(claim_id)
                    || *revision == 0
                    || !check_reference_list(supporting_evidence_ids)
                    || !check_reference_list(contradicting_evidence_ids)
                {
                    add_error(&mut errors, event.sequence, "claim identity, revision or evidence references are invalid");
                }
                let previous = latest_claims.get(claim_id);
                match (revision, previous, event.parent_event_id.as_deref()) {
                    (1, None, None) => {}
                    (next, Some(prior), Some(parent_id))
                        if *next == prior.revision + 1 && parent_id == prior.event_id => {}
                    _ => add_error(&mut errors, event.sequence, "claim revision must point to the immediately previous revision"),
                }
                let supporting: BTreeSet<&str> =
                    supporting_evidence_ids.iter().map(String::as_str).collect();
                let contradicting: BTreeSet<&str> =
                    contradicting_evidence_ids.iter().map(String::as_str).collect();
                if supporting.intersection(&contradicting).next().is_some() {
                    add_error(&mut errors, event.sequence, "supporting and contradicting evidence must be disjoint");
                }
                for evidence_id in supporting_evidence_ids.iter().chain(contradicting_evidence_ids) {
                    match evidence.get(evidence_id) {
                        Some(item)
                            if item.verified
                                && !item.rejected
                                && item.viewers.contains(&event.actor_id) => {}
                        Some(_) => add_error(&mut errors, event.sequence, "claim author has not viewed independently verified evidence"),
                        None => add_error(&mut errors, event.sequence, "claim references unknown evidence"),
                    }
                }
                match disposition {
                    ClaimDisposition::Supported
                        if supporting_evidence_ids.is_empty() || !contradicting_evidence_ids.is_empty() =>
                    {
                        add_error(&mut errors, event.sequence, "supported claim requires supporting evidence and no unresolved contradiction");
                    }
                    ClaimDisposition::Disputed
                        if supporting_evidence_ids.is_empty() || contradicting_evidence_ids.is_empty() =>
                    {
                        add_error(&mut errors, event.sequence, "disputed claim requires both supporting and contradicting evidence");
                    }
                    ClaimDisposition::Disputed => {
                        let supporting_sources: BTreeSet<&str> = supporting_evidence_ids
                            .iter()
                            .filter_map(|id| evidence.get(id).map(|item| item.source_origin_id.as_str()))
                            .collect();
                        let contradicting_sources: BTreeSet<&str> = contradicting_evidence_ids
                            .iter()
                            .filter_map(|id| evidence.get(id).map(|item| item.source_origin_id.as_str()))
                            .collect();
                        if !supporting_sources.is_disjoint(&contradicting_sources) {
                            add_error(&mut errors, event.sequence, "disputed claim requires independent supporting and contradicting origins");
                        }
                    }
                    ClaimDisposition::Unknown
                        if !supporting_evidence_ids.is_empty() || !contradicting_evidence_ids.is_empty() =>
                    {
                        add_error(&mut errors, event.sequence, "unknown claim must not cite evidence as support or contradiction");
                    }
                    _ => {}
                }
                let state = ClaimState {
                    event_id: event.event_id.clone(),
                    revision: *revision,
                    disposition: *disposition,
                };
                claims.insert((claim_id.clone(), *revision), state.clone());
                latest_claims.insert(claim_id.clone(), state);
            }
            LedgerEventKind::DecisionRecorded {
                decision_id,
                revision,
                disposition,
                claims: claim_refs,
            } => {
                if !is_nonempty(decision_id) || *revision == 0 {
                    add_error(&mut errors, event.sequence, "decision id and revision must be valid");
                }
                let previous = decisions.get(decision_id);
                match (revision, previous, event.parent_event_id.as_deref()) {
                    (1, None, None) => {}
                    (next, Some(prior), Some(parent_id))
                        if *next == prior.revision + 1 && parent_id == prior.event_id => {}
                    _ => add_error(&mut errors, event.sequence, "decision revision must point to the immediately previous revision"),
                }
                if claim_refs.is_empty() && *disposition == DecisionDisposition::Decide {
                    add_error(&mut errors, event.sequence, "a decision requires at least one explicit claim premise");
                }
                let mut refs_seen = BTreeSet::new();
                let mut all_supported = true;
                for claim_ref in claim_refs {
                    if !refs_seen.insert((claim_ref.claim_id.as_str(), claim_ref.revision)) {
                        add_error(&mut errors, event.sequence, "decision contains duplicate claim references");
                    }
                    match claims.get(&(claim_ref.claim_id.clone(), claim_ref.revision)) {
                        Some(claim) => {
                            all_supported &= claim.disposition == ClaimDisposition::Supported;
                            if latest_claims
                                .get(&claim_ref.claim_id)
                                .is_none_or(|latest| latest.revision != claim_ref.revision)
                            {
                                add_error(&mut errors, event.sequence, "decision references a stale claim revision");
                            }
                        }
                        None => {
                            all_supported = false;
                            add_error(&mut errors, event.sequence, "decision references unknown claim revision");
                        }
                    }
                }
                if *disposition == DecisionDisposition::Decide && !all_supported {
                    add_error(&mut errors, event.sequence, "decisive outcome requires supported, non-disputed claim premises");
                }
                decisions.insert(
                    decision_id.clone(),
                    DecisionState {
                        event_id: event.event_id.clone(),
                        revision: *revision,
                    },
                );
            }
        }

        if let Some(parent) = parent
            && parent.sequence >= event.sequence
        {
            add_error(&mut errors, event.sequence, "parent event must precede child event");
        }
        seen_events.insert(event.event_id.clone(), event);
    }

    LedgerValidationReport {
        valid: errors.is_empty(),
        event_count: events.len(),
        root_digest: events.last().map(|event| event.event_digest.clone()),
        errors,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn append(
        events: &mut Vec<LedgerEvent>,
        event_id: &str,
        timestamp_ms: u64,
        actor_id: &str,
        recipient_id: Option<&str>,
        parent_event_id: Option<&str>,
        kind: LedgerEventKind,
    ) {
        let previous = events
            .last()
            .map(|event| event.event_digest.as_str())
            .unwrap_or_else(|| {
                // Kept separate from the event builder to make the genesis constant explicit.
                ""
            });
        let previous = if previous.is_empty() {
            genesis_digest()
        } else {
            previous.to_owned()
        };
        let event = LedgerEvent::seal(
            events.len() as u64 + 1,
            event_id,
            timestamp_ms,
            actor_id,
            recipient_id.map(str::to_owned),
            parent_event_id.map(str::to_owned),
            previous,
            kind,
        )
        .unwrap();
        events.push(event);
    }

    fn valid_ledger() -> Vec<LedgerEvent> {
        let digest = "a".repeat(64);
        let mut events = Vec::new();
        append(
            &mut events,
            "assigned-1",
            1,
            "holder",
            None,
            None,
            LedgerEventKind::InformationAssigned {
                information_id: "fact-1".into(),
                source_origin_id: "source-record-1".into(),
                permission: DisclosurePermission::Scoped,
                permitted_recipients: vec!["requester".into()],
            },
        );
        append(
            &mut events,
            "request-1",
            2,
            "requester",
            Some("holder"),
            Some("assigned-1"),
            LedgerEventKind::RequestCreated {
                request_id: "r-1".into(),
                information_id: "fact-1".into(),
            },
        );
        append(
            &mut events,
            "delivered-1",
            3,
            "transport",
            Some("holder"),
            Some("request-1"),
            LedgerEventKind::RequestDelivered {
                request_id: "r-1".into(),
            },
        );
        append(
            &mut events,
            "disclosure-1",
            4,
            "holder",
            Some("requester"),
            Some("delivered-1"),
            LedgerEventKind::Disclosure {
                request_id: "r-1".into(),
                outcome: DisclosureOutcome::Granted,
                evidence_id: Some("evidence-1".into()),
                payload_digest: Some(digest.clone()),
            },
        );
        append(
            &mut events,
            "verified-1",
            5,
            "independent-verifier",
            None,
            Some("disclosure-1"),
            LedgerEventKind::EvidenceVerified {
                evidence_id: "evidence-1".into(),
                outcome: VerificationOutcome::Verified,
                payload_digest: digest,
            },
        );
        append(
            &mut events,
            "viewed-1",
            6,
            "requester",
            None,
            Some("disclosure-1"),
            LedgerEventKind::InformationViewed {
                evidence_id: "evidence-1".into(),
            },
        );
        append(
            &mut events,
            "claim-1",
            7,
            "requester",
            None,
            None,
            LedgerEventKind::ClaimRevised {
                claim_id: "claim-1".into(),
                revision: 1,
                disposition: ClaimDisposition::Supported,
                supporting_evidence_ids: vec!["evidence-1".into()],
                contradicting_evidence_ids: vec![],
            },
        );
        append(
            &mut events,
            "decision-1",
            8,
            "decision-maker",
            None,
            None,
            LedgerEventKind::DecisionRecorded {
                decision_id: "decision-1".into(),
                revision: 1,
                disposition: DecisionDisposition::Decide,
                claims: vec![ClaimReference {
                    claim_id: "claim-1".into(),
                    revision: 1,
                }],
            },
        );
        events
    }

    #[test]
    fn empty_ledger_is_not_a_valid_evidence_record() {
        let report = validate_ledger(&[]);
        assert!(!report.valid);
        assert!(report.root_digest.is_none());
    }

    #[test]
    fn valid_causal_evidence_chain_passes() {
        let report = validate_ledger(&valid_ledger());
        assert!(report.valid, "{:?}", report.errors);
        assert_eq!(report.event_count, 8);
        assert!(report.root_digest.is_some());
    }

    #[test]
    fn changed_event_is_detected_by_hash_chain() {
        let mut events = valid_ledger();
        events[3].actor_id = "different-holder".into();
        let report = validate_ledger(&events);
        assert!(!report.valid);
        assert!(report.errors.iter().any(|error| error.contains("event digest")));
    }

    #[test]
    fn out_of_scope_disclosure_fails() {
        let mut events = valid_ledger();
        if let LedgerEventKind::InformationAssigned {
            permitted_recipients,
            ..
        } = &mut events[0].kind
        {
            *permitted_recipients = vec!["other-person".into()];
        }
        events[0].event_digest = events[0].calculate_digest();
        let mut previous = genesis_digest();
        for event in &mut events {
            event.previous_digest = previous;
            event.event_digest = event.calculate_digest();
            previous = event.event_digest.clone();
        }
        let report = validate_ledger(&events);
        assert!(!report.valid);
        assert!(report.errors.iter().any(|error| error.contains("rights do not authorize")));
    }

    #[test]
    fn evidence_cannot_support_claim_before_verification() {
        let mut events = valid_ledger();
        events.remove(4);
        let mut previous = genesis_digest();
        for (index, event) in events.iter_mut().enumerate() {
            event.sequence = index as u64 + 1;
            event.previous_digest = previous;
            event.event_digest = event.calculate_digest();
            previous = event.event_digest.clone();
        }
        let report = validate_ledger(&events);
        assert!(!report.valid);
        assert!(report.errors.iter().any(|error| error.contains("not viewed independently verified")));
    }

    #[test]
    fn rights_based_withholding_is_a_valid_non_disclosure_outcome() {
        let mut events = Vec::new();
        append(
            &mut events,
            "assigned-1",
            1,
            "holder",
            None,
            None,
            LedgerEventKind::InformationAssigned {
                information_id: "fact-secret".into(),
                source_origin_id: "source-secret-1".into(),
                permission: DisclosurePermission::WithheldByRights,
                permitted_recipients: vec![],
            },
        );
        append(
            &mut events,
            "request-1",
            2,
            "requester",
            Some("holder"),
            Some("assigned-1"),
            LedgerEventKind::RequestCreated {
                request_id: "r-secret".into(),
                information_id: "fact-secret".into(),
            },
        );
        append(
            &mut events,
            "delivered-1",
            3,
            "transport",
            Some("holder"),
            Some("request-1"),
            LedgerEventKind::RequestDelivered {
                request_id: "r-secret".into(),
            },
        );
        append(
            &mut events,
            "withheld-1",
            4,
            "holder",
            Some("requester"),
            Some("delivered-1"),
            LedgerEventKind::Disclosure {
                request_id: "r-secret".into(),
                outcome: DisclosureOutcome::WithheldByRights,
                evidence_id: None,
                payload_digest: None,
            },
        );
        let report = validate_ledger(&events);
        assert!(report.valid, "{:?}", report.errors);
    }

    fn append_conflicting_evidence(
        events: &mut Vec<LedgerEvent>,
        holder_id: &str,
        source_origin_id: &str,
    ) {
        let digest = "a".repeat(64);
        append(
            events,
            "assigned-2",
            9,
            holder_id,
            None,
            None,
            LedgerEventKind::InformationAssigned {
                information_id: "fact-2".into(),
                source_origin_id: source_origin_id.into(),
                permission: DisclosurePermission::Shareable,
                permitted_recipients: vec![],
            },
        );
        append(
            events,
            "request-2",
            10,
            "requester",
            Some(holder_id),
            Some("assigned-2"),
            LedgerEventKind::RequestCreated {
                request_id: "r-2".into(),
                information_id: "fact-2".into(),
            },
        );
        append(
            events,
            "delivered-2",
            11,
            "transport",
            Some(holder_id),
            Some("request-2"),
            LedgerEventKind::RequestDelivered {
                request_id: "r-2".into(),
            },
        );
        append(
            events,
            "disclosure-2",
            12,
            holder_id,
            Some("requester"),
            Some("delivered-2"),
            LedgerEventKind::Disclosure {
                request_id: "r-2".into(),
                outcome: DisclosureOutcome::Granted,
                evidence_id: Some("evidence-2".into()),
                payload_digest: Some(digest.clone()),
            },
        );
        append(
            events,
            "verified-2",
            13,
            "independent-verifier-2",
            None,
            Some("disclosure-2"),
            LedgerEventKind::EvidenceVerified {
                evidence_id: "evidence-2".into(),
                outcome: VerificationOutcome::Verified,
                payload_digest: digest,
            },
        );
        append(
            events,
            "viewed-2",
            14,
            "requester",
            None,
            Some("disclosure-2"),
            LedgerEventKind::InformationViewed {
                evidence_id: "evidence-2".into(),
            },
        );
        append(
            events,
            "claim-2",
            15,
            "requester",
            None,
            Some("claim-1"),
            LedgerEventKind::ClaimRevised {
                claim_id: "claim-1".into(),
                revision: 2,
                disposition: ClaimDisposition::Disputed,
                supporting_evidence_ids: vec!["evidence-1".into()],
                contradicting_evidence_ids: vec!["evidence-2".into()],
            },
        );
    }

    #[test]
    fn distinct_origins_can_be_independent_even_with_one_holder() {
        let mut events = valid_ledger();
        append_conflicting_evidence(&mut events, "holder", "source-record-2");
        let report = validate_ledger(&events);
        assert!(report.valid, "{:?}", report.errors);
    }

    #[test]
    fn different_holders_repeating_one_origin_are_not_independent() {
        let mut events = valid_ledger();
        append_conflicting_evidence(&mut events, "holder-2", "source-record-1");
        let report = validate_ledger(&events);
        assert!(!report.valid);
        assert!(report.errors.iter().any(|error| error.contains("independent supporting and contradicting origins")));
    }

    #[test]
    fn claim_must_link_to_prior_revision() {
        let mut events = valid_ledger();
        let second = LedgerEvent::seal(
            9,
            "claim-2",
            9,
            "requester",
            None,
            Some("not-the-prior-event".into()),
            events[7].event_digest.clone(),
            LedgerEventKind::ClaimRevised {
                claim_id: "claim-1".into(),
                revision: 2,
                disposition: ClaimDisposition::Unknown,
                supporting_evidence_ids: vec![],
                contradicting_evidence_ids: vec![],
            },
        )
        .unwrap();
        events.push(second);
        let report = validate_ledger(&events);
        assert!(!report.valid);
        assert!(report.errors.iter().any(|error| error.contains("immediately previous revision")));
    }
}
