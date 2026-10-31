//! Synthetic evidence-custody and verification-boundary evaluator.
//! It verifies generic digital event histories; it does not verify physical truth or law.

use std::collections::{BTreeMap, HashSet};

pub const FIXTURE: &str = include_str!("../fixtures/claim-events-v0.1.csv");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActorRole {
    Declarant,
    Custodian,
    Claimant,
    AuthorizedInspector,
    TreatyInstitution,
    Software,
}

impl ActorRole {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "declarant" => Ok(Self::Declarant),
            "custodian" => Ok(Self::Custodian),
            "claimant" => Ok(Self::Claimant),
            "authorized_inspector" => Ok(Self::AuthorizedInspector),
            "treaty_institution" => Ok(Self::TreatyInstitution),
            "software" => Ok(Self::Software),
            _ => Err(format!("invalid actor_role: {value}")),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityStatus {
    Valid,
    Revoked,
    Expired,
    OutOfScope,
    NotAuthorized,
}

impl AuthorityStatus {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "valid" => Ok(Self::Valid),
            "revoked" => Ok(Self::Revoked),
            "expired" => Ok(Self::Expired),
            "out_of_scope" => Ok(Self::OutOfScope),
            "not_authorized" => Ok(Self::NotAuthorized),
            _ => Err(format!("invalid authority_status: {value}")),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventKind {
    Declaration,
    EvidenceReceived,
    InspectionAttestation,
    TreatyDetermination,
    DisputeOpened,
    DisputeResolved,
}

impl EventKind {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "declaration" => Ok(Self::Declaration),
            "evidence_received" => Ok(Self::EvidenceReceived),
            "inspection_attestation" => Ok(Self::InspectionAttestation),
            "treaty_determination" => Ok(Self::TreatyDetermination),
            "dispute_opened" => Ok(Self::DisputeOpened),
            "dispute_resolved" => Ok(Self::DisputeResolved),
            _ => Err(format!("invalid event_kind: {value}")),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReportedState {
    Pending,
    Unregistered,
    Declared,
    EvidenceReceived,
    CustodyComplete,
    InspectionAttested,
    Disputed,
    TreatyDetermined,
}

impl ReportedState {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "pending" => Ok(Self::Pending),
            "unregistered" => Ok(Self::Unregistered),
            "declared" => Ok(Self::Declared),
            "evidence_received" => Ok(Self::EvidenceReceived),
            "custody_complete" => Ok(Self::CustodyComplete),
            "inspection_attested" => Ok(Self::InspectionAttested),
            "disputed" => Ok(Self::Disputed),
            "treaty_determined" => Ok(Self::TreatyDetermined),
            _ => Err(format!("invalid reported_state: {value}")),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Event {
    pub history_id: String,
    pub claim_id: String,
    pub event_id: String,
    pub sequence: u32,
    pub previous_digest: String,
    pub event_digest: String,
    pub protocol_version: String,
    pub actor_role: ActorRole,
    pub authority_status: AuthorityStatus,
    pub event_kind: EventKind,
    pub record_integrity_valid: bool,
    pub claim_binding: bool,
    pub external_reference: String,
    pub protected_data_disclosed: bool,
    pub disclosure_authorized: bool,
    pub reported_state: ReportedState,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Report {
    pub history_count: u32,
    pub event_count: u32,
    pub missing_lineage: u32,
    pub duplicate_event_ids: u32,
    pub custody_chain_breaks: u32,
    pub record_integrity_failures: u32,
    pub claim_binding_mismatches: u32,
    pub protocol_version_mismatches: u32,
    pub invalid_declaration_events: u32,
    pub invalid_evidence_events: u32,
    pub unauthorized_inspection_attestations: u32,
    pub unauthorized_treaty_determinations: u32,
    pub verification_overclaims: u32,
    pub treaty_overclaims: u32,
    pub custody_overclaims: u32,
    pub dispute_suppressions: u32,
    pub false_dispute_states: u32,
    pub protected_disclosures: u32,
    pub summary_state_mismatches: u32,
    pub valid_custody_histories: u32,
    pub valid_inspection_histories: u32,
    pub valid_treaty_histories: u32,
}

impl Report {
    pub fn fixture_oracle_matches(&self) -> bool {
        self.history_count == 12
            && self.event_count == 30
            && self.missing_lineage == 0
            && self.duplicate_event_ids == 1
            && self.custody_chain_breaks == 1
            && self.record_integrity_failures == 0
            && self.claim_binding_mismatches == 1
            && self.protocol_version_mismatches == 0
            && self.invalid_declaration_events == 0
            && self.invalid_evidence_events == 0
            && self.unauthorized_inspection_attestations == 2
            && self.unauthorized_treaty_determinations == 1
            && self.verification_overclaims == 3
            && self.treaty_overclaims == 1
            && self.custody_overclaims == 3
            && self.dispute_suppressions == 1
            && self.false_dispute_states == 0
            && self.protected_disclosures == 1
            && self.summary_state_mismatches == 8
            && self.valid_custody_histories == 7
            && self.valid_inspection_histories == 3
            && self.valid_treaty_histories == 1
    }
}

fn parse_bool(value: &str, field: &str) -> Result<bool, String> {
    match value {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => Err(format!("{field} must be true or false, got {value}")),
    }
}

fn parse_u32(value: &str, field: &str) -> Result<u32, String> {
    value
        .parse()
        .map_err(|_| format!("{field} must be an unsigned 32-bit integer, got {value}"))
}

pub fn parse_fixture(input: &str) -> Result<Vec<Event>, String> {
    const HEADER: &str = "history_id,claim_id,event_id,sequence,previous_digest,event_digest,protocol_version,actor_role,authority_status,event_kind,record_integrity,claim_binding,external_reference,protected_data_disclosed,disclosure_authorized,reported_state";
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("NRVDEF fixture header does not match frozen v0.1 contract".into());
    }

    let mut events = Vec::new();
    for (line_idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 16 {
            return Err(format!("line {}: expected 16 fields, got {}", line_idx + 2, f.len()));
        }

        let event = Event {
            history_id: f[0].to_owned(),
            claim_id: f[1].to_owned(),
            event_id: f[2].to_owned(),
            sequence: parse_u32(f[3], "sequence")?,
            previous_digest: f[4].to_owned(),
            event_digest: f[5].to_owned(),
            protocol_version: f[6].to_owned(),
            actor_role: ActorRole::parse(f[7])?,
            authority_status: AuthorityStatus::parse(f[8])?,
            event_kind: EventKind::parse(f[9])?,
            record_integrity_valid: match f[10] {
                "valid" => true,
                "invalid" => false,
                other => return Err(format!("invalid record_integrity: {other}")),
            },
            claim_binding: parse_bool(f[11], "claim_binding")?,
            external_reference: f[12].to_owned(),
            protected_data_disclosed: parse_bool(f[13], "protected_data_disclosed")?,
            disclosure_authorized: parse_bool(f[14], "disclosure_authorized")?,
            reported_state: ReportedState::parse(f[15])?,
        };

        if event.history_id.trim().is_empty()
            || event.claim_id.trim().is_empty()
            || event.event_id.trim().is_empty()
            || event.event_digest.trim().is_empty()
            || event.protocol_version.trim().is_empty()
        {
            return Err(format!("line {}: required identity/digest/protocol field is empty", line_idx + 2));
        }
        if event.sequence == 0 {
            return Err(format!("line {}: sequence must start at 1", line_idx + 2));
        }
        events.push(event);
    }

    if events.is_empty() {
        return Err("NRVDEF fixture has no events".into());
    }
    Ok(events)
}

#[derive(Default)]
struct Derived {
    declared: bool,
    evidence_received: bool,
    inspection_attested: bool,
    treaty_determined: bool,
    dispute_open: bool,
}

impl Derived {
    fn reported_state(&self) -> ReportedState {
        if self.dispute_open {
            ReportedState::Disputed
        } else if self.treaty_determined {
            ReportedState::TreatyDetermined
        } else if self.inspection_attested {
            ReportedState::InspectionAttested
        } else if self.declared && self.evidence_received {
            ReportedState::CustodyComplete
        } else if self.evidence_received {
            ReportedState::EvidenceReceived
        } else if self.declared {
            ReportedState::Declared
        } else {
            ReportedState::Unregistered
        }
    }
}

fn evaluate_history(events: &[&Event], report: &mut Report) {
    if events.is_empty() {
        return;
    }

    let first = events[0];
    let last = events[events.len() - 1];
    if first.claim_id.trim().is_empty()
        || first.protocol_version.trim().is_empty()
        || last.reported_state == ReportedState::Pending
    {
        report.missing_lineage += 1;
    }

    let mut derived = Derived::default();
    let mut event_ids = HashSet::new();
    let history_claim = first.claim_id.as_str();
    let history_protocol = first.protocol_version.as_str();
    let mut previous_digest = "GENESIS";
    let mut previous_sequence = 0u32;
    let mut chain_valid = true;

    for (index, event) in events.iter().enumerate() {
        if event.claim_id.as_str() != history_claim {
            report.claim_binding_mismatches += 1;
            chain_valid = false;
        }
        if event.protocol_version.as_str() != history_protocol {
            report.protocol_version_mismatches += 1;
            chain_valid = false;
        }
        if !event_ids.insert(event.event_id.as_str()) {
            report.duplicate_event_ids += 1;
            chain_valid = false;
        }

        let expected_sequence = previous_sequence.saturating_add(1);
        if event.sequence != expected_sequence || event.previous_digest != previous_digest {
            report.custody_chain_breaks += 1;
            chain_valid = false;
        }

        if !event.record_integrity_valid {
            report.record_integrity_failures += 1;
            chain_valid = false;
        }
        if event.protected_data_disclosed && !event.disclosure_authorized {
            report.protected_disclosures += 1;
        }

        previous_sequence = event.sequence;
        previous_digest = event.event_digest.as_str();

        if !chain_valid {
            continue;
        }

        match event.event_kind {
            EventKind::Declaration => {
                if index == 0
                    && event.actor_role == ActorRole::Declarant
                    && event.authority_status == AuthorityStatus::Valid
                    && event.claim_binding
                {
                    derived.declared = true;
                } else {
                    report.invalid_declaration_events += 1;
                }
            }
            EventKind::EvidenceReceived => {
                if derived.declared
                    && event.actor_role == ActorRole::Custodian
                    && event.authority_status == AuthorityStatus::Valid
                    && event.claim_binding
                {
                    derived.evidence_received = true;
                } else {
                    report.invalid_evidence_events += 1;
                }
            }
            EventKind::InspectionAttestation => {
                if derived.evidence_received
                    && event.actor_role == ActorRole::AuthorizedInspector
                    && event.authority_status == AuthorityStatus::Valid
                    && event.claim_binding
                    && !event.external_reference.trim().is_empty()
                {
                    derived.inspection_attested = true;
                } else {
                    report.unauthorized_inspection_attestations += 1;
                }
            }
            EventKind::TreatyDetermination => {
                if derived.inspection_attested
                    && event.actor_role == ActorRole::TreatyInstitution
                    && event.authority_status == AuthorityStatus::Valid
                    && event.claim_binding
                    && !event.external_reference.trim().is_empty()
                {
                    derived.treaty_determined = true;
                } else {
                    report.unauthorized_treaty_determinations += 1;
                }
            }
            EventKind::DisputeOpened => {
                if event.authority_status == AuthorityStatus::Valid
                    && event.claim_binding
                    && !event.external_reference.trim().is_empty()
                {
                    derived.dispute_open = true;
                } else {
                    report.invalid_evidence_events += 1;
                }
            }
            EventKind::DisputeResolved => {
                if derived.dispute_open
                    && event.actor_role == ActorRole::TreatyInstitution
                    && event.authority_status == AuthorityStatus::Valid
                    && event.claim_binding
                    && !event.external_reference.trim().is_empty()
                {
                    derived.dispute_open = false;
                } else {
                    report.invalid_evidence_events += 1;
                }
            }
        }
    }

    if chain_valid && derived.declared && derived.evidence_received {
        report.valid_custody_histories += 1;
    }
    if chain_valid && derived.inspection_attested {
        report.valid_inspection_histories += 1;
    }
    if chain_valid && derived.treaty_determined {
        report.valid_treaty_histories += 1;
    }

    let expected = derived.reported_state();
    if last.reported_state == ReportedState::InspectionAttested && !derived.inspection_attested {
        report.verification_overclaims += 1;
    }
    if last.reported_state == ReportedState::TreatyDetermined && !derived.treaty_determined {
        report.treaty_overclaims += 1;
    }
    if last.reported_state == ReportedState::CustodyComplete
        && !(chain_valid && derived.declared && derived.evidence_received)
    {
        report.custody_overclaims += 1;
    }
    if derived.dispute_open && last.reported_state != ReportedState::Disputed {
        report.dispute_suppressions += 1;
    }
    if !derived.dispute_open && last.reported_state == ReportedState::Disputed {
        report.false_dispute_states += 1;
    }
    if last.reported_state != expected {
        report.summary_state_mismatches += 1;
    }
}

pub fn evaluate(events: &[Event]) -> Report {
    let mut histories: BTreeMap<&str, Vec<&Event>> = BTreeMap::new();
    for event in events {
        histories.entry(event.history_id.as_str()).or_default().push(event);
    }

    let mut report = Report {
        history_count: histories.len() as u32,
        event_count: events.len() as u32,
        ..Report::default()
    };

    for history in histories.values() {
        evaluate_history(history, &mut report);
    }

    report
}

pub fn fixture_report() -> Result<Report, String> {
    Ok(evaluate(&parse_fixture(FIXTURE)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixture_is_frozen_and_matches_expected_negative_controls() {
        fn fnv1a64(bytes: &[u8]) -> u64 {
            bytes.iter().fold(0xcbf29ce484222325u64, |mut hash, byte| {
                hash ^= u64::from(*byte);
                hash.wrapping_mul(0x100000001b3)
            })
        }
        assert_eq!(fnv1a64(FIXTURE.as_bytes()), 0xb6014db30ed2b435);
        let events = parse_fixture(FIXTURE).unwrap();
        assert_eq!(events.len(), 30);
        assert!(fixture_report().unwrap().fixture_oracle_matches());
    }

    #[test]
    fn valid_custody_history_does_not_claim_inspection_or_treaty_status() {
        let events = parse_fixture(FIXTURE).unwrap();
        let subset: Vec<_> = events.iter().filter(|e| e.history_id == "valid-custody").cloned().collect();
        let report = evaluate(&subset);
        assert_eq!(report.valid_custody_histories, 1);
        assert_eq!(report.valid_inspection_histories, 0);
        assert_eq!(report.valid_treaty_histories, 0);
        assert_eq!(report.summary_state_mismatches, 0);
    }

    #[test]
    fn claimant_and_revoked_inspector_cannot_attest_inspection() {
        let report = fixture_report().unwrap();
        assert_eq!(report.unauthorized_inspection_attestations, 2);
        assert_eq!(report.verification_overclaims, 3);
    }

    #[test]
    fn only_external_treaty_institution_decision_can_derive_treaty_state() {
        let report = fixture_report().unwrap();
        assert_eq!(report.unauthorized_treaty_determinations, 1);
        assert_eq!(report.treaty_overclaims, 1);
        assert_eq!(report.valid_treaty_histories, 1);
    }

    #[test]
    fn broken_chain_replay_and_claim_mismatch_are_not_custody_complete() {
        let report = fixture_report().unwrap();
        assert_eq!(report.custody_chain_breaks, 1);
        assert_eq!(report.duplicate_event_ids, 1);
        assert_eq!(report.claim_binding_mismatches, 1);
        assert_eq!(report.custody_overclaims, 3);
    }

    #[test]
    fn unresolved_dispute_cannot_disappear_from_summary() {
        let report = fixture_report().unwrap();
        assert_eq!(report.dispute_suppressions, 1);
        assert_eq!(report.false_dispute_states, 0);
    }

    #[test]
    fn unauthorized_protected_disclosure_is_reported() {
        let report = fixture_report().unwrap();
        assert_eq!(report.protected_disclosures, 1);
    }

    #[test]
    fn no_synthetic_event_can_make_physical_truth_or_legal_status_intrinsic() {
        let report = fixture_report().unwrap();
        assert_eq!(report.valid_inspection_histories, 3);
        assert_eq!(report.valid_treaty_histories, 1);
        // These are imported external attestations/determinations, not software-created truths.
    }
}
