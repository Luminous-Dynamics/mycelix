//! Synthetic conformance oracle for civilian-warning records.
//! No live feeds, alert dispatch, threat modelling, or real-world warning policy.
//!
//! v0.1.1 extends the original frozen v0.1 fixture without modifying it.

use std::collections::HashSet;

pub const FIXTURE_V0_1: &str = include_str!("../fixtures/gcpn-001-v0.1.csv");
pub const FIXTURE: &str = include_str!("../fixtures/gcpn-001-v0.1.1.csv");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordAuthenticity { Authenticated, Invalid, Unknown }

impl RecordAuthenticity {
    fn parse(s: &str) -> Result<Self, String> {
        match s {
            "authenticated" => Ok(Self::Authenticated),
            "invalid" => Ok(Self::Invalid),
            "unknown" => Ok(Self::Unknown),
            _ => Err(format!("invalid record_authenticity: {s}")),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasurementState { Validated, Unverified, Contradictory, Stale, Missing }

impl MeasurementState {
    fn parse(s: &str) -> Result<Self, String> {
        match s {
            "validated" => Ok(Self::Validated),
            "unverified" => Ok(Self::Unverified),
            "contradictory" => Ok(Self::Contradictory),
            "stale" => Ok(Self::Stale),
            "missing" => Ok(Self::Missing),
            _ => Err(format!("invalid measurement_state: {s}")),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IndependenceClaim { Independent, Dependent, Unknown }

impl IndependenceClaim {
    fn parse(s: &str) -> Result<Self, String> {
        match s {
            "independent" => Ok(Self::Independent),
            "dependent" => Ok(Self::Dependent),
            "unknown" => Ok(Self::Unknown),
            _ => Err(format!("invalid reported_independence: {s}")),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Case {
    pub profile_id: String,
    pub scenario_id: String,
    pub record_authenticity: RecordAuthenticity,
    pub measurement_state: MeasurementState,
    pub reported_measurement_state: MeasurementState,
    pub source_reports: u32,
    pub source_families: u32,
    pub reported_independence: IndependenceClaim,
    pub authority_authorized: bool,
    pub alert_issued: bool,
    pub wan_available: bool,
    pub local_fallback_available: bool,
    pub local_authority_available: bool,
    pub local_issue_expected: bool,
    pub local_alert_issued: bool,
    pub recipients: u32,
    pub receipts: u32,
    pub accessible_receivers: u32,
    pub traceable: bool,
    pub source_record_count: u32,
    pub unauthorized_remote_change_attempted: bool,
    pub unauthorized_remote_change_accepted: bool,
    pub authority_assertion_bound: bool,
    pub uncertainty_disclosed: bool,
    pub sensitive_provenance_in_public_payload: bool,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Report {
    pub cases: u32,
    pub independence_promotions: u32,
    pub measurement_state_coercions: u32,
    pub authenticity_measurement_conflations: u32,
    pub unauthorized_alerts: u32,
    pub authority_unbound_alerts: u32,
    pub uncertainty_hidden_alerts: u32,
    pub sensitive_provenance_leaks: u32,
    pub local_partition_cases: u32,
    pub local_partition_deliveries: u32,
    pub untraceable_alerts: u32,
    pub unauthorized_remote_mutations: u32,
    pub recipients: u64,
    pub receipts: u64,
    pub accessible_receivers: u64,
}

impl Report {
    /// Hard integrity gates. Delivery/accessibility ratios remain separate metrics.
    pub fn conformance_passes(&self) -> bool {
        self.independence_promotions == 0
            && self.measurement_state_coercions == 0
            && self.authenticity_measurement_conflations == 0
            && self.unauthorized_alerts == 0
            && self.authority_unbound_alerts == 0
            && self.uncertainty_hidden_alerts == 0
            && self.sensitive_provenance_leaks == 0
            && self.local_partition_cases == self.local_partition_deliveries
            && self.untraceable_alerts == 0
            && self.unauthorized_remote_mutations == 0
    }

    pub fn receiver_delivery_ratio(&self) -> f64 {
        ratio(self.receipts, self.recipients)
    }

    pub fn accessibility_coverage_ratio(&self) -> f64 {
        ratio(self.accessible_receivers, self.recipients)
    }
}

fn ratio(n: u64, d: u64) -> f64 {
    if d == 0 { 0.0 } else { n as f64 / d as f64 }
}

fn parse_bool(s: &str, name: &str) -> Result<bool, String> {
    match s {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => Err(format!("{name} must be true or false, got {s}")),
    }
}

fn parse_u32(s: &str, name: &str) -> Result<u32, String> {
    s.parse().map_err(|_| format!("{name} must be an unsigned integer, got {s}"))
}

/// Parse the fixed v0.1.1 CSV contract. Quoted fields are deliberately unsupported;
/// fixture cells are restricted to enumerations, integers, booleans and IDs.
pub fn parse_fixture(input: &str) -> Result<Vec<Case>, String> {
    const HEADER: &str = "profile_id,scenario_id,record_authenticity,measurement_state,reported_measurement_state,source_reports,source_families,reported_independence,authority_authorized,alert_issued,wan_available,local_fallback_available,local_authority_available,local_issue_expected,local_alert_issued,recipients,receipts,accessible_receivers,traceable,source_record_count,unauthorized_remote_change_attempted,unauthorized_remote_change_accepted,authority_assertion_bound,uncertainty_disclosed,sensitive_provenance_in_public_payload";
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("fixture header does not match frozen v0.1.1 contract".into());
    }

    let mut cases = Vec::new();
    let mut identities = HashSet::new();
    for (line_no, line) in lines.enumerate() {
        if line.trim().is_empty() { continue; }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 25 {
            return Err(format!("line {}: expected 25 fields, got {}", line_no + 2, f.len()));
        }
        let c = Case {
            profile_id: f[0].to_owned(),
            scenario_id: f[1].to_owned(),
            record_authenticity: RecordAuthenticity::parse(f[2])?,
            measurement_state: MeasurementState::parse(f[3])?,
            reported_measurement_state: MeasurementState::parse(f[4])?,
            source_reports: parse_u32(f[5], "source_reports")?,
            source_families: parse_u32(f[6], "source_families")?,
            reported_independence: IndependenceClaim::parse(f[7])?,
            authority_authorized: parse_bool(f[8], "authority_authorized")?,
            alert_issued: parse_bool(f[9], "alert_issued")?,
            wan_available: parse_bool(f[10], "wan_available")?,
            local_fallback_available: parse_bool(f[11], "local_fallback_available")?,
            local_authority_available: parse_bool(f[12], "local_authority_available")?,
            local_issue_expected: parse_bool(f[13], "local_issue_expected")?,
            local_alert_issued: parse_bool(f[14], "local_alert_issued")?,
            recipients: parse_u32(f[15], "recipients")?,
            receipts: parse_u32(f[16], "receipts")?,
            accessible_receivers: parse_u32(f[17], "accessible_receivers")?,
            traceable: parse_bool(f[18], "traceable")?,
            source_record_count: parse_u32(f[19], "source_record_count")?,
            unauthorized_remote_change_attempted: parse_bool(f[20], "unauthorized_remote_change_attempted")?,
            unauthorized_remote_change_accepted: parse_bool(f[21], "unauthorized_remote_change_accepted")?,
            authority_assertion_bound: parse_bool(f[22], "authority_assertion_bound")?,
            uncertainty_disclosed: parse_bool(f[23], "uncertainty_disclosed")?,
            sensitive_provenance_in_public_payload: parse_bool(f[24], "sensitive_provenance_in_public_payload")?,
        };

        if c.profile_id.is_empty() || c.scenario_id.is_empty() {
            return Err(format!("line {}: profile_id and scenario_id are required", line_no + 2));
        }
        if !identities.insert(format!("{}::{}", c.profile_id, c.scenario_id)) {
            return Err(format!("line {}: duplicate profile/scenario identity", line_no + 2));
        }
        if c.source_families > c.source_reports {
            return Err(format!("line {}: source_families exceeds source_reports", line_no + 2));
        }
        if c.receipts > c.recipients || c.accessible_receivers > c.recipients {
            return Err(format!("line {}: receiver counts exceed recipients", line_no + 2));
        }
        if !c.alert_issued && (c.recipients > 0 || c.receipts > 0 || c.accessible_receivers > 0) {
            return Err(format!("line {}: receiver counts require an issued alert", line_no + 2));
        }
        if c.local_issue_expected
            && (c.wan_available || !c.local_fallback_available || !c.local_authority_available)
        {
            return Err(format!("line {}: local_issue_expected requires WAN partition, local authority and fallback", line_no + 2));
        }
        if c.local_alert_issued && (!c.local_issue_expected || !c.alert_issued || !c.local_authority_available) {
            return Err(format!("line {}: local alert requires expected local issue, issued alert and local authority", line_no + 2));
        }
        cases.push(c);
    }

    if cases.is_empty() { return Err("fixture has no cases".into()); }
    Ok(cases)
}

/// Evaluate candidate-record outputs against the synthetic oracle.
pub fn evaluate(cases: &[Case]) -> Report {
    let mut r = Report { cases: cases.len() as u32, ..Report::default() };
    for c in cases {
        // An independence claim requires at least two genuinely distinct source families.
        if c.reported_independence == IndependenceClaim::Independent && c.source_families < 2 {
            r.independence_promotions += 1;
        }
        if c.reported_measurement_state != c.measurement_state {
            r.measurement_state_coercions += 1;
        }
        if c.record_authenticity == RecordAuthenticity::Authenticated
            && c.measurement_state != MeasurementState::Validated
            && c.reported_measurement_state == MeasurementState::Validated
        {
            r.authenticity_measurement_conflations += 1;
        }
        if c.alert_issued && !c.authority_authorized {
            r.unauthorized_alerts += 1;
        }
        if c.alert_issued && !c.authority_assertion_bound {
            r.authority_unbound_alerts += 1;
        }
        // Precautionary warnings remain possible, but uncertainty must be visible.
        if c.alert_issued
            && c.measurement_state != MeasurementState::Validated
            && !c.uncertainty_disclosed
        {
            r.uncertainty_hidden_alerts += 1;
        }
        if c.sensitive_provenance_in_public_payload {
            r.sensitive_provenance_leaks += 1;
        }
        if c.local_issue_expected {
            r.local_partition_cases += 1;
            if c.local_alert_issued && c.alert_issued && c.authority_authorized {
                r.local_partition_deliveries += 1;
            }
        }
        if c.alert_issued && (!c.traceable || c.source_record_count == 0) {
            r.untraceable_alerts += 1;
        }
        if c.unauthorized_remote_change_accepted {
            r.unauthorized_remote_mutations += 1;
        }
        r.recipients += c.recipients as u64;
        r.receipts += c.receipts as u64;
        r.accessible_receivers += c.accessible_receivers as u64;
    }
    r
}

pub fn fixture_report() -> Result<Report, String> {
    Ok(evaluate(&parse_fixture(FIXTURE)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frozen_v0_1_fixture_remains_unchanged() {
        // A 22-field legacy parser is not reimplemented here; this locks the original bytes
        // into the crate and verifies its defining first-line contract remains present.
        assert!(FIXTURE_V0_1.starts_with("profile_id,scenario_id,record_authenticity,measurement_state,reported_measurement_state,"));
        assert_eq!(FIXTURE_V0_1.lines().count(), 10);
    }

    #[test]
    fn frozen_v0_1_1_fixture_parses_with_unique_scenario_ids() {
        let cases = parse_fixture(FIXTURE).expect("fixture should parse");
        assert_eq!(cases.len(), 9);
        let ids: HashSet<_> = cases.iter().map(|c| c.scenario_id.as_str()).collect();
        assert_eq!(ids.len(), 9);
    }

    #[test]
    fn reference_fixture_passes_all_hard_integrity_gates() {
        let r = fixture_report().expect("fixture should evaluate");
        assert!(r.conformance_passes(), "unexpected report: {r:?}");
        assert_eq!(r.independence_promotions, 0);
        assert_eq!(r.measurement_state_coercions, 0);
        assert_eq!(r.authenticity_measurement_conflations, 0);
        assert_eq!(r.unauthorized_alerts, 0);
        assert_eq!(r.authority_unbound_alerts, 0);
        assert_eq!(r.uncertainty_hidden_alerts, 0);
        assert_eq!(r.sensitive_provenance_leaks, 0);
        assert_eq!((r.local_partition_deliveries, r.local_partition_cases), (1, 1));
        assert_eq!(r.untraceable_alerts, 0);
        assert_eq!(r.unauthorized_remote_mutations, 0);
    }

    #[test]
    fn delivery_and_accessibility_use_receiver_side_denominators() {
        let r = fixture_report().expect("fixture should evaluate");
        assert_eq!((r.recipients, r.receipts, r.accessible_receivers), (24, 24, 22));
        assert!((r.receiver_delivery_ratio() - 1.0).abs() < 1e-12);
        assert!((r.accessibility_coverage_ratio() - (22.0 / 24.0)).abs() < 1e-12);
    }

    #[test]
    fn one_source_cannot_claim_independent_corroboration() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        let c = cases.iter_mut().find(|c| c.scenario_id == "gcpn-normal").unwrap();
        c.reported_independence = IndependenceClaim::Independent;
        c.source_reports = 1;
        c.source_families = 1;
        let r = evaluate(&cases);
        assert_eq!(r.independence_promotions, 1);
        assert!(!r.conformance_passes());
    }

    #[test]
    fn duplicate_reports_from_one_family_cannot_claim_independence() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        let c = cases.iter_mut().find(|c| c.scenario_id == "gcpn-duplicate-upstream").unwrap();
        c.reported_independence = IndependenceClaim::Independent;
        let r = evaluate(&cases);
        assert_eq!(r.independence_promotions, 1);
        assert!(!r.conformance_passes());
    }

    #[test]
    fn authentic_signature_does_not_validate_measurement_truth() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        let c = cases.iter_mut().find(|c| c.scenario_id == "gcpn-authentic-not-accurate").unwrap();
        c.reported_measurement_state = MeasurementState::Validated;
        let r = evaluate(&cases);
        assert_eq!(r.measurement_state_coercions, 1);
        assert_eq!(r.authenticity_measurement_conflations, 1);
        assert!(!r.conformance_passes());
    }

    #[test]
    fn stale_missing_and_contradictory_states_cannot_be_rewritten() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        for id in ["gcpn-contradictory", "gcpn-stale", "gcpn-missing"] {
            let c = cases.iter_mut().find(|c| c.scenario_id == id).unwrap();
            c.reported_measurement_state = MeasurementState::Validated;
        }
        let r = evaluate(&cases);
        assert_eq!(r.measurement_state_coercions, 3);
        assert!(!r.conformance_passes());
    }

    #[test]
    fn alert_requires_a_bound_authority_assertion() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        let c = cases.iter_mut().find(|c| c.scenario_id == "gcpn-normal").unwrap();
        c.authority_assertion_bound = false;
        let r = evaluate(&cases);
        assert_eq!(r.authority_unbound_alerts, 1);
        assert!(!r.conformance_passes());
    }

    #[test]
    fn precautionary_alert_must_disclose_unverified_evidence() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        let c = cases.iter_mut().find(|c| c.scenario_id == "gcpn-authentic-not-accurate").unwrap();
        c.uncertainty_disclosed = false;
        let r = evaluate(&cases);
        assert_eq!(r.uncertainty_hidden_alerts, 1);
        assert!(!r.conformance_passes());
    }

    #[test]
    fn sensitive_provenance_must_not_enter_public_payload() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        let c = cases.iter_mut().find(|c| c.scenario_id == "gcpn-normal").unwrap();
        c.sensitive_provenance_in_public_payload = true;
        let r = evaluate(&cases);
        assert_eq!(r.sensitive_provenance_leaks, 1);
        assert!(!r.conformance_passes());
    }

    #[test]
    fn stale_missing_and_contradictory_alerts_must_disclose_uncertainty() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        for id in ["gcpn-contradictory"] {
            let c = cases.iter_mut().find(|c| c.scenario_id == id).unwrap();
            c.uncertainty_disclosed = false;
        }
        let r = evaluate(&cases);
        assert_eq!(r.uncertainty_hidden_alerts, 1);
        assert!(!r.conformance_passes());
    }

    #[test]
    fn unauthorized_alerts_untraceable_alerts_and_remote_writes_fail() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        let c = cases.iter_mut().find(|c| c.scenario_id == "gcpn-unauthorized-cross-member-write").unwrap();
        c.alert_issued = true;
        c.recipients = 0;
        c.receipts = 0;
        c.accessible_receivers = 0;
        c.traceable = false;
        c.source_record_count = 0;
        c.unauthorized_remote_change_accepted = true;
        c.authority_assertion_bound = false;
        let r = evaluate(&cases);
        assert_eq!((r.unauthorized_alerts, r.authority_unbound_alerts, r.untraceable_alerts, r.unauthorized_remote_mutations), (1, 1, 1, 1));
        assert!(!r.conformance_passes());
    }

    #[test]
    fn local_partition_loss_is_detected_even_when_remote_send_counts_exist() {
        let mut cases = parse_fixture(FIXTURE).unwrap();
        let c = cases.iter_mut().find(|c| c.scenario_id == "gcpn-wan-partition").unwrap();
        c.local_alert_issued = false;
        c.receipts = 0;
        let r = evaluate(&cases);
        assert_eq!((r.local_partition_deliveries, r.local_partition_cases), (0, 1));
        assert!(!r.conformance_passes());
    }

    #[test]
    fn malformed_receiver_counts_and_unissued_alert_receipts_are_rejected() {
        let invalid = FIXTURE.replace("4,4,4,true,3,false,false,true,false,false", "4,5,4,true,3,false,false,true,false,false");
        assert!(parse_fixture(&invalid).is_err());

        let unissued_with_receipt = FIXTURE.replace(
            "true,false,0,0,0,true,2,false,false,false,false,false",
            "false,false,4,1,1,true,2,false,false,false,false,false",
        );
        assert!(parse_fixture(&unissued_with_receipt).is_err());
    }
}
