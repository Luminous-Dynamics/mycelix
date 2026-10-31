//! Deterministic synthetic multi-channel last-mile warning evaluation.
//! No live messaging, subscriber data, location tracking or operational alert dispatch.

use std::collections::HashSet;

pub const COHORT_FIXTURE: &str = include_str!("../fixtures/receiver-cohorts-v0.1.csv");
pub const SCENARIO_FIXTURE: &str = include_str!("../fixtures/channel-fault-matrix-v0.1.csv");
pub const CHANNEL_NAMES: [&str; 4] = [
    "cell-broadcast",
    "radio-tv",
    "mobile-data",
    "community-relay",
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Cohort {
    pub cohort_id: String,
    pub recipients: u32,
    /// Channel order is CHANNEL_NAMES.
    pub channel_access: [bool; 4],
    /// A channel can be accessible only when the cohort can receive that channel.
    pub accessible_format: [bool; 4],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Scenario {
    pub scenario_id: String,
    /// Channel order is CHANNEL_NAMES.
    pub channel_up: [bool; 4],
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CohortOutcome {
    pub cohort_id: String,
    pub intended: u64,
    pub unique_delivered: u64,
    pub accessible_delivered: u64,
    pub channel_receipts: [u64; 4],
    pub duplicate_deliveries: u64,
}

impl CohortOutcome {
    pub fn uncovered(&self) -> u64 {
        self.intended - self.unique_delivered
    }

    pub fn delivered_without_accessible_format(&self) -> u64 {
        self.unique_delivered - self.accessible_delivered
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Evaluation {
    pub scenario_id: String,
    pub intended_receivers: u64,
    pub unique_receivers_reached: u64,
    pub accessible_receivers_reached: u64,
    pub uncovered_receivers: u64,
    pub reached_without_accessible_format: u64,
    pub duplicate_channel_deliveries: u64,
    /// Counts channel-level receipts. One receiver may appear in multiple channels.
    pub channel_receipts: [u64; 4],
    pub cohorts: Vec<CohortOutcome>,
}

impl Evaluation {
    pub fn channel_receipt_total(&self) -> u64 {
        self.channel_receipts.iter().sum()
    }

    pub fn delivery_coverage(&self) -> f64 {
        if self.intended_receivers == 0 {
            0.0
        } else {
            self.unique_receivers_reached as f64 / self.intended_receivers as f64
        }
    }

    pub fn accessible_coverage(&self) -> f64 {
        if self.intended_receivers == 0 {
            0.0
        } else {
            self.accessible_receivers_reached as f64 / self.intended_receivers as f64
        }
    }

    pub fn accessibility_among_reached(&self) -> f64 {
        if self.unique_receivers_reached == 0 {
            0.0
        } else {
            self.accessible_receivers_reached as f64 / self.unique_receivers_reached as f64
        }
    }
}

fn parse_bool(value: &str, field: &str) -> Result<bool, String> {
    match value {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => Err(format!("{field} must be true or false, got {value}")),
    }
}

fn parse_recipient_count(value: &str, field: &str) -> Result<u32, String> {
    // Bound individual synthetic cohort sizes to u32 so aggregate u64 sums
    // and duplicate-delivery multiplication remain overflow-safe.
    let count: u32 = value
        .parse()
        .map_err(|_| format!("{field} must be an unsigned 32-bit integer, got {value}"))?;
    if count == 0 {
        return Err(format!("{field} must be greater than zero"));
    }
    Ok(count)
}

pub fn parse_cohorts(input: &str) -> Result<Vec<Cohort>, String> {
    const HEADER: &str = "cohort_id,recipients,cell_broadcast_access,radio_tv_access,mobile_data_access,community_relay_access,cell_broadcast_accessible,radio_tv_accessible,mobile_data_accessible,community_relay_accessible";
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("receiver cohort header does not match frozen v0.1 contract".into());
    }

    let mut cohorts = Vec::new();
    let mut ids = HashSet::new();
    for (line_idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 10 {
            return Err(format!("line {}: expected 10 cohort fields, got {}", line_idx + 2, f.len()));
        }

        let cohort = Cohort {
            cohort_id: f[0].to_owned(),
            recipients: parse_recipient_count(f[1], "recipients")?,
            channel_access: [
                parse_bool(f[2], "cell_broadcast_access")?,
                parse_bool(f[3], "radio_tv_access")?,
                parse_bool(f[4], "mobile_data_access")?,
                parse_bool(f[5], "community_relay_access")?,
            ],
            accessible_format: [
                parse_bool(f[6], "cell_broadcast_accessible")?,
                parse_bool(f[7], "radio_tv_accessible")?,
                parse_bool(f[8], "mobile_data_accessible")?,
                parse_bool(f[9], "community_relay_accessible")?,
            ],
        };

        if cohort.cohort_id.trim().is_empty() {
            return Err(format!("line {}: cohort_id is required", line_idx + 2));
        }
        if cohort.recipients == 0 {
            return Err(format!("line {}: cohort recipients must be non-zero", line_idx + 2));
        }
        if !ids.insert(cohort.cohort_id.clone()) {
            return Err(format!("line {}: duplicate cohort_id", line_idx + 2));
        }
        if cohort.accessible_format.iter().zip(cohort.channel_access.iter()).any(|(accessible, can_receive)| *accessible && !*can_receive) {
            return Err(format!("line {}: accessible format requires channel access", line_idx + 2));
        }
        cohorts.push(cohort);
    }

    if cohorts.is_empty() {
        return Err("receiver cohort fixture contains no rows".into());
    }
    Ok(cohorts)
}

pub fn parse_scenarios(input: &str) -> Result<Vec<Scenario>, String> {
    const HEADER: &str = "scenario_id,cell_broadcast_up,radio_tv_up,mobile_data_up,community_relay_up";
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("channel fault matrix header does not match frozen v0.1 contract".into());
    }

    let mut scenarios = Vec::new();
    let mut ids = HashSet::new();
    for (line_idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 5 {
            return Err(format!("line {}: expected 5 scenario fields, got {}", line_idx + 2, f.len()));
        }

        let scenario = Scenario {
            scenario_id: f[0].to_owned(),
            channel_up: [
                parse_bool(f[1], "cell_broadcast_up")?,
                parse_bool(f[2], "radio_tv_up")?,
                parse_bool(f[3], "mobile_data_up")?,
                parse_bool(f[4], "community_relay_up")?,
            ],
        };
        if scenario.scenario_id.trim().is_empty() {
            return Err(format!("line {}: scenario_id is required", line_idx + 2));
        }
        if !ids.insert(scenario.scenario_id.clone()) {
            return Err(format!("line {}: duplicate scenario_id", line_idx + 2));
        }
        scenarios.push(scenario);
    }

    if scenarios.is_empty() {
        return Err("channel fault matrix contains no scenarios".into());
    }
    Ok(scenarios)
}

/// Calculate unique delivery coverage and accessible-format coverage.
/// Each cohort member is counted once for unique reach, irrespective of how many
/// channels carry the alert. Additional channel receipts are reported separately.
pub fn evaluate(scenario: &Scenario, cohorts: &[Cohort]) -> Evaluation {
    let mut evaluation = Evaluation {
        scenario_id: scenario.scenario_id.clone(),
        ..Evaluation::default()
    };

    for cohort in cohorts {
        evaluation.intended_receivers += cohort.recipients;
        let mut active_channels = 0u64;
        let mut has_accessible_delivery = false;
        let mut channel_receipts = [0u64; 4];

        for i in 0..4 {
            if cohort.channel_access[i] && scenario.channel_up[i] {
                active_channels += 1;
                channel_receipts[i] = cohort.recipients;
                evaluation.channel_receipts[i] += cohort.recipients;
                if cohort.accessible_format[i] {
                    has_accessible_delivery = true;
                }
            }
        }

        let unique_delivered = if active_channels > 0 { recipients } else { 0 };
        let accessible_delivered = if has_accessible_delivery { recipients } else { 0 };
        let duplicate_deliveries = if active_channels > 1 {
            recipients * (active_channels - 1)
        } else {
            0
        };

        let result = CohortOutcome {
            cohort_id: cohort.cohort_id.clone(),
            intended: recipients,
            unique_delivered,
            accessible_delivered,
            channel_receipts,
            duplicate_deliveries,
        };

        evaluation.unique_receivers_reached += result.unique_delivered;
        evaluation.accessible_receivers_reached += result.accessible_delivered;
        evaluation.uncovered_receivers += result.uncovered();
        evaluation.reached_without_accessible_format += result.delivered_without_accessible_format();
        evaluation.duplicate_channel_deliveries += result.duplicate_deliveries;
        evaluation.cohorts.push(result);
    }

    evaluation
}

pub fn evaluate_all(scenarios: &[Scenario], cohorts: &[Cohort]) -> Vec<Evaluation> {
    scenarios.iter().map(|scenario| evaluate(scenario, cohorts)).collect()
}

pub fn fixture_evaluations() -> Result<Vec<Evaluation>, String> {
    let cohorts = parse_cohorts(COHORT_FIXTURE)?;
    let scenarios = parse_scenarios(SCENARIO_FIXTURE)?;
    Ok(evaluate_all(&scenarios, &cohorts))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cohorts() -> Vec<Cohort> {
        parse_cohorts(COHORT_FIXTURE).unwrap()
    }

    fn scenario(id: &str) -> Scenario {
        parse_scenarios(SCENARIO_FIXTURE)
            .unwrap()
            .into_iter()
            .find(|s| s.scenario_id == id)
            .unwrap()
    }

    #[test]
    fn fixtures_have_unique_ids_expected_totals_and_pinned_fingerprints() {
        fn fnv1a64(bytes: &[u8]) -> u64 {
            bytes.iter().fold(0xcbf29ce484222325u64, |mut hash, byte| {
                hash ^= u64::from(*byte);
                hash.wrapping_mul(0x100000001b3)
            })
        }
        assert_eq!(fnv1a64(COHORT_FIXTURE.as_bytes()), 0x59f956045ca16c3d);
        assert_eq!(fnv1a64(SCENARIO_FIXTURE.as_bytes()), 0x2a7269d9a7d4caee);

        let c = cohorts();
        let s = parse_scenarios(SCENARIO_FIXTURE).unwrap();
        assert_eq!(c.len(), 5);
        assert_eq!(s.len(), 10);
        assert_eq!(c.iter().map(|x| u64::from(x.recipients)).sum::<u64>(), 2_700);
        assert_eq!(c.iter().map(|x| x.cohort_id.as_str()).collect::<HashSet<_>>().len(), 5);
        assert_eq!(s.iter().map(|x| x.scenario_id.as_str()).collect::<HashSet<_>>().len(), 10);
    }

    #[test]
    fn all_channels_up_counts_unique_people_once_and_duplicates_separately() {
        let c = cohorts();
        let e = evaluate(&scenario("all-channels-up"), &c);
        assert_eq!(e.intended_receivers, 2_700);
        assert_eq!(e.unique_receivers_reached, 2_700);
        assert_eq!(e.accessible_receivers_reached, 2_700);
        assert_eq!(e.duplicate_channel_deliveries, 3_750);
        assert_eq!(e.channel_receipts, [1_400, 2_200, 1_400, 1_450]);
        assert_eq!(e.channel_receipt_total(), 6_450);
        assert!((e.delivery_coverage() - 1.0).abs() < 1e-12);
        assert!((e.accessible_coverage() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn mobile_data_loss_exposes_accessibility_gap_without_losing_all_receivers() {
        let c = cohorts();
        let e = evaluate(&scenario("mobile-data-down"), &c);
        assert_eq!(e.unique_receivers_reached, 2_700);
        assert_eq!(e.accessible_receivers_reached, 2_450);
        assert_eq!(e.reached_without_accessible_format, 250);
        let screen_reader = e.cohorts.iter().find(|x| x.cohort_id == "screen-reader-mobile").unwrap();
        assert_eq!(screen_reader.unique_delivered, 250);
        assert_eq!(screen_reader.accessible_delivered, 0);
    }

    #[test]
    fn community_relay_loss_leaves_low_connectivity_cohort_unreached() {
        let c = cohorts();
        let e = evaluate(&scenario("community-relay-down"), &c);
        assert_eq!(e.unique_receivers_reached, 2_200);
        assert_eq!(e.uncovered_receivers, 500);
        let low = e.cohorts.iter().find(|x| x.cohort_id == "low-connectivity-community").unwrap();
        assert_eq!(low.unique_delivered, 0);
    }

    #[test]
    fn only_community_relay_reaches_only_community_enabled_cohorts() {
        let c = cohorts();
        let e = evaluate(&scenario("only-community-relay"), &c);
        assert_eq!(e.unique_receivers_reached, 1_450);
        assert_eq!(e.accessible_receivers_reached, 1_450);
        assert_eq!(e.uncovered_receivers, 1_250);
        assert_eq!(e.duplicate_channel_deliveries, 0);
    }

    #[test]
    fn no_channels_up_means_zero_unique_reach_and_zero_accessible_reach() {
        let c = cohorts();
        let e = evaluate(&scenario("no-channels"), &c);
        assert_eq!(e.unique_receivers_reached, 0);
        assert_eq!(e.accessible_receivers_reached, 0);
        assert_eq!(e.uncovered_receivers, 2_700);
        assert_eq!(e.channel_receipt_total(), 0);
        assert_eq!(e.delivery_coverage(), 0.0);
    }

    #[test]
    fn mobile_and_cell_broadcast_loss_does_not_double_count_recipients() {
        let c = cohorts();
        let e = evaluate(&scenario("cell-broadcast-mobile-data-down"), &c);
        assert_eq!(e.unique_receivers_reached, 2_700);
        assert_eq!(e.accessible_receivers_reached, 2_450);
        assert_eq!(e.duplicate_channel_deliveries, 950);
        assert_eq!(e.channel_receipt_total(), 3_650);
        assert_eq!(e.channel_receipt_total() - e.unique_receivers_reached, e.duplicate_channel_deliveries);
    }

    #[test]
    fn channel_union_is_deduplicated_for_every_cohort() {
        let c = cohorts();
        let e = evaluate(&scenario("all-channels-up"), &c);
        assert_eq!(
            e.unique_receivers_reached,
            e.cohorts.iter().map(|x| x.unique_delivered).sum::<u64>()
        );
        assert_eq!(
            e.accessible_receivers_reached,
            e.cohorts.iter().map(|x| x.accessible_delivered).sum::<u64>()
        );
        for cohort in &e.cohorts {
            assert!(cohort.accessible_delivered <= cohort.unique_delivered);
            assert!(cohort.unique_delivered <= cohort.intended);
        }
    }

    #[test]
    fn accessible_format_cannot_be_claimed_for_an_unavailable_channel() {
        let invalid = COHORT_FIXTURE.replace(
            "urban-general,1000,true,true,true,false,true,true,true,false",
            "urban-general,1000,true,true,true,false,false,true,true,true",
        );
        assert!(parse_cohorts(&invalid).is_err());
    }

    #[test]
    fn malformed_channel_matrix_fails_closed() {
        let invalid = SCENARIO_FIXTURE.replace("all-channels-up,true", "all-channels-up,yes");
        assert!(parse_scenarios(&invalid).is_err());
    }
}
