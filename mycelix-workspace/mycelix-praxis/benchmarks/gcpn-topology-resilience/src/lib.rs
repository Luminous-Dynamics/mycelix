//! Deterministic structural comparison of civilian warning distribution topologies.
//! This is a logical route-dependency model, not a production network simulator.

use std::collections::HashSet;

pub const FIXTURE: &str = include_str!("../fixtures/topology-fault-matrix-v0.1.csv");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Architecture {
    CentralizedHubSpoke,
    RegionalFederation,
    LocalFirstFederation,
}

impl Architecture {
    pub const ALL: [Self; 3] = [
        Self::CentralizedHubSpoke,
        Self::RegionalFederation,
        Self::LocalFirstFederation,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::CentralizedHubSpoke => "centralized",
            Self::RegionalFederation => "regional",
            Self::LocalFirstFederation => "local-first",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Scenario {
    pub scenario_id: String,
    pub alert_authorized: bool,
    pub central_hub_up: bool,
    pub origin_to_central_up: bool,
    pub central_remote_path_up: bool,
    pub origin_regional_service_up: bool,
    pub interregional_federation_up: bool,
    pub remote_regional_services_up: bool,
    pub local_first_local_path_up: bool,
    pub origin_last_mile_up: bool,
    pub remote_last_miles_up: bool,
    pub local_receivers: u32,
    pub remote_a_receivers: u32,
    pub remote_b_receivers: u32,
}

impl Scenario {
    pub fn intended_receivers(&self) -> u64 {
        u64::from(self.local_receivers)
            + u64::from(self.remote_a_receivers)
            + u64::from(self.remote_b_receivers)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Outcome {
    pub local_delivered: u64,
    pub remote_a_delivered: u64,
    pub remote_b_delivered: u64,
    pub intended_receivers: u64,
}

impl Outcome {
    pub fn delivered(&self) -> u64 {
        self.local_delivered + self.remote_a_delivered + self.remote_b_delivered
    }

    pub fn missed(&self) -> u64 {
        self.intended_receivers - self.delivered()
    }

    pub fn remote_delivered(&self) -> u64 {
        self.remote_a_delivered + self.remote_b_delivered
    }

    pub fn delivery_ratio(&self) -> f64 {
        if self.intended_receivers == 0 {
            0.0
        } else {
            self.delivered() as f64 / self.intended_receivers as f64
        }
    }

    pub fn local_continuity(&self, scenario: &Scenario) -> f64 {
        if scenario.local_receivers == 0 {
            0.0
        } else {
            self.local_delivered as f64 / f64::from(scenario.local_receivers)
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComparisonRow {
    pub scenario_id: String,
    pub architecture: Architecture,
    pub outcome: Outcome,
}

/// Route decision for one authorized synthetic civil-warning event.
/// Every topology sees the same authority state, cohorts and service faults.
pub fn evaluate(scenario: &Scenario, architecture: Architecture) -> Outcome {
    let mut outcome = Outcome {
        intended_receivers: scenario.intended_receivers(),
        ..Outcome::default()
    };

    // No architecture may bypass the competent authority gate.
    if !scenario.alert_authorized {
        return outcome;
    }

    match architecture {
        Architecture::CentralizedHubSpoke => {
            // All origin traffic traverses the central hub.
            if scenario.central_hub_up && scenario.origin_to_central_up {
                if scenario.origin_last_mile_up {
                    outcome.local_delivered = u64::from(scenario.local_receivers);
                }
                if scenario.central_remote_path_up && scenario.remote_last_miles_up {
                    outcome.remote_a_delivered = u64::from(scenario.remote_a_receivers);
                    outcome.remote_b_delivered = u64::from(scenario.remote_b_receivers);
                }
            }
        }
        Architecture::RegionalFederation => {
            // Local and cross-region delivery depend on the origin regional service.
            if scenario.origin_regional_service_up {
                if scenario.origin_last_mile_up {
                    outcome.local_delivered = u64::from(scenario.local_receivers);
                }
                if scenario.interregional_federation_up
                    && scenario.remote_regional_services_up
                    && scenario.remote_last_miles_up
                {
                    outcome.remote_a_delivered = u64::from(scenario.remote_a_receivers);
                    outcome.remote_b_delivered = u64::from(scenario.remote_b_receivers);
                }
            }
        }
        Architecture::LocalFirstFederation => {
            // Local continuity is independent of WAN/federation availability.
            if scenario.local_first_local_path_up && scenario.origin_last_mile_up {
                outcome.local_delivered = u64::from(scenario.local_receivers);
            }
            // Cross-region sharing remains dependent on the federation path.
            if scenario.origin_regional_service_up
                && scenario.interregional_federation_up
                && scenario.remote_regional_services_up
                && scenario.remote_last_miles_up
            {
                outcome.remote_a_delivered = u64::from(scenario.remote_a_receivers);
                outcome.remote_b_delivered = u64::from(scenario.remote_b_receivers);
            }
        }
    }

    outcome
}

pub fn compare(scenarios: &[Scenario]) -> Vec<ComparisonRow> {
    let mut rows = Vec::with_capacity(scenarios.len() * Architecture::ALL.len());
    for scenario in scenarios {
        for architecture in Architecture::ALL {
            rows.push(ComparisonRow {
                scenario_id: scenario.scenario_id.clone(),
                architecture,
                outcome: evaluate(scenario, architecture),
            });
        }
    }
    rows
}

fn parse_bool(s: &str, field: &str) -> Result<bool, String> {
    match s {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => Err(format!("{field} must be true or false, got {s}")),
    }
}

fn parse_u32(s: &str, field: &str) -> Result<u32, String> {
    s.parse().map_err(|_| format!("{field} must be an unsigned integer, got {s}"))
}

pub fn parse_fixture(input: &str) -> Result<Vec<Scenario>, String> {
    const HEADER: &str = "scenario_id,alert_authorized,central_hub_up,origin_to_central_up,central_remote_path_up,origin_regional_service_up,interregional_federation_up,remote_regional_services_up,local_first_local_path_up,origin_last_mile_up,remote_last_miles_up,local_receivers,remote_a_receivers,remote_b_receivers";

    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("topology fixture header does not match frozen v0.1 contract".into());
    }

    let mut scenarios = Vec::new();
    let mut ids = HashSet::new();

    for (line_idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 14 {
            return Err(format!("line {}: expected 14 fields, got {}", line_idx + 2, f.len()));
        }

        let scenario = Scenario {
            scenario_id: f[0].to_owned(),
            alert_authorized: parse_bool(f[1], "alert_authorized")?,
            central_hub_up: parse_bool(f[2], "central_hub_up")?,
            origin_to_central_up: parse_bool(f[3], "origin_to_central_up")?,
            central_remote_path_up: parse_bool(f[4], "central_remote_path_up")?,
            origin_regional_service_up: parse_bool(f[5], "origin_regional_service_up")?,
            interregional_federation_up: parse_bool(f[6], "interregional_federation_up")?,
            remote_regional_services_up: parse_bool(f[7], "remote_regional_services_up")?,
            local_first_local_path_up: parse_bool(f[8], "local_first_local_path_up")?,
            origin_last_mile_up: parse_bool(f[9], "origin_last_mile_up")?,
            remote_last_miles_up: parse_bool(f[10], "remote_last_miles_up")?,
            local_receivers: parse_u32(f[11], "local_receivers")?,
            remote_a_receivers: parse_u32(f[12], "remote_a_receivers")?,
            remote_b_receivers: parse_u32(f[13], "remote_b_receivers")?,
        };

        if scenario.scenario_id.trim().is_empty() {
            return Err(format!("line {}: scenario_id is required", line_idx + 2));
        }
        if !ids.insert(scenario.scenario_id.clone()) {
            return Err(format!("line {}: duplicate scenario_id", line_idx + 2));
        }
        if scenario.local_receivers == 0
            || scenario.remote_a_receivers == 0
            || scenario.remote_b_receivers == 0
        {
            return Err(format!("line {}: all synthetic receiver cohorts must be non-zero", line_idx + 2));
        }
        scenarios.push(scenario);
    }

    if scenarios.is_empty() {
        return Err("topology fixture contains no scenarios".into());
    }
    Ok(scenarios)
}

pub fn fixture_comparison() -> Result<Vec<ComparisonRow>, String> {
    Ok(compare(&parse_fixture(FIXTURE)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario(id: &str) -> Scenario {
        parse_fixture(FIXTURE)
            .unwrap()
            .into_iter()
            .find(|s| s.scenario_id == id)
            .unwrap()
    }

    fn get(s: &Scenario, a: Architecture) -> Outcome {
        evaluate(s, a)
    }

    #[test]
    fn fixture_is_frozen_and_unique() {
        fn fnv1a64(bytes: &[u8]) -> u64 {
            bytes.iter().fold(0xcbf29ce484222325u64, |mut hash, byte| {
                hash ^= u64::from(*byte);
                hash.wrapping_mul(0x100000001b3)
            })
        }
        assert_eq!(fnv1a64(FIXTURE.as_bytes()), 0x0cf24a2151b682ad);
        let scenarios = parse_fixture(FIXTURE).unwrap();
        assert_eq!(scenarios.len(), 10);
        let ids: HashSet<_> = scenarios.iter().map(|s| s.scenario_id.as_str()).collect();
        assert_eq!(ids.len(), 10);
        assert!(scenarios.iter().all(|s| s.intended_receivers() == 2_000));
    }

    #[test]
    fn baseline_all_architectures_deliver_all_synthetic_receivers() {
        let s = scenario("all-available");
        for a in Architecture::ALL {
            let o = get(&s, a);
            assert_eq!(o.delivered(), 2_000, "{}: {o:?}", a.label());
            assert_eq!(o.missed(), 0);
            assert!((o.delivery_ratio() - 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn central_hub_outage_only_breaks_hub_spoke_reference() {
        let s = scenario("central-hub-outage");
        assert_eq!(get(&s, Architecture::CentralizedHubSpoke).delivered(), 0);
        assert_eq!(get(&s, Architecture::RegionalFederation).delivered(), 2_000);
        assert_eq!(get(&s, Architecture::LocalFirstFederation).delivered(), 2_000);
    }

    #[test]
    fn origin_central_uplink_loss_does_not_disable_local_or_regional_federation() {
        let s = scenario("origin-central-uplink-loss");
        assert_eq!(get(&s, Architecture::CentralizedHubSpoke).delivered(), 0);
        assert_eq!(get(&s, Architecture::RegionalFederation).delivered(), 2_000);
        assert_eq!(get(&s, Architecture::LocalFirstFederation).delivered(), 2_000);
    }

    #[test]
    fn interregional_partition_preserves_origin_region_delivery() {
        let s = scenario("interregional-partition");
        let c = get(&s, Architecture::CentralizedHubSpoke);
        let r = get(&s, Architecture::RegionalFederation);
        let l = get(&s, Architecture::LocalFirstFederation);
        assert_eq!(c.delivered(), 2_000);
        assert_eq!(r.local_delivered, 1_000);
        assert_eq!(r.remote_delivered(), 0);
        assert_eq!(l.local_delivered, 1_000);
        assert_eq!(l.remote_delivered(), 0);
        assert_eq!(l.local_continuity(&s), 1.0);
    }

    #[test]
    fn regional_coordinator_failure_leaves_local_first_fallback_available() {
        let s = scenario("origin-regional-service-outage");
        let c = get(&s, Architecture::CentralizedHubSpoke);
        let r = get(&s, Architecture::RegionalFederation);
        let l = get(&s, Architecture::LocalFirstFederation);
        assert_eq!(c.delivered(), 2_000);
        assert_eq!(r.delivered(), 0);
        assert_eq!(l.local_delivered, 1_000);
        assert_eq!(l.remote_delivered(), 0);
    }

    #[test]
    fn local_first_path_outage_does_not_silently_count_local_receivers() {
        let s = scenario("local-first-path-outage");
        let l = get(&s, Architecture::LocalFirstFederation);
        assert_eq!(l.local_delivered, 0);
        assert_eq!(l.remote_delivered(), 1_000);
        assert_eq!(l.missed(), 1_000);
    }

    #[test]
    fn last_mile_faults_are_not_hidden_by_healthy_backhaul() {
        let local_s = scenario("origin-last-mile-outage");
        let remote_s = scenario("remote-last-mile-outage");
        for a in Architecture::ALL {
            assert_eq!(get(&local_s, a).local_delivered, 0, "{}", a.label());
            assert_eq!(get(&remote_s, a).remote_delivered(), 0, "{}", a.label());
        }
    }

    #[test]
    fn authorization_failure_blocks_every_architecture() {
        let s = scenario("origin-authority-unavailable");
        for a in Architecture::ALL {
            assert_eq!(get(&s, a).delivered(), 0, "{}", a.label());
        }
    }

    #[test]
    fn remote_regional_service_failure_only_affects_federated_remote_routes() {
        let s = scenario("remote-regional-services-outage");
        assert_eq!(get(&s, Architecture::CentralizedHubSpoke).delivered(), 2_000);
        assert_eq!(get(&s, Architecture::RegionalFederation).delivered(), 1_000);
        assert_eq!(get(&s, Architecture::LocalFirstFederation).delivered(), 1_000);
    }

    #[test]
    fn comparison_uses_every_scenario_for_every_architecture() {
        let scenarios = parse_fixture(FIXTURE).unwrap();
        let rows = compare(&scenarios);
        assert_eq!(rows.len(), scenarios.len() * 3);
        for s in &scenarios {
            assert_eq!(rows.iter().filter(|r| r.scenario_id.as_str() == s.scenario_id.as_str()).count(), 3);
        }
    }

    #[test]
    fn malformed_and_duplicate_fixture_rows_fail_closed() {
        let malformed = FIXTURE.replace("all-available,true", "all-available,yes");
        assert!(parse_fixture(&malformed).is_err());

        let first = FIXTURE.lines().nth(1).unwrap();
        let duplicated = format!("{}\n{}\n", FIXTURE.trim_end(), first);
        assert!(parse_fixture(&duplicated).is_err());
    }
}
