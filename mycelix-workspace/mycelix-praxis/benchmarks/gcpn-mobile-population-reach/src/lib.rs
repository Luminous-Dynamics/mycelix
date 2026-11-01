//! Aggregate-only mobile-population warning reach benchmark.
//!
//! No person/device identifiers, GPS traces, persistent movement histories,
//! immigration-status inference or individual targeting are represented.
//! Aggregate receipt unions must be supplied by a governed deduplication
//! process; summing channel receipts is never treated as unique reach.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum CohortClass {
    Resident,
    Commuter,
    Transit,
    Seasonal,
    DisplacedSynthetic,
    MovementConstrained,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct PopulationKey {
    pub cohort_id: String,
    pub zone_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PopulationCount {
    pub key: PopulationKey,
    pub count: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AggregateFlow {
    pub cohort_id: String,
    pub from_zone: String,
    pub to_zone: String,
    pub count: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlowResult {
    pub populations: Vec<PopulationCount>,
    pub total_before: u64,
    pub total_after: u64,
    pub moved: u64,
}

/// Apply a time-bucketed aggregate flow matrix.
///
/// Counts are integer cohort aggregates. Every departure must be available in
/// its origin cell. The transition conserves cohort totals exactly; no
/// individual is represented or assigned a persistent ID.
pub fn apply_aggregate_flows(
    populations: &[PopulationCount],
    flows: &[AggregateFlow],
) -> Result<FlowResult, String> {
    let mut cells: BTreeMap<PopulationKey, u64> = BTreeMap::new();
    for item in populations {
        if cells.insert(item.key.clone(), item.count).is_some() {
            return Err(format!(
                "duplicate population cell: {}/{}",
                item.key.cohort_id, item.key.zone_id
            ));
        }
    }

    let total_before: u64 = cells.values().sum();
    let mut outgoing: BTreeMap<PopulationKey, u64> = BTreeMap::new();
    let mut incoming: BTreeMap<PopulationKey, u64> = BTreeMap::new();
    let mut moved = 0u64;

    for flow in flows {
        if flow.count == 0 {
            return Err("zero-count flow is not a valid movement record".into());
        }
        let from = PopulationKey {
            cohort_id: flow.cohort_id.clone(),
            zone_id: flow.from_zone.clone(),
        };
        let to = PopulationKey {
            cohort_id: flow.cohort_id.clone(),
            zone_id: flow.to_zone.clone(),
        };
        if !cells.contains_key(&from) {
            return Err(format!("unknown flow origin: {}/{}", from.cohort_id, from.zone_id));
        }
        if !cells.contains_key(&to) {
            return Err(format!("unknown flow destination: {}/{}", to.cohort_id, to.zone_id));
        }
        *outgoing.entry(from).or_default() += flow.count;
        *incoming.entry(to).or_default() += flow.count;
        moved += flow.count;
    }

    let mut after = BTreeMap::new();
    for (key, count) in &cells {
        let departures = outgoing.get(key).copied().unwrap_or(0);
        if departures > *count {
            return Err(format!(
                "flow overdraws origin cell: {}/{}",
                key.cohort_id, key.zone_id
            ));
        }
        let retained = count - departures;
        let arrivals = incoming.get(key).copied().unwrap_or(0);
        after.insert(key.clone(), retained + arrivals);
    }

    let total_after: u64 = after.values().sum();
    if total_before != total_after {
        return Err(format!(
            "aggregate population conservation failed: before={total_before}, after={total_after}"
        ));
    }

    Ok(FlowResult {
        populations: after
            .into_iter()
            .map(|(key, count)| PopulationCount { key, count })
            .collect(),
        total_before,
        total_after,
        moved,
    })
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReachCell {
    pub cohort_id: String,
    pub cohort_class: CohortClass,
    pub zone_id: String,
    pub time_bucket: u32,
    /// Aggregate exposure/person-time denominator for this zone/time.
    pub exposed: u64,
    /// Governed, pre-deduplicated union across channels. None means unknown.
    pub unique_timely_receipts: Option<u64>,
    /// Aggregate union of exposed people receiving an appropriate format.
    pub unique_accessible_receipts: Option<u64>,
    /// Sum of channel receipts; can exceed unique reach and is never substituted.
    pub channel_receipt_count: u64,
    pub accessibility_required: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CohortReach {
    pub cohort_id: String,
    pub cohort_class: Option<CohortClass>,
    pub exposed: u64,
    pub timely_covered: u64,
    pub accessible_covered: u64,
    pub accessibility_required_unserved: u64,
    pub unknown_unique_reach_cells: u32,
    pub unknown_accessibility_cells: u32,
    pub channel_receipts: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReachReport {
    pub cells_seen: u32,
    pub exposure_units: u64,
    pub timely_unique_reach: u64,
    pub accessible_reach: u64,
    pub accessibility_required_unserved: u64,
    pub unique_reach_unknown_cells: u32,
    pub accessibility_unknown_cells: u32,
    pub channel_receipt_total: u64,
    pub duplicate_receipts_above_union: Option<u64>,
    pub effective_timely_reach: Option<f64>,
    pub effective_accessible_reach: Option<f64>,
    pub per_cohort: Vec<CohortReach>,
    pub worst_cohort_effective_reach: Option<f64>,
}

pub fn evaluate_reach(cells: &[ReachCell]) -> Result<ReachReport, String> {
    let mut keys = BTreeSet::new();
    let mut total_exposed = 0u64;
    let mut timely = 0u64;
    let mut accessible = 0u64;
    let mut accessibility_required_unserved = 0u64;
    let mut unknown_unique = 0u32;
    let mut unknown_accessibility = 0u32;
    let mut channel_total = 0u64;
    let mut per: BTreeMap<String, CohortReach> = BTreeMap::new();

    for cell in cells {
        let key = (
            cell.cohort_id.as_str(),
            cell.zone_id.as_str(),
            cell.time_bucket,
        );
        if !keys.insert(key) {
            return Err("duplicate cohort/zone/time reach cell".into());
        }
        if let Some(count) = cell.unique_timely_receipts {
            if count > cell.exposed {
                return Err("unique timely receipts exceed exposed denominator".into());
            }
            if cell.channel_receipt_count < count {
                return Err("channel receipt total cannot be smaller than unique receipt union".into());
            }
        } else {
            unknown_unique += 1;
        }
        if let Some(count) = cell.unique_accessible_receipts {
            if count > cell.exposed {
                return Err("accessible receipts exceed exposed denominator".into());
            }
            match cell.unique_timely_receipts {
                Some(timely_count) if count > timely_count => {
                    return Err("accessible reach cannot exceed timely unique reach".into());
                }
                None => {
                    return Err("accessible unique union cannot be known when timely unique union is unknown".into());
                }
                _ => {}
            }
        } else {
            unknown_accessibility += 1;
        }

        total_exposed += cell.exposed;
        timely += cell.unique_timely_receipts.unwrap_or(0);
        accessible += cell.unique_accessible_receipts.unwrap_or(0);
        if cell.accessibility_required {
            if let (Some(timely_count), Some(accessible_count)) = (cell.unique_timely_receipts, cell.unique_accessible_receipts) {
                accessibility_required_unserved += timely_count - accessible_count;
            }
        }
        channel_total += cell.channel_receipt_count;
        let row = per.entry(cell.cohort_id.clone()).or_insert_with(|| CohortReach {
            cohort_id: cell.cohort_id.clone(),
            cohort_class: Some(cell.cohort_class),
            ..CohortReach::default()
        });
        if row.cohort_class != Some(cell.cohort_class) {
            return Err("cohort class changes across cells".into());
        }
        row.exposed += cell.exposed;
        row.timely_covered += cell.unique_timely_receipts.unwrap_or(0);
        row.accessible_covered += cell.unique_accessible_receipts.unwrap_or(0);
        if cell.accessibility_required {
            if let (Some(timely_count), Some(accessible_count)) = (cell.unique_timely_receipts, cell.unique_accessible_receipts) {
                row.accessibility_required_unserved += timely_count - accessible_count;
            }
        }
        row.unknown_unique_reach_cells += if cell.unique_timely_receipts.is_none() { 1 } else { 0 };
        row.unknown_accessibility_cells += if cell.unique_accessible_receipts.is_none() { 1 } else { 0 };
        row.channel_receipts += cell.channel_receipt_count;
    }

    let effective_timely_reach = (unknown_unique == 0 && total_exposed > 0)
        .then_some(timely as f64 / total_exposed as f64);
    let effective_accessible_reach =
        (unknown_accessibility == 0 && total_exposed > 0)
            .then_some(accessible as f64 / total_exposed as f64);
    let worst_cohort_effective_reach = if per.values().all(|row| {
        row.unknown_unique_reach_cells == 0 && row.exposed > 0
    }) {
        per.values()
            .map(|row| row.timely_covered as f64 / row.exposed as f64)
            .min_by(f64::total_cmp)
    } else {
        None
    };

    Ok(ReachReport {
        cells_seen: cells.len() as u32,
        exposure_units: total_exposed,
        timely_unique_reach: timely,
        accessible_reach: accessible,
        accessibility_required_unserved,
        unique_reach_unknown_cells: unknown_unique,
        accessibility_unknown_cells: unknown_accessibility,
        channel_receipt_total: channel_total,
        duplicate_receipts_above_union: (unknown_unique == 0).then_some(channel_total.saturating_sub(timely)),
        effective_timely_reach,
        effective_accessible_reach,
        per_cohort: per.into_values().collect(),
        worst_cohort_effective_reach,
    })
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityHandoff {
    pub source_jurisdiction: String,
    pub destination_jurisdiction: String,
    pub request_recorded_at: u64,
    pub acknowledged_at: Option<u64>,
    pub destination_authority_id: String,
    pub scope_matches_hazard_and_zone: bool,
    pub permission_to_reissue: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HandoffStatus {
    AcceptedForRelay,
    AcceptedForReissue,
    AwaitingAcknowledgement,
    RejectedInvalidHandoff,
    RejectedScopeMismatch,
    RejectedNoReissuePermission,
}

pub fn validate_handoff(handoff: &AuthorityHandoff) -> HandoffStatus {
    if handoff.source_jurisdiction.trim().is_empty()
        || handoff.destination_jurisdiction.trim().is_empty()
        || handoff.destination_authority_id.trim().is_empty()
        || handoff.acknowledged_at.is_some_and(|ack| ack < handoff.request_recorded_at)
    {
        return HandoffStatus::RejectedInvalidHandoff;
    }
    if !handoff.scope_matches_hazard_and_zone {
        return HandoffStatus::RejectedScopeMismatch;
    }
    if handoff.acknowledged_at.is_none() {
        return HandoffStatus::AwaitingAcknowledgement;
    }
    if handoff.permission_to_reissue {
        HandoffStatus::AcceptedForReissue
    } else {
        HandoffStatus::AcceptedForRelay
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PublicCell {
    pub cell_id: String,
    pub count: u64,
}

/// Simple minimum-cell suppression control. This is not differential privacy
/// and does not protect against arbitrary composition/differencing attacks.
pub fn suppress_small_cells(cells: &[PublicCell], minimum_cell_size: u64) -> Vec<Option<u64>> {
    cells
        .iter()
        .map(|cell| (cell.count >= minimum_cell_size).then_some(cell.count))
        .collect()
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AggregateQuery {
    pub query_id: String,
    pub member_cell_ids: BTreeSet<String>,
    pub published_total: u64,
}

/// Detect the simplest differencing disclosure: two published aggregate
/// queries whose sets differ by exactly one small suppressed cell. This is an
/// explicit negative control, not a complete privacy attack model.
pub fn detect_single_cell_differencing(
    queries: &[AggregateQuery],
    cell_sizes: &BTreeMap<String, u64>,
    minimum_cell_size: u64,
) -> Vec<(String, String, String)> {
    let mut findings = Vec::new();
    for left_index in 0..queries.len() {
        for right_index in (left_index + 1)..queries.len() {
            let left = &queries[left_index];
            let right = &queries[right_index];
            let left_only: Vec<_> = left
                .member_cell_ids
                .difference(&right.member_cell_ids)
                .collect();
            let right_only: Vec<_> = right
                .member_cell_ids
                .difference(&left.member_cell_ids)
                .collect();
            if left_only.len() == 1 && right_only.is_empty() {
                let cell_id = left_only[0];
                if let Some(cell_count) = cell_sizes.get(cell_id) {
                    let difference = left.published_total.checked_sub(right.published_total);
                    if *cell_count < minimum_cell_size && difference == Some(*cell_count) {
                        findings.push((
                            left.query_id.clone(),
                            right.query_id.clone(),
                            cell_id.clone(),
                        ));
                    }
                }
            } else if right_only.len() == 1 && left_only.is_empty() {
                let cell_id = right_only[0];
                if let Some(cell_count) = cell_sizes.get(cell_id) {
                    let difference = right.published_total.checked_sub(left.published_total);
                    if *cell_count < minimum_cell_size && difference == Some(*cell_count) {
                        findings.push((
                            right.query_id.clone(),
                            left.query_id.clone(),
                            cell_id.clone(),
                        ));
                    }
                }
            }
        }
    }
    findings
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pop(cohort: &str, zone: &str, count: u64) -> PopulationCount {
        PopulationCount {
            key: PopulationKey {
                cohort_id: cohort.into(),
                zone_id: zone.into(),
            },
            count,
        }
    }

    #[test]
    fn aggregate_flows_conserve_totals_and_keep_constrained_cohort() {
        let population = vec![pop("commuter", "home", 100), pop("commuter", "work", 0),
            pop("constrained", "home", 40), pop("constrained", "work", 0)];
        let flows = vec![AggregateFlow {
            cohort_id: "commuter".into(), from_zone: "home".into(), to_zone: "work".into(), count: 65,
        }];
        let result = apply_aggregate_flows(&population, &flows).unwrap();
        assert_eq!(result.total_before, 140);
        assert_eq!(result.total_after, 140);
        assert_eq!(result.moved, 65);
        let constrained = result.populations.iter().find(|p| p.key.cohort_id == "constrained" && p.key.zone_id == "home").unwrap();
        assert_eq!(constrained.count, 40);
    }

    #[test]
    fn overdrawn_and_unknown_flows_fail_closed() {
        let population = vec![pop("cohort", "a", 10), pop("cohort", "b", 0)];
        let overdraw = vec![AggregateFlow { cohort_id:"cohort".into(),from_zone:"a".into(),to_zone:"b".into(),count:11 }];
        assert!(apply_aggregate_flows(&population,&overdraw).unwrap_err().contains("overdraws"));
        let unknown = vec![AggregateFlow { cohort_id:"cohort".into(),from_zone:"a".into(),to_zone:"c".into(),count:1 }];
        assert!(apply_aggregate_flows(&population,&unknown).unwrap_err().contains("unknown flow destination"));
    }

    #[test]
    fn duplicate_population_cells_are_rejected() {
        let population = vec![pop("c","a",1),pop("c","a",1)];
        assert!(apply_aggregate_flows(&population,&[]).is_err());
    }

    fn cell(cohort:&str,class:CohortClass,zone:&str,exposed:u64,timely:Option<u64>,accessible:Option<u64>,receipts:u64)->ReachCell{
        ReachCell{cohort_id:cohort.into(),cohort_class:class,zone_id:zone.into(),time_bucket:1,exposed,unique_timely_receipts:timely,unique_accessible_receipts:accessible,channel_receipt_count:receipts,accessibility_required:true}
    }

    #[test]
    fn aggregate_channel_receipts_never_substitute_for_unique_reach(){
        let cells=vec![cell("commuters",CohortClass::Commuter,"work",100,Some(80),Some(70),150)];
        let report=evaluate_reach(&cells).unwrap();
        assert_eq!(report.timely_unique_reach,80);
        assert_eq!(report.channel_receipt_total,150);
        assert_eq!(report.duplicate_receipts_above_union,Some(70));
        assert_eq!(report.effective_timely_reach,Some(0.8));
        assert_eq!(report.accessibility_required_unserved,10);
    }

    #[test]
    fn missing_unique_union_makes_effective_and_duplicate_reach_unknown(){
        let cells=vec![cell("transit",CohortClass::Transit,"hub",50,None,None,30)];
        let report=evaluate_reach(&cells).unwrap();
        assert_eq!(report.effective_timely_reach,None);
        assert_eq!(report.unique_reach_unknown_cells,1);
        assert_eq!(report.duplicate_receipts_above_union,None);
    }

    #[test]
    fn accessible_union_cannot_be_claimed_without_timely_union(){
        let cells=vec![cell("transit",CohortClass::Transit,"hub",50,None,Some(20),30)];
        assert!(evaluate_reach(&cells).unwrap_err().contains("timely unique union is unknown"));
    }

    #[test]
    fn accessible_reach_cannot_exceed_timely_unique_reach(){
        let cells=vec![cell("c",CohortClass::Resident,"z",10,Some(5),Some(6),6)];
        assert!(evaluate_reach(&cells).is_err());
    }

    #[test]
    fn cross_jurisdiction_handoff_never_reissues_without_ack_or_scope(){
        let base=AuthorityHandoff{source_jurisdiction:"A".into(),destination_jurisdiction:"B".into(),request_recorded_at:10,acknowledged_at:Some(12),destination_authority_id:"authority-B".into(),scope_matches_hazard_and_zone:true,permission_to_reissue:false};
        assert_eq!(validate_handoff(&base),HandoffStatus::AcceptedForRelay);
        let mut no_ack=base.clone();no_ack.acknowledged_at=None;
        assert_eq!(validate_handoff(&no_ack),HandoffStatus::AwaitingAcknowledgement);
        let mut bad_scope=base.clone();bad_scope.scope_matches_hazard_and_zone=false;
        assert_eq!(validate_handoff(&bad_scope),HandoffStatus::RejectedScopeMismatch);
        let mut time_reversed=base.clone();time_reversed.acknowledged_at=Some(9);
        assert_eq!(validate_handoff(&time_reversed),HandoffStatus::RejectedInvalidHandoff);
    }

    #[test]
    fn small_cell_suppression_masks_below_threshold_counts(){
        let cells=vec![PublicCell{cell_id:"large".into(),count:30},PublicCell{cell_id:"small".into(),count:3}];
        assert_eq!(suppress_small_cells(&cells,10),vec![Some(30),None]);
    }

    #[test]
    fn simple_differencing_attack_on_suppressed_cell_is_detected(){
        let cells=BTreeMap::from([("a".to_string(),20),("b".to_string(),2)]);
        let queries=vec![
            AggregateQuery{query_id:"q1".into(),member_cell_ids:BTreeSet::from(["a".to_string(),"b".to_string()]),published_total:22},
            AggregateQuery{query_id:"q2".into(),member_cell_ids:BTreeSet::from(["a".to_string()]),published_total:20},
        ];
        let findings=detect_single_cell_differencing(&queries,&cells,10);
        assert_eq!(findings,vec![("q1".into(),"q2".into(),"b".into())]);
        let mut inconsistent=queries.clone();
        inconsistent[0].published_total=99;
        assert!(detect_single_cell_differencing(&inconsistent,&cells,10).is_empty());
    }

    #[test]
    fn duplicate_reach_cells_and_receipt_overcounts_are_visible(){
        let first=cell("c",CohortClass::Resident,"z",10,Some(5),Some(5),12);
        let mut duplicate=first.clone();
        assert!(evaluate_reach(&[first.clone(),duplicate.clone()]).is_err());
        duplicate.time_bucket=2;
        let report=evaluate_reach(&[first,duplicate]).unwrap();
        assert_eq!(report.duplicate_receipts_above_union,Some(14));
    }
}
