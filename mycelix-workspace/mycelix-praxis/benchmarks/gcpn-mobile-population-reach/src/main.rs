use gcpn_mobile_population_reach::{
    apply_aggregate_flows, evaluate_reach, AggregateFlow, CohortClass, PopulationCount,
    PopulationKey, ReachCell,
};

fn main() {
    let populations = vec![
        PopulationCount { key: PopulationKey { cohort_id: "commuter".into(), zone_id: "home".into() }, count: 100 },
        PopulationCount { key: PopulationKey { cohort_id: "commuter".into(), zone_id: "work".into() }, count: 0 },
        PopulationCount { key: PopulationKey { cohort_id: "movement-constrained".into(), zone_id: "home".into() }, count: 40 },
        PopulationCount { key: PopulationKey { cohort_id: "movement-constrained".into(), zone_id: "work".into() }, count: 0 },
    ];
    let flows = vec![AggregateFlow { cohort_id: "commuter".into(), from_zone: "home".into(), to_zone: "work".into(), count: 65 }];
    let transitioned = apply_aggregate_flows(&populations, &flows).expect("valid synthetic aggregate flow");
    let reach = vec![
        ReachCell { cohort_id:"commuter".into(),cohort_class:CohortClass::Commuter,zone_id:"work".into(),time_bucket:1,exposed:65,unique_timely_receipts:Some(50),unique_accessible_receipts:Some(45),channel_receipt_count:90,accessibility_required:true },
        ReachCell { cohort_id:"movement-constrained".into(),cohort_class:CohortClass::MovementConstrained,zone_id:"home".into(),time_bucket:1,exposed:40,unique_timely_receipts:Some(32),unique_accessible_receipts:Some(30),channel_receipt_count:45,accessibility_required:true },
    ];
    let report = evaluate_reach(&reach).expect("valid synthetic aggregate reach fixture");
    println!("GCPN aggregate mobile-population reach v0.1");
    println!("status: synthetic_aggregate_demo");
    println!("population_before: {}", transitioned.total_before);
    println!("population_after: {}", transitioned.total_after);
    println!("moved_aggregate_count: {}", transitioned.moved);
    println!("exposure_person_time_units: {}", report.exposure_units);
    println!("unique_timely_reach: {}", report.timely_unique_reach);
    println!("channel_receipts_not_unique: {}", report.channel_receipt_total);
    println!("duplicate_receipts_above_union: {:?}", report.duplicate_receipts_above_union);
    println!("effective_timely_reach: {:?}", report.effective_timely_reach);
    println!("effective_accessible_reach: {:?}", report.effective_accessible_reach);
    println!("worst_cohort_effective_reach: {:?}", report.worst_cohort_effective_reach);
    println!("non_claim: aggregate synthetic counts are not observed individual receipt or comprehension");
}
