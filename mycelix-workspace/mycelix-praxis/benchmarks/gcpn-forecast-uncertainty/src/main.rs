use gcpn_forecast_uncertainty::fixture_report;

fn main() {
    match fixture_report() {
        Ok(r) => {
            println!("GCPN paired event-cluster bootstrap uncertainty v0.1");
            println!("source_records: {}", r.source_record_count);
            println!("source_fixture_fnv1a64: 0x{:016x}", r.source_fixture_fingerprint);
            println!("complete_event_clusters: {}", r.diagnostics.complete_event_clusters);
            println!("exact_duplicate_rows_removed: {}", r.diagnostics.exact_duplicate_rows);
            println!("partition_leak_events_excluded: {}", r.diagnostics.partition_leak_events);
            println!("metadata_mismatch_events_excluded: {}", r.diagnostics.metadata_mismatch_events);
            println!("bootstrap_seed: 0x{:016x}", r.seed);
            println!("bootstrap_replicates: {}", r.replicates);
            println!("candidate_brier: {:.8}", r.candidate_brier);
            println!("climatology_brier: {:.8}", r.climatology_brier);
            println!("lead_reference_brier: {:.8}", r.lead_reference_brier);
            println!();
            println!("comparison,point_candidate_minus_baseline,percentile_95_low,percentile_95_high");
            for c in [&r.candidate_vs_climatology, &r.candidate_vs_lead_reference] {
                println!("{},{:.8},{:.8},{:.8}", c.baseline_model, c.point_mean_difference, c.percentile_95_low, c.percentile_95_high);
            }
            println!();
            println!("interpretation: both synthetic intervals cross zero; point scores do not establish superiority");
            println!("limitation: event_id is an assumed resampling unit in this fixture, not a validated dependence cluster");
        }
        Err(e) => {
            eprintln!("GCPN forecast uncertainty benchmark failed: {e}");
            std::process::exit(1);
        }
    }
}
