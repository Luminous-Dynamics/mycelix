use gcpn_human_outcomes::fixture_report;

fn main() {
    match fixture_report() {
        Ok(report) => {
            println!("GCPN synthetic human-usable warning benchmark v0.1");
            println!("fixture_rows: {}", report.rows.len());
            println!("fixture_fingerprint_fnv1a64: 0x{:016x}", report.fixture_fingerprint);
            println!("fixture_integrity_oracle: PASS");
            println!("counting_invariants: PASS");
            println!("All values are synthetic test vectors, not human-subject observations.");
            println!();
            println!("variant,trials,accessible_rate,comprehension_given_access,effective_comprehension,action_intent_given_access,effective_action_intent,uncertainty_given_access,effective_uncertainty,false_certainty_given_access,worst_profile_comprehension,gap,max_p90_seconds");
            for s in &report.variant_summaries {
                println!("{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{}",
                    s.variant.label(), s.trials, s.accessibility_rate,
                    s.comprehension_given_access.unwrap_or(0.0), s.effective_comprehension,
                    s.action_intent_given_access.unwrap_or(0.0), s.effective_action_intent,
                    s.uncertainty_given_access.unwrap_or(0.0), s.effective_uncertainty_understanding,
                    s.false_certainty_given_access.unwrap_or(0.0),
                    s.minimum_profile_effective_comprehension,
                    s.effective_comprehension_profile_gap, s.maximum_profile_p90_seconds);
            }
            println!();
            println!("profile,variant,accessible_rate,effective_comprehension,effective_action_intent,effective_uncertainty,false_certainty_given_access,p90_seconds");
            for s in &report.profile_summaries {
                println!("{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{}",
                    s.profile.label(), s.variant.label(), s.accessibility_rate,
                    s.effective_comprehension, s.effective_action_intent,
                    s.effective_uncertainty_understanding,
                    s.false_certainty_given_access.unwrap_or(0.0), s.p90_response_seconds);
            }
            println!("non_claim: comprehension tasks are not actual protective behaviour or causal impact");
        }
        Err(error) => {
            eprintln!("GCPN human-outcomes fixture failed: {error}");
            std::process::exit(1);
        }
    }
}
