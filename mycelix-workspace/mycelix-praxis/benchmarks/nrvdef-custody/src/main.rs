use nrvdef_custody::fixture_report;

fn main() {
    match fixture_report() {
        Ok(report) => {
            println!("NRVDEF synthetic evidence-custody conformance v0.1");
            println!("fixture_oracle: {}", if report.fixture_oracle_matches() { "PASS" } else { "FAIL" });
            println!("histories: {}", report.history_count);
            println!("events: {}", report.event_count);
            println!("missing_lineage: {}", report.missing_lineage);
            println!("duplicate_event_ids: {}", report.duplicate_event_ids);
            println!("custody_chain_breaks: {}", report.custody_chain_breaks);
            println!("record_integrity_failures: {}", report.record_integrity_failures);
            println!("claim_binding_mismatches: {}", report.claim_binding_mismatches);
            println!("protocol_version_mismatches: {}", report.protocol_version_mismatches);
            println!("invalid_declaration_events: {}", report.invalid_declaration_events);
            println!("invalid_evidence_events: {}", report.invalid_evidence_events);
            println!("unauthorized_inspection_attestations: {}", report.unauthorized_inspection_attestations);
            println!("unauthorized_treaty_determinations: {}", report.unauthorized_treaty_determinations);
            println!("verification_overclaims: {}", report.verification_overclaims);
            println!("treaty_overclaims: {}", report.treaty_overclaims);
            println!("custody_overclaims: {}", report.custody_overclaims);
            println!("dispute_suppressions: {}", report.dispute_suppressions);
            println!("false_dispute_states: {}", report.false_dispute_states);
            println!("protected_disclosures: {}", report.protected_disclosures);
            println!("summary_state_mismatches: {}", report.summary_state_mismatches);
            println!("valid_custody_histories: {}", report.valid_custody_histories);
            println!("valid_inspection_histories: {}", report.valid_inspection_histories);
            println!("valid_treaty_histories: {}", report.valid_treaty_histories);
            println!("interpretation: fixture PASS means expected negative controls were detected; no physical truth or legal status is certified");
            if !report.fixture_oracle_matches() {
                std::process::exit(1);
            }
        }
        Err(error) => {
            eprintln!("fixture error: {error}");
            std::process::exit(2);
        }
    }
}
