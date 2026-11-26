//! Emits the memory-specific 48-sequence longitudinal leakage panel.
//! A separate executable keeps the generic #5460 report stable.

fn main() {
    print!(
        "{}",
        life_disclosure_observable_traces::memory_metadata_leakage::report_csv()
    );
}
