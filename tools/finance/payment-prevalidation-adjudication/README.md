# Payment Prevalidation Independent Adjudication Evaluator

Effects-disabled Rust reference evaluator for the independent adjudication corpus in issue 5830.

Run the following commands from the repository root:

cargo fmt --manifest-path tools/finance/payment-prevalidation-adjudication/Cargo.toml -- --check

cargo test --manifest-path tools/finance/payment-prevalidation-adjudication/Cargo.toml

cargo run --manifest-path tools/finance/payment-prevalidation-adjudication/Cargo.toml -- --self-test

The evaluator qualifies two independent, output-blinded primary adjudicators per case; third-adjudicator resolution when they disagree; append-only correction lineage; merchant, participant, payee-identity and time-block disjoint partitions; separate confusion matrices; false-ready, false-quarantine and warning-omission numerators and denominators; raw agreement and Cohen's kappa where defined; and suppression of confidence intervals when the synthetic sample is too small.

System outputs are never used as ground truth. The schema does not admit evaluator-expected-output fields. A name match is not proof of merchant legitimacy or a valid debt. A warn-and-permit-choice result is not an automatic block.

The fixture has nine fictional cases and deliberately includes both a Mycelix false-ready and false-quarantine held-out example. This mixed outcome is an evaluator/report control, not an observed performance result. Reports keep development, calibration and held-out test metrics separate. The configured bootstrap uses 2,000 replicates and fixed seed `20261106`, resampling merchant/time-block clusters. Intervals are suppressed for synthetic data and when fewer than 30 independent clusters exist. That minimum is a software gate, not a power guarantee. A real study needs authorised minimised evidence, independent qualified adjudicators, pre-registration, cluster-aware sampling and a representative held-out set.
