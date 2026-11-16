# NeuroTwin transportability and selective-prediction audit

**Status:** research utility; synthetic tests only. Not a clinical risk score, model trainer, calibration service or proof that abstention is safe.  
**Issue:** [NEUROTWIN-004 / #6287](https://github.com/Luminous-Dynamics/mycelix/issues/6287).

This isolated Rust tool evaluates already-locked binary prediction outputs across prespecified domains (for example, trial, site or time period). It reports discrimination, proper scoring rules, calibration and selective Brier risk. It never trains or updates a model.

## Why this exists

A model can preserve AUROC while its probabilities become badly miscalibrated in a new trial. A model can also be accurate on the cases it confidently accepts while failing on cases outside its support. These behaviours are invisible in a single pooled AUROC.

Clinical external-validation guidance calls for discrimination, calibration across the full prediction range, and clinical usefulness when predictions are intended to direct decisions. TRIPOD+AI supplies a reporting checklist; PROBAST+AI separately assesses model development quality and evaluation risk of bias/applicability. See the NeuroTwin evidence landscape for the broader research context.

## Metrics

- AUROC: tie-aware ranking discrimination; undefined if a domain has one outcome class.
- AUPRC: average precision with equal-score groups handled together; undefined if there are no positive outcomes.
- Brier score and log loss: proper scoring rules that reflect probability quality.
- Calibration intercept and slope: logistic calibration regression for evaluation only. The tool does not fit or apply a correction. A non-identifiable or non-convergent fit is reported as null.
- Observed-to-expected event ratio.
- Selective curve: retained-subset Brier score at requested support-score coverages of 100%, 90%, 80%, 70%, 60% and 50%. Ties at a cutoff are retained together; actual coverage may exceed requested coverage. Metrics are suppressed when the retained subset is below 20.

Original domain labels are replaced with stable local ordinal indices and never emitted. Domain rows below 20 are suppressed without a denominator. The overall report requires at least 20 observations. A stricter provider-specific disclosure rule always takes precedence.

## Input

Strict TSV header:

```text
observation_key<TAB>domain<TAB>outcome<TAB>predicted_probability<TAB>support_score
```

- `observation_key`: local-only key for duplicate detection.
- `domain`: local-only trial/site/time key; never emitted.
- `outcome`: `0` or `1`.
- `predicted_probability`: locked probability in [0,1].
- `support_score`: locked model-support score in [0,1], where larger means retain first.

The file must be evaluated in its approved local environment. Do not place participant-level predictions or sensitive derivatives in public repositories, public DHT, IPFS or logs.

## Run

```bash
cargo fmt -- --check
cargo test --locked
cargo clippy --all-targets --locked -- -D warnings
cargo run --locked -- predictions.tsv aggregate-report.json
```

## Interpretation limits

- This tool cannot verify that the model, support score, preprocessing or thresholds were locked before evaluation.
- The support score is not a clinical certainty score. Abstention is not a safe action unless its workflow and harms are independently evaluated.
- This v0.1 report does not estimate confidence intervals, net benefit, decision thresholds or causal treatment effects.
- AUROC/AUPRC do not prove treatment selection utility.
- Recalibration on the evaluation cohort changes the validation context and must be reported as model updating, not untouched external validation.
- Small-cell suppression does not guarantee anonymity; the approved custodian must review every aggregate disclosure.
