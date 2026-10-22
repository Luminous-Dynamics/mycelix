# NRVCA-016: External-validity protocol for KPI comparability v0.1

**Status:** preregistered protocol; not executed; zero real adjudicated pairs acquired  
**Tracking issue:** [#5443](https://github.com/Luminous-Dynamics/mycelix/issues/5443)  
**Pre-registration fixture:** [machine-readable bundle](../../../fixtures/nrvca-external-validity-bundle-v0.1.json)  
**Validator implementation:** `mycelix-workspace/tools/nrvca-validator/src/external_validity.rs`

## Paradigm shift: evaluate selective risk, not fixture pass rate

The benchmark asks whether a comparator is safe *and useful when uncertain*. Exact synthetic known answers only verify contract behavior. External validity requires that test labels be grounded in reproducible source evidence, generated independently of the model, rights-cleared for the intended use, and held out by provider/vintage so near-duplicate pairs cannot leak across folds.

A benchmark can be gamed by abstaining on every pair. It can also report high accuracy because most cases are easy matches. Therefore, report false-comparable risk on hard semantic mismatches separately from false-incomparable errors on matches, plus exact three-way agreement, coverage on adequately evidenced cases, abstention rate/reasons, and provider/field-level errors.

## Source admission and rights

Every admitted source record must identify the publisher, source URL, publication/dataset ID, vintage, capture timestamp, acquired source bytes and SHA-256, extraction method/tool version, and one or more exact locators. Each source also needs rights evidence. Rights state is explicit: `open_reuse`, `permission_granted`, or `evaluation_only` can qualify for evaluation if their evidence and permissions are coherent; `restricted`, `prohibited`, or `unknown` blocks evaluation. Evaluation permission does **not** imply redistribution permission.

This matters for likely industry comparators. AWWA's public page describes aggregate results by utility type, size, and region and provides participant-specific access; it does not make it legitimate to assume its individual-level data are an unrestricted benchmark. IBNET's Use Data page says management-practices data are confidential and shared only with the registered utility. We need the exact terms and authorized definition material before evaluation; the protocol forbids scraping or redistributing protected data.

Sources: [AWWA Utility Benchmarking Program](https://www.awwa.org/programs/benchmarking/), [IBNET Use Data](https://newibnet.org/use-data), [SDMX content-oriented guidance](https://sdmx.org/guidelines/), and [W3C PROV Primer](https://www.w3.org/TR/prov-primer/).

## Human adjudication

Two independent raters label each pair while blinded to comparator output. Labels are `comparable`, `not_comparable`, or `indeterminate`; insufficient source evidence must remain indeterminate. If the first two raters disagree, a third independent adjudicator must record a decision and rationale. Store the initial labels and disagreement instead of overwriting them with one "gold" label.

A known hard mismatch is eligible for the hard-negative denominator only when the evidence is sufficient and the adjudicated label is `not_comparable`. The validator rejects a hard mismatch relabelled as comparable or indeterminate. It also rejects duplicate rater identities, model-visible ratings, unresolved source locators, weak rights states, missing source hashes, and cross-split provider/contract/source-byte leakage.

## Splits and external validity

The primary evaluation uses disjoint provider groups across `development`, `validation`, and `provider_held_out_test`. Random pair splitting is prohibited. Contract IDs and source digests may not cross primary partitions, and the same underlying pair cannot be registered in multiple partitions. A separate temporal-vintage test measures revision robustness, and its results are reported separately rather than quietly mixed into provider-held-out results.

## Proposed qualification targets (not current results)

- At least **300** adjudicated, evidence-sufficient hard mismatch pairs.
- **Zero** `comparable` predictions among those hard mismatches. Under an independent binomial approximation, zero errors in 300 trials yields a one-sided 95% upper error-rate bound near 1%; clustered providers, biased sampling, or dependence weaken that interpretation.
- At least **95%** exact three-way agreement on held-out data, and at least **80%** coverage among adequately evidenced pairs. Report both so abstention cannot inflate apparent accuracy.
- **100%** abstention for unresolved critical semantics absent a known mismatch, as measured in a dedicated missingness challenge set.
- Report false-comparable and false-incomparable rates, per-provider and per-field error, confidence intervals, rater agreement, coverage/abstention, and the performance of a transparent naïve baseline plus each production mapping that is legally available.

The 300-pair threshold and 95%/80% targets are proposed, not empirically optimized. They need review before execution. No claim of state-of-the-art superiority follows from the preregistration itself.

## Current evidence boundary

The fixture contains **zero real sources and zero adjudicated pairs**. This is intentional: no source bytes, rights grants, or human labels have been fabricated. The current artifact must validate as a preregistration but remain blocked for external evaluation. The Rust validator checks that manifests declare acquired bytes, digest syntax, exact locators, extraction metadata, and coherent rights; it does **not** recompute SHA-256 from external bytes, independently verify a license, or prove the truth of any source assertion. A future intake step must compute the actual digest from the captured bytes and retain a reproducible acquisition receipt before an external bundle is treated as byte-verified. Synthetic protocol tests can qualify gate behavior; they cannot establish real-world semantic accuracy.
