# Craft EVSI known-answer oracle V0.1

Independent Go reference for a small, finite-state, one-step decision-focused evidence-acquisition model. It is test-only and is not a production planner, field-control system, source-trust verifier or calibrated water-quality model.

## What it computes

For current admitted evidence state D, finite domain state Theta, safe action set A_safe and candidate observation Y, it computes:

- baseline minimum expected decision loss;
- expected post-observation minimum decision loss;
- EVSI as the difference;
- expected information gain (EIG) in bits;
- monetary, delay, burden and risk cost components;
- normalized acquisition cost under an explicitly named loss/cost profile;
- net decision value (EVSI minus normalized cost).

Observation likelihoods must already be conditioned on current admitted evidence and relevant common-cause dependencies. If the model is unavailable, out-of-domain, inadmissible for the decision or not conditioned on current evidence, the oracle abstains. It does not estimate or authenticate those likelihoods.

Mandatory evidence is a hard constraint: an open mandatory-evidence requirement returns AcquireMandatoryEvidence before optional EVSI selection. It is never traded against cost or expected information gain.

## The two benchmark counterexamples

- **EVSI-001:** a 0.5% catastrophic hazard has only about 0.0454 bits of entropy, but a perfect, decision-relevant observation has EVSI 9.95 loss units. With explicitly normalized costs of 1.6, net decision value is 8.35.
- **EVSI-002:** a perfect observation yields 1 bit of information, but policy only allows the same hold action regardless of result. EVSI is zero and normalized cost is 0.3, so the optional observation is not acquired.

These cases are designed to falsify the idea that lower entropy alone means a better operational decision.

## Frozen suite

`fixtures/evsi-known-answer-v0.1.json` contains 12 scenarios covering decision value vs EIG, costs/delay/burden, shared calibration lineage, unknown dependencies, mandatory evidence, unavailable or out-of-domain observations, unqualified samples, missing safe actions and non-discriminating observations.

The unit tests independently load the JSON fixture and compare statuses and numerical outputs. Malformed likelihood distributions fail as input errors. The cost profile makes normalization explicit; deployments must not fabricate weights for non-monetizable or policy-hard constraints.

## Run

```sh
gofmt -w oracle/*.go
go test ./...
go test -race ./...
go vet ./...
go test -cover ./...
```

## Local validation

Go 1.23.2:

- 12 frozen numerical/structural KAT cases passed;
- mandatory-evidence, malformed-likelihood and abstention controls passed;
- go test -race ./... passed;
- go vet ./... passed;
- statement coverage smoke: 87.7%.

These are local reference-module results, not hosted exact-head CI qualification or field-performance results.

## Claim ceiling

This oracle proves only the declared finite-state arithmetic and safe-action constraints in its fixture profile. It does not establish that state probabilities or likelihoods are calibrated, that observations are independent, that evidence is authentic, that actions are legally authorized, or that water-quality/physical outcomes occurred. It issues no dispatch, permit, sampling, advisory or closure mutation. Field calibration, water quality, legal authority, physical outcome and autonomous action remain NotEvaluated / prohibited.
