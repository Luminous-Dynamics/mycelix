# Craft Evidence Acquisition Proposal (EAP) oracle V0.1

Test-only structural receipt validator for advisory evidence-acquisition proposals.

The EAP connects three immutable objects:

1. **Prior Decision Basis Capsule (DBC):** what was justified at the exact accepted frontier.
2. **Evidence Acquisition Proposal:** what optional observation, mandatory measurement or authorized review could be useful next, and under which explicit EVSI/loss/cost assumptions.
3. **Next DBC:** a new capsule after new evidence has actually been acquired, admitted, verified and replayed.

The EAP references the prior DBC digest. The prior DBC does not embed or hash the EAP. This avoids cyclic commitments and keeps a current decision basis separate from a future evidence-acquisition policy. The EVSI receipt must bind the same DBC decision ID, scope digest and frontier digest, plus the same explicit decision-loss-profile digest.

## Hard safety boundary

- Wire capability is fixed to `AdvisoryOnly`.
- The EAP cannot dispatch, create work orders, authorize sampling, amend a permit, issue a public advisory, change an action denominator or close an episode.
- Mandatory safety/legal/professional/water-quality evidence cannot be ranked away by optional EVSI.
- A recommendation is bound to a prior DBC, exact frontier, decision scope, candidate catalogue and observation model. The candidate's candidate-set and observation-model digests must match the EVSI receipt exactly. A changed prerequisite invalidates it.
- Unknown, uncalibrated, unavailable or out-of-domain likelihoods require abstention or authorized human review.
- Common-cause dependencies must be considered; different sensors, vendors, IDs or signing keys do not prove independence.
- Predicted EVSI, realized EVSI, acquired observations and physical outcomes are distinct.

## Validation scope

The Go reference validates a typed EAP model and deterministic domain-separated commitment. It verifies receipt references, DBC/frontier/scope binding, EVSI/cost/loss-profile binding, mandatory-vs-optional disposition, candidate safety/admissibility, result-impact coverage, independence prerequisites, expiry ordering, advisory-only capability and absence of operational effects.

It does not authenticate the referenced DBC, EVSI model, loss function, observation likelihoods, calibration, professional role or field source. Those remain with their owners. It does not recompute the upstream EVSI evaluator.

## KATs

The frozen JSON known-answer suite contains 15 named scenarios, each with an independently authored expected violation/disposition. The Go runner applies each mutation to one fully specified positive control. Positive and negative controls prevent the benchmark from rewarding blanket abstention. Strict JSON encoding/decoding rejects duplicate keys, unknown fields, trailing JSON and null/missing required arrays; round-trip canonical-digest parity is tested.

## Run

```sh
gofmt -w oracle/*.go
go test ./...
go test -race ./...
go vet ./...
go test -cover ./...
```

## Claim ceiling

A valid EAP proves only that an advisory proposal is structurally bound to one specific prior DBC, frontier, loss profile, evaluator and candidate. It does not authorize or perform field observation, prove calibrated probabilities, authenticate source truth, establish legal authority, water-quality safety, physical repair or improved service outcomes.
