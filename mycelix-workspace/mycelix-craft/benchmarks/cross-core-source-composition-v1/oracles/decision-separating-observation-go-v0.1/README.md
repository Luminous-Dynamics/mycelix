# Craft Decision-Separating Observation Oracle V0.1

A dependency-free Go reference that evaluates deterministic observation partitions over a finite, declared set of possible worlds. It does not assign probabilities.

## Run

```sh
go test -count=1 ./...
go test -race -count=1 ./...
go vet ./...
```

## What it proves within the declared world set

- An observation outcome mapping is total over every listed world before any separation result is produced.
- Every result branch is evaluated independently for robust-safe and invariantly optimal actions.
- An observation is decision-separating within the listed worlds only if every non-empty branch has at least one invariantly optimal robust-safe action.
- Branch-specific actions may differ. This supports adaptive decisions without asserting one action is best across all outcomes.
- The worst branch minimax regret is compared with the pre-observation minimax regret.
- Missing outcome mappings are NotEvaluated, never treated as a favorable or negative observation.
- Shared failure-domain labels are reported explicitly. Distinct outcomes do not prove source independence.
- Acquisition cost, delay and burden are reported separately; no single utility score is invented.

## Frozen controls

- `DSOP-001`: one observation separates two worlds whose optimal actions differ.
- `DSOP-002`: branch-specific hard safety excludes an unsafe action.

Additional tests cover same-outcome non-separation, incomplete mappings, shared failure domains, input permutations and strict JSON parsing.

## Limits

The world set is only as complete as its upstream scope. `declared_set` is conditional and `not_evaluated` cannot make a global claim. Outcomes are deterministic fixture mappings, not calibrated likelihoods. This oracle does not authenticate sources, prove the world set complete, validate observation models, establish legal authority, or execute an acquisition.

It complements rather than replaces sequential EVSI (#6858), robust EVSI (#6862), and EAP (#6860). Those components may use declared probability/model assumptions to value observations; this module checks whether the observation can resolve an action distinction without inventing a probability distribution.
