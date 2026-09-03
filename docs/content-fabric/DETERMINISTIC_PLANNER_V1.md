# Content Fabric Deterministic Planner v1

Status: **CF-06B contract**

## Purpose

CF-06B provides a deterministic reference planner over a CF-06A `PolicyQualifiedPoolV1`.

It is a recommendation engine, not an authority boundary.

The intended flow is:

```text
validated StorageIntentV1
        +
PolicyQualifiedPoolV1
        +
fresh soft evidence
        +
normalization profile
        |
        v
CF-06B deterministic scoring
        |
        v
policy-preserving subset selection
        |
        v
CF-06A validate_selection()
        |
        v
PlacementProposalV1
```

A later Symthaea planner may implement a better proposal strategy, but it must consume the same hard-qualified boundary and its output must pass the same CF-06A selection validator.

## Intent binding

The planner requires the complete validated `StorageIntentV1` in addition to the qualified pool.

It refuses planning unless:

- `StorageIntentV1::validate()` succeeds;
- `pool.storage_intent_id == intent.id`;
- `pool.requirements == intent.requirements`;
- the pool target belongs to the intent object; and
- the pool timestamp/candidates remain internally consistent.

This prevents a caller from swapping different cost/latency/energy/locality preferences onto a hard-qualified pool after qualification. The stable storage-intent ID already commits those preferences.

## Soft evidence

`CandidateSoftEvidenceV1` is bound to the exact canonical availability action, parent advertisement and provider. It may contain:

- cost in microunits/GiB;
- latency in milliseconds;
- energy in millijoules/GiB;
- locality distance in kilometres.

Each evidence record carries `observed_at_unix_ms` and an exclusive `valid_until_unix_ms` freshness bound.

Soft evidence has **no hard-policy authority**. It cannot establish jurisdiction, retention, encryption, replica independence, read authorization or content integrity.

## Conservative missing-data rule

Every metric is converted to an integer penalty in `[0, 1_000_000]` ppm, where lower is better.

For every weighted dimension, any of the following receives the maximum `1_000_000 ppm` penalty:

- missing metric value;
- no evidence record;
- stale evidence;
- future-dated evidence;
- malformed freshness window;
- provider/advertisement identity mismatch;
- conflicting evidence records for the same candidate.

A provider therefore cannot improve its score by suppressing an unfavorable weighted metric.

A zero-weight metric cannot affect the weighted score.

## Explicit normalization profile

`PlannerNormalizationProfileV1` freezes non-zero ceilings for cost, latency, energy and locality and carries a stable `PlannerProfileIdV1`.

Values at or above a ceiling receive the maximum penalty. No floating-point arithmetic is used.

Latency honors `PlacementPreferencesV1::target_latency_ms`: latency at or below the target receives zero penalty; excess latency is normalized against the explicit ceiling.

## Deterministic score

The v1 weighted penalty is the exact integer sum:

```text
cost_weight     * cost_penalty_ppm
+ latency_weight * latency_penalty_ppm
+ energy_weight  * energy_penalty_ppm
+ locality_weight * locality_penalty_ppm
```

No division is required for ranking. Ties are broken by the canonical availability action reference.

## Policy-preserving selection

Naive top-N is forbidden because the best-ranked N candidates may violate failure-domain requirements.

CF-06B v1 uses `PolicyPreservingEliminationV1`:

1. rank the entire already-qualified pool from best to worst;
2. start with every qualified candidate selected;
3. iterate from worst to best;
4. tentatively remove a candidate;
5. keep the removal only if `PolicyQualifiedPoolV1::validate_selection()` still succeeds;
6. validate the final subset again before returning a proposal.

The resulting baseline is deterministic and policy-valid. It is intentionally a reference strategy rather than a claim of globally optimal combinatorial placement.

## Replay commitments

`PlannerInputIdV1` commits:

- storage-intent identity;
- exact placement target;
- evaluation time;
- canonical qualified candidate identities/endpoints;
- accepted jurisdiction/failure-domain facts;
- all relevant raw soft-evidence records after exact-duplicate normalization.

Unknown-candidate telemetry is ignored and does not perturb the target decision.

`PlacementProposalIdV1` additionally commits:

- planner input ID;
- normalization-profile ID;
- target fields;
- planner authority and strategy;
- full best-to-worst ranking;
- total/component penalties;
- evidence states and missing weighted dimensions;
- final selected subset.

Identical qualified state + intent + profile + relevant telemetry therefore produces an identical proposal identity.

## Authority boundary

Every proposal carries:

```text
PlannerAuthorityV1::RecommendationOnly
```

CF-06B cannot:

- create or sign a lease;
- authorize an Iroh read;
- mutate or delete CAS data;
- write Holochain coordination state;
- pay or settle with a provider;
- weaken hard constraints;
- declare content correct;
- mint attestation assurance.

An executor must still validate policy and establish its own execution authorization.

## Symthaea integration

The deterministic planner is the baseline, not the final intelligence layer.

A future Symthaea planner should be evaluated against this reference implementation using the same `PolicyQualifiedPoolV1` and auditable input evidence. Symthaea may improve expected cost, latency, energy, locality or repair behavior, but it receives no additional authority.
