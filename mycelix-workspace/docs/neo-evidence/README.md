# NEO source-correlation evidence profile v0.1

**Status:** evidence/provenance contract and synthetic control only. It does not estimate physical correlations or establish observation independence.

## Why this matters

An astrometric observation is not an independent evidence unit merely because it has a different observation ID, instrument label, observatory, provider or sensor modality. Measurements can share star catalogues, reduction software, timing services, calibration campaigns, environmental conditions, facilities or telemetry paths.

If a pair of observations shares a common calibration bias, multiplying their marginal likelihoods as though they were independent can create overconfident posteriors and overstate the value of another observation. Conversely, assuming a fixed numeric correlation coefficient without evidence is also unjustified.

The profile records dependence as provenance and model constraints. It deliberately does **not** invent a scalar correlation coefficient.

## Files

- `source-correlation-profile-v0.1.schema.json` — Draft 2020-12 JSON Schema.
- `fixtures/shared-calibration-unknown-cross-source.v0.1.json` — synthetic optical-source pair sharing a catalogue lineage plus an optical/radar pair whose dependence is explicitly unknown.

## Dependence assessments

| Assessment | Meaning | Required handling |
|---|---|---|
| `dependent_documented` | A shared catalogue, calibration, clock, processing or other common-cause domain is documented | Preserve the common-cause / latent dependency identity in the joint likelihood |
| `dependent_estimated` | A versioned model estimates dependence with a declared validity scope | Reference the model, preserve its uncertainty and avoid collapsing it to an unexplained coefficient |
| `independent_evidenced` | Evidence supports independence for this precise scope | Record the basis and evidence references; independence is not inferred from provider count |
| `unknown` | Available evidence does not establish dependence or independence | Do not assume independence; abstain or explicitly model dependence uncertainty |

**Unknown does not mean dependent.** It means that an independent likelihood product is not justified yet. A planner may abstain, request lineage evidence or evaluate a conservative model set.

## Invariants

1. Every source has a stable source ID and declared failure-domain IDs.
2. Every assertion names at least two source IDs and a validity interval.
3. A documented dependency names its common-cause ID and dependence kind.
4. An estimated dependency references a versioned dependence model; no free-floating correlation number is accepted.
5. An independence assertion requires an explicit basis and at least one evidence reference.
6. An unknown assertion must set `independence_assumption: not_assumed`.
7. Production-evidence profiles require a digest for the observation set and source records, plus evidence references for dependency assertions.
8. The profile stores evidence links, digests and metadata—not raw telescope/radar telemetry.
9. Corrections, supersessions and retractions must append new evidence events; they must not rewrite the earlier lineage.
10. Source count, provider diversity and social consensus are never substitutes for demonstrated statistical independence.

## Composition

- **Symthaea #11112 / PR #11165:** observation candidate ranking and EVSI. Its finite fixture demonstrates why common-source calibration must be retained.
- **Symtropy #4217 / #4199:** validated orbit/posterior and decision-loss computation. Mycelix does not calculate the orbit or official impact probability.
- **Mycelix #5952:** durable source identities, observation digests, correction history, calibration lineage and dependency assertions.
- **Sol-Atlas #640:** show source groups and uncertainty provenance; visualization is not an authority for physical independence.

## Claim ceiling

The included fixture is synthetic and is only a schema/example control. It does not describe real observatories, actual catalogue lineage, a numeric correlation, astrometry, orbit determination or impact risk. A profile records the provenance basis for a dependency claim; a separate physics/statistical validation is required before using any likelihood or covariance model operationally.
