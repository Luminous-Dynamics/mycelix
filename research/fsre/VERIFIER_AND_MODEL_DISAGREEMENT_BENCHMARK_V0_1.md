# FSRE verifier independence and model-disagreement benchmark v0.1

**Date:** 2026-11-13  
**Status:** Research benchmark protocol; no production verifier accuracy or multi-model advantage is claimed.

## 1. Why this benchmark exists

FSRE produces qualification claims: a service trajectory is calculated from stated observations; a recovery option is or is not qualified for a scenario; a portfolio is compared against declared constraints; and an outcome can be independently reproduced.

The verifier itself can be wrong. It can accept stale, wrong-scope or incomplete owner evidence; mishandle clock uncertainty; count duplicate records; omit provenance; or report a qualified option when a source model is outside its domain. A second component that merely reuses the same parser, helper functions, assumptions or source records may reproduce the same mistake and is not independent in the relevant sense.

Likewise, several domain models agreeing does not establish independent corroboration if they share a solver, calibration set, weather data, asset inventory, assumptions, vendor or source.

The proposed contribution is to make the verifier and the evidence path themselves benchmark subjects.

## 2. Non-negotiable ownership boundaries

- Mycelix owns canonical provenance, temporal assertions, dependencies, authority and completed-effect evidence.
- Sol-Atlas #338 owns the canonical benchmark manifest and execution receipt; spatial owners own geospatial evidence and alternatives.
- Symtropy owns physical model execution, adequacy, solver semantics and model-domain receipts.
- Symthaea owns causal hypotheses, discriminating experiments and model/decision uncertainty reasoning.
- The operator owns workforce, resources, safety and operational control.
- FSRE owns the service-level challenge protocol, composition and independent report profile.

The verifier benchmark must not add another evidence ledger, trust root, clock service, dependency graph, spatial model, physical solver, causal scheduler, identity/authority service or benchmark-manifest format.

## 3. Benchmark layers

### V0 — Deterministic evaluator conformance

For a frozen manifest and reference inputs, compare:
- the production/reference evaluator;
- a separately implemented reference evaluator with independently authored code;
- a human-reviewed calculation for small hand-verifiable cases.

The reference implementation must not call production helpers or import production parsing/aggregation code. It can use a different language or a minimal independently authored implementation. Any disagreement blocks a pass and is retained as a defect, not resolved by choosing the preferred output.

### V1 — Metamorphic and invariant tests

Test properties that should remain true under transformations that do not alter the semantic claim:
- Reorder debt/evidence rows where row order is non-semantic: report semantics remain identical.
- Reorder independent option candidates: sorted output and per-option dispositions remain identical.
- Change a display label without changing a stable ID or source digest: identity binding does not change.
- Duplicate an option display name while retaining distinct IDs: options are not merged or falsely made independent.
- Duplicate an identical predicate record: reject the ambiguous duplicate rather than silently choosing first or last.
- Add an unknown schema field: reject or explicitly ignore only under a declared extension policy; never silently reinterpret it.
- Change line endings or harmless whitespace only where canonical serialisation explicitly defines normalisation; otherwise a digest mismatch is expected.
- Permute independent services/cohorts: no cross-service contamination.
- Add an unrelated debt item: other option dispositions remain stable, while the debt report changes only by the expected item/group delta.

### V2 — Negative-control mutation suite

Start from a synthetic positive candidate with all required predicates qualified, then mutate one property at a time and in selected combinations.

| Family | Mutation | Required result |
|---|---|---|
| Missing evidence | Remove one of seven predicates | NOT_QUALIFIED with exact missing predicate |
| Wrong owner | Supply an owner from another subsystem | Reject input; never relabel it |
| Wrong schema | Supply a schema namespace not owned by the predicate owner | Reject input |
| Wrong service | Bind predicate to another service/cohort | Reject input |
| Wrong scenario | Change scenario ID or digest | Reject input |
| Malformed digest | Non-SHA256, wrong length or uppercase/non-hex encoding when canonical form is lowercase | Reject input |
| Time-basis mismatch | Supply evidence from an unnormalised clock domain | Reject input |
| Clock uncertainty | Make observation/evaluation intervals overlap | Reject or mark temporally indeterminate; never accept as current |
| Validity boundary | Set exclusive validity end equal to latest possible evaluation | Mark stale |
| Future evidence | Move observation interval beyond evaluation | Reject input |
| Revoked authority | Change authority disposition to REVOKED | NOT_QUALIFIED |
| Wrong purpose | Change authority to WRONG_SCOPE | NOT_QUALIFIED |
| Expired authority | Change authority to EXPIRED | NOT_QUALIFIED |
| Hidden common cause | Change canonical dependency disposition to NOT_INDEPENDENT | NOT_QUALIFIED |
| Unknown independence | Remove/mark unknown the common-cause result | NOT_QUALIFIED; never infer independence |
| Physical model limit | Change Symtropy result to OUT_OF_DOMAIN | NOT_QUALIFIED |
| Spatial staleness | Make Sol-Atlas access evidence stale | NOT_QUALIFIED |
| Workforce gap | Make staffing unavailable | NOT_QUALIFIED |
| Resource shortage | Make critical spares/resources unavailable | NOT_QUALIFIED |
| Safety uncertainty | Make safety unknown or conflicting | NOT_QUALIFIED |
| Mobilisation beyond horizon | Increase time beyond declared scenario window | NOT_QUALIFIED |
| Duplicate identity | Clone an option under a second display name but share the same failed dependency evidence | No extra independent qualified option |
| Evidence substitution | Change source evidence reference/digest while preserving the old disposition | Reject until owner record is re-resolved and reverified |
| Report omission | Remove debt/predicate evidence lines from a report | Golden/output schema test fails |
| Authority inflation | Feed a qualified report to an actuation/approval adapter | Adapter rejects; report explicitly has no execution authority |

Run both single-mutation and selected compound-mutation cases. Preserve the raw mutation, expected disposition/reason and actual result.

### V3 — Verifier fault injection

Inject defects into the verifier implementation in a controlled test build:
- reverse stale comparison;
- ignore one predicate;
- skip service or scenario binding;
- omit one evidence record from output;
- sort inputs destructively or pick the first duplicate;
- treat unknown as qualified;
- drop uncertainty bounds;
- collapse conflicting records into last-write-wins;
- fail to include the non-authorisation statement.

Each injected defect should be detected by at least one independent test or golden differential. Mutation score is reported by fault family; it is not interpreted as general software correctness.

### V4 — Multi-model disagreement

Use at least two qualified model configurations or independent model families for the same frozen scenario where practical. Compare:
- service trajectory (level, deficit, duration, quality exposure);
- capacity and physical feasibility;
- recovery mobilisation and resource requirements;
- option feasibility/dispositions;
- portfolio ranking and regret;
- uncertainty range and out-of-domain boundaries.

Record shared dependencies between models: source inventory, demand model, weather forcing, calibration data, solver library, parameter source, vendor or model lineage. Two wrappers around the same solver are not independent models.

Do not average incompatible outputs. Report:
- pairwise model disagreement by metric;
- common data/assumption lineage;
- leave-one-model-out option/portfolio rank sensitivity;
- interval overlap only when quantities and uncertainty definitions are comparable;
- reasoned abstention where models materially disagree and no independent observation resolves the dispute.

Symthaea can propose a discriminating inspection or measurement based on the decision affected by disagreement. Model disagreement itself is not causal evidence.

### V5 — Independent operational reproduction

A second organisation receives a frozen package containing the declared source-owner references/digests, schema versions, scenario manifest, evaluator version, metric code, uncertainty and claim ceiling. It should reproduce the named result without relying on an FSRE-hosted service or privileged hidden state.

Measure:
- reproducibility rate;
- unresolved/missing evidence rate;
- analyst time and required expertise;
- compute and integration cost;
- data permissions and time to access owner records;
- differences in result and reason codes;
- ability to export, re-host and independently continue after the original provider is unavailable.

The independent evaluator should be separate from intervention proposal, execution and primary outcome measurement where practical. Separation of organisations alone is not enough if they share source, vendor, incentives or unchecked evidence.

## 4. Metrics

Report separate measures, not a single verifier score:

1. **Exact differential agreement:** exact agreement with independent reference outputs on deterministic fixtures.
2. **False-qualified rate:** proportion of invalid/unknown options incorrectly reported qualified.
3. **False-independent rate:** recovery options incorrectly called independent under hidden-common-cause and identifier-inflation controls.
4. **False-safe rate:** unknown or quality-violating service intervals incorrectly classified safe.
5. **Reason-code precision/recall:** whether the system reports the correct cause of non-qualification.
6. **Temporal-boundary accuracy:** stale/current/overlap classification under clock uncertainty.
7. **Evidence-lineage completeness:** required owner references, schema versions, scenario digest and time bounds present in the report.
8. **Metamorphic consistency:** output invariance under non-semantic input permutations and transformations.
9. **Mutation-detection score:** proportion of seeded verifier defects detected, by family.
10. **Abstention precision and coverage:** whether the system abstains when a claim is not identifiable and remains useful on adequately evidenced cases.
11. **Model disagreement sensitivity:** rate at which disagreement changes an option/portfolio decision and rate of justified information acquisition.
12. **Independent reproduction cost:** time, staff effort, integration burden, permissions and compute needed by a second organisation.

## 5. Provisional qualification targets

These are proposed research targets, not achieved results or universal industry standards:

- 100% exact differential agreement on frozen deterministic reference fixtures.
- 0 false-qualified outcomes in the declared frozen negative-control suite.
- 0 false-independent outcomes in hidden-common-cause and identity-inflation controls.
- 0 false-safe classifications in quality-failure and unknown-observation controls.
- 100% required evidence-lineage completeness for qualified outputs.
- 100% of authority, safety, time-basis and scenario mismatch controls rejected or downgraded as specified.
- All declared verifier mutations detected by at least one independent control.
- Second-organisation reproduction of 100% of the published deterministic reference package; unresolved differences are published, not hidden.

The size and diversity of the challenge suite must be reported. Zero observed failures on a small suite does not imply a low field failure probability or zero operational risk. A production reliability claim requires an appropriately powered independent evaluation.

## 6. Benchmark task-set design

Start with a versioned public synthetic set of at least 50 deterministic cases:
- 10 positive cases across different service/scenario scopes;
- 10 incomplete/unknown evidence cases;
- 10 wrong-owner/schema/scope/digest cases;
- 10 temporal freshness/clock-uncertainty cases;
- 5 hidden-common-cause/false-redundancy cases;
- 5 authority/resource/safety/mobilisation compound cases.

Then add a sealed, independently held challenge set with varied task families and hidden combinations. Do not publish sealed answer keys or root-cause labels in the agent-visible repository. Use Sol-Atlas #338 for the canonical pre-registered manifest and Mycelix's owner-native evidence/receipt systems for source identity.

## 7. Comparison with the infrastructure interdependency literature

Shamsi, Helmrich and Deegan's 7 November 2026 comparative review identified seven explicitly cross-sector interdependency tools: IN-CORE, InfraRisk, REFIT, CIrcle, PRIIA, Kaleidoscope and CISIA-Pro. Its comparison covers interdependency breadth, data requirements, validation strength, integration potential, purpose fit, methods, context, readiness and sector coverage; it notes the absence of comparable runtime/hardware benchmark data across the tools.

FSRE can contribute a common service-level verifier benchmark on top of tool-specific outputs. It should not force participatory CIrcle, spatial Kaleidoscope, probabilistic REFIT, hypergraph PRIIA and integrated IN-CORE into one false task or claim that service-outcome reporting replaces each tool's native strengths.

Reference: https://doi.org/10.1007/s10669-026-10162-2

## 8. Implementation boundary and falsification

The first executable suite should extend existing Rust reference tools only where their semantics are stable. An independent second evaluator can be separately authored in a different codebase/language if that reduces common-mode implementation risk. Do not reuse production parser/evaluator internals in the reference.

This research direction should be narrowed or rejected if:
- differential agreement cannot be achieved on deterministic fixtures;
- the independent verifier shares the same unexamined evidence or implementation assumptions;
- challenge coverage cannot expose realistic failure modes;
- independent reproduction cost exceeds the value of assurance;
- the benchmark becomes an administrative burden that does not reduce false confidence or service loss;
- incumbent products already supply equivalent independently reproducible service qualification at lower lifecycle cost.

The purpose is to demonstrate when a service and recovery claim is warranted—and to make it easy to discover when it is not.
