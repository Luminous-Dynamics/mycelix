# IPB benchmark adapter conformance contract

**Status:** Draft v0.1 for issue #6117  
**Scope:** Read-only benchmark observation projection, initially for IPB-S2  
**Non-goal:** A universal governance event model or authoritative Mycelix data store

## 1. Purpose

An adapter translates a platform's native records into a small, non-authoritative projection that allows an independent evaluator to compare the same frozen institutional task across implementations.

The adapter is not allowed to make decisions, grant authority, mutate the source platform, or replace its record system. It exposes what the platform actually recorded and marks unsupported or unknown properties explicitly.

An adapter must never promote:
- a discussion or poll result into legal/institutional authority;
- a credential signature into proof that its claims are true;
- a workflow status into evidence that an external action occurred;
- an execution acknowledgement into an independently observed outcome;
- absent telemetry into a safe/pass result.

## 2. Frozen S2 task

The initial task is defined in [IPB_S2_MATCHED_SCENARIO.yaml](IPB_S2_MATCHED_SCENARIO.yaml). It is the 20-case community-fund expenditure authorization fixture implemented by the S2 research crate.

The validated source fixture is commit `c9a366095ad204f2561ac2c76ff7de8a4dd5b73c`, CI run #13845. This is fixture-level validation only, not production zome equivalence.

## 3. Adapter output projection

The benchmark projection should contain:

| Field | Requirement | Meaning |
|---|---|---|
| `schema` | Required | Adapter projection version |
| `scenario_id` / `scenario_version` | Required | Frozen benchmark input identity |
| `case_id` | Required | Exact case in the scenario manifest |
| `system_id` / `system_version` | Required | Product/workflow and version under test |
| `configuration_digest` | Required | Digest of relevant configuration and policy setup |
| `observed_decision` | Required where in scope | `permit`, `deny`, `not_executed`, or `unknown` |
| `evidence_disposition` | Required | One of the typed states below |
| `reason_codes` | Required where available | Native reason or adapter-mapped reason with mapping version |
| `native_references` | Required for claimed support | Stable references into source records |
| `event_time` | Required when source provides it | When the source says the action/event occurred |
| `observed_at` | Required when source provides it | When the evaluator/adapter learned the fact |
| `metric_values` | Required | Named measures, units, denominators and missingness |
| `limitations` | Required | Explicit unsupported, incomplete or disputed properties |

### Typed evidence disposition

- `supported`: source records directly support the benchmark claim.
- `contradicted`: source records directly conflict with the claim.
- `contested`: material disagreement is open and recorded.
- `indeterminate`: evidence is insufficient to resolve the claim.
- `unavailable`: required evidence or access is not available.
- `out_of_scope`: platform is not being evaluated for a function outside its intended role.
- `invalid_trace`: a required record chain is malformed, broken or inconsistent.

None of these values may be silently coerced into `supported`.

### Native reference

Each reference should contain:
- source system identifier;
- artifact kind (proposal, poll, grant, decision, workflow event, payment evidence, appeal, etc.);
- stable native record identifier or resolvable URI;
- source content digest where available;
- event time and observation time, if provided by the source;
- mapping version used by the adapter.

Do not copy entire native records into a second authoritative store. The benchmark can retain minimal redacted snapshots only when necessary, with consent, access controls and source licensing permitting it.

### Pseudonymous participation

Participant references must be stable within a scenario run but must not expose names, email addresses, account IDs or other personal identifiers in the public result. A separate authorized evaluator may retain a protected re-identification key where legally and ethically approved. Public benchmark results should publish only pseudonymous actor references and group-level metrics.

## 4. Example projection (illustrative)

```json
{
  "schema": "lise.ipb.adapter-projection.v0",
  "scenario_id": "community-fund-expenditure-authorization",
  "scenario_version": 1,
  "case_id": "s2-negative-screened-conflict",
  "system_id": "example-workflow",
  "system_version": "1.2.0",
  "configuration_digest": "sha256:<computed-by-adapter>",
  "observed_decision": "deny",
  "evidence_disposition": "supported",
  "reason_codes": [
    {
      "native_code": "conflict_blocked",
      "mapped_code": "conflict_of_interest",
      "mapping_version": "0.1"
    }
  ],
  "native_references": [
    {
      "system_id": "example-workflow",
      "artifact_kind": "approval-attempt",
      "native_id": "opaque-record-reference",
      "content_digest": "sha256:<source-digest>",
      "event_time": "fixture-time-or-source-time",
      "observed_at": "evaluator-observation-time"
    }
  ],
  "metric_values": [
    {
      "name": "authorization_latency_ms",
      "value": 120,
      "unit": "ms",
      "denominator": 1
    }
  ],
  "limitations": []
}
```

The example is illustrative; placeholder digests/timestamps are not evidence.

## 5. Fair platform scope

| Baseline | In-scope contribution for S2 | Not to be inferred |
|---|---|---|
| Manual process | Documented role/grant check, conflict declaration, approval/denial and retained records | Completeness or independence not evidenced by a process document alone |
| Conventional workflow/access-control tooling | Scope, identity, approval and audit workflow where configured | A workflow status alone is proof of external execution |
| Mycelix S2 fixture | Deterministic policy decision under the exact 20-case input contract | Production Holochain enforcement or physical payment |
| Decidim | Participation process, proposal and decision context where configured | That every vote or proposal is a legally authorized expenditure |
| Loomio | Discussion, proposal, poll and member decision record where configured | That a poll outcome is the required delegated spending authority |
| Polis | Opinion discovery and deliberative input | That opinion-group consensus constitutes institutional authorization |
| Formal voting power analysis | Voting pivotality under the exact declared game, if the scenario actually involves a vote | Non-voting delegation, treasury custody, informal influence or oversight independence |

For the S2 authorization subtest, a platform with no authorization primitive should emit `out_of_scope` for that claim rather than a fabricated decision or a failure. It can still be evaluated on relevant upstream participation functions in a separate experiment.

## 6. Independent scoring and reporting

The evaluator (not the adapter) joins the observed projection to the frozen expectation and reports:
- true permits, false permits, true denials and false denials;
- unsupported/out-of-scope cases separately from errors;
- missing/invalid traces and incomplete authority references;
- latency and participant burden with units and denominators;
- exact system, adapter, scenario, configuration and evaluator versions.

No single composite governance score. Publish per-case outcomes, confusion matrices, missingness, subgroup distributions, uncertainty intervals and deviations from preregistration.

## 7. Reproducibility and independence

- The same manifest and case IDs must be used across arms.
- The adapter records exact software and configuration versions.
- The reference evaluator is independent of the implementation under test.
- Each output is bound to the manifest digest, adapter version and evaluator version.
- Held-out scenarios must not be used to tune mappings.
- Two independent evaluators review the first public comparison.
- At least three independent adapters/evaluators are required before general cross-platform interoperability claims.
- Human pilots require informed consent, minimization of personal data and pre-registered outcome measures.

## 8. Existing owner boundaries

- Mycelix #2953/#3117/#6069/#3126/#4835/#6113 own lifecycle, authority, trace, uncertainty, independent evaluation and integration.
- Mycelix #5762/#5695/#1243/#1371 own dependency and common-cause closure.
- Symtropy #3684 owns formal power indices and related game structures.
- Symthaea #3054/#3095/#1872 own strategic and adversarial agent experiments.
- LISE/#6117 owns the benchmark scenario, adapter conformance and matched experimental design.

The projection is strictly an evaluation view; the source platform and its owner-native evidence model remain authoritative.
