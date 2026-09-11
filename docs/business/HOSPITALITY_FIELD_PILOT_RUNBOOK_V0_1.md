# Hospitality Field Pilot Runbook v0.1

Status: **read-only shadow qualification protocol**

This runbook turns the Mycelix Business qualification stack into an operational field-pilot procedure for a real food-service location without granting autonomous business write authority.

It is brand-, POS-vendor-, country-, and currency-neutral. A specific restaurant supplies only its scope, export mapping, model lineage, time rules, and independently issued control/authenticity evidence.

## 1. Pilot claim boundary

The pilot may establish evidence about:

- deterministic ingestion of a declared export format;
- data quality under a preregistered policy;
- candidate demand-forecast quality versus a preregistered baseline;
- performance on preregistered daypart/weekend slices;
- reconciliation of canonical replay against separately issued control totals; and
- externally verified authenticity of those control documents.

The pilot does **not** establish:

- causal profit, savings, waste reduction, labour improvement, or customer impact;
- physical-world completeness of the provider's records;
- semantic/legal correctness of the provider's metric;
- external legal authority of the control issuer;
- institutional authority for Symthaea to act; or
- permission for purchase, scheduling, refund, payment, personnel, or other business writes.

All Business components used by this protocol remain read-only.

## 2. Freeze the evidence lineage before collection

At least 24 hours before the evaluation interval, freeze and retain the exact digests/identities for:

1. Hospitality pilot registration and shadow protocol.
2. Candidate model lineage.
3. Seasonal-naive baseline lineage.
4. Delimited-ingress adapter semantic ID and adapter build digest.
5. Exact ordered source headers / source-schema digest.
6. Mapping digest and declared normalized input.
7. Business/location scope.
8. Transition-aware local-time schedule and schedule digest.
9. Exact forecast target plan.
10. Transaction actual-projection specification (`Sum` for v0.1).
11. Control-coverage plan, including every exact control window.
12. Control-authenticity coverage plan: verifier domain, verification method, and verification-policy digest.
13. Registered data-quality thresholds and slice predicates.

A change to any frozen semantic input starts a new evidence lineage. Do not reinterpret previously collected evidence under the new version.

### Conservative v0.1 policy defaults

The current hospitality pilot preset uses:

- minimum preregistration lead: **24 hours**;
- minimum evaluation duration: **28 days**;
- minimum overall forecast cases: **56**;
- minimum cases per required slice: **10**;
- maximum candidate abstention: **10%**;
- maximum missing-data rate: **1%**;
- maximum conflicting-record rate: **0%**;
- maximum stale-record rate: **2%**; and
- maximum observed ingest delay: **36 hours**.

These are versioned pilot defaults, not universal hospitality rules. If changed, the replacement policy must be preregistered before evidence collection.

## 3. Required slices

The standard v0.1 hospitality profile preregisters:

- breakfast: local 05:00–11:00;
- lunch: local 11:00–15:00;
- evening: local 15:00–23:00; and
- weekend.

Slice membership is derived from the preregistered transition-aware local-time schedule. Do not assign cases to slices by hand after outcomes are known.

A forecast window crossing an offset transition is not silently forced into a daypart. It must follow the time-evidence fail-closed rule.

## 4. Daily evidence collection

For each collection/control window:

### 4.1 Preserve the source export

Store the exact original export bytes without editing rows, headers, timestamps, decimal representation, sorting, or identifiers.

Record the ingestion timestamp separately. The diagnostic/replay path derives the source-file digest, row counts, rejection classes, schema/mapping identity, temporal coverage, and ingest delay.

Do not remove malformed, duplicate, stale, conflicting, or inconvenient rows before qualification. Rejected rows remain visible in denominator/data-quality evidence.

### 4.2 Preserve the separately issued control report

After the exact control window closes, retain the provider/auditor control document used for reconciliation. The control statement must bind:

- its source-document digest;
- exact control contract/window;
- expected unique source-event count; and
- expected exact fixed-point total for the preregistered metric.

A control report issued before its window closes is invalid for this theorem.

### 4.3 Preserve external authenticity verification

For each control document, retain a verification receipt from the preregistered external verifier domain. The Business layer records the receipt reference but does not verify cryptography itself.

The receipt must bind the exact source-document digest and claimed issuer, plus:

- verifier receipt digest;
- verification method and policy;
- verifier epoch and receipt sequence;
- credential identity and credential epoch;
- revocation frontier; and
- validity interval.

Before the authenticity coverage theorem may narrow any limitation, each receipt must still pass revalidation against the current verifier epoch, credential epoch, and revocation frontier.

A changed frontier requires re-verification. It is not silently treated as proof of revocation or proof of continued validity.

## 5. Forecast collection rules

The exact forecast target set is preregistered before evaluation.

For every target, retain one candidate forecast and one baseline forecast with their exact model lineages and issuance times. The evaluator—not the model or operator—constructs the target actual from canonical transaction replay over the preregistered half-open target interval `[start, end)`.

The candidate cannot:

- omit a difficult target;
- replace a planned target;
- supply its own actual;
- choose which transactions count toward the actual;
- supply slice pass/fail verdicts; or
- hide an abstention by dropping the case.

Candidate abstention remains in the denominator and receives the baseline error under the current shadow protocol.

## 6. Daily stop / quarantine conditions

Do not silently continue one qualification lineage if any of the following occurs:

- source headers/schema change;
- adapter or mapping semantics change;
- candidate or baseline model lineage changes;
- local-time rule/schedule changes outside the preregistered schedule;
- an accepted source-event ID is reused across campaign files;
- a source timestamp lies in the future under the registered evidence rules;
- the campaign contains an unexplained conflicting-record event where the plan allows zero conflict;
- a control count or value fails reconciliation;
- a control window is missing, duplicated, overlapping, or leaves a coverage gap;
- an authenticity receipt targets the wrong source document or issuer;
- verifier policy/domain/method drifts;
- verifier or credential epoch changes without fresh verification;
- revocation frontier changes without fresh verification; or
- evidence artifacts cannot reproduce their registered digests.

Quarantine the affected evidence and investigate. If a frozen semantic input actually changed, start a new lineage instead of editing historical evidence.

## 7. Qualification sequence

The evaluation order is strict:

**Q0 — Import reproducibility**

Exact file → deterministic diagnostics → canonical replay. No model claim yet.

**Q1 — Exact evaluation denominator**

Every preregistered forecast target appears exactly once. Transaction-derived actuals are constructed internally, including evidence-backed zero-transaction windows.

**Q2 — Shadow model qualification**

Candidate is evaluated against the registered baseline, abstention ceiling, overall case floor, and required slices.

**Q3 — Field-data qualification**

Registered missing/conflict/stale/ingest-delay gates pass. This still does not prove upstream provider completeness.

**Q4 — Control reconciliation coverage**

Every preregistered control window reconciles count and fixed-point value, with no gaps/overlaps across the full qualification interval.

This may narrow:

- `upstream-export-completeness-unverified` → `control-source-external-reality-unverified`; and
- `aggregation-semantic-authority-unverified` → `control-metric-semantic-authority-unverified`.

It does not prove physical reality or source authenticity.

**Q5 — Control authenticity coverage**

Every admitted control document has a live externally verified authenticity receipt under the preregistered verifier policy.

This may narrow:

- `control-source-authenticity-unverified` → `control-source-issuer-authority-unverified`.

It still does not establish that the authenticated issuer possessed institutional/legal authority to define the economic fact.

## 8. Evidence bundle at pilot close

Retain the complete evidence bundle, including:

- immutable pilot registration/protocol;
- candidate and baseline model lineage IDs;
- source adapter/schema/mapping digests;
- transition-aware time schedule;
- exact target plan and projection spec;
- original source files and import-diagnostic manifests;
- canonical campaign replay evidence;
- forecast submissions, derived actuals, overall scorecard, and slice reports;
- field data-quality evidence;
- immutable transaction hospitality report;
- every control contract and source control document;
- every per-window reconciliation and full control-coverage evidence;
- every external authenticity receipt/revalidation context;
- full authenticity-coverage evidence;
- controlled and authenticated additive qualification envelopes; and
- all unresolved limitations.

Never replace an older report with a stronger one. New evidence references and narrows earlier limitations while preserving the earlier artifact verbatim.

## 9. Human review fields

For operational usefulness, collect a separate human review stream that is **not** used to manufacture ground truth:

- recommendation understood? yes/no;
- recommendation operationally useful? yes/no;
- manager would have acted on it? yes/no;
- important context missing? free text/category;
- unusual event / promotion / outage / staffing issue? category;
- suspected data error? category + evidence reference.

Human review may motivate future hypotheses or model changes. It must not retroactively change the preregistered evaluation target, actual, slice, or baseline.

## 10. Promotion ceiling

This protocol qualifies read-only observation/forecasting evidence only. A successful field pilot does not automatically promote any capability into business execution.

Any later `recommend → draft → delegated write` progression must pass the separate Business authority, Action Contract, Decision Capsule, coordination/reservation, material-revalidation, and execution-reconciliation stack. New model versions begin at their own qualified autonomy level rather than inheriting prior operational authority by version number.

## 11. Pilot success language

Permitted wording should stay close to the evidence actually established, for example:

> Under preregistered protocol X, model lineage Y met the registered shadow forecast gate against baseline Z across the full target set and required hospitality slices. The evaluated transaction export reconciled to the separately issued control statements across every preregistered control window. Those documents were externally authenticated under verification policy V. Remaining limitations are recorded in the evidence envelope.

Avoid wording such as “the AI saved 15%,” “the POS data is complete truth,” “fully audited,” or “safe for autonomous purchasing” unless a later independent theorem actually establishes those claims.
