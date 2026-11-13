# FSRE recovery-option interoperability contract v0.1

**Status:** Integration contract proposal, not a production wire protocol.  
**Date:** 2026-11-13  
**Purpose:** Define the minimum owner-qualified result needed to evaluate whether a proposed fallback is genuinely executable for one declared service, scenario and time horizon.

## 1. Scope

The FSRE recovery-option evaluator composes pre-qualified results from existing subsystems. It does not recalculate the source systems' semantics.

A recovery option is qualified only for the declared service, disruption family, scenario revision, observation frontier, mobilisation horizon and evaluation time. Qualification is not a permanent property of an asset, organisation or supplier.

The seven required predicate families are:

1. Physical feasibility — Symtropy.
2. Spatial access — Sol-Atlas.
3. Dependency independence — Mycelix canonical time-qualified dependency/common-cause owners.
4. Staffing — responsible operator.
5. Resources — responsible operator.
6. Authority — Mycelix canonical authority/grant owners.
7. Safety — responsible operator and relevant independent assurance sources.

The evaluator's expected owner labels are a narrow v0.1 integration contract. Real adapters must bind to exact canonical record IDs and schema versions; they must not translate an absent record into a qualified result.

## 2. Paradigm: claims are qualified at the boundary

A candidate is not qualified because:
- an asset exists;
- an alternative graph path exists;
- two identifiers differ;
- a model predicts spare capacity;
- an operator has a general contract with a supplier;
- a signed request or action receipt exists;
- a previous scenario qualified the same option.

For the declared scenario, all seven owner-qualified predicates must be current and `QUALIFIED`. Mobilisation time must fit the declared horizon. Unknown, missing, disputed, out-of-domain, not-independent, expired, revoked, wrong-scope or unavailable results block qualification and remain visible as reason codes.

This is a deterministic conjunction for a declared scenario, not a risk score or probability of success.

## 3. Predicate result envelope

Production adapters should emit an immutable result with at least:

| Field | Meaning |
|---|---|
| `schema_id` / `schema_version` | Exact version of the predicate contract |
| `option_id` | Stable identifier for this candidate recovery option |
| `service_id` / `cohort_id` | Exact service and protected service lane the evidence qualifies |
| `scenario_id` / `scenario_digest` | Exact disruption scenario and immutable revision |
| `service_id` / `cohort_id` | Declared service and protected service lane |
| `scenario_id` / `scenario_digest` | Exact frozen disruption and boundary |
| `predicate` | One of the seven required predicate families |
| `disposition` | `QUALIFIED`, `UNKNOWN`, `NOT_INDEPENDENT`, `UNAVAILABLE`, `EXPIRED`, `REVOKED`, `WRONG_SCOPE`, `OUT_OF_DOMAIN`, or `CONFLICT` |
| `source_owner` | Canonical subsystem / operator that owns the assertion |
| `source_schema_version` | Exact schema/version of the owner's evidence contract; non-empty and validated |
| `source_record_ref` | Exact opaque record identifier in that owner's system |
| `source_schema_version` | Exact source record schema version |
| `evidence_digest` | Digest of the canonical source record/evidence bundle, not a model-generated replacement |
| `time_basis_id` | Canonical time basis to which times were normalised |
| `observed_at` / `valid_until` | Observation time and exclusive end of validity |
| `clock_uncertainty` | Bound on the normalisation/clock uncertainty |
| `claim_scope` | Exact service, jurisdiction, failure-domain, model and validity scope |
| `reason_codes` | Machine-readable reasons, including missing evidence and unresolved dependencies |

The local reference evaluator v0.1 consumes a subset in its PSV fixtures: option ID, service ID, scenario ID/digest, source schema version, predicate, disposition, owner, opaque evidence reference, time-basis identifier, observation time, exclusive validity end and clock-uncertainty bound. It rejects cross-service, cross-scenario and missing-schema reuse. It does not authenticate the referenced source record. Production integration must resolve the owner record and independently verify its digest and schema before treating the disposition as authoritative.

## 4. Time and uncertainty rules

Different federated clocks, time domains and validity conventions must not be compared as though they were exact UTC timestamps.

The v0.1 evaluator requires every source record to be normalised to the declared scenario `time_basis_id`. The canonical temporal owner must attest the conversion and its uncertainty; the evaluator does not create or operate a clock service.

Let:
- (t_e) be the nominal evaluation time;
- (u_e) be the scenario evaluation-time uncertainty;
- (t_o) be the nominal observation time;
- (u_o) be the source record's clock/normalisation uncertainty;
- (t_v) be the exclusive validity end.

The evaluator uses conservative bounds:
- latest possible observation: (t_o + u_o);
- earliest possible evaluation: (t_e - u_e);
- latest possible evaluation: (t_e + u_e);
- earliest possible validity end: (t_v - u_o).

Evidence is rejected when it cannot be proven to predate evaluation ((t_o+u_o ge t_e-u_e)), or when its conservative validity interval is empty. Evidence is stale when (t_v-u_o le t_e+u_e). Overflow, underflow or a time-basis mismatch is an input error, not a qualified result.

The validity end is exclusive. This is intentionally conservative. A future extension may support richer interval-valued temporal assertions, but must reuse Mycelix #2631 and the existing clock-domain owners rather than add another time service.

## 5. Evaluation semantics

For each candidate:
1. Validate the scenario contract, time basis and horizon.
2. Require one distinct predicate record for each of the seven predicate families.
3. Reject duplicated predicate records and records for unknown option IDs.
4. Verify exact expected source owner.
5. Verify the source reference is non-empty and the time interval is valid/current.
6. Preserve each non-qualified disposition as a reason code.
7. Reject mobilisation time beyond the declared scenario horizon.
8. Qualify only when the reason set is empty.

Missing rows yield explicit `MISSING_PREDICATE:<name>` reasons. They are not parser defaults.

A qualified result should include the exact input/owner record references and digests, the scenario and evaluator versions, and the full reason set. It does not grant permission to dispatch, spend, contract, operate equipment or perform work.

## 6. Revalidation and authority separation

Qualification is time-bounded and scenario-scoped. Before an operator performs a consequential action, the action path must independently revalidate:
- current authority, purpose, jurisdiction and revocation state;
- current source-evidence validity;
- physical/model operating constraints and safety envelope;
- resource and workforce availability;
- scenario changes and mobilisation feasibility.

A previously qualified recovery option can become unqualified when a grant expires, a route becomes inaccessible, a model leaves its validity domain, a supplier becomes unavailable or new common-cause evidence arrives.

The evaluator returns a qualification statement only. It is not an authorisation token and must never be accepted by a physical actuator as the sole basis for action.

## 7. Federation, privacy and minimum disclosure

An organisation may disclose only the predicate disposition and minimum evidence reference/digest required for a declared purpose. Sensitive OT asset details, staff personal data, precise security-sensitive routes and credentials remain with their source owners. The source owner must retain the evidence required for independent audit and lawful review.

Federation should support:
- signed, versioned, purpose-bound result envelopes;
- expiry and revocation;
- local operation during network partitions;
- explicit unresolved/conflicting state during reconciliation;
- idempotent receipt identity;
- export and exit without central service dependency;
- independent re-evaluation from owner-native evidence.

No last-write-wins rule may silently resolve a conflict in authority, safety, dependency independence or currentness.

## 8. Benchmark and adversarial requirements

The integration benchmark must include:
- wrong-owner and schema-version substitution;
- absent or duplicated predicate rows;
- time-basis mismatch;
- clock uncertainty that makes evidence freshness ambiguous;
- stale, future-dated and overlapping validity intervals;
- expired/revoked/wrong-purpose authority;
- shared failure-domain alternatives;
- nominal alternatives with no staff, spares or route access;
- mobilising after the scenario horizon;
- partition, stale cache and late revocation;
- changed scenario/source digests after qualification.

Hard gate: no candidate may be reported qualified when any required predicate is missing, non-qualified, stale, temporally ambiguous, out-of-scope or sourced from the wrong owner.

## 9. Canonical implementation owners

- Mycelix #5923/#5981 and temporal assertion/clock owners: dependency independence, common-cause semantics, evidence validity, clock domain and authority.
- Sol-Atlas #21 and provenance/geometry owners: spatial reachability/access and alternative-path disposition.
- Symtropy #4392/#3787: physical capacity, operating envelope and model adequacy.
- Symthaea #10366: causal investigation, competing hypotheses, discriminating observations and experiment-selection receipts.
- Operator: staffing, resources, safety and final operational control.
- Sol-Atlas #338: canonical preregistered run manifest and frozen benchmark identities.

No second dependency graph, spatial graph, physical simulator, temporal ledger, authority service or experiment scheduler is introduced.

## 10. Claim ceiling

This v0.1 contract and synthetic evaluator demonstrate only deterministic composition semantics. They do not authenticate live upstream records, establish real asset independence, validate physical capacity, prove water quality, certify safety, authorise operations or demonstrate societal improvement.
