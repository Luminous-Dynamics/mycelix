# MCRB Per-Operation Outcome Attestation v0.1

**Status:** Normative candidate contract for implementation; not yet implemented or qualified  
**Tracking:** [#6357 — MCRB-OUTCOME-ATTEST-001](https://github.com/Luminous-Dynamics/mycelix/issues/6357)  
**Parent:** [#6322 — Platform Stewardship and MCRB](https://github.com/Luminous-Dynamics/mycelix/issues/6322)

## 1. The missing benchmark proposition

The MCRB offer ledger fixes what the workload intended to attempt. Metric integrity verifies how the measurement was timed. Correctness-gate attestations record whether scenario invariants passed. None of those proves that an individual offered operation actually reached its reported terminal outcome.

The current diagnostic API still accepts a caller-supplied `Vec<OperationOutcomeSample>`. A caller can therefore label every offered ID `QualifiedSuccess` without supplying evidence that the operation succeeded. The reducer currently withholds QOPS rates while this remains true. That fail-closed behavior is correct and must remain until the outcome verifier is integrated.

The next capability is an opaque `VerifiedOutcomeLedger`, derived from signed, operation-bound evidence and the frozen offer ledger.

## 2. Paradigm shift: distinguish the operation from the assertion about it

Do not model an operation as one mutable status field. Preserve four separate evidence dimensions:

| Dimension | Example states | What it answers |
|---|---|---|
| Attempt state | not attempted, dispatched, acknowledged, timed out, reconciled | What happened to the request path? |
| Effect assertion | applied, confirmed not applied, partial, unknown | What effect is being asserted? |
| Evidence kind | application record, provider acknowledgement, provider query, independent observation, policy decision, failure trace, reconciliation | What proposition can this source support? |
| Source relationship and coverage | qualified independent, known shared dependency, unknown; complete, partial, missing, duplicated, out-of-plan | How independent and complete is the evidence? |

These dimensions must not be collapsed into one trust score.

A provider acknowledgement is evidence that a provider acknowledged a request. It is not automatically evidence that the external effect occurred. A successful local Holochain zome call is evidence that the local call returned successfully; it is not evidence that the record converged to another peer. A second observer name is not independent corroboration when both observers depend on the same provider, identity system, cloud account, software agent, network path, source event or sensor.

A cryptographic signature authenticates the signed assertion. It does not prove that a physical effect occurred or that the signer is independent.

## 3. Evidence flow

```text
frozen scenario contract
        |
verified offer ledger (denominator fixed before dispatch)
        |
dispatch / attempt events
        |
application, provider or network observations
        |
independent source and dependency qualification
        |
signed per-operation outcome attestations
        |
verified outcome ledger (exactly one record per offer)
        |
verified timing + correctness gates
        |
QOPS reducer
```

The runner is not the sole authority for its own numerator. The denominator, outcome assertion, metric timing, correctness gates and evidence provenance are separate inputs with separate verifiers.

## 4. Binding requirements

Every per-operation attestation must bind to the exact entry in `VerifiedOfferLedger`:

- run ID and MCRB scenario ID;
- exact scenario-contract digest;
- operation ID and offer sequence;
- request digest;
- idempotency-key digest where present;
- offer-ledger digest;
- effect assertion and evidence kind;
- evidence artifact digests;
- evidence source ID and source role;
- attempt identity and event/observation time;
- exact source commit and tree SHA;
- runtime identity and topology-profile digests;
- builder identity;
- verifier identity, version and binary digest;
- trust-policy ID and digest;
- signature and DSSE/in-toto envelope digest.

The verifier must look up the operation in the verified offer ledger. It must not accept caller-supplied operation IDs, request digests, idempotency keys, run IDs or scenario context as independent truth.

An attestation for an operation with a different request digest or idempotency key is invalid even when the signature is valid.

## 5. DSSE and in-toto representation

Use a DSSE v1 envelope carrying an in-toto Statement v1:

- payload type: `application/vnd.in-toto+json`;
- statement type: `https://in-toto.io/Statement/v1`;
- predicate type: `https://luminousdynamics.io/attestations/mcrb-operation-outcome/v0.1`;
- each in-toto subject binds a SHA-256 evidence artifact digest;
- the predicate binds the operation, offer, runtime, topology, source, evidence and policy context.

Canonicalize the predicate before signing. Use DSSE pre-authentication encoding exactly; never sign an ordinary concatenated string or a non-canonical JSON representation. Reject duplicate JSON keys, unknown fields, wrong payload types, invalid base64, duplicate signatures where a single signer is required, subject/predicate mismatch, malformed digests and non-canonical payload bytes.

Outcome signatures must be verified with strict Ed25519 verification. The expected trust-policy digest must be supplied by protected verifier configuration outside the workload runner's mutable input.

## 6. Trust policy and source independence

A pinned outcome trust policy is an externally governed input. It identifies trusted sources and their dependency closures.

Each trusted source record should contain:

- source ID and signing key ID;
- public key;
- source role (effect executor, provider, policy engine, independent observer, reconciliation authority or telemetry);
- allowed evidence kinds and outcome assertions;
- authorized builder identities;
- validity window and revocation state;
- declared dependency IDs;
- a completeness disposition for that dependency closure.

The policy also binds the effect-producing system's dependency closure for the run/scenario profile. The verifier classifies source relationship; the attestation submitter must not self-assert it.

Classification rules:

1. **KnownSharedDependency:** the source dependency closure intersects the effect path's dependency closure, or the source is itself the effect provider/executor for a proposition that requires independent observation.
2. **QualifiedIndependent:** the source has a complete, trusted dependency closure, the relevant dependency sets are disjoint, the source role is permitted to observe the proposition, and the signer is valid for the run.
3. **Unknown:** the dependency closure is incomplete, missing, stale, not assessed or cannot be tied to the trusted source identity.

Unknown source relationship must never be promoted to QualifiedIndependent because two records have different names, keys, organizations or network addresses.

This is an owner-governed classification of declared dependency relationships. It is not a mathematical proof that a source is honest or that no undisclosed common cause exists. The receipt must preserve the trust-policy digest and the full claim ceiling.

## 7. Evidence semantics

### QualifiedSuccess

A QualifiedSuccess requires evidence that satisfies the operation's declared success proposition.

For an external effect, provider acknowledgement alone is insufficient. A provider status query is a provider assertion, not an independent observation. QualifiedSuccess requires an allowed effect receipt or independent observation under the frozen scenario policy, complete coverage of the expected effect, and no unresolved contradictory evidence.

For a Holochain publication scenario, distinguish:
- local zome-call result;
- accepted authored source-chain record;
- validation/integration event;
- record observed by an expected remote peer;
- cross-host timing evidence.

A scenario requiring remote visibility cannot count success from a local zome-call return alone.

### CorrectDenial

Requires a signed policy-decision proposition bound to the exact request digest, principal, resource, action, authority context, policy digest and decision time. The verifier must be able to distinguish an expected denial from an unavailable policy service or a malformed request.

### UnexpectedDenial

Requires the same exact decision evidence plus the scenario's frozen expectation that the request should have been allowed. It is not a qualified success.

### Failed / confirmed not applied

A failure must be attributable and bounded. A timeout, dropped acknowledgement or absent log does not prove that no effect occurred. If the effect cannot be determined, the state remains Unknown. A failed operation with a possible partial external effect must not be collapsed into Failed/no-effect.

### Partial

Partial completion is a separate assertion. Preserve the successful prefix, missing suffix and exact expected action set. Do not represent partial execution as either complete success or no-effect failure.

### Unknown

Unknown is a valid recorded state, not an error to hide. It remains in the offered denominator and makes eligible QOPS unavailable until a later reconciliation attestation resolves it.

### DuplicateEffect

A verified duplicate external effect is a hard correctness failure. It cannot receive throughput credit. Preserve each duplicate receipt and the common idempotency key for audit.

## 8. Exact coverage and ledger rules

The outcome ledger is derived from the verified offer ledger:

- exactly one terminal outcome record or explicit Unknown per offered operation;
- no missing, duplicate, reordered or unoffered operation;
- no second terminal record that silently supersedes the first;
- correction/reconciliation is append-only and binds the prior outcome attestation digest;
- no operation may disappear from the denominator after dispatch;
- unknown and partial outcomes remain explicit;
- the ledger digest is domain-separated and commits to ordered canonical bytes.

For segmented ledgers, each checkpoint must bind the previous checkpoint digest and preserve sequence continuity. A new segment cannot rewrite the earlier offered set.

## 9. QOPS eligibility rules

The reducer should eventually accept only:

`VerifiedOfferLedger + VerifiedOutcomeLedger + VerifiedMetricIntegrity + VerifiedCorrectnessGateSet`

It must not accept `Vec<OperationOutcomeSample>` as a numerator authority.

QOPS is eligible only when:
- the outcome ledger is verified under the exact pinned trust policy;
- all offered operations have exactly one reconciled record;
- no outcome is Unknown or Partial;
- no DuplicateEffect or invalid contradictory record exists;
- every QualifiedSuccess satisfies the success evidence proposition for that scenario;
- required source-independence and remote-visibility conditions are qualified;
- metric integrity and scenario correctness gates are Valid for the same run, scenario, contract, source, runtime and topology context.

Otherwise:
- Invalid means an invariant, signature, source binding, duplicate, contradiction or context check failed;
- Incomplete means required evidence or reconciliation is missing or unresolved;
- NotEvaluated must remain distinct from Invalid, Incomplete and Valid for any optional evidence family.

Correct denials, unexpected denials, failures, partials, unknowns and duplicate effects remain separately reportable. Only QualifiedSuccess contributes to the QOPS numerator. Every offered operation remains in the denominator.

## 10. Required negative controls

The implementation must include at least these frozen cases:

1. Fabricated all-success outcome vector with no attestations.
2. Correct signature over the wrong operation ID.
3. Correct signature over the wrong request digest.
4. Wrong idempotency-key digest.
5. Missing operation attestation.
6. Duplicate terminal attestations for one operation.
7. Unoffered operation inserted into the outcome ledger.
8. Reordered or truncated outcome ledger.
9. Replayed attestation under a different run/scenario/source/runtime.
10. Unknown, expired, revoked or unauthorized signer.
11. Provider acknowledgement labelled as independent observation.
12. Provider status query labelled as physical effect.
13. Two observer identities with a known shared dependency.
14. Unknown dependency closure promoted to independent.
15. Duplicate/mirrored evidence artifact counted twice.
16. Partial completion with a missing expected suffix.
17. Timeout or missing log incorrectly labelled confirmed-not-applied.
18. Conflicting observations silently majority-voted into certainty.
19. Holochain local zome-call success incorrectly counted as remote DHT convergence.
20. Cross-peer observation whose record digest differs from the published record.

## 11. Metrics

Report separate dimensions, not one trust score:

- outcome-attestation verification status;
- per-operation evidence coverage;
- qualified-independent / known-shared / unknown source counts;
- provider-report-only count;
- unknown and partial outcome rates;
- duplicate-effect count;
- missing and duplicate outcome-attestation counts;
- contradictory evidence count;
- late-reconciliation count and reconciliation latency;
- qualified success fraction and QOPS;
- offer rate and QOPS under offered load;
- verifier CPU, memory and evidence bytes per qualified operation.

A useful comparison is the gap between raw acknowledged throughput and qualified throughput. A system that acknowledges quickly but cannot demonstrate outcomes should not win a coordination benchmark by counting unverified acknowledgements.

## 12. Integration with existing work

Compose with, do not duplicate:

- #6342 — offer-first denominator and append-only workload schedule;
- #6335 — raw monotonic and cross-host measurement provenance;
- #6346 — scenario-bound correctness-gate attestations;
- #6140 — external-effect evidence kind, source independence and coverage;
- #6322 — Platform Stewardship / MCRB.

The outcome verifier should consume a frozen dependency-relationship qualification from #6140 or a separately reviewed equivalent. It should not implement a second generic evidence store, provider client, event log or dependency graph.

## 13. Implementation sequence

1. Freeze the canonical outcome record, DSSE/in-toto envelope and source trust-policy schemas.
2. Add typed source roles, evidence kinds, effect assertions and coverage states.
3. Implement canonical domain-separated encoding and strict signature/context verification.
4. Derive expected operation identities exclusively from `VerifiedOfferLedger`.
5. Verify dependency-aware source relationship from the externally pinned trust policy; preserve Unknown.
6. Create opaque `VerifiedOutcomeLedger`.
7. Change QOPS to consume the opaque verified output and remove caller-supplied outcomes from its authority path.
8. Extend the run receipt with outcome ledger, trust-policy, verifier and per-operation evidence references.
9. Add the positive/adversarial fixture corpus and exact-head Rust validation.

## 14. Claim ceiling

This contract and its schemas are design artifacts, not an implemented verifier. They do not establish a live independent observer, source honesty, physical truth, application effect, Holochain DHT convergence, external payment finality, or benchmark superiority. No QOPS rate is eligible until the complete outcome verification path is implemented and independently qualified.
