# Maritime Coordinator Portability and Reconstructible Stay Views v0.1

**Status:** research profile proposal; not normative protocol and not executed  
**Tracking:** [#5938](https://github.com/Luminous-Dynamics/mycelix/issues/5938)  
**Parents:** [#5916](https://github.com/Luminous-Dynamics/mycelix/issues/5916), [#5918](https://github.com/Luminous-Dynamics/mycelix/issues/5918), [#5920](https://github.com/Luminous-Dynamics/mycelix/issues/5920), [#5921](https://github.com/Luminous-Dynamics/mycelix/issues/5921)  
**Generic patterns to compose:** [FIN-SYNC-004 #2525](https://github.com/Luminous-Dynamics/mycelix/issues/2525), [LEX-NET-028 #1215](https://github.com/Luminous-Dynamics/mycelix/issues/1215), [LEX-NET-025 #1149](https://github.com/Luminous-Dynamics/mycelix/issues/1149), [LOG-COMMONS #4900/#4901](https://github.com/Luminous-Dynamics/mycelix/issues/4900)

## 1. The claim being tested

A maritime stay is composed from independently authoritative provider effects. A coordinator may make those effects easier to discover, request and inspect, but its database is not the canonical record of a provider commitment.

This profile tests whether a stay's derived view can be reconstructed from an exact, bounded evidence frontier after the original coordinator is replaced or lost.

The intended property is:

> Two independent evaluators, given the same exact evidence frontier, historical evaluation instant, semantic profile, policy snapshot and verifier version, produce the same canonical historical disposition without contacting providers or performing effects.

This does not mean that a missing provider record can be reconstructed from a digest. A digest commits to evidence; it does not recreate unavailable source bytes or prove the underlying claim true. A missing, inaccessible or unverifiable prerequisite remains Unknown / NeedsReconciliation.

## 2. Why this is a separate profile

Maritime Stay Protocol v0.1 defines the initial StayIntent, ProviderOffer, ReservationHold, ProviderCommitment, EffectReceipt and StayManifest messages. Its schema is structural and does not itself provide a complete issuer-signed envelope or coordinator-recovery theorem.

Do not silently widen v0.1.

This profile adds a review boundary around:
- historical manifest reconstruction;
- exact evidence-frontier identity;
- coordinator replacement;
- read-only versus effectful coordinator roles;
- current actionability requalification;
- native event/wire-format adaptation.

The generic crash journal, coordinator fencing, receipt algebra, identity, capability, provenance and event log must be reused from their existing owners after their exact contracts are reviewed. This document is not a new generic event store or durable journal.

## 3. Core state dimensions

Keep these dimensions separate:

| Dimension | Question | Authority |
|---|---|---|
| Provider effect | Did the provider actually commit, cancel, refund or fulfil? | Provider or separately recognized external evidence |
| Coordinator observation | What did this coordinator receive, durably record or fail to observe? | Coordinator-local history |
| Evidence disposition | Is the evidence structurally valid, attributable, recognized, current and applicable? | Typed verifier/recognition stages |
| Historical stay view | What follows from one frozen evidence frontier and evaluation profile? | Deterministic independent evaluator |
| Current actionability | May a new exact action be attempted now? | Current provider state + local authority + current policy/capability |
| Coordinator execution ownership | Which coordinator epoch may attempt the already-authorized operation? | Existing qualified lease/fencing mechanism |

A historical manifest is not an executable capability. Reconstructing it does not transfer a provider credential, payment token, session, reservation hold or action lease.

## 4. Evidence frontier

An evidence frontier is a closed, explicitly identified set of evidence references consumed by one evaluation. It should describe:

- stay-intent identity and exact component set;
- provider/source identity and relevant subject/resource references;
- typed receipt references with exact content commitments;
- source-native event references and protected raw-artifact commitments where permitted;
- provider event identity and revision/sequence only when the provider actually supplies them;
- event occurrence, provider revision, delivery attempt and local receipt timestamps as distinct fields;
- exact protocol, adapter, translation, local recognition and policy profile versions;
- verifier/evaluator identity and version;
- evidence intentionally excluded and why;
- unresolved gaps, contradictions and equivocation groups;
- the historical evaluation instant and declared clock/currentness policy;
- resulting component and aggregate dispositions;
- explicit claim ceiling and nonclaims.

The evidence frontier must not contain:
- secrets or signing keys;
- bearer credentials or reusable capability leases;
- payment-card data;
- unneeded customer or crew PII;
- private source payloads copied solely to simplify coordinator portability.

A portable package may contain references and commitments to protected evidence. The next evaluator must prove it can access and verify each prerequisite within the same authorized purpose. Otherwise it returns an indeterminate disposition.

## 5. Deterministic reconstruction theorem

Define a canonical view identity over the exact frontier and evaluator inputs. Reuse the repository's established canonicalization and content-commitment primitives only after inspecting their current contract; do not create a second canonicalization/digest subsystem.

The reconstruction function should be pure over already admitted inputs:

`View = Evaluate(Frontier, EvaluationInstant, ProtocolProfile, PolicySnapshot, VerifierIdentity)`

For identical inputs it must produce byte-identical canonical output. Input collection order must not affect the result when the evidence semantics establish a determinate order. If two updates are incomparable because the source provides no trustworthy revision/sequence, preserve both and return Conflict / NeedsReconciliation; do not invent an order based on delivery time.

Historical reconstruction and present actionability are distinct:

- Historical reconstruction uses the exact prior policy/profile/evaluation instant. It preserves what was concluded then.
- Current actionability evaluates current provider evidence, issuer/key status, recognition agreements, local policy, expiry and exact action capability.
- A later policy change or revocation creates a new current disposition; it does not rewrite the historical manifest.
- A new verifier version creates a new evaluation identity. It does not mutate the old verdict.

No network call, live system clock, randomness, ambient environment variable, provider mutation or current-policy lookup may be hidden inside deterministic historical evaluation. Those inputs must be explicit and committed.

## 6. Coordinator replacement and effect safety

A read-only replacement coordinator may rebuild a historical view from an authorized evidence frontier without acquiring the old coordinator's credentials.

An effect-capable coordinator needs a separate, current execution profile. Before it can dispatch:
1. bind the exact provider, resource, operation, canonical request digest and provider idempotency identity;
2. classify any previous attempt as NotSent only when durable evidence proves no external invocation could have occurred;
3. otherwise retain Unknown / NeedsReconciliation until provider-authoritative reconciliation or a qualified same-operation replay theorem resolves it;
4. verify current provider key/profile, recognition and local policy;
5. acquire the existing exact plan/operation execution lease and fencing epoch where the provider adapter can enforce it;
6. obtain a current exact-purpose local capability;
7. invoke only the provider-authorized operation and record typed receipts without promoting a transport acknowledgement to effect confirmation.

A fencing token enforced only by local processes cannot stop a paused old coordinator at an external provider. Where the provider cannot enforce fencing, the profile must rely on provider-owned durable idempotency and a qualified single-writer constraint, or refuse effectful failover. Never state a global exactly-once guarantee.

An old coordinator that resumes after replacement may still present historical evidence. It cannot start a new effect with an expired/stale execution epoch.

Compensation is a new provider effect with its own authority, idempotency and receipt lineage. Reconstructing a partial stay never authorizes automatic cancellation/refund.

## 7. Wire and standards neutrality

Do not prescribe CloudEvents as the sole event envelope.

DCSA Port Call 2.0 uses a DCSA-defined OpenAPI publisher/subscriber contract and leaves event-subscriber registration and bilateral authentication/authorization out of scope. DCSA's API design guidance also says it avoids CloudEvents for notifications. CloudEvents 1.0.2 remains a useful optional adapter for providers that actually use it.

Therefore:
- preserve provider-native bytes or an authorized protected source commitment;
- define explicit per-source parsing and semantic mapping;
- issue typed origin, translation/loss, recognition, transport and effect receipts at their respective stages;
- preserve source event identity and timestamps without conflating them with delivery identity;
- never assume all sources expose monotonic revisions;
- never infer source truth from a signature or transport acknowledgement;
- never rewrite DCSA timestamp meaning merely to fit the stay model.

DCSA Port Call events concern commercial port-call service coordination. They are not the recreational stay protocol, marina booking authority or statutory clearance.

## 8. Receipt typing and local authority

Compose the Mycelix typed receipt algebra. The following are different facts and cannot substitute for one another:

- TransportReceipt: a message was delivered;
- OriginVerificationReceipt: a source signature/origin check was satisfied under a specific verification profile;
- TranslationReceipt: a specific target projection and its semantic loss were recorded;
- RecognitionReceipt: a local authority recognized an evidence projection for a bounded purpose;
- AuthorizationReceipt: a local authority authorized an exact action;
- ExecutionReceipt: an attempt was made under a specific execution context;
- ExternalEffectReceipt: a provider-authoritative or otherwise recognized external effect was established.

A signature-valid delivery is not a locally recognized provider. A recognized provider is not automatically authorized to mutate every resource. A valid execution attempt is not proof of a successful effect. Replaying a weak receipt inside a stronger-looking envelope must not upgrade its type or disposition.

## 9. Independent fault corpus

The companion file coordinator-portability-cases-v0.1.json declares 20 synthetic cases. It tests reconstruction equality, permutations, incomplete evidence, incomparable revisions, crash windows, lost provider responses, fencing, actionability changes, secret leakage, receipt substitution and native event mapping.

Each case specifies:
- frozen evidence-frontier inputs or named references to existing corpora;
- required comparison implementations;
- expected historical disposition;
- expected current-actionability disposition;
- effect-permission ceiling;
- invariant and failure reason;
- exact claim ceiling.

The corpus is declarative and not executed. It must be reconciled with final #5918/#5921 semantics before becoming a normative verifier oracle.

## 10. MCB-COORD benchmark family

Add coordinator portability as an independent family in the Maritime Commons Benchmark. Compare:

A. central orchestration with one durable coordinator;
B. high-availability/replicated orchestration using a shared durable journal;
C. replaceable read-only evaluators reconstructing stay views from participant/provider evidence, with effect authority requalified separately.

Measure:
- historical manifest equality across independent evaluators;
- cold-start reconstruction time;
- recovery/replacement time and active operator minutes;
- frontier size and number of inaccessible evidence references;
- divergence/conflict detection time;
- provider read-back requirement and latency;
- duplicate provider effect count after crash/failover;
- stale fencing-epoch dispatch acceptance;
- percentage of fields/claims reconstructible without copying protected source payloads;
- provider-native availability while the coordinator is offline;
- current-actionability requalification cost after policy/key revocation.

Report p50/p95 and sample sizes. Keep safety gates non-compensatory. A faster recovery that duplicates a provider effect is a failed run.

## 11. Proposed gates — not results

- 100% canonical equality for identical frozen frontier/evaluator inputs;
- zero mutation of historical receipts/manifests during reconstruction;
- zero external side effects from read-only reconstruction;
- zero duplicate effects in bounded schedules where provider idempotency is explicitly present and qualified;
- zero new actions accepted under a stale execution epoch under the tested enforcement profile;
- zero local actions without current exact-purpose authority;
- zero receipt-type substitution;
- 100% Unknown / NeedsReconciliation for unresolved provider outcomes;
- 100% explicit preservation or accounting of native mapping loss;
- zero secrets or unnecessary PII in portable recovery artifacts.

These are synthetic qualification goals, not operational failure-rate claims.

## 12. Dependencies, exclusions and claim ceiling

This profile must compose #2525, #1215, #1149, #4900/#4901, #5918, #5920 and #5921. Any generic primitive gap is raised to the existing owner rather than cloned in maritime code.

No new generic event log, durable journal, lease service, identity/DID registry, capability engine, receipt algebra, canonicalization subsystem, provider inventory database, payment rail or coordinator SaaS is in scope.

No live provider endpoint, booking, hold, cancellation, refund, payment or credential transfer is in scope without separate explicit provider authorization and a reviewed operational contract.

A PASS establishes deterministic reconstruction and declared failure behavior only for the exact synthetic inputs, profiles, evaluator and software identity. It does not prove provider honesty, missing source-state recovery, operator support for fencing/idempotency, legal enforceability, physical truth, safety, field interoperability or production readiness.
