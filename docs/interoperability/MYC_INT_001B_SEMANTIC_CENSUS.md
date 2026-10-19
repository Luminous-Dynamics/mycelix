# MYC-INT-001B — Interoperable Evidence and Authority Semantic Census

Status: architecture census; documentation-only

Issue: #3115
Parent interoperability contract: #3114 (`MYC-INT-001A`)
Base examined: `main` at `4a190a9c6ad8d9f1e291f10916472a183f01eddc`

## 1. Purpose

This census inventories existing Mycelix concepts that could otherwise be accidentally duplicated by the Integral interoperability work. Its purpose is not to redesign every domain around a universal evidence model. It establishes where semantics already live, where names collide, which concepts are genuinely cross-domain, and which boundaries must remain explicit before shared Rust types are introduced.

The governing rule is:

> Domain payloads remain domain-owned. Shared interoperability code carries references, provenance, epistemic kind, verification state, schema identity, and authority boundaries unless broader reuse is independently demonstrated.

This document makes no executable compatibility claim and does not modify the older `MYC-EVID-002A` qualification lineage. The `MYC-EVID-002A` identifier had already been assigned in the semantic/evidence program referenced by #2661. Its exact qualifier #2662 is not PASS; this work therefore uses the independent `MYC-INT-*` namespace.

## 2. Safety and semantic invariants

The interoperability layer must preserve at least the following distinctions:

```text
Recommendation != Decision
Decision       != Authorization
Authorization  != ExecutionReceipt

Prediction     != Observation
ActorReport    != InstrumentMeasurement
Inference      != SourceObservation
NormativeClaim != DescriptiveClaim

Claim          != Evidence
Evidence       != Verification
Evidence       != Authority
Verification   != Endorsement

Identity       != Reputation
Reputation     != Expertise
Expertise      != Standing
Standing       != Authority

Schema-name equality != Semantic equivalence
External authority   != Local authority
Schema compatibility != Policy agreement
Imported object      != Locally authored object
```

Where conversion is needed, conversion produces a new attributable object or translation receipt. It must not rewrite source provenance.

## 3. Inventory

| Area | Exact owner | Existing semantics | Census disposition |
|---|---|---|---|
| Core epistemics | `crates/mycelix-core-types/src/epistemic.rs` | Cross-ecosystem E/N/M/H-style classification, epistemic contexts, testimonial quality, verification states including contested/refuted/superseded | Treat as an existing semantic root; do not create a second generic E/N/M system |
| Core crate contract | `crates/mycelix-core-types/src/lib.rs` | Declares core types as fundamental shared ecosystem types | Any new generic primitive requires a stronger reuse case than a single adapter |
| Knowledge claims | `mycelix-knowledge/zomes/claims/integrity/src/lib.rs` | `Claim`, `ClassifiedClaim`, local E/N/M/H types, `Evidence`, `EvidenceType`, `ClaimChallenge`, classification votes/consensus | Keep domain payloads local; reconcile namespace collision before any cross-schema mapping |
| Knowledge fact-checking | `mycelix-knowledge/zomes/factcheck/integrity/src/lib.rs` | `FactCheckRequest`, `FactCheckResult`, local `EpistemicPosition`, claim relationships, advisory `SuggestedAction` | Preserve advisory/non-executive meaning; do not convert suggested actions into authority |
| Civic justice evidence | `mycelix-civic/zomes/justice-evidence/integrity/src/lib.rs` | Complaint-bound evidence, encrypted-content support, verifier records, verification status, disputes | Keep justice payload local; reusable concepts are evidence reference/provenance/verification, not the whole struct |
| Attribution | `mycelix-attribution/zomes/usage/integrity/src/lib.rs` | Immutable `UsageReceipt`, ZK-STARK-backed `UsageAttestation`, verifier material, predecessor links | Reuse as a proof that receipts/attestations are domain assertions; do not create a universal attestation payload |
| Governance proposals | `mycelix-governance/zomes/proposals/integrity/src/lib.rs` | Proposal lifecycle, executable actions, amendments, discussion contributions, stance, advisory discussion reflection | Deliberation currently has useful advisory signals but no generic typed objection graph |
| Governance signing | `mycelix-governance/zomes/threshold-signing/integrity/src/lib.rs` | Scoped signing committees, threshold signatures, signature shares, finality evidence | Treat as authority/finality evidence that a future authorization can reference, not replace |
| Governance execution | `mycelix-governance/zomes/execution/integrity/src/lib.rs` | Timelock, execution, execution status, veto/override, fund allocation | Existing `Execution` is an execution-receipt analogue; authority is currently implicit in proposal/signature/timelock lineage |
| Bridge entry types | `crates/mycelix-bridge-entry-types/src/lib.rs` | Shared cross-cluster query/event types, source agent, related hashes, schema versioning/migration | Strong precedent for shared versioned types; insufficient by itself for external federation provenance |

## 4. P0 semantic collision: E/N/M names are not equivalent schemas

The most important finding is not missing functionality. It is existing semantic drift.

`mycelix-core-types` and the Knowledge claims zome both define types named like `EmpiricalLevel`, `NormativeLevel`, `MaterialityLevel`, and `EpistemicClassification`, but the meanings are materially different.

A concrete example:

- Core uses its highest empirical level for **cryptographically verifiable** evidence.
- Knowledge uses its highest empirical level for **established / robust empirical consensus**.

Those are not interchangeable claims. A result can be cryptographically bound yet scientifically weak, or scientifically well established without being represented by a cryptographic proof. Mapping one value to the other merely because both are named `E4` would create false semantics.

The normative and materiality scales also differ in both meaning and cardinality. Therefore:

```text
core::EmpiricalLevel::E4
    !=
knowledge::EmpiricalLevel::E4
```

The same-name rule is now explicitly rejected.

### Required treatment

Until #3131 resolves the namespace/conversion question:

1. serialized objects must identify which epistemic schema/version they use;
2. adapters must keep core and Knowledge classifications namespaced;
3. no implicit `From`/`Into` conversion should be introduced;
4. lossy or impossible mappings must be representable explicitly;
5. translated objects retain the original classification and conversion provenance;
6. round-trip tests must detect semantic loss rather than silently normalize it.

This is a compatibility prerequisite, not a reason to immediately rewrite either domain.

## 5. Evidence is not one universal payload

The repository already demonstrates several legitimate meanings of “evidence”:

### Knowledge evidence

Knowledge evidence is claim-centered. It records a claim reference, evidence type, source URI, content, strength, submitter, and submission time. It is immutable and can be challenged through a separate `ClaimChallenge` lineage.

### Justice evidence

Justice evidence is case/complaint-centered. It includes complaint identity, submitter, evidence type, title/description, content hash, optional encrypted content, and explicit verifier/dispute records.

### Attribution attestation

Attribution does not call its main domain assertion `Evidence`. It uses immutable `UsageReceipt` and proof-bearing `UsageAttestation` entries, including witness commitments, proof bytes, expiry, verifier key and signature material.

These are not defects that should be flattened. They are examples of healthy domain specialization.

The shared layer should therefore prefer references/envelopes:

```text
EvidenceRef -> identifies evidence without owning its domain payload
Provenance  -> identifies source, schema, lineage and transformation history
VerificationReceipt -> says who checked what, under which method/policy/version
EpistemicKind -> describes how the underlying assertion was produced
```

It should not define:

```text
enum GlobalEvidence {
    Justice(...),
    Science(...),
    Finance(...),
    Health(...),
    Integral(...),
    ...
}
```

Such an enum would make core depend on every domain and turn interoperability into central schema ownership.

## 6. Verification should be a separate assertion from source evidence

The existing code contains both good examples and a useful warning.

Justice has a separate `EvidenceVerification` object and `EvidenceDispute` object. This preserves the difference between the submitted evidence and later judgments about it.

Attribution's `UsageAttestation`, in contrast, contains mutable-looking verification fields (`verified`, verifier key/signature). Its integrity source explicitly flags that update validation is currently too permissive and relies on coordinator checks for which fields may change.

For future shared semantics, prefer:

```text
SourceAssertion / Attestation   immutable
        |
        +--> VerificationReceipt A
        +--> VerificationReceipt B
        +--> Challenge / Dispute
        +--> Supersession / Expiry
```

rather than mutating the original assertion into “verified”.

This does not require an immediate Attribution migration; it defines the interoperability direction and avoids copying the same update pattern into new shared types.

## 7. Current governance authority path

Mycelix already contains a meaningful authority chain, but several of its semantic roles are implicit:

```text
Proposal
   |
   v
Vote / approval
   |
   v
ThresholdSignature
   |
   v
Timelock
   |
   v
Execution
```

`ThresholdSignature` provides scoped collective finality evidence. `Timelock` provides a bounded execution gate. `Execution` records the executor, status, result/error, and timestamp.

This is useful existing machinery. The interoperability work should not replace it.

However, the generic semantic distinction needed by #3117/#3118 is currently not a first-class object:

```text
Decision
   !=
Authorization
```

A governance decision may justify issuing an authorization, but an execution API should not have to infer permission from a loosely related proposal ID, free-form action JSON, or advisory output.

The future model should therefore allow a domain-specific authority path to issue/reference a bounded authorization while keeping the existing governance objects intact.

## 8. Advisory outputs already exist and should remain non-executive

Two current examples are especially useful:

- Governance `DiscussionReflection` includes `ready_for_vote` and readiness reasoning.
- Knowledge fact-checking emits `SuggestedAction` values such as request verification, challenge a claim, add context, wait, or escalate to governance.

These are already semantically closer to recommendations than commands.

They establish the desired rule for Symthaea/Integral FRS interoperability:

```text
analysis / reflection / fact-check / simulation
                  |
                  v
            Recommendation
                  |
           [NO EXECUTION]
                  |
                  v
      authorized process decides
```

Confidence, epistemic strength, reputation, expertise, or model quality must not automatically manufacture execution authority.

## 9. Proposed minimal shared interoperability vocabulary

No Rust type is introduced by this census. The following are candidates to validate in follow-up work. A candidate moves into shared code only after at least two independent domains demonstrate the need.

### 9.1 `SemanticRef`

Purpose: identify a typed object without importing its payload type.

Candidate fields:

```text
namespace / authority domain
object identifier or content/action hash
schema identifier
schema version
object version / lineage reference where applicable
```

This can become the basis for `EvidenceRef`, `ClaimRef`, `DecisionRef`, `ModelRef`, or `PolicyRef` aliases/newtypes without moving domain payloads into core.

### 9.2 `EpistemicKind`

This is orthogonal to E/N/M quality/classification. It answers “how was this assertion produced?”, not “how strong is it?”

Candidate values:

```text
DirectObservation
InstrumentMeasurement
ActorReport
DerivedInference
Simulation
ForecastPrediction
NormativeAssertion
ExternalImport
```

A scientific confidence model, Knowledge E/N/M classification, or community epistemic framework can still be attached separately.

### 9.3 `ProvenanceEnvelope`

Candidate responsibility:

```text
source actor / issuer
source node or namespace
source object reference
schema identity/version
creation/assertion time
predecessor/supersession lineage
integrity/signature/proof refs
translation/import receipts
```

Provenance should be immutable history. Local review state should not rewrite it.

### 9.4 `VerificationReceipt`

Candidate responsibility:

```text
subject ref
verifier / evaluator
method or policy ref + version
result
reason / notes / machine-readable finding refs
verification time
supporting evidence refs
```

A receipt records a verifier's conclusion. It does not mutate the source object and does not imply universal endorsement.

### 9.5 authority references, not a universal governance model

The generic layer may need `DecisionRef`, `AuthorizationRef`, and `ExecutionReceiptRef`, but the full domain objects should not be promoted until concrete reuse exists.

Authority transitions must remain explicit:

```text
Recommendation
      X  cannot authorize directly

Decision
      |
      | policy/governance-specific derivation
      v
Authorization
      |
      | scope + validity checked
      v
ExecutionReceipt
```

## 10. Federation requirements

`mycelix-bridge-entry-types` already establishes two good precedents:

- shared cross-cluster entry definitions can have a single source of truth;
- schema versions and additive migration rules are explicit.

External interoperability needs more than the current bridge event/query envelope. A foreign record may require:

```text
source node / namespace
source actor / issuing authority
source object identity and version
schema URI/type and version
integrity/signature/proof reference
epistemic kind
source assertion state
local verification state
import time
adapter/translation version
policy/jurisdiction scope when relevant
```

The receiving node must be able to store and verify an external object without politically endorsing it or granting it local authority.

This leads to four independent predicates:

```text
syntactically understood?
cryptographically/integrity verified?
locally trusted/accepted as evidence?
locally authorized to cause effects?
```

No predicate implies the next one.

## 11. Ownership decisions

The census freezes the following preliminary ownership decisions.

| Concept | Ownership decision |
|---|---|
| Full Knowledge `Claim` | Knowledge-owned |
| Full Knowledge `Evidence` | Knowledge-owned |
| Justice `Evidence` | Civic/Justice-owned |
| `UsageReceipt` / `UsageAttestation` | Attribution-owned |
| Governance `Proposal` | Governance-owned |
| `ThresholdSignature` | Governance-owned authority/finality evidence |
| `Timelock` | Governance-owned execution-control primitive |
| `Execution` | Governance-owned execution-receipt analogue |
| E/N/M/H quality model | Existing owner must be explicit; core and Knowledge are currently distinct schemas |
| Object/evidence references | Candidate shared primitive |
| Epistemic production kind | Candidate shared primitive, orthogonal to E/N/M |
| Provenance envelope | Candidate shared primitive |
| Verification receipt/envelope | Candidate shared primitive if multiple domains converge |
| Recommendation/Decision/Authorization distinction | Cross-domain invariant; payload types remain domain-specific until reuse is proven |
| Integral CDS/FRS payloads | Adapter-owned, not core |

## 12. Do-not-unify list

The following changes are specifically rejected by this census unless a later proof demonstrates otherwise:

- replacing all domain `Evidence` structs with one universal enum;
- aliasing Knowledge E/N/M values to core E/N/M values by ordinal or variant name;
- turning `FactCheckResult` or `DiscussionReflection` into execution permission;
- treating a threshold signature as a universal authorization schema;
- treating `verified == true` as equivalent to global truth or endorsement;
- turning identity/reputation/expertise/standing into one scalar authority score;
- allowing external decisions to execute locally without a local authorization rule;
- rewriting imported objects into local schemas without retaining source identity and translation provenance;
- introducing Integral-specific consensus, ITC, ecological, or fairness semantics into generic Mycelix types.

## 13. Migration strategy

The safe sequence is additive and explicit.

### Phase A — identity and namespaces

1. assign explicit schema identifiers/version identities to shared/interoperable representations;
2. freeze the core-vs-Knowledge epistemic distinction;
3. add compile/test fixtures that prove same-named schemas cannot be interchanged accidentally.

### Phase B — references and provenance

1. introduce the smallest semantic reference type if reuse is demonstrated;
2. add provenance/translation metadata without moving domain payloads;
3. prove deterministic serialization and cross-cluster transport.

### Phase C — epistemic production kind

1. add an orthogonal `EpistemicKind` only after mapping existing domain concepts;
2. prove prediction cannot satisfy an observation-required API;
3. preserve local E/N/M/credibility models alongside it.

### Phase D — verification receipts

1. model verification as an attributable assertion about another object;
2. keep source object immutable;
3. support conflicting verifier receipts and unresolved states.

### Phase E — authority seam

1. type recommendation, decision, authorization, and execution references distinctly;
2. adapt existing governance signature/timelock/execution flow rather than replacing it;
3. prove direct recommendation-to-execution paths fail.

Only after these phases should the Integral water-system adapter in #3119 be used as an end-to-end neutrality test.

## 14. Required qualification tests for code follow-ups

When code begins, the minimum regression set should include:

### Schema safety

- core E4 cannot be silently accepted as Knowledge E4;
- same local ID under different namespaces remains distinct;
- unknown future schema versions fail closed or remain opaque, never guessed;
- lossy translation emits explicit loss metadata.

### Epistemic safety

- prediction cannot satisfy a direct-observation requirement;
- actor report remains attributable to the reporter;
- simulation retains model/version/input refs;
- normative assertion remains distinguishable from descriptive evidence.

### Provenance safety

- serialization round-trip preserves source namespace/object/schema/version;
- bridge/federation transport preserves provenance exactly;
- translated local views retain a reference to the immutable source object;
- local verification does not rewrite source provenance.

### Authority safety

- recommendation cannot be passed where `AuthorizationRef` is required;
- decision alone does not satisfy execution capability unless domain policy issues authorization;
- expired/revoked/out-of-scope authorization cannot validate a new execution receipt;
- execution receipt cannot claim a missing authorization;
- foreign authority cannot produce local effects without an explicit local policy seam.

### Open-world safety

- conflicting verifications can coexist;
- contested evidence is not deleted when a review concludes;
- unknown/indeterminate state remains first-class;
- supersession preserves predecessor lineage.

## 15. Integral interoperability implications

The census strengthens rather than weakens the Integral compatibility plan.

Integral does not need Mycelix to adopt CDS, OAD, ITC, COS, or FRS as core policy. An Integral adapter can map its objects into neutral Mycelix references and envelopes while retaining Integral's own decision rules.

Likewise, Symthaea can contribute simulations, diagnoses, forecasts, and recommendations as evidence-bearing artifacts without becoming an authority.

A future water-system proof should therefore look like:

```text
observation / report
        |
        v
issue + claims + evidence refs
        |
        v
simulation / analysis
        |
        v
recommendation
        |
       [no authority]
        |
        v
Integral CDS or another governance adapter
        |
        v
decision
        |
        v
bounded authorization
        |
        v
implementation / execution receipt
        |
        v
outcome observations
        |
        v
review / reaffirm / amend / supersede
```

Mycelix's neutrality is proven when another governance adapter can reuse the same evidence/provenance/authority seams while applying different participation, consensus, and legitimacy rules.

## 16. Result and next gate

This census does **not** justify a new global `Evidence`, `Claim`, `Attestation`, or `Decision` Rust type.

It does justify three immediate next steps:

1. resolve or explicitly namespace the core-vs-Knowledge E/N/M semantic collision (#3131);
2. prototype the smallest namespace-safe semantic reference/provenance envelope against at least two independent existing domains;
3. design verification as a separate receipt/assertion before introducing any new cross-domain verification status.

The implementation gate remains conservative: promote a type into shared core only when at least two independent domains need the same semantics, and reject any promotion that carries one domain's policy into another.
