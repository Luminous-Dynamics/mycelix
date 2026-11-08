# Institutional State Machines and Causal Identity v0.1

Status: **normative candidate / documentation only**

This profile freezes lifecycle and identity semantics required before a transport-neutral Business kernel can be implemented.

It addresses two classes of failure that ordinary workflow systems often collapse:

1. **causal identity** — distinguishing one logical institutional effect from its retries, execution attempts, source events, and transport deliveries; and
2. **institutional disposition** — distinguishing satisfaction, waiver, breach, dispute, compensation, termination, and policy-approved closure with exceptions.

It does not add another business-domain primitive. It constrains the existing vocabulary from `INSTITUTIONAL_SEMANTICS.md`.

---

# 1. Four causal identities MUST remain distinguishable

## 1.1 Logical intent identity

Answers:

> What exact institutional effect is being requested?

Its semantic identity SHOULD commit to the material operation facts defined by the owning domain/profile, including the relevant subset of:

- organization/context;
- operation type/profile;
- subject/resource;
- parties;
- value/quantity/currency;
- destination;
- purpose/scope;
- policy-relevant parameters;
- parent causal/workflow context;
- semantic profile/version.

A material semantic change MUST change logical operation identity.

Examples:

- pay Supplier A $500 != pay Supplier A $600;
- refund Order X != refund Order Y;
- transfer Item 1 A->B != Item 1 A->C.

Logical intent identity is not authority.

## 1.2 Execution-attempt identity

Answers:

> Which concrete attempt to realize that logical intent is this?

One logical intent MAY have multiple execution attempts when owning-domain policy permits retry.

Attempts MUST remain distinguishable across:

- prepared/requested;
- dispatched;
- accepted/rejected by provider/domain;
- `ProvenNotApplied`;
- `OutcomeUnknown`;
- later authorized retry.

A new attempt MUST NOT silently become a new logical institutional effect.

## 1.3 Source-event identity

Answers:

> Which exact externally/domain-generated event is being observed?

Examples include:

- provider event identity + provider identity;
- bank transaction identity;
- carrier event identity;
- exact domain/DHT action identity;
- registry response/object identity.

Repeated delivery/observation of the same source-event identity is duplicate delivery, not a new event merely because it was received again.

A provider's unique identifier is provenance/deduplication evidence; it is not Mycelix authority merely because the provider calls it unique.

## 1.4 Transport-delivery identity

Answers:

> Which concrete message/webhook/queue/network delivery carried the information?

Several transport deliveries MAY carry the same source event or logical intent.

Transport identity MUST NOT be used as logical-effect identity.

---

# 2. Causal metadata

Future envelopes SHOULD be capable of retaining the relevant subset of:

- `logical_intent_ref`;
- stable logical-operation commitment/digest under a named semantic profile;
- `attempt_ref`;
- `source_event_ref`;
- transport/delivery reference where diagnostically useful;
- `caused_by`;
- `workflow_ref`;
- `organization_context_ref`;
- `source_domain`;
- semantic profile/version;
- idempotency scope defined by the owning domain;
- exact evidence references.

These are semantic requirements, not frozen Rust field names.

No timestamp, nonce, transport request ID, database row ID, or provider request ID is sufficient by itself to prove logical-operation equivalence.

## 2.1 Semantic equality belongs to the owning domain

Two operations MAY be treated as the same logical effect only when the owning domain can establish equivalence under the exact registered operation profile.

Business orchestration MUST NOT decide equality by comparing arbitrary serialization bytes or a convenient subset of fields.

## 2.2 Replay rule

Replay/re-delivery MUST NOT create a second consequential effect merely because:

- transport retried;
- orchestrator restarted;
- provider webhook was redelivered;
- queue delivered twice;
- user refreshed/resubmitted;
- workflow replayed during recovery.

The owning domain MUST instead establish one of:

1. the logical operation/effect is already represented -> return/reference existing result;
2. prior attempt is `ProvenNotApplied` -> explicit policy may authorize a new attempt bound to the same logical intent;
3. outcome is unknown -> preserve ambiguity unless domain idempotency/reconciliation policy makes another attempt safe.

## 2.3 `OutcomeUnknown != ProvenNotApplied`

A consequential attempt with `OutcomeUnknown` MUST NOT be blindly retried as though non-application were established.

A new attempt is allowed only when the owning domain's explicit idempotency/reconciliation semantics make duplicate application impossible or explicitly acceptable.

Otherwise remain awaiting evidence/reconciliation or enter an authorized exception path.

## 2.4 Compensation identity

Compensation is a new logical effect with its own identity and a causal edge to the outcome it addresses.

Example:

`payment P -> refund obligation R -> refund intent RI -> refund attempt RA`

The refund is not a mutated/reused identity of payment P.

---

# 3. Claim / observation / accepted fact pipeline

`Claim` remains a source-attributed assertion expressed through the existing observation/assertion/evidence vocabulary rather than a second truth store.

These meanings are non-substitutable:

1. **source claim** — what a party/provider asserts;
2. **observation record** — Mycelix retained that assertion/evidence;
3. **validated observation** — owning domain established required source/structure/authenticity/freshness conditions;
4. **reconciled fact** — owning domain related accepted evidence to an exact institutional subject under policy;
5. **projection** — downstream interpretation.

Duplicate delivery of one source event MUST NOT multiply confidence, quorum, value, quantity, satisfaction, or evidence weight merely because it arrived more than once.

---

# 4. Obligation lifecycle

An `Obligation` is an active institutional duty. It may be instantiated from a commitment, agreement, policy/law, resolution, or another domain-owned normative source.

Its lifecycle MUST distinguish performance from discharge or disposition for other reasons.

A conceptual v0.1 state set is:

- `Active`;
- `PartiallySatisfied`;
- `Satisfied`;
- `Breached`;
- `Disputed`;
- `Waived`;
- `Superseded`;
- `Terminated`.

Exact Rust names may be refined later; the semantic distinctions may not be silently collapsed.

## 4.1 Satisfaction

`Satisfied` means the owning domain/policy accepted evidence that the exact satisfaction predicate was met.

Business orchestration cannot self-satisfy an obligation.

## 4.2 Partial satisfaction

`PartiallySatisfied` retains:

- accepted satisfied portion/conditions;
- remaining duty;
- exact evidence/reconciliation references for what was satisfied.

Partial satisfaction MUST NOT project as full satisfaction.

## 4.3 Breach

`Breached` means a breach predicate was established under applicable normative policy.

Breach does not automatically discharge or terminate the duty.

It may remain enforceable, become disputed, be waived/superseded/terminated, or create remedial obligations.

## 4.4 Waiver

`Waived` means valid authority discharged all/part of the duty without claiming performance occurred.

`Waived != Satisfied`

## 4.5 Supersession

`Superseded` means another explicitly linked normative record replaces the duty going forward.

Historical responsibility and prior performance evidence remain visible.

## 4.6 Termination

`Terminated` ends the duty according to applicable authority/policy without implying satisfaction.

## 4.7 Dispute

`Disputed` preserves a contest over existence, scope, amount, authority, performance, or satisfaction.

Dispute MUST retain claims/evidence rather than manufacture a factual answer.

---

# 5. PowerGrant lifecycle is not authorization lifecycle

A `PowerGrant` is normative authority state, not a positive operation authorization receipt.

A conceptual grant lifecycle is:

- `Proposed`;
- `Granted`;
- `Active`;
- `Suspended` where supported;
- `Expired`;
- `Revoked`;
- `Superseded`.

For consequential execution, the owning authority subsystem SHOULD evaluate the exact current:

- grant identity;
- holder/principal;
- logical operation identity;
- subject/scope;
- purpose;
- constraints/limits;
- policy generation/profile;
- revocation/supersession state;
- validity boundary;
- required approvals/quorum;
- qualification time.

Only the resulting domain-owned authorization decision may justify execution.

Therefore:

`role label != PowerGrant != current qualified authorization`

---

# 6. Observation lifecycle

A conceptual observation lifecycle is:

- `Observed`;
- `Validated`;
- `Rejected`;
- `Conflicted`;
- `Reconciled`;
- `Superseded/Corrected` where source semantics support correction.

## 6.1 Validation is not reconciliation

Validation answers whether the exact observation meets required source/structure/authentication/freshness/policy conditions.

It MUST NOT automatically establish the institutional conclusion for which the observation may later be used.

## 6.2 Conflict

`Conflicted` preserves incompatible claims/observations.

Do not choose "latest" or "most convenient" unless a named owning-domain policy authorizes that rule and the ordering source is trustworthy for the purpose.

## 6.3 Reconciliation

`Reconciled` binds an exact observation/outcome set to an exact institutional subject under an exact policy/profile.

It SHOULD retain:

- source-event/observation identities;
- subject/obligation/event refs;
- reconciliation policy identity/version;
- authority/evidence basis;
- reconciliation time;
- unresolved variance.

---

# 7. Workflow lifecycle

Workflow **progress state** is separate from **closure meaning**.

A conceptual progress lifecycle is:

- `Proposed`;
- `Authorized`;
- `Executing`;
- `PartiallyPerformed`;
- `AwaitingEvidence`;
- `Disputed`;
- `CompensationRequired`;
- `Compensating`;
- `ClosureCandidate`;
- `Closed`.

`ClosureCandidate` means only that enough referenced evidence appears available to evaluate the named closure policy.

Only a closure qualification under that policy may produce `Closed`.

---

# 8. Closure class

Every consequential `WorkflowClosureReceipt` SHOULD retain an explicit closure class.

Conceptual v0.1 classes:

### `Satisfied`

All closure-policy-required duties/effects are accepted as satisfied/reconciled.

### `Waived`

At least one required duty was discharged through valid waiver rather than performance, and the closure policy permits this disposition.

### `Compensated`

A prior adverse/partial outcome was addressed through completed compensating duties/actions under policy.

Original history remains visible.

### `ResolvedWithExceptions`

The closure policy explicitly permits terminality with an exact retained unresolved/deferred/write-off/exception set.

Exceptions remain exceptions; terminality does not strengthen them into satisfied/failed/proven-not-applied facts.

### `Terminated`

The workflow ended through valid termination without representing intended performance as satisfied.

The exact implementation set may be narrowed later. The invariant is that the **reason for terminality remains explicit**.

## 8.1 Forbidden equivalence

`Satisfied == Waived == Compensated == ResolvedWithExceptions == Terminated`

is forbidden as a semantic model.

A coarse UI may group these as "not active", but audit/accounting/compliance/export must preserve the exact class.

## 8.2 Unknown effects and exception closure

A workflow may close with an unresolved effect only when the exact closure policy explicitly allows that exception class.

Example:

A month-end close may be `ResolvedWithExceptions` while a bank item remains unresolved.

That does **not** authorize the corresponding invoice/payment projection to say definitively "settled".

---

# 9. WorkflowClosureReceipt minimum semantics

A future receipt SHOULD bind the relevant subset of:

- workflow identity;
- organization context;
- exact workflow semantic profile/version;
- closure-policy identity/version;
- closure class;
- required predicate set/profile;
- referenced obligation dispositions;
- referenced domain outcomes;
- referenced reconciliations;
- required authorization-decision references;
- retained evidence references;
- unresolved/deferred/exception set;
- compensation refs;
- qualification/closure time;
- effective validity boundary where closure depends on freshness;
- supersession/reopen reference when later corrected.

A closure receipt is evidence of qualification under one exact policy/profile. It is not universal truth outside that semantics.

---

# 10. Semantic profiles and historical interpretation

Business records can outlive the software that created them.

Consequential shared envelopes SHOULD remain interpretable under an exact registered semantic profile/version.

Schema migration MAY translate representation while preserving prior meaning, add optional presentation metadata, or explicitly adapt old records with provenance.

Migration MUST NOT silently:

- strengthen an observation into accepted truth;
- reinterpret `Waived` as `Satisfied`;
- reinterpret `OutcomeUnknown` as failure/non-application;
- lengthen historical validity;
- substitute newer authority policy for historical authorization;
- change logical-operation identity;
- erase closure/discharge reasons.

Material semantic change requires explicit supersession/adaptation/requalification lineage.

---

# 11. Reference privacy and correlation

A reference has at least two separate permissions:

1. **possession/transport** — carry/compare/store the reference as allowed;
2. **dereference** — retrieve protected underlying data.

Possession of `PartyRef`, `SubjectRef`, workflow/evidence/organization refs MUST NOT imply dereference authority.

## 11.1 Contextual correlation

Where global correlation is unnecessary, implementations SHOULD prefer contextual/pairwise references or protected binding mechanisms over universal identifiers.

Cross-domain correlation itself follows purpose/authority constraints.

## 11.2 Disclosure before projection

Viewer/purpose disclosure policy MUST be applied before exposing sensitive joined state.

A universally readable unredacted Business join hidden only by UI controls is insufficient.

## 11.3 Evidence disclosure

Evidence references SHOULD allow the required proposition to be established without automatically exposing unrelated sensitive source material.

Future selective-disclosure cryptography may strengthen this, but v0.1 freezes the semantic requirement now.

---

# 12. Qualification vectors

Future executable contracts MUST cover at least:

## Causal identity / replay

- same transport redelivery -> no new logical effect;
- same provider event delivered twice -> one source event;
- same logical intent + new transport ID -> same logical effect;
- changed amount/beneficiary/destination -> different logical effect unless exact domain profile proves otherwise;
- `OutcomeUnknown` -> blind retry denied;
- `ProvenNotApplied` -> policy may authorize new attempt bound to same logical intent;
- compensation -> distinct logical intent causally linked to original.

## Obligation

- policy/law/resolution may instantiate obligation without fictional voluntary commitment;
- partial performance cannot become `Satisfied`;
- waiver cannot become `Satisfied`;
- breach does not silently terminate duty;
- supersession preserves old history;
- Business orchestrator cannot construct satisfaction without owning-domain result/evidence.

## Power

- expired/revoked grant -> authorization denied;
- grant for wrong subject/scope/purpose -> denied;
- role string without grant -> denied;
- stale policy when current policy required -> denied.

## Observation/reconciliation

- duplicate source event does not multiply value/quantity/quorum;
- validated observation != reconciled fact;
- conflict remains conflict until explicit policy resolution/reconciliation;
- stale required observation cannot produce fresh accepted state;
- reconciliation binds exact source set and subject.

## Closure

- final workflow step without closure predicates -> not closed;
- unresolved required effect -> not `Satisfied`;
- waiver -> may close `Waived`, never `Satisfied`;
- completed compensation -> may close `Compensated`, original history retained;
- policy-approved exceptions -> `ResolvedWithExceptions` with exact exception set;
- closure policy/profile substitution -> denied;
- later correction/reopen -> explicit supersession lineage.

## Privacy/versioning

- reference possession without dereference authority -> access denied;
- viewer lacks purpose/field authorization -> field not disclosed;
- schema migration cannot reinterpret old closure/discharge class;
- semantic profile mismatch -> fail closed or explicit adapter/requalification.

---

# 13. Runtime exit gate

Before `mycelix-business-core` encodes these semantics, review SHOULD establish that:

1. logical intent, execution attempt, source event, and transport delivery are not collapsed;
2. obligation satisfaction remains distinct from waiver, breach, termination, dispute, and supersession;
3. obligations can arise from legitimate normative sources without inventing a voluntary commitment;
4. `PowerGrant` cannot be confused with current per-operation authorization;
5. observation cannot self-promote into reconciled truth;
6. workflow progress is separate from closure class;
7. closure-with-exceptions cannot strengthen exception truth;
8. migrations preserve historical meaning;
9. reference possession is separate from dereference authority;
10. all six golden paths remain expressible without another foundational business primitive.

If any item fails, the corpus remains provisional.
