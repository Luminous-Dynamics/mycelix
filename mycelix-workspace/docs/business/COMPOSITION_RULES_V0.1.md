# Cross-Domain Composition and Qualification Rules v0.1

Status: **normative candidate / documentation only**

This profile constrains how independently authoritative Mycelix domains may be composed into one consequential Business conclusion.

It does not add a new authority-owning Business domain. It closes four cross-domain failure classes that can survive otherwise-correct local state machines:

1. circular/self-justifying qualification;
2. mixed-generation or non-reproducible closure;
3. reuse of one conserved real-world effect to satisfy several incompatible duties; and
4. reuse of stale authorization after the facts that justified it changed.

The rules here apply together with `BUSINESS_CONSTITUTION.md`, `INSTITUTIONAL_SEMANTICS.md`, `DOMAIN_AUTHORITY_MATRIX.md`, `STATE_MACHINES_V0.1.md`, and the six golden paths.

---

# 1. Consequential conclusions have an explicit dependency graph

Every consequential positive Business conclusion SHOULD be explainable as a directed graph whose leaves are domain-owned facts, observations, authority decisions, policies, or evidence and whose internal nodes are named derivations/reconciliations/closure predicates.

A node may depend only on already independently meaningful prerequisites.

## 1.1 No self-justifying cycles

A conclusion MUST NOT be used, directly or indirectly, as evidence for a prerequisite required to establish that same conclusion.

Forbidden shape:

`BusinessClosure -> AccountingAcceptedEvent -> BusinessClosure`

Also forbidden:

`AuthorizedOperation -> policy fact derived only from AuthorizedOperation`

`ReconciledSettlement -> observation whose acceptance depends on ReconciledSettlement`

A cycle is not repaired merely by splitting the nodes across different services, DNAs, databases, providers, or workflow steps.

## 1.2 Accounting may be downstream without becoming circular

A Business workflow MAY require an Accounting-owned acceptance/recording result as a closure predicate only when Accounting can derive that result from independently accepted source-domain facts without requiring the same Business closure as an input.

Valid shape:

`Commerce/Finance/SupplyChain accepted facts -> EconomicEvent projection -> Accounting acceptance -> Business closure predicate`

Invalid shape:

`Business closed -> Accounting accepts event -> therefore Business closed`

If Accounting policy needs a workflow disposition, it must consume a pre-closure domain fact or an independently meaningful disposition record, not the final closure result that it is being asked to justify.

## 1.3 Resolution cannot bootstrap its own evidence

A dispute resolution may establish institutional disposition going forward. It MUST NOT make its own prerequisites true by declaring them true inside the resolution.

For example, a resolution may waive a payment obligation. It cannot simultaneously claim that the payment was historically settled merely because the waiver makes the workflow terminal.

---

# 2. Qualification uses an exact evidence cut

A consequential authorization, reconciliation, or closure MUST be attributable to one exact qualification input set.

Conceptually this is a **qualification snapshot/evidence cut**. It is metadata about the derivation, not a new truth-owning domain object.

The relevant subset SHOULD bind:

- organization context;
- logical intent/workflow/subject identity;
- exact domain record/outcome references and versions;
- exact observation/source-event set;
- exact reconciliation/allocation references;
- exact authority-decision references;
- policy/profile identifiers and versions;
- exception/dispute set;
- qualification time supplied explicitly to the evaluator;
- prerequisite validity/freshness boundaries;
- semantic profile/version.

## 2.1 No Frankenstein qualification

A positive conclusion MUST NOT be assembled from mutually incompatible generations merely because each prerequisite was individually valid at some different time.

Example:

- old approval policy says Alice may approve;
- new role state says Alice remains an employee;
- old pricing terms say $500;
- new purchase order says $600.

Those records cannot be freely mixed into one authorization/closure unless the exact owning-domain profiles establish that the versions are jointly applicable.

## 2.2 Inputs changing during evaluation

If a prerequisite changes while a qualification is being evaluated, an implementation MUST do one of:

1. finish against the exact already-captured immutable input cut and label the result as qualification of that cut; or
2. abort/restart qualification against a new coherent cut.

It MUST NOT silently substitute some newer inputs while retaining older others.

## 2.3 Historical qualification vs current truth

A qualification receipt can remain valid evidence that a conclusion was justified under an exact historical cut even after some prerequisites later expire or change.

That does not automatically make the same conclusion current forever.

Current projections MUST separately obey any currentness/revalidation requirements of the underlying semantics.

Thus:

`historically qualified at t != necessarily current at t+n`

while also:

`later expiry != historical qualification never happened`.

---

# 3. Derivations SHOULD be deterministic over their declared inputs

For a fixed semantic profile, fixed explicit qualification time, and the same exact input cut, a pure authorization/reconciliation/closure evaluator SHOULD produce the same result.

Hidden dependencies are forbidden for consequential qualification.

Examples of hidden dependencies that MUST NOT silently affect a result:

- ambient wall-clock reads not represented in the qualification input;
- process-local random values;
- mutable global configuration not bound by profile/version;
- whichever provider response happened to arrive first;
- iteration/hash-map order;
- UI state;
- cached data whose version/source is not retained.

If current time matters, `qualification_time` is an explicit input.

If external entropy or human discretion is materially required, the resulting authorized decision/observation/resolution must be represented explicitly rather than hidden inside an otherwise deterministic evaluator.

---

# 4. Authorization is bound to an exact operation and validity basis

A current authorization decision is not a reusable bearer token for arbitrary future operations.

A consequential positive authorization SHOULD bind the relevant subset of:

- exact logical-intent identity;
- operation semantic profile/version;
- organization/context;
- holder/principal;
- subject/resource;
- purpose;
- material amount/quantity/destination parameters;
- policy/grant/approval/quorum basis;
- qualification time;
- prerequisite validity/freshness bounds;
- a decision validity boundary when continued currentness is required.

## 4.1 Authorization validity cannot outlive prerequisites

Where an authorization decision is usable for a period rather than one instantaneous evaluation, its usable validity horizon MUST be no later than the earliest required upstream expiry/revocation/currentness boundary.

Conceptually:

`authorization_valid_until <= min(required prerequisite validity boundaries)`

Unknown required expiry/currentness MUST NOT be replaced by an invented permissive duration.

## 4.2 Authorization-to-execution drift

If a policy-relevant fact changes after authorization but before execution, the owning domain must either prove that the existing authorization decision remains applicable under its exact profile or require requalification.

Examples:

- approval grant revoked;
- beneficiary/destination changed;
- amount changed;
- account changed;
- purchase terms changed;
- required policy generation changed.

Transport success cannot repair a stale authorization.

## 4.3 Authorization is not execution evidence

`Authorized` means the operation was permitted under the bound qualification basis.

It does not mean:

- dispatched;
- accepted;
- applied;
- externally effective;
- reconciled;
- satisfied.

---

# 5. Conserved effects require allocation semantics

Not all evidence behaves the same way.

Some evidence is **corroborative/non-consumptive**: one signed agreement or registry attestation may legitimately support several derived claims.

Other observations/outcomes represent **conserved capacity/value**: one $500 settlement, one serialized inventory unit, ten received items, or a defined quantity of remitted value cannot normally be counted again without an owning-domain rule explaining the allocation.

The owning domain/profile MUST declare the applicable semantics.

Business orchestration MUST NOT infer conservation or non-conservation on its own.

## 5.1 No double allocation

A value-bearing/conserved source outcome MUST NOT satisfy more total obligation/effect quantity than the owning domain accepts as available under the exact reconciliation/allocation policy.

Examples:

- one $500 bank settlement cannot satisfy two separate $500 invoices;
- one serialized item cannot fulfill two orders simultaneously;
- receiving 10 units cannot satisfy 15 units of receiving obligations;
- a $100 refund cannot compensate $200 of recognized refund obligation.

## 5.2 Partial allocation is allowed when the domain supports it

One source effect MAY satisfy several obligations when the owning domain explicitly supports divisible allocation and preserves the exact allocation set.

Example:

A $1,000 payment may be allocated $600 to Invoice A and $400 to Invoice B.

The accepted allocation must preserve:

- source effect identity;
- total available quantity/value;
- exact obligation/subject destinations;
- exact allocated amounts/units;
- remaining unallocated amount where applicable;
- policy/profile;
- supersession/correction lineage.

## 5.3 Many-to-one satisfaction is also explicit

One obligation MAY be satisfied by multiple source effects where policy permits.

Example:

Invoice A for $1,000 may be reconciled to two accepted settlements of $600 and $400.

Satisfaction follows the obligation's exact satisfaction predicate over accepted allocations; it is not inferred merely because several payments exist nearby in time.

## 5.4 Duplicate source delivery adds zero capacity

Redelivery of the same source-event identity does not create additional allocatable value/quantity.

Idempotency and conservation reinforce one another:

`duplicate observation -> same source capacity`, not `new capacity`.

## 5.5 Evidence reuse and value reuse are different

A single evidence artifact MAY support several claims when the semantic profile permits corroborative reuse.

That does not authorize reuse of any conserved value represented by that evidence.

For example, the same bank statement may evidence several transactions listed within it, but one transaction line may not be double-allocated merely because the statement document is reusable evidence.

---

# 6. Allocation authority stays with the owning domain

The domain that owns the conserved effect owns its allocation/reconciliation semantics.

Examples:

- Finance owns payment/refund/remittance allocation;
- Supply Chain owns serialized inventory and quantity allocation;
- a workforce/time domain owns accepted work/time allocation;
- an asset/custody domain owns exclusive custody transitions.

The Business layer MAY reference accepted allocation results and use them in closure predicates.

It MUST NOT create a parallel generic allocation ledger that can overrule domain ownership.

A future transport-neutral kernel may provide generic typed references/digests for allocation provenance, but the semantic rules remain domain-owned.

---

# 7. Closure is a pure qualification over a declared dependency set

A `WorkflowClosureReceipt` is strongest when it can be treated as the reproducible result of evaluating one exact closure policy over one exact qualification cut.

Conceptually:

`ClosureReceipt = qualify(closure_profile, explicit_time, exact_dependency_cut)`

The receipt SHOULD additionally retain:

- dependency-set commitment/digest or equivalent immutable references;
- closure class;
- exact obligation dispositions;
- allocation/reconciliation references used for satisfaction;
- authorization-decision references required by policy;
- exact exception set;
- exact policy/profile versions;
- deterministic qualification result/reason.

## 7.1 Closure cannot create its prerequisites

Closing a workflow MUST NOT cause a prerequisite domain record to become accepted solely so that the close remains valid.

If closure triggers a new downstream action, that action is a new causal effect whose result may later support another workflow/state transition.

## 7.2 Reopen/correction is forward history

If later evidence reveals that a historical closure needs correction, the system MUST preserve the original closure receipt and create explicit reopen/supersession/correction lineage.

It MUST NOT mutate the original qualification cut to pretend the new evidence had always been present.

## 7.3 Closure may depend on Accounting only acyclically

Where a workflow closure policy requires Accounting acceptance, the exact Accounting result must be derivable independently from pre-closure accepted facts.

Otherwise Accounting completion should be downstream of Business closure rather than a prerequisite of the same closure.

The profile must choose one direction; it may not rely on both simultaneously.

---

# 8. Cross-domain projection cannot erase dependency uncertainty

If a Business projection summarizes several domains, the projection's certainty cannot exceed the weakest material dependency required for the displayed claim.

Examples:

- accepted order + unknown settlement != "Paid";
- payment settled + disputed fulfillment != "Completed sale";
- inventory observation unavailable != "Out of stock";
- accounting period closed-with-exceptions != every included item reconciled.

A task-oriented UI may hide detail until requested, but the user-visible state must not reverse the semantic distinction.

---

# 9. Privacy is compatible with reproducible qualification

An exact dependency cut does not require every evaluator or viewer to receive every sensitive source record.

A cut may contain protected references, commitments/digests, qualified propositions, or selectively disclosed evidence sufficient for the required predicate.

The qualification mechanism MUST preserve:

- source/domain attribution;
- semantic profile/version;
- freshness/validity semantics;
- exact proposition justified;
- viewer/purpose disclosure rules.

Therefore reproducibility is not permission to build a universal plaintext warehouse.

---

# 10. Required cross-domain qualification vectors

Future executable qualification MUST include at least:

## Dependency cycles

- Business closure requires Accounting result whose derivation requires same Business closure -> deny as circular;
- reconciliation whose source acceptance depends on that reconciliation -> deny;
- resolution that invents historical evidence required for itself -> deny;
- explicit acyclic Accounting-before-close profile -> permit when all prerequisites are independent.

## Qualification cut

- same exact cut + same profile + same explicit time -> same result;
- policy changes during evaluation -> old coherent cut or restart, never mixed cut;
- source record superseded during evaluation -> old coherent cut or restart;
- incompatible generations mixed -> deny;
- later source expiry -> historical receipt remains historical evidence, current projection revalidates when required.

## Authorization drift

- authorization bound to amount A, execution requests amount B -> deny/requalify;
- grant revoked between qualification and execution -> deny unless exact owning-domain profile proves the decision remains valid;
- authorization lease outlives required grant/policy/evidence expiry -> deny;
- unknown required expiry -> no invented positive lease.

## Conservation/allocation

- same $500 settlement allocated to two $500 obligations -> deny;
- same serialized item allocated to two deliveries -> deny;
- 10 received units allocated as 15 -> deny;
- $1,000 settlement split $600/$400 under divisible policy -> permit;
- two settlements $600/$400 satisfy one $1,000 obligation -> permit under exact satisfaction policy;
- duplicate webhook/source event -> allocation capacity unchanged;
- one corroborative agreement evidence supports several derived obligations -> permit where profile allows non-consumptive reuse.

## Closure

- final step reached but dependency cut incomplete -> not closed;
- closure depends on its own projection -> deny;
- close with exceptions -> exact exceptions retained;
- late correction after close -> explicit reopen/supersession lineage;
- same cut evaluated twice -> semantically identical closure result.

## Privacy

- verifier receives only qualified proposition/reference needed for predicate -> may qualify without unrelated source disclosure;
- exact dependency cut does not grant dereference authority;
- export preserves dependency/provenance semantics without requiring export of inaccessible third-party secrets.

---

# 11. Runtime exit gate

Before `mycelix-business-core` encodes cross-domain composition, review SHOULD establish that:

1. every consequential positive conclusion has an acyclic dependency story;
2. qualification is bound to one coherent immutable input cut;
3. hidden clock/random/global-state dependencies cannot change consequential qualification;
4. authorization decisions cannot float free of the exact operation and prerequisite validity that justified them;
5. conserved real-world effects cannot be double-allocated across workflows;
6. divisible allocations and many-to-one satisfaction preserve exact quantities and provenance;
7. closure cannot create the facts required to justify itself;
8. historical closure remains historical evidence while current projections still obey currentness rules;
9. later correction creates forward supersession/reopen history rather than retroactive mutation; and
10. reproducibility does not require universal disclosure.

If any of these require Business orchestration to manufacture domain truth or authority, the composition model is not ready for runtime implementation.
