# mycelix-business-core

Transport-neutral executable contracts for the Mycelix Business institutional substrate.

This crate is intentionally small. It encodes structural invariants from the v0.1
business normative corpus without becoming an authority-owning domain.

It has no HDK, Holochain, persistence, networking, provider, UI, AI, ambient clock,
environment, random, or external-effect dependencies.

## What it enforces

- typed causal identities (`logical intent != attempt != source event != delivery`);
- source-scoped external event identity;
- domain-scoped authorization/reconciliation/allocation/obligation/exception identity;
- committed logical intent semantics (`intent id + operation profile + owning-domain semantic commitment`);
- production authorization applicability only to the exact committed operation;
- authorization validity derived from an exact domain-owned decision result, never supplied as a free-standing Business lease;
- explicit time/freshness bounds with fail-closed unknown required validity;
- exact qualification cuts with domain-local generation-conflict rejection;
- deterministic set semantics for qualification inputs;
- acyclic derivation graphs;
- authorization validity that cannot outlive its exact decision source or prerequisite cut;
- authorization decision identity and exact decision source must both be present in the cut;
- provenance-bound attempt outcomes and retry safety (`OutcomeUnknown != ProvenNotApplied`);
- structural conservation for accepted allocation statements with summaries bound to the exact validated statement;
- policy-declared required-obligation completeness for workflow closure;
- production closure dispositions bound to exact domain-owned results in the same cut;
- compensated closure bound to a full `CommittedIntent` plus an exact supporting result in the same cut;
- retained exceptions bound to exact same-domain results plus exact cut↔receipt exception-set equality;
- sealed positive authorization, retry, allocation-summary, exception, and closure objects whose validated basis cannot be externally rewritten after construction;
- obligation-disposition and closure-class non-substitutability.

## Sealed positive objects

A validation function is not an invariant boundary if callers can bypass it by
constructing or mutating the resulting positive object directly.

Production positive objects therefore keep invariant-bearing state private or
crate-private and expose read-only accessors. This includes:

- `AuthorizationBinding`;
- `AttemptOutcomeBinding`;
- `RetrySafety`;
- `AllocationSummary`;
- `ObligationDispositionBinding`;
- `CompensationBinding`;
- `ExceptionBinding`;
- `WorkflowClosureReceipt`.

External callers cannot:

- forge a positive authorization with a hand-written struct literal;
- replace the decision source after authorization qualification;
- widen an authorization's effective validity horizon;
- claim `ProvenNotApplied` as retry authority without an exact attempt-result binding;
- claim duplicate-safe retry from a free-standing boolean/enum;
- detach an allocation summary from the exact statement it validated;
- inject a bare exception ID as proof of an accepted retained exception;
- replace a closure receipt's exact qualification cut;
- change `Waived`/`Compensated`/`Terminated` into `Satisfied` after validation;
- inject or remove obligation, exception, or compensation evidence after closure.

Validated constructors are the only production construction paths for these
positive claims. `Clone` only duplicates an already validated immutable value; it
does not reopen its fields. Compile-fail doctests qualify key external visibility
boundaries.

Candidate/domain-input structures such as `QualifiedInputRef`, `AllocationStatement`,
and adapter-supplied enum interpretations remain constructible because authoritative
adapters must be able to supply candidate evidence into the kernel. Sealing the
positive derived object does not make Business authoritative for those source facts.

## Exact-reference requirement

A reference placed into a consequential `QualificationCut` must identify the exact
historical domain result intended by the owning semantic profile. A mutable alias
whose referent can change while the identifier remains stable is not sufficient
unless the adapter also binds whatever immutable revision/content identity the
owning domain requires.

The generic Business core deliberately does not invent a universal `version: u64`
for authorization, reconciliation, allocation, disposition, compensation, exception,
retry, or attempt-outcome results: some domains may use immutable action hashes,
content-addressed IDs, monotonic revisions, or another profile-defined exact
identity. The adapter/domain contract must normalize that into an exact stable
reference before Business composes it.

## Semantic-commitment boundary

`OperationCommitment` is opaque to Business. The owning domain/profile defines
which fields are material, how they are canonicalized, and how the commitment is
produced. Business does not recompute or bless that commitment.

Once supplied, however, the commitment is part of `CommittedIntent`, so reusing a
logical intent ID after changing amount, beneficiary, destination, purpose,
operation-profile version, or another material fact cannot carry an existing
`AuthorizationBinding`, retry-safety binding, or compensation identity unless the
owning profile supplied the exact same committed operation.

## Authorization provenance boundary

Production `AuthorizationBinding::bind` does not accept a caller-selected
`ValidityWindow`.

It receives:

- the domain-scoped authorization decision identity;
- the exact committed operation;
- the exact `QualifiedInputRef` supplied by the owning authorization adapter/profile
  as the decision result;
- the exact `QualificationCut`.

The kernel requires that:

1. the decision identity is present in the cut;
2. the exact decision source is present in the cut;
3. the decision source belongs to the same authoritative domain as the decision;
4. the source's validity positively includes the explicit qualification time; and
5. every other prerequisite in the cut is also positively current.

Business derives only:

`effective_valid_until = min(decision_source.valid_until, all required cut validity)`

There is no separate production argument from which Business can manufacture or
widen an authorization lease.

The owning authorization domain/profile is still responsible for the substantive
correspondence between its decision identity and the exact source record it supplies.
The generic kernel preserves that explicit pair and its provenance; it does not
invent a universal rule saying how every domain must encode authorization records.

### Execution-time freshness boundary

`effective_valid_until` is an upper bound derived from the exact cut's declared
validity windows. It is **not** proof that an authority source cannot be revoked,
superseded, or contradicted before that natural horizon unless the owning profile
explicitly gives the source that semantic guarantee.

A profile that requires current revocation/supersession state for an irreversible
effect must construct a fresh `AuthorizationBinding` from a cut qualified at the
action's relevant time boundary. Business cannot refresh authority by merely
advancing a timestamp on an old binding.

## Retry and attempt-outcome provenance boundary

Production retry permission is never granted by a naked positive enum.

`AttemptOutcomeBinding` retains:

- the exact `AttemptRef`;
- the exact `CommittedIntent`;
- the owning adapter/profile's interpreted `AttemptOutcome`;
- the exact `QualifiedInputRef` supporting that interpretation.

Only a current provenance-bound `ProvenNotApplied` result may qualify retry without
additional duplicate-safety evidence.

For `OutcomeUnknown`, production additionally requires a `RetrySafety` binding for
the exact same committed operation, backed by its own exact current domain result.
Both bindings are revalidated against the cut at retry-decision time. Therefore a
retry-safety proof cannot float to a changed amount/beneficiary/profile, another cut,
or past the supporting result's validity horizon.

The generic kernel still does **not** decide whether an owning adapter correctly
interpreted a provider/domain result as `ProvenNotApplied`, `OutcomeUnknown`, or
duplicate-safe. It proves exact provenance and non-substitution, not the substantive
external-world meaning.

## Allocation-conservation boundary

`AllocationStatement` is candidate/domain-owned input. It remains constructible.

`validate_conservation()` checks unit consistency, zero consequential lines,
arithmetic overflow, `allocated <= declared source capacity`, and exact remainder.
Its positive `AllocationSummary` is sealed and retains an immutable clone of the
exact statement it validated.

A caller therefore cannot validate statement A and present the resulting summary as
proof for a later-mutated statement B. The generic kernel still does not establish
that the declared capacity is real or that the allocation is institutionally
legitimate.

## Obligation-disposition boundary

`ObligationDispositionBinding` prevents Business closure from consuming a naked
`ObligationDisposition` in production.

Each binding retains:

- the exact domain-scoped obligation identity;
- the disposition asserted by the owning adapter/profile;
- the exact `QualifiedInputRef` from which that adapter/profile derived the disposition.

The source result must:

1. belong to the same authoritative domain as the obligation;
2. be present in the exact `QualificationCut` used by the closure receipt; and
3. remain valid at the cut's explicit qualification time.

Bindings are revalidated when a receipt is built, so a binding created against one
cut cannot be replayed into another cut after its source result has disappeared.
Duplicate disposition bindings for one obligation are rejected.

This still does **not** make Business an obligation oracle. The owning domain/profile
is responsible for interpreting its source result and establishing that it really
maps to `Satisfied`, `Waived`, `Terminated`, or another disposition. Business proves
provenance and composition, not the substantive domain interpretation.

## Compensation provenance boundary

The normative corpus defines compensation as a **distinct logical effect** and
`Compensated` closure as a prior adverse/partial outcome addressed through
**completed** compensating duties/actions. A proposed refund/remediation intent is
therefore not enough by itself.

Production `CompensationBinding` retains:

- the full `CommittedIntent`, including operation-profile version and material
  semantic commitment; and
- the exact `QualifiedInputRef` supplied by the owning adapter/profile as the
  supporting compensation result.

That exact result must be present in the same `QualificationCut` used by the final
receipt, and the binding is revalidated when the receipt is constructed. A binding
created against cut A therefore cannot float into cut B after its supporting result
has disappeared. Stale supporting results fail through normal exact-cut freshness
qualification.

Business still does **not** decide whether that result substantively proves the
compensating action completed or whether the compensation is legally/economically
adequate. The owning domain/profile establishes those meanings. The generic kernel
prevents a bare intent or semantically weakened logical ID from being used as proof
of completed compensation.

`WorkflowClosureReceipt::compensating_intents()` preserves the historical semantic
label for migration/readability, but it exposes an immutable set of
`CompensationBinding` values rather than bare `LogicalIntentRef`s.

## Exception provenance and exact retention

Retained closure/dispute exceptions are domain-scoped. Equal local identifiers from
different authoritative domains remain distinct:

`finance/exception:1 != commerce/exception:1`

Production closure does not accept a naked exception reference. `ExceptionBinding`
retains:

- the exact `DomainExceptionRef`; and
- the exact `QualifiedInputRef` from which the owning adapter/profile derived that
  retained exception.

The source must belong to the same authoritative domain and be present in the exact
cut. Bindings are revalidated when the receipt is constructed, and duplicate
bindings for one exception are rejected.

The bound exception key set must then exactly equal `QualificationCut::exceptions()`.
A receipt therefore cannot hide an exception that qualified the cut, invent one
absent from the cut, or collapse equal local exception IDs from different domains.

This proves provenance and exact retention only. Whether the source really means the
claimed exception and whether a closure policy is substantively allowed to terminate
with it remain policy/domain responsibilities.

## What it does not enforce

This crate does **not** decide:

- whether a party is truly identified;
- whether a `PowerGrant` is valid;
- whether an authorization decision is substantively correct;
- whether an authorization adapter correctly associates its decision identity with its exact source record;
- whether an owning domain computed an operation commitment correctly;
- whether an attempt-result adapter correctly interprets an exact result as `ProvenNotApplied`, `OutcomeUnknown`, or another outcome;
- whether a retry-safety adapter correctly establishes duplicate application as impossible or acceptable;
- whether a payment actually settled;
- whether inventory really exists;
- whether work was actually performed;
- whether an external observation is authentic;
- whether a domain-specific allocation is legitimate or its declared source capacity is true;
- whether an owning adapter correctly maps its exact result to an obligation disposition;
- whether a compensation result really proves completion or institutional adequacy of the compensating effect;
- whether an exception adapter correctly maps its exact result to the retained exception;
- which obligations a closure policy should require;
- whether a particular closure policy's business predicates are substantively satisfied;
- whether a particular exact exception set is substantively permitted by a closure policy.

Those remain the responsibility of the authoritative domains and their qualified
policies. This crate can preserve and structurally validate their references and
composition, but it cannot mint their truth.
