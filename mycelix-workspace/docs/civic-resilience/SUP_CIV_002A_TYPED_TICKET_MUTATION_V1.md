# SUP-CIV-002A — Typed support-ticket mutation and lifecycle contract v1

Status: semantic contract only; stacked and blocked on SUP-CIV-001A qualification  
Parent subject: `9a5148b9927c630d4073d486f464824fb429d8c6`  
Tracking issue: #2049  
Hardening program: #2030  
Civic discovery: #2028

## Purpose

Freeze the Civic-safe mutation waist for Commons Support before any Civic Resilience adapter is allowed to mutate a support ticket.

The current generic coordinator accepts a whole replacement `SupportTicket`. That remains existing Support behavior until separately migrated; this contract does not pretend otherwise. It defines the stricter surface that a future CIV-RES adapter must use.

## Core theorem

```text
validated stable ticket identity
+ exact expected revision
+ one typed command
+ command-specific authority/policy evidence
+ explicit transition rule
= one reviewable operational mutation
```

not:

```text
caller-supplied replacement record = authorized transition
caller-supplied to_status = lifecycle authority
last write seen = current state
```

## Typed command vocabulary

The closed v1 command vocabulary is:

```text
StartWork
RequestUserInput
ResumeWork
MarkOperationallyResolved
CloseOperationalTicket
ReopenOperationalTicket
AssignTicket
ChangeOperationalPriority
ChangeSupportCategory
AmendDescriptiveMetadata
```

A runtime implementation may use Rust enum names matching these tokens. Adding a new command changes the contract and requires a new qualified subject.

There is deliberately no generic `SetStatus`, `ReplaceTicket`, `PatchArbitraryFields`, or caller-selected `to_status` command.

## Lifecycle transition relation

The closed v1 status relation is:

```text
Open         + StartWork                 -> InProgress
Open         + RequestUserInput          -> AwaitingUser
InProgress   + RequestUserInput          -> AwaitingUser
AwaitingUser + ResumeWork                -> InProgress
Open         + MarkOperationallyResolved -> Resolved
InProgress   + MarkOperationallyResolved -> Resolved
AwaitingUser + MarkOperationallyResolved -> Resolved
Resolved     + CloseOperationalTicket    -> Closed
Resolved     + ReopenOperationalTicket   -> InProgress
Closed       + ReopenOperationalTicket   -> InProgress
```

All unlisted status/command pairs are refused by default.

The relation is operational Support workflow semantics only. It does not encode municipal administrative procedure or real-world completion truth.

## Exact-base requirement

Every typed mutation must bind an exact expected ticket revision reference.

```text
StableSupportTicketId
!= ExpectedRevisionRef
```

The stable ID selects the logical ticket. The expected revision identifies the state against which the caller formed the command.

A later executable/currentness tranche must refuse a mutation when the expected revision is stale, divergent, unrelated to the stable identity, or not the admitted current revision.

```text
stale base -> explicit conflict/refusal
```

No last-write-wins assumption is admitted by this contract.

## Immutable identity-bound fields

The Civic-safe mutation surface must never rewrite:

```text
requester
created_at
```

These are identity-bound intake facts under SUP-CIV-001A.

A correction to historical intake facts must be represented as new provenance/amendment evidence under an owning procedure, not destructive identity rewriting.

## Historical intake provenance

`title`, `description`, and `category` may require operational correction, but current values must not erase the original intake evidence.

```text
CurrentDescription != OriginalIntakeDescription
CurrentCategory != OriginalIntakeCategory
```

`AmendDescriptiveMetadata` and `ChangeSupportCategory` therefore mean append-only attributable changes projected into current operational state, not history deletion.

## Command write-sets

A runtime implementation must enforce narrow command-specific write-sets.

### StartWork

May change only operational status according to the transition relation plus derived update/provenance metadata.

### RequestUserInput

May change only operational status according to the transition relation plus derived update/provenance metadata.

### ResumeWork

May change only operational status according to the transition relation plus derived update/provenance metadata.

### MarkOperationallyResolved

May change only operational status according to the transition relation plus resolution-event provenance.

It does not write Civic completion verification.

### CloseOperationalTicket

May change only operational status according to the transition relation plus closure-event provenance.

It does not create administrative finality, external municipal closure, payment authority, or outcome evidence.

### ReopenOperationalTicket

May change only operational status according to the transition relation plus reopening provenance. Prior resolved/closed history remains preserved.

### AssignTicket

May change only assignment-related operational state and provenance.

```text
Assignee != ExecutionAuthority
Assignee != InstitutionalAuthority
```

### ChangeOperationalPriority

May change only Support-domain priority and provenance.

```text
TicketPriority != CivicPriorityAuthority
```

### ChangeSupportCategory

May change only current Support classification and provenance while retaining original intake classification.

```text
SupportCategory != UniversalCivicTaxonomy
```

### AmendDescriptiveMetadata

May amend current human-readable operational description/title under explicit provenance while retaining original intake evidence.

It cannot change requester, created_at, stable identity, status, assignment, priority, autonomy authority, or completion evidence.

## Caller-controlled timestamp refusal

The Civic-safe path must not treat a freely supplied `updated_at` as trusted chronology.

The future runtime must derive mutation chronology from action/event evidence and document how any retained legacy `updated_at` projection relates to that evidence.

```text
caller supplied updated_at != trusted currentness
```

## Operational resolution boundary

`Resolved` and `Closed` remain Support workflow states.

```text
TicketStatus::Resolved != CompletionVerification
TicketStatus::Resolved != BeneficiaryAcceptance
TicketStatus::Resolved != ExternalMunicipalClosure
TicketStatus::Resolved != OutcomeImprovement
TicketStatus::Closed != CompletionVerification
TicketStatus::Closed != AdministrativeFinality
TicketStatus::Closed != ExternalMunicipalClosure
TicketStatus::Closed != PaymentAuthority
TicketStatus::Closed != OutcomeImprovement
```

CIV-RES-001C owns the separate completion-claim/evidence/verification boundary.

## Missing closure semantics

The current status enum has no explicit `Cancelled`, `Duplicate`, `Rejected`, or `Transferred` state.

This contract refuses to overload `Closed` as a synonym for those meanings:

```text
Closed != Cancelled
Closed != Duplicate
Closed != Rejected
Closed != Transferred
```

A later tranche must decide whether these are typed closure reasons, status extensions with migration semantics, or external administrative procedure states.

## Authority boundary

A typed command still needs the authority/policy evidence appropriate to its domain. The command type itself is not authority.

```text
TypedCommand != AuthorityToMutate
SupportCapability != MunicipalAuthority
Assignment != AuthorityToExecuteExternalEffect
```

Civic institutional legitimacy and consequential effect authority remain externally owned.

## Legacy API boundary

The existence of this stricter contract does not retroactively make the existing generic whole-record `update_ticket` path conformant.

A future runtime tranche must either:

- add a new typed Civic-safe surface while clearly marking legacy update behavior as non-Civic; or
- migrate/deprecate the generic mutation API under a separately qualified compatibility plan.

CIV-RES must consume only the qualified typed surface.

## Required runtime refusals

At minimum, the executable tranche must prove:

- arbitrary `Open -> Closed` refusal;
- caller-selected `to_status` absent/refused;
- requester rewrite refused;
- created_at rewrite refused;
- status+priority combined arbitrary patch refused;
- backdated/free `updated_at` cannot establish chronology;
- stale expected revision refused;
- unrelated revision/stable-ID pair refused;
- `Resolved` cannot satisfy CIV completion verification;
- `Closed` cannot satisfy administrative finality;
- `Critical` priority cannot establish public priority authority;
- assignment cannot establish execution authority;
- reopen preserves prior resolved/closed provenance.

## Required non-equivalences

```text
WholeRecordReplacement != AuthorizedSemanticTransition
CallerSelectedToStatus != LifecycleAuthority
StableSupportTicketId != ExpectedRevisionRef
LatestSeenRevision != ProvenCurrentState
TicketStatus::Resolved != CompletionVerification
TicketStatus::Resolved != BeneficiaryAcceptance
TicketStatus::Resolved != ExternalMunicipalClosure
TicketStatus::Resolved != OutcomeImprovement
TicketStatus::Closed != CompletionVerification
TicketStatus::Closed != AdministrativeFinality
TicketStatus::Closed != ExternalMunicipalClosure
TicketStatus::Closed != PaymentAuthority
TicketStatus::Closed != OutcomeImprovement
TicketPriority != CivicPriorityAuthority
SupportCategory != UniversalCivicTaxonomy
Assignee != ExecutionAuthority
Assignee != InstitutionalAuthority
TypedCommand != AuthorityToMutate
SupportCapability != MunicipalAuthority
Closed != Cancelled
Closed != Duplicate
Closed != Rejected
Closed != Transferred
CurrentDescription != OriginalIntakeDescription
CurrentCategory != OriginalIntakeCategory
CallerSuppliedUpdatedAt != TrustedCurrentness
```

## Deferred theorems

This tranche deliberately defers:

- executable command implementation;
- stable-identity runtime implementation;
- revision lineage/current-state admission;
- stale index repair and current index projection;
- exact closure-reason extension/migration;
- autonomy/effect authority hardening;
- CIV-RES adapter implementation;
- completion verification runtime;
- municipal administrative procedure binding.

## Qualification dependency

This subject is stacked on SUP-CIV-001A head `9a5148b9927c630d4073d486f464824fb429d8c6`.

A green SUP-CIV-002A qualifier cannot create qualification inheritance. The contract remains blocked for promotion if SUP-CIV-001A is not itself qualified under its dedicated exact-subject qualifier.

## Claim ceiling

A PASS may establish only that this exact semantic subject freezes the closed command vocabulary, closed operational transition relation, immutable/provenance boundaries, stale-base requirement, authority/completion non-equivalences, and runtime refusal obligations.

It does not establish runtime enforcement, current-state correctness, ticket authorization, completion truth, administrative finality, civic priority legitimacy, municipal authority, privacy compliance, Johannesburg readiness, or deployment readiness.
