# SUP-CIV-001A — Stable support-ticket identity / intake-lineage contract v1

Status: semantic composition contract only  
Parent subject: `main@a85369699099d4c7524e502e531735eed4ab36f4`  
Tracking issue: #2044  
Parent discovery: SUP-CIV-000 / #2030  
Civic dependency: CIV-RES-002A0 / #2028

## Purpose

Freeze a migration-safe identity theorem for the existing Commons Support ticket family before Civic Resilience consumes it as an operational substrate.

The current `SupportTicket` is already a serialized Holochain entry type. SUP-CIV-001A therefore refuses the tempting but unsafe shortcut of adding a new required logical-ID field directly to that legacy payload and pretending migration has been solved.

Instead, the contract defines a separate immutable canonical identity entry whose content-addressed `EntryHash` is the stable logical support-ticket identifier.

This tranche changes no Rust, Holochain DNA, zome behavior, authorization, municipal semantics, or Civic authority.

## Current-main facts

The source audited at `a85369699099d4c7524e502e531735eed4ab36f4` establishes the following relevant facts:

- `SupportTicket` contains title, description, category, priority, status, requester, assignee, autonomy metadata, system information, preemptive/prediction metadata, and timestamps;
- ticket creation returns a Holochain action-backed record;
- ticket update creates another action revision;
- no separate stable logical ticket ID is present in the serialized `SupportTicket` payload;
- existing status/category/requester links are operational indexes, not a logical identity theorem;
- `TicketStatus`, `TicketPriority`, `AutonomyLevel`, `AutonomousAction` and prediction fields are Support-domain semantics only.

These facts motivate the identity split. They do not establish that the current Support system is unsafe for its existing use case.

## Identity theorem

The v1 canonical relation is:

```text
one exact genesis SupportTicket create action
        |
        v
CanonicalSupportTicketIdentity
        |
        v
StableSupportTicketId := EntryHash(CanonicalSupportTicketIdentity)
```

The stable ID is therefore:

- content-addressed;
- immutable for one exact canonical identity payload;
- distinct in type and meaning from mutable ticket `ActionHash` revisions;
- derivable without changing the legacy `SupportTicket` serialization;
- suitable as the logical join key for later revision/current-state adapters.

## Canonical identity payload

The v1 payload is deliberately minimal:

```text
CanonicalSupportTicketIdentity {
  genesis_ticket_action_ref,
  requester_ref,
  intake_created_at_ref
}
```

A future runtime implementation may represent these refs with native Holochain types, but it must preserve their meaning.

### Genesis ticket action

`genesis_ticket_action_ref` must identify an actual `Create` action whose application entry is a `SupportTicket`.

It must not point to:

- an update action;
- a delete action;
- a comment;
- an autonomous action;
- a preemptive alert;
- an unrelated application entry;
- an arbitrary caller-supplied hash with no validated ticket record.

### Requester binding

`requester_ref` must equal the requester encoded in the referenced genesis `SupportTicket`.

The identity layer does not independently decide whether that requester field was authorized at intake. Intake authorization remains a separate theorem.

### Intake time binding

`intake_created_at_ref` must equal the `created_at` value encoded in the referenced genesis `SupportTicket`.

It is an identity-binding fact, not trusted wall-clock truth by itself.

## Canonicality

For one exact valid genesis ticket, the canonical identity payload is deterministic.

Therefore:

```text
same canonical payload -> same EntryHash
```

A duplicate create of the same identity content may create another action that stores the same entry, but the logical `StableSupportTicketId` remains the same entry hash.

This avoids requiring a mutable global uniqueness registry merely to establish logical identity.

It does **not** prove uniqueness of human meaning, deduplicate semantically similar tickets, or prevent two distinct genesis tickets from describing the same real-world problem.

## Immutability

`CanonicalSupportTicketIdentity` is append-only immutable identity material.

A later executable implementation must reject updates that attempt to mutate an identity entry.

If a correction is required because the original intake record was wrong, the correction belongs in ticket/revision/procedure semantics. It must not rewrite the logical identity payload after the fact.

```text
identity correction request != mutate identity history
```

## Legacy compatibility

Existing tickets created before identity support may have no canonical identity entry.

The only valid states are explicit:

```text
identity present and validated
legacy/unmigrated identity absent
migration attempt failed/refused
```

A consumer must not silently synthesize a successful identity migration from a current action hash.

A later migration procedure may create the canonical identity for a legacy genesis ticket only after resolving the exact genesis action and validating the canonical payload.

```text
LegacyTicketWithoutIdentity != MigratedTicket
```

## Revision boundary

SUP-CIV-001A freezes **identity**, not revision lineage/currentness.

A later layer may bind update revisions to the stable identity, but it must prove that relation independently.

In particular:

```text
LinkToIdentity != ValidatedLineage
LatestSeenRevision != ProvenCurrentState
UpdateAction != StableSupportTicketId
```

A link/index is a discoverability mechanism until its semantic validation theorem is established.

## Status / completion boundary

Stable ticket identity says nothing about operational or civic completion.

```text
StableSupportTicketId != TicketStatus
TicketStatus::Resolved != CompletionVerification
TicketStatus::Closed != CompletionVerification
TicketStatus::Closed != AdministrativeFinality
TicketStatus::Closed != ExternalMunicipalClosure
TicketStatus::Closed != OutcomeImprovement
```

CIV-RES-001C remains the semantic owner of completion claim/evidence/verification separation for Civic Resilience.

## Priority boundary

The existing Support `TicketPriority::{Low,Medium,High,Critical}` remains operational metadata.

```text
TicketPriority != CivicPriorityAuthority
```

The stable identity layer neither freezes nor legitimizes a public allocation/triage decision.

## Category boundary

The current Support category enum is IT/support oriented.

```text
SupportCategory != UniversalCivicTaxonomy
```

Stable identity does not make that enum a municipal ontology and does not require expanding it in this tranche.

## Autonomy / execution boundary

The identity layer grants no meaning to caller-visible support autonomy/action fields beyond their existing domain.

```text
AutonomyLevel::FullAutonomous != CivicAuthority
AutonomousAction.approved != InstitutionalAuthority
AutonomousAction.executed != ConfirmedExternalEffect
AutonomousAction.success != OutcomeEffect
PredictionConfidence != ObservationTruth
```

SUP-CIV-001A does not authorize Symthaea or any other actor to take consequential civic actions.

## Civic boundary

A stable Support identity may later become an operational reference inside a Civic Resilience adapter.

It is never sufficient by itself to establish:

- municipal jurisdiction;
- public authority;
- service entitlement;
- resident identity;
- legal basis;
- prioritization legitimacy;
- completion verification;
- payment authority;
- administrative finality;
- outcome improvement;
- causal effect.

## Required non-equivalences

The closed v1 registry is:

```text
SupportTicketActionHash != StableSupportTicketId
StableSupportTicketId != CurrentTicketState
StableSupportTicketId != TicketStatus
StableSupportTicketId != CivicAuthority
IdentityEntry != CurrentRevision
IdentityEntry != CompletionVerification
IdentityEntry != AdministrativeFinality
IdentityEntry != ExternalMunicipalClosure
IdentityEntry != OutcomeImprovement
IdentityEntry != CausalEffect
LegacyTicketWithoutIdentity != MigratedTicket
LinkToIdentity != ValidatedLineage
LatestSeenRevision != ProvenCurrentState
TicketPriority != CivicPriorityAuthority
SupportCategory != UniversalCivicTaxonomy
AutonomyLevel::FullAutonomous != CivicAuthority
AutonomousAction.approved != InstitutionalAuthority
AutonomousAction.executed != ConfirmedExternalEffect
AutonomousAction.success != OutcomeEffect
PredictionConfidence != ObservationTruth
```

## Required executable refusals for SUP-CIV-001B

The implementation tranche must demonstrate at least:

1. canonical identity can be created for a valid genesis support ticket;
2. canonical identity resolves to the same entry hash for the same canonical payload;
3. identity creation refuses an update action presented as genesis;
4. identity creation refuses a non-ticket genesis record;
5. requester mismatch is refused;
6. intake-created-at mismatch is refused;
7. identity entry updates are refused;
8. legacy ticket without an identity returns explicit absence rather than a fabricated ID;
9. existing `SupportTicket` serialization remains unchanged;
10. no ticket status/priority/autonomy/action boolean is converted into Civic authority or completion truth.

## Deferred theorems

The following remain explicitly deferred:

- revision lineage and current-state projection;
- allowed status-transition algebra;
- typed assignment/priority/category changes;
- current index maintenance;
- access/accountability composition;
- Civic adapter refs;
- completion-verification binding;
- administrative procedure binding;
- municipal external-reference reconciliation;
- deployment/privacy profile composition.

## Intended continuation

```text
SUP-CIV-001A  stable identity contract                     <- this tranche
SUP-CIV-001B  executable canonical identity
SUP-CIV-002A  transition algebra / immutable-field rules
SUP-CIV-002B  revision lineage + current-state projection
SUP-CIV-003   current indexes / projections
SUP-CIV-004   autonomy/effect authority split
SUP-CIV-005   narrow CIV-RES adapter
```

The sequence may be refined by evidence. The semantic boundaries above must not be weakened merely to reduce PR count.

## Qualification claim ceiling

A PASS may establish only that this exact contract freezes a migration-safe, content-addressed stable support-ticket identity model; preserves legacy ticket serialization as an explicit constraint; separates identity from revision/currentness/status/authority/completion/outcome semantics; and records the required executable refusals and non-equivalences.

It does not establish runtime identity code, migration success, authorization, transition safety, current-state correctness, Holochain index correctness, Civic adoption, municipal legitimacy, privacy compliance, Johannesburg deployment, or deployment readiness.
