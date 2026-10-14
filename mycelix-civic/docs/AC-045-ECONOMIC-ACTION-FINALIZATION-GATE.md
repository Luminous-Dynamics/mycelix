# AC-045 — Economic Action Finalization Gate

## Purpose

AC-045 creates a clean close-out boundary for an economic action after execution
and lifecycle evidence have been recorded.

The lifecycle reaching `Completed` is not, by itself, a sufficient finalization
certificate. Finalization must establish that the action's explicit execution
controls were satisfied, the completion event itself is evidenced, the current
substrate/reciprocity state is clean, and no unresolved impact exposure remains.

The gate is an assessment only. It does not mutate the lifecycle or erase failed
reconciliations.

## Invariants

### Normal completion

Only `EconomicActionStage::Completed` is eligible for clean finalization.

A terminated action is not silently treated as successfully completed.

### Explicit completion evidence

At least one execution constraint of kind `Completion` must exist, and it must
reference the current Completed lifecycle revision.

The corresponding reconciliation must be `Conformant`.

### Complete execution reconciliation

Every supplied required execution constraint must have at least one matching
reconciliation.

Every matching reconciliation must be conformant. A later conformant result
cannot hide an earlier non-conformant result in the append-only history.

This deliberately treats failed execution as durable evidence. Corrective
execution may be represented with new, explicitly governed records rather than
rewriting the failed record.

### Historical execution is preserved

Execution controls for payment, delivery, and milestones may reference an earlier
Contracted or Implementation lifecycle revision.

Finalization validates those constraints against the immutable lifecycle history
rather than falsely requiring every historical execution record to use the
current Completed revision.

### Exact authorization binding

Each required constraint must match its referenced lifecycle revision's:

- action reference;
- scope ID;
- scope fingerprint.

Execution kinds are also checked against the stage of the referenced revision.

### Clean integrity state

The current AC-017/AC-018 scoped assessment must be exactly `Allowed`.

Warnings, insufficient evidence, ordinary blocks, and emergency escalation do not
produce a clean finalization certificate.

### No unresolved substrate-impact exposure

Open depletion impacts, remediation still in progress, and blocking restoration
obligations remain visible and prevent clean finalization.

There is no compensation rule in which a healthy substrate dimension offsets an
unresolved impact elsewhere.

## Decision ordering

The gate is fail-closed and deterministic:

1. emergency escalation;
2. invalid/non-completed lifecycle;
3. missing completion or other required controls;
4. non-conformant execution evidence;
5. unresolved impact exposure;
6. non-clean integrity state;
7. ready.

The assessment retains the IDs needed to explain the selected decision without
altering the underlying ledgers.

## Relationship to existing architecture

AC-025 composes substrate and reciprocity gates.

AC-035 binds lifecycle history to a stable action and scope.

AC-036 binds scope contents to a versioned fingerprint.

AC-039 binds execution receipts to exact lifecycle authorization.

AC-041 reconciles execution receipts against explicit constraints.

AC-043 and AC-044 harden reconciliation chronology and persisted ledger shape.

AC-045 is the first close-out boundary that composes those controls into a
single deterministic finalization eligibility decision.

## External interoperability research

The design seam is consistent with the shape of established public-sector
contracting data without claiming semantic equivalence.

The Open Contracting Data Standard (OCDS) models one contracting process across
tendering, awarding, contracting and implementation, linked by a stable
contracting-process identifier. Its implementation stage includes payments,
progress updates, extensions, amendments, and completion or termination
information. This supports retaining one stable action reference while keeping
stage-specific evidence explicit.

OCDS also uses releases and records to preserve a change history, and distinguishes
updates from formal amendments. That is directly analogous to Mycelix's append-only
lifecycle history and explicit scope-amendment path.

SEEA Ecosystem Accounting is complementary rather than a lifecycle model: its
framework organizes ecosystem condition, extent, services, and asset information
and links environmental change to economic and other human activity. AC-045
therefore consumes substrate/impact state as typed evidence and gate inputs rather
than turning ecological accounting into a fungible financial score.

## Non-goals

AC-045 does not:

- define procurement law;
- create universal monetary prices for ecological or social conditions;
- invent execution tolerances;
- assert that the known-impact set is omniscient;
- rewrite non-conformant evidence;
- make termination equivalent to successful completion.

## Test coverage

The reference tests cover:

- clean completion with a historical delivery reconciliation;
- missing completion control;
- missing reconciliation;
- an earlier non-conformant reconciliation that cannot be hidden by a later
  conformant one;
- unresolved impact exposure;
- warning-state integrity blocking clean finalization;
- non-completed lifecycle rejection;
- completion constraints bound to a historical revision being rejected.

## Research references

- Open Contracting Data Standard 1.1.5, “How does the OCDS work?”:
  https://standard.open-contracting.org/latest/en/primer/how/
- Open Contracting Data Standard 1.1.5, “How is OCDS data published?”:
  https://standard.open-contracting.org/latest/en/primer/releases_and_records/
- Open Contracting Data Standard 1.1.5, “Updates and amendments”:
  https://standard.open-contracting.org/latest/en/guidance/map/amendments/
- United Nations System of Environmental-Economic Accounting, “Introduction to
  SEEA Ecosystem Accounting”:
  https://seea.un.org/en/Introduction-to-Ecosystem-Accounting
