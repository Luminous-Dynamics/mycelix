# Mycelix Business Institutional Substrate v0.1

Status: **normative candidate / documentation only**

This directory freezes the first semantic contract for building a coherent business experience on top of Mycelix without creating a new authority monolith.

The target is not "another ERP domain." The target is a small institutional substrate that lets existing domain clusters compose while preserving their independent authority boundaries.

## Core rule

> Business orchestration coordinates domain authority. It does not manufacture domain authority.

The Business layer may relate records, retain causality, evaluate workflow closure, and produce projections. It may not become the source of truth for identity, governance, finance, inventory, fulfillment, or other domain-owned facts.

## Documents

- [`INSTITUTIONAL_SEMANTICS.md`](./INSTITUTIONAL_SEMANTICS.md) — the four-plane truth model, primitive vocabulary, temporal semantics, claims, commitments, powers, observations, reconciliation, disputes, and projections.
- [`BUSINESS_CONSTITUTION.md`](./BUSINESS_CONSTITUTION.md) — frozen invariants that every future Business runtime, workflow, UI, adapter, and AI integration must preserve.
- [`DOMAIN_AUTHORITY_MATRIX.md`](./DOMAIN_AUTHORITY_MATRIX.md) — exact ownership rules for cross-domain truth and the rules for ambiguous or externally observed facts.
- [`GOLDEN_PATHS_V0.1.md`](./GOLDEN_PATHS_V0.1.md) — six normative business workflows used to test whether the ontology composes without special-case expansion.
- [`STATE_MACHINES_V0.1.md`](./STATE_MACHINES_V0.1.md) — causal/idempotency identity, obligation/power/observation/workflow lifecycles, typed closure classes, semantic-version preservation, and reference-privacy rules.

## Intended implementation sequence

1. Qualify this normative corpus.
2. Freeze logical-intent / execution-attempt / source-event identity and the state-machine semantics needed by the golden paths.
3. Encode only the transport-neutral vocabulary required by the qualified corpus.
4. Property-test commitment, obligation, power, observation, reconciliation, replay/idempotency, and workflow-closure state machines.
5. Integrate a service-business path using existing Identity, Commerce, Finance, and future Accounting boundaries.
6. Add adversarial qualification before exposing a general Business Cockpit.

## Deliberate non-features

This v0.1 corpus does **not** define or authorize:

- a new Business DNA;
- a generic workflow/BPM language;
- autonomous AI execution;
- legal-contract execution;
- universal payroll or tax engines;
- provider trust policy;
- new payment authority;
- new identity authority;
- new inventory authority;
- new governance authority;
- a production accounting implementation.

## Qualification question

The immediate exit gate is intentionally semantic:

> Can product sale, service sale, procurement, compensation, refund/dispute, and month-end close all be expressed using this vocabulary without introducing another foundational primitive or allowing orchestration to strengthen truth?

The deeper state-machine gate additionally requires that the corpus preserve these non-substitutabilities:

- logical intent != execution attempt != source event;
- obligation satisfaction != waiver != compensation != termination;
- PowerGrant != current operation authorization;
- observation != validated observation != reconciled fact;
- workflow progress != closure class;
- possession of a reference != authority to dereference it;
- representation migration != permission to reinterpret historical meaning.

If either gate fails, the ontology is not frozen and runtime implementation must wait.
