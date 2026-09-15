# mycelix-institutional-core

Portable institutional primitives for authority, evidence, intent, consent, agreements, actions, decisions, challenges, appeals, and remedies.

## Core rule

> **Observation is not authority.**

A score, model output, reputation signal, consciousness measurement, recommendation, anomaly score, or other assessment may support deliberation or satisfy an explicitly declared evidence requirement. It does **not** directly grant a role, capability, vote, veto, execution permission, or other institutional power.

`AuthoritySourceKind` therefore contains only explicit authority-bearing sources:

- credential
- consent
- agreement
- governance decision
- delegation
- bounded emergency mandate

There is intentionally no model-score, reputation, Φ, consciousness, or advisory-signal authority source.

## Evaluation flow

```text
AuthorityGrant + AuthorityRequirement + EvidenceRef[]
                       |
                       v
               evaluate_authority
                       |
        +--------------+--------------+
        |              |              |
        v              v              v
      Allow           Deny      NeedsEvidence
        |
        v
      Intent
        |
        +--> ConsentReceipt / AgreementRef
        |
        v
    ActionRequest
  exact payload digest
  fresh nonce + lifetime
        |
        v
     execution
        |
        v
    ActionEvent
        |
        v
 Decision / Challenge / Appeal / Remedy
```

## Project boundaries

This crate is deliberately wire-neutral and has no Holochain, Xenia, Symthaea, signature, storage, or transport dependency.

- **Mycelix** adapters persist institutional events, resolve rulebooks, credentials, jurisdictions, and governance processes.
- **Xenia** adapters authenticate sessions, bind nonces/transcripts, verify protected key use, and enforce replay-resistant delivery.
- **Symthaea** may produce `AdvisorySignal` and `EvidenceRef` values, but the authority evaluator does not accept advisory signals as authority.

## Important invariants in v0.1

- authority grants are explicit, scoped, rulebook-bound, sourced, and expiring;
- requirements name exact capabilities, roles, jurisdiction, evidence, and rulebook;
- missing evidence is represented explicitly rather than guessed or silently denied;
- action requests bind the exact payload digest that the intent described;
- action requests require a non-zero freshness nonce and cannot outlive the intent;
- consent and agreements bind exact term/scope digests;
- execution events preserve the rulebook, payload digest, evidence, proof reference, and optional predecessor digest;
- advisory confidence is never interpreted as civic weight or authority.

## Relationship to `mycelix-sovereign-access`

`mycelix-sovereign-access` is a focused accountable person-linked lookup protocol. This crate is the generic institutional substrate beneath many domains. The access protocol can later map its `QueryCapability`, authority, receipt, and contest semantics onto these shared primitives without making all institutional actions look like data lookups.

## Next tranches

1. Add canonical serialization/digest profiles.
2. Add append-only `InstitutionEventEnvelope` and deterministic projections.
3. Add Holochain adapters and multi-agent tests.
4. Migrate governance execution to `ActionRequest` + verified `AuthorityGrant` semantics.
5. Migrate credential and recovery workflows to independently authored events.
6. Add rulebook/trust-registry adapters and conformance vectors.

## Validation

Run:

```bash
cargo test --manifest-path crates/mycelix-institutional-core/Cargo.toml
```

The crate is experimental until CI, multi-implementation conformance vectors, and independent security/governance review are complete.
