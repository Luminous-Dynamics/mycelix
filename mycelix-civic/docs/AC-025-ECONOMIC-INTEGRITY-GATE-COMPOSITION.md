# AC-025 — Economic Integrity Gate Composition

## Status

Reference hardening tranche for the economics SDK.

Branch: `ac-025-economic-integrity-gate-composition`

Parent: AC-018 mainline

Scope: compose AC-017 substrate integrity and AC-018 impact/reciprocity into one auditable decision boundary.

## Problem

AC-017 and AC-018 intentionally expose separate gates.

That separation is useful: substrate health measures whether the system is consuming the capacities required for future activity, while impact reciprocity measures whether actions have created attributable obligations that have not been repaired.

The separation also creates a composition risk. A caller that evaluates only one layer can accidentally bypass the other:

- a healthy impact ledger must not make a breached substrate look spendable;
- a healthy substrate ledger must not make an unresolved restoration obligation disappear;
- missing evidence in either layer must remain visible;
- emergency escalation must not be silently converted into ordinary approval.

AC-025 adds an explicit composition primitive.

## Primitive

`EconomicIntegrityGate::assess` evaluates:

1. the AC-017 `SubstrateLedger::gate`;
2. the AC-018 `ImpactLedger::gate`;
3. a deterministic combined decision.

The returned `EconomicIntegrityAssessment` preserves both component decisions as well as the combined result. This makes the result inspectable without requiring a caller to reverse-engineer why a transaction was rejected.

## Combined decision

The combination is fail-closed and uses this severity ordering:

`EmergencyEscalationRequired > Blocked > InsufficientEvidence > AllowedWithWarning > Allowed`

The ordering is deliberately monotone: the combined decision can never be more permissive than either component decision.

This is not a claim that these states form a universal moral hierarchy. It is an implementation rule for preventing accidental weakening during composition.

## What this does not decide

AC-025 does not invent policy about which substrate dimensions are relevant to an action.

The caller must still supply the required dimensions. That keeps scope selection a policy/governance question rather than silently turning the reference SDK into a universal economic rule engine.

Likewise, AC-025 does not resolve disputed impacts or establish boundary legitimacy. Those responsibilities remain in AC-018, AC-019, and the surrounding evidence/governance layers.

## Research basis

The composition follows the same separation principle reflected in the UN System of Environmental-Economic Accounting (SEEA): ecosystem accounting is an integrated system of distinct accounts, and individual accounts remain valuable information in their own right rather than being collapsed into one universal indicator. SEEA also explicitly supports use of the accounts in policy and scenario analysis.

That makes a layered architecture preferable to a single "sustainability score":

- evidence can remain typed;
- disagreements can remain visible;
- hard boundaries can remain non-compensable;
- policy can consume several independently meaningful signals.

Relevant background: UN SEEA Ecosystem Accounting, including its core-account model and policy-scenario guidance.

## Invariants

### No cross-layer compensation

A passing substrate gate cannot compensate for a blocking impact gate, and vice versa.

### No hidden evidence failure

An `InsufficientEvidence` or `InsufficientAttribution` result cannot be converted to ordinary allowance because another layer is healthy.

### No silent emergency bypass

Emergency escalation remains explicit even when another component would otherwise allow the action.

### Reason preservation

The assessment retains the exact substrate and impact decisions. Combining states does not erase the underlying cause.

### Warning visibility

Warnings and open impacts remain visible without being silently promoted to a hard block.

### Deterministic composition

Given the same component decisions and action purpose, the combined result is deterministic and independent of caller ordering.

## Validation added

The test suite covers:

- both layers healthy -> allowed;
- hard substrate breach + healthy impacts -> blocked;
- restoration obligation + healthy substrate -> blocked;
- insufficient impact attribution + healthy substrate -> insufficient evidence;
- substrate warning -> visible warning;
- open impact during maintenance -> visible warning;
- emergency state -> explicit escalation;
- duplicate required dimensions -> unchanged result;
- exhaustive component-state combinations -> combined decision is never more permissive than either layer.

## Security posture

AC-025 is intentionally small. It does not create a new economic score, price externalities, choose winners between competing evidence sources, or bypass existing governance.

Its security contribution is compositional: the economic path now has a canonical place where independent integrity constraints can be combined without weakening either one.

The next architectural hardening target should therefore move upward from "two ledgers are checked" toward "the required scope and evidence bundle for a specific action are themselves explicit and tamper-evident." 
