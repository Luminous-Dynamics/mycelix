# Binding Governance Execution Preflight Invariants

Status: **normative pre-merge contract for PR #59**

The pure preflight kernel is necessary but not sufficient until the Holochain execution coordinator resolves every input itself and runs it at both authority boundaries.

## Required runtime chain

`mark_timelock_ready` and `execute_timelock` MUST each independently perform:

1. resolve the exact current `Timelock` and recompute its execution action digest;
2. resolve `proposal_authority.get_verified_proposal_authority_context(proposal_id)`;
3. reject missing, stale, expired, ambiguous, or differently-digested proposal authority;
4. resolve `constitution_transition.get_verified_current_constitution()`;
5. reject any constitution response whose statement digest does not recompute exactly;
6. resolve the proposal's **constitutional epoch binding** — the exact constitution statement digest under which the proposal became binding-authoritative;
7. require the current verified constitution digest to equal that bound digest;
8. resolve `binding_voting.get_verified_binding_tally(proposal_id)`;
9. require the tally to have been verified under the exact same proposal-authority binding and constitutional epoch;
10. construct `ExecutionPreflightInput` internally and require `authorize_execution_preflight` to return a permit;
11. verify threshold authorization for the exact action digest and institutionally authorized signing policy;
12. only then promote Ready or, on the second independent pass, perform side effects.

No caller-supplied `verified`, `eligible`, `approved`, current-constitution, authority-binding, tally-binding, or committee-legitimacy boolean is sufficient.

## Constitutional epoch rule

**A proposal does not automatically survive a constitutional transition merely because institution and rulebook identifiers still compare equal.**

When a proposal crosses into binding authority, the runtime must persist or otherwise immutably verify the exact current `ConstitutionStatement` digest. This is the proposal's constitutional epoch.

If the constitution advances before Ready promotion or before execution:

- the old preflight becomes stale;
- Ready promotion or execution MUST deny;
- an existing Ready record does not grandfather execution;
- the proposal must follow an explicit rulebook-defined reauthorization/rebinding process if carry-forward is intended.

This prevents a proposal authorized under constitution N from executing under constitution N+1 after rights, amendment policy, parameter set, charter semantics, or another constitutional field changes while retaining the same superficial rulebook ID/version.

## Two-pass rule

A successful Ready-time preflight is not a lease for later execution.

Execution MUST repeat the full current-constitution, proposal-authority, binding-tally and threshold checks immediately before external effects. Any intervening constitutional change, authority expiry/revocation, tally invalidation, signing-policy epoch change, or action-byte change denies.

## Failure semantics

All dependency failures are denial:

- missing zome or function;
- network/call error;
- decode error;
- absent record;
- ambiguous record set;
- digest/profile mismatch;
- stale/expired authority;
- constitution epoch mismatch;
- verifier outage;
- receipt mismatch;
- unsupported signature algorithm;
- replay/finality ambiguity.

There is no Phi/reputation/stake/model-score fallback and no legacy-voting fallback.

## Adversarial tests required before merge-ready

At minimum:

- constitution changes after proposal activation but before Ready;
- constitution changes after Ready but before execution;
- rulebook ID/version unchanged but charter/parameters/amendment-policy digest changes;
- proposal action bytes change after tally;
- tally belongs to another proposal-authority binding;
- proposal authority expires between Ready and execution;
- signing policy epoch/key changes between Ready and execution;
- verifier outage at either pass;
- stale locally cached constitution while the verified lineage has advanced;
- duplicate/replayed execution attempt after a successful effect.
