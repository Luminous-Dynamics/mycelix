# mycelix-governance-authority

Institution-bound governance signing and execution authority for Mycelix.

## Security question this crate answers

A cryptographically valid signature proves that a key signed something. It does **not** prove that the key or committee had institutional authority to govern.

This crate makes that authority explicit and fail-closed.

A proposal binds:

- institution;
- jurisdiction;
- exact rulebook identity + digest;
- governing body;
- action class;
- exact executable action digest;
- exact signing-policy ID + digest;
- proposal lifetime.

The signing policy binds:

- the same institution, jurisdiction, rulebook, and governing body;
- exactly one authorized committee;
- exact committee key digest;
- committee epoch;
- majority-or-stronger signer threshold;
- member count;
- allowed action classes;
- allowed signature algorithms;
- PQ-required policy where applicable;
- validity window;
- institutional authority grant + proof reference that adopted the policy.

The threshold verifier then emits evidence binding the signature to the proposal, signing policy, committee, key digest, epoch, exact action digest, signer count, algorithm, and verification proof.

Finally, execution uses a short-lived `ExecutionAttempt` with a non-zero freshness nonce. Only when all four objects agree does `evaluate_governance_authority()` produce an `ExecutionPermit`.

## Central invariant

> A committee is not governance authority merely because its signature verifies.

An arbitrary self-created committee fails because it is not the committee named by the institution-approved signing policy. A stale committee fails because its epoch or key digest differs. A valid committee signing the wrong action plan fails because the exact action digest differs.

## Fail-closed checks

The evaluator rejects:

- wrong institution;
- wrong jurisdiction;
- wrong rulebook/version/digest;
- wrong governing body;
- wrong or rebound signing policy;
- action class outside policy scope;
- arbitrary committee ID;
- wrong committee key digest;
- stale/wrong epoch;
- signer count below policy threshold or above committee size;
- policy thresholds below a simple majority protocol floor;
- disallowed signature algorithms;
- classical-only verification when PQ is required;
- wrong proposal ID;
- wrong action digest;
- wrong signature reference;
- expired proposal/policy/execution attempts;
- signature or verification timestamps outside the authority lifetime;
- replay-prone all-zero execution nonce.

## Trust boundary

This crate deliberately does **not** pretend that strings are cryptographic proof.

Integrating code must verify:

1. `SigningPolicy.authorized_by` and `policy_proof_ref` trace to a real institutional governance decision/authority grant;
2. `ThresholdVerificationEvidence.verification_proof_ref` represents actual cryptographic verification against the key whose digest is bound by policy;
3. the execution nonce is checked against replay state / a Xenia capability transcript;
4. proposal authority-context fields cannot be mutated outside the governed proposal-amendment process.

## Relationship to PRs #31 and #32

This crate is designed as a stacked follow-on to the institutional primitives in PR #31 and the fail-closed containment in PR #32.

PR #32 should remain fail-closed until the governance Holochain adapters can provide:

`ProposalAuthorityContext`
→ verified `SigningPolicy`
→ cryptographic `ThresholdVerificationEvidence`
→ fresh `ExecutionAttempt`
→ `ExecutionPermit`
→ side effect

That sequence is the bridge from institutional legitimacy to cryptographic authorization without allowing either layer to silently substitute for the other.
