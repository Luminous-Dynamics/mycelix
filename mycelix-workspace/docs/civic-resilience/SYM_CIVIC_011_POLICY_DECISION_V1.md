# SYM-CIVIC-011 — monotonic policy-decision provenance boundary v1

Status: synthetic research provenance qualification only

Parent: qualified SYM-CIVIC-010 / `6e49198c934ff7d215134c9ccbcd3f3951982128`

Tracking issue: #3872

## Purpose

Qualify the boundary between composed verification evidence and a derived policy decision without allowing the decision layer to silently become authority, scientific truth, or an unreviewable mutable state.

The research separation is:

`EvidenceComposition != PolicyDecision != PolicyVersion != DecisionFinality != Authorization != ScientificTruth != CivicAuthority`

## Standards basis

The current in-toto Attestation Framework specification is v1.2. Its monotonic principle defines a policy as monotonic when ignoring an attestation or field can never turn a DENY into an ALLOW, and it recommends consumers design policies that way. The framework also treats authenticated statements, subjects, predicates, and policy consumption as separate layers. The current SVR v0.2 predicate records point-in-time verified properties and the policies used by a verifier, but does not define a universal downstream policy-decision algebra.

This tranche therefore qualifies a synthetic decision function rather than claiming that in-toto itself defines the civic or operational semantics below.

## Contract

A policy decision is admissible only when:

- the decision is bound to the exact target subject;
- the composed evidence identity and decisive evidence identities are explicitly retained;
- the evaluated policy has immutable identity, version, digest, and matching policy bytes;
- mutable `latest` policy references are rejected;
- decision evaluation occurs within the policy validity interval;
- exact canonical decision identity material is embedded in the decision, including policy validity, required-positive semantics, evaluation time, decision expiry, and outcome; a changed identity material requires a new decision identity;
- expired decisions cannot be silently replayed as current;
- exact replay of identical decision bytes is idempotent;
- missing required positive evidence yields DENY rather than ALLOW;
- deletion of required evidence may move ALLOW to DENY/UNRESOLVED, but never DENY to ALLOW;
- negative evidence cannot be silently discarded to manufacture ALLOW;
- stale evidence cannot be removed in a way that creates ALLOW from DENY/UNRESOLVED;
- conflicting policy outcomes remain UNRESOLVED unless an explicit external ordering exists;
- evidence-consumption order cannot change the decision;
- quorum/count thresholds cannot become authorization;
- decision outcomes cannot be promoted into scientific truth or civic authority.

## Semantic decision identity

The qualifier treats the semantic decision identity as the canonical commitment over:

- exact target subject digest;
- exact policy ID and version;
- exact policy digest;
- exact policy validity interval;
- exact required-positive property set;
- exact composed-evidence digest;
- exact decisive evidence identity set;
- exact evaluation time;
- exact decision validity horizon; and
- exact policy outcome (allow, deny, or unresolved).

The semantic identity digest is derived by the qualifier and exposed in the final receipt as decision_identity_digest. It is not a substitute for the opaque decision record identifier, and it is not authority by itself.

A changed semantic identity with the same historical decision identifier is treated as identity reuse unless an explicit new decision identifier and predecessor lineage are present.

Metamorphic probes independently mutate semantic inputs and recompute the resulting disposition. Fixture annotations such as before/after are never used as the source of the transition result.

## Typed dispositions

- `REJECT_DECISION_PROVENANCE`: provenance, policy, identity, temporal, monotonicity, conflict, or authority-boundary failure.
- `DECISION_DENY`: the exact bound policy evaluates the admissible evidence set without the required positive evidence.
- `DECISION_ALLOW`: the exact bound policy evaluates the admissible evidence set with all required positive evidence and no unresolved conflict.
- `DECISION_UNRESOLVED`: the evidence or policy outcomes cannot be reduced to a single decision under the declared semantics.

## Corpus

D-01 missing required positive evidence -> DENY
D-02 complete required positive evidence -> ALLOW
D-03 deletion of required evidence -> ALLOW to DENY
D-04 deletion of required evidence -> DENY to ALLOW
D-05 negative evidence silently discarded -> ALLOW
D-06 stale evidence removal -> DENY to ALLOW
D-07 conflict silently resolved -> ALLOW
D-08 policy version change with reused decision identity
D-09 mutable latest policy reference
D-10 decisive evidence identities omitted
D-11 policy digest differs from evaluated bytes
D-12 decision evaluated outside policy validity
D-13 expired decision replayed as current
D-14 exact replay of identical decision
D-15 contradictory policy outcomes explicitly unresolved
D-16 quorum threshold treated as authorization
D-17 decision presented as scientific truth
D-18 decision presented as civic authority
D-19 order-dependent evidence consumption
D-20 policy-field omission cannot weaken a DENY
D-21 irrelevant evidence deletion leaves decision unchanged

The qualifier derives dispositions from semantic predicates and generates a canonical receipt. Fixture files contain no expected verdicts or embedded oracles.

## Qualification ceiling

PASS establishes only that this synthetic benchmark preserves decision provenance, policy identity, monotonicity, conflict semantics, temporal validity, and authority boundaries.

It does not establish correctness of any real policy, truth of any underlying scientific claim, legitimacy to act, safety of deployment, or civic authority.

No runtime implementation is proposed.

## Standards alignment

The current in-toto framework is v1.2, while SVR is v0.2. The framework's monotonic principle is the primary external basis for the deletion invariants here. SVR requires `verifier.policies`, records `timeCreated`, and allows multiple SVRs for the same subject; it does not prescribe this synthetic decision algebra.

References:
- https://github.com/in-toto/attestation/blob/main/spec/v1/README.md
- https://github.com/in-toto/attestation/blob/main/spec/predicates/svr.md
- https://github.com/in-toto/attestation/blob/main/spec/v1/bundle.md
