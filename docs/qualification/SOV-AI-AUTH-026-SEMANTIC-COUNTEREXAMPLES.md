# SOV-AI-AUTH-026 — semantic completeness boundary and counterexamples

## Two relations, two failure meanings

This candidate deliberately keeps these predicates separate:

- Denotational containment: the child admits no request the parent rejects in the declared finite universe.
- Structural compound subsumption: every parent conjunction obligation has a distinct child witness, or every child disjunct is covered by a parent disjunct.

For an all expression, the reference denotation is the intersection of its atomic denotations. Injective clause matching is a conservative structural rule, not automatically a complete decision procedure for arbitrary request-set inclusion. A single child atom can denote a subset of two parent atoms while being unable to witness both parent obligations one-to-one.

The oracle distinguishes these outcomes:

- STRUCTURAL_SUBSUMPTION_PASS: bounded denotational containment and the declared structural relation both pass.
- AUTHORITY_EXPANSION: a concrete request exists in child-minus-parent (or child effective-policy minus parent effective-policy).
- STRUCTURAL_FALSE_NEGATIVE: bounded denotational containment holds but structural matching rejects; no authority-expansion request exists.
- POLICY_ATTENUATION_VIOLATION: effective request-set containment may hold, but at least one separate attenuation obligation fails—allow containment, deny preservation, or conflict-rule preservation.
- UNSUPPORTED_OR_UNDECIDABLE: an extension, cross-type pair, mode, or conflict rule has no admitted rule.
- EFFECTIVE_POLICY_CONTAINMENT_PASS: effective containment and the separate allow/deny/conflict-rule obligations all pass.

A structural false negative is not an authorization expansion. A finite-domain pass is not a claim about unbounded values, arbitrary predicates, or arbitrary policy languages.

## Counterexample oracle

The compound_subsumption_counterexamples.py tool accepts a JSON scenario and emits deterministic JSON. For an authority expansion it reports the smallest request under the order declared by the scenario universe, the first divergent parent constraint, all parent constraints rejecting that request, clauses supporting child admission, and a deterministic minimal clause core that preserves the request's admission/rejection outcome.

Effective-policy mode evaluates allow and deny denotations under an explicit conflict rule. It separates actual effective authority expansions from attenuation-component violations. Controls cover (a) deny deletion that really expands effective access, (b) deny deletion that is masked by an unchanged allow set and therefore does not change current effective access, and (c) an allow expansion masked by a deny. The latter two are still reported as POLICY_ATTENUATION_VIOLATION rather than passing solely because the effective request set happened not to change. Conflict-rule substitution is separately exercised.

Receipts include input and result SHA-256 digests. The companion control runner independently replays the reported request against the raw fixture data and enumerates the entire finite universe to confirm that a structural false negative has no child-minus-parent request.

## Boundedness and proof ceiling

The finite evaluator is complete only for the supplied universe, its all/any grammar, the four atomic dimensions in this candidate, and the explicitly supported conflict rules. Unknown extension identifiers fail closed. The implementation does not establish completeness for infinite numeric domains, arbitrary predicates, or all policy compositions.

Research/specification only. Hosted exact-head evidence and independent receipt review are required; no production authorization qualification is claimed.


## Differential corpus

The candidate now includes an exhaustive, deterministic differential corpus:

- 8 frozen atomic constraint templates;
- 2 compound operators: conjunction (all) and disjunction (any);
- 1- and 2-clause ordered expressions, with no repeated atomic template within a single expression;
- 128 syntactic expressions and 16,384 ordered parent/child pairs;
- a finite request domain of 32 tuples, giving 524,288 ordered-pair/request combinations.

For each supported same-operator pair, the production reference is compared with two separate implementations: a raw-JSON denotation evaluator that does not call the oracle's atom matcher, and a brute-force injective matching reference that enumerates candidate injections instead of using the oracle's augmenting-path algorithm. Cross-operator compound pairs are required to return unsupported/fail-closed.

The corpus checks:
1. the reported authority-expansion status and denotation cardinalities against independently computed child-minus-parent denotation;
2. that any authority-expansion witness is the first request under the frozen total order;
3. that structural matching agrees with brute-force existence of the declared structural witness;
4. that emitted conjunction witness maps use known clause IDs, use each child witness at most once, and contain only independently valid edges;
5. that first-divergence diagnostics, rejected parent constraints, child-supporting clauses, and minimal witness cores match independent replay;
6. that clause-order permutations preserve status, witness and ID-mapped matching;
7. that a structural false negative has no request in child-minus-parent;
8. that a mismatch is reduced deterministically by deleting clauses and then reducing atom dimensions while preserving the failure category.

A second mutation guard deliberately tries 14 changes to the frozen differential manifest, including universe shrinkage, weaker atoms, dropped operators, reduced pair counts, changed generation semantics and missing invariants. Each mutation must be rejected by the checker; its result is recorded in a hash-bound JSON receipt.

When a differential mismatch occurs, the job emits the original scenario, minimized scenario, observed output and minimized mismatch to the evidence artifact. The shrinker is a deterministic delta reducer; minimality is relative to its reduction operations, not a claim of globally minimum representation.

## Research basis

This uses the same broad verification-guided pattern described by the Cedar project: an executable reference/model checked against a separate implementation through differential testing, supplemented by property-based checks. Cedar's formalization and testing infrastructure is public at https://github.com/cedar-policy/cedar-spec and its verification-guided development paper is at https://arxiv.org/abs/2407.01688.

The IETF Attenuating Authorization Tokens Internet-Draft (June 2026, version -01) requires extension subsumption to be decidable, sound and deterministic, and permits conservative false negatives rather than unsound positives: https://datatracker.ietf.org/doc/draft-niyikiza-oauth-attenuating-agent-tokens/.

These are research references, not certification of this implementation. The differential corpus establishes evidence only for the frozen finite grammar and universe. It does not prove completeness for arbitrary constraints, infinite numeric domains, extension predicates, or production enforcement.


## Oracle mutation sensitivity

The hosted exact-head workflow now injects four deterministic defects into in-memory oracle functions and requires the separate differential checker to detect each one: opening atom subsumption, forcing denotations empty, reusing one child witness for two parent obligations, and retaining a redundant constraint in a reported witness core. The guard first verifies that each unmutated fixture agrees with independent replay, checks the expected mismatch category for each mutant, restores the original function in a `finally` path, and records a receipt bound to source hashes.

This is a mutation-sensitivity smoke test, not a proof that the checker detects all possible defects. It complements—rather than replaces—the bounded exhaustive corpus, raw-fixture replay, and frozen manifest mutation guard. Any hosted result remains bounded research/specification evidence; production qualification is not claimed.


## Effective-policy independent replay and mutation sensitivity

The policy-level checker is separate from compound-clause matching. It evaluates the four-dimensional request universe directly from raw JSON, computes allow and deny sets independently, applies the explicit conflict rule, and independently derives effective access plus the three attenuation-component obligations:

- child effective access must be a subset of parent effective access;
- child allow denotation must be a subset of parent allow denotation, even when a deny happens to mask the difference;
- every parent-denied request must remain denied, even when the allow policy happens not to overlap that request;
- the conflict rule must remain unchanged in this profile.

The hosted guard first independently replays four baseline cases over all 32 requests. It then injects six deterministic policy-layer defects: erase deny denotations, skip deny-overrides in effective access, reinterpret allow-overrides as deny-overrides, accept an allow expansion masked by deny, bypass the deny-preservation rejection, and forge the conflict-rule-preserved receipt field. Each mutant must produce the expected independent mismatch category; baseline and mutant receipts include relevant source hashes.

The independent request-set replay is intentionally finite and raw-fixture based. It is evidence against these identified defect classes only. It is not a formal proof of the checker, arbitrary policies, an infinite argument domain, or production enforcement. Qualification remains NOT_CLAIMED pending exact-head hosted runs and artifact inspection.

This follows the general verification-guided pattern described by Cedar's research: separately model semantics and compare implementation behavior while testing properties that a model may not fully capture (Disselkoen et al., *How We Built Cedar: A Verification-Guided Approach*, 2024, https://arxiv.org/abs/2407.01688). The current AAT document is still an individual Internet-Draft, not an endorsed IETF standard; its requirement that subsumption checks be decidable, sound, and deterministic is a design reference, not a certification claim (revision -01, updated 2026-06-15, https://datatracker.ietf.org/doc/draft-niyikiza-oauth-attenuating-agent-tokens/).


## Multi-hop delegation-chain attenuation

The new `delegation_chain_counterexamples.py` candidate evaluates every adjacent parent-to-child policy pair and every descendant against the root. The companion `test_delegation_chain_counterexamples.py` independently calculates effective/allow/deny request sets directly from raw JSON for the finite universe, compares each relation's result, verifies relation ordering/cardinality and top-level status, and injects four regressions that must be detected.

The frozen chain scenarios include: a monotonic four-edge delegation; allow expansion at an intermediate hop that is masked by a deny and happens not to change effective access; authority reintroduced at a later hop; a parent deny removed where the current allow set makes that deny ineffective; and an unsupported extension at the leaf. This tests why root-to-leaf containment alone is not enough as an audit record: each hop must be checked against its immediate parent, and root anchoring should be reported independently.

The chain evaluator is a semantic-policy model only. It does not validate token signatures, issuer trust anchors, byte-level parent commitments, proof-of-possession, delegation authorization, expiry monotonicity, maximum depth, revocation, or JWT/JWS parsing. Those need their own typed inputs and verified implementations before the chain could qualify as an enforcement verifier. The IETF AAT Internet-Draft -01 describes an agent delegation chain linked to parents and calls for monotonic attenuation, bounded depth, and monotonic expiry; it remains an Internet-Draft and is used here as a design reference, not as a certification source: https://datatracker.ietf.org/doc/html/draft-niyikiza-oauth-attenuating-agent-tokens-01.

The workflow now compiles and runs this separate checker and uploads `delegation-chain-differential.json`. Qualification remains NOT_CLAIMED until exact-head runs complete and their receipts are inspected. The chain corpus is finite and does not prove arbitrary-policy completeness or cryptographic chain validity.


The chain evaluator has an explicit eight-hop ceiling (root included), independently frozen by the test harness. Over-depth chains fail closed as unsupported, and duplicate hop IDs are rejected before relations are evaluated. These bounds constrain candidate work; they are not a negotiated protocol limit and must not be presented as one.
