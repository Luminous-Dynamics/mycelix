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


The policy-chain evaluator has an explicit maximum delegation depth of eight edges, allowing at most nine tokens including the root; this is independently frozen by the test harness. The separate claims, key-link, and `par_hash` checkers use the same nine-token chain-size ceiling. Over-depth chains fail closed as unsupported, and duplicate hop IDs are rejected before relations are evaluated. These bounds constrain candidate work; they are not a negotiated protocol limit and must not be presented as one.


## Delegation-chain claims: depth, lifetime, and fixture linkage

The new `delegation_chain_claims.py` checker evaluates parsed, deterministic fixture claims independently from policy denotation. It checks unique token IDs, the root depth being zero, exactly-one depth increments, descendant depth within the parent's budget, non-increasing depth ceilings, child expiry not exceeding parent expiry, non-decreasing `iat`, expiry after both current time and issuance, a finite per-token lifetime, a future-issued-at skew bound, and a parent-envelope SHA-256 commitment over the preceding fixture node.

The independent harness repeats all numeric limits in its own frozen constants, computes the canonical JSON digest without importing the implementation's helper, independently derives the set of expected invariant findings, and tests ten malformed/violating cases plus four injected omissions of required finding codes. The workflow emits `delegation-chain-claims-differential.json` alongside the policy-semantic chain evidence.

**Important protocol boundary:** the fixture's `parent_envelope_sha256` is a custom canonical-JSON commitment. It is not the AAT draft's `par_hash` wire representation, is not derived from a compact signed JWT, and provides no authentication. This checker does not parse or verify token signatures, issuer authority, derived-key thumbprints, proof-of-possession, audience semantics, or revocation. It checks only the supplied parsed fixture claims. A claim-level PASS must never be treated as a cryptographic-chain PASS.

The published AAT draft -01 (dated 15 June 2026, expiring 17 December 2026) supplies useful independent design context: its depth invariant increments `del_depth` by exactly one at each link and bounds `del_max_depth`; its TTL invariant requires child expiry no later than the parent, issued-at no earlier than the parent, an unexpired token, and a bounded lifetime. It recommends finite maximum-depth enforcement and documents a 30-second maximum future-issued-at skew and 90-day maximum-token-lifetime upper bound. This candidate adopts those last two values as frozen *test-profile values*, not as a claim that the draft is an endorsed standard or that the entire AAT profile is implemented. See https://datatracker.ietf.org/doc/draft-niyikiza-oauth-attenuating-agent-tokens/ and RFC 8693's express distinction between token exchange and deployment-specific token validity/trust semantics: https://www.rfc-editor.org/rfc/rfc8693.

All evidence remains research/specification only. The exact-head workflow must complete and its artifacts must be reviewed before any bounded-test PASS can be reported; cryptographic enforcement qualification remains explicitly outside this candidate.


## Derived issuer and holder-key linkage (parsed claims)

A standalone `delegation_chain_key_linkage.py` candidate now checks the AAT-style relation between each derived token's parsed `iss` claim and the preceding token's `cnf.jwk`. The implemented narrow profile is public OKP/Ed25519 only: it rejects private JWK members, malformed `x`, other key types/curves, a root issuer that uses a derived-token thumbprint URI, and any derived issuer that does not equal the preceding holder key's RFC 7638 SHA-256 thumbprint rendered as the RFC 9278 URI:

`urn:ietf:params:oauth:jwk-thumbprint:sha-256:<base64url-thumbprint>`

The independent test harness implements its own required-member canonicalization and SHA-256/base64url computation, covers a valid four-hop link plus eight negative controls, and injects three missing-finding regressions. Unsupported key profiles fail closed; this does not claim support for RSA, EC, or other OKP curves.

The existing `parent_envelope_sha256` field remains a separate *test-fixture* consistency link. It is **not** the draft's `par_hash` (which commits to the parent's JWS signing input), and it is not a substitute for derived issuer linkage. The RFC 7638 thumbprint binds the issuer claim to the public JWK's required members; it does not authenticate any token.

Normative cryptographic verification remains out of scope: the candidate does not parse compact JWS/JWT, validate signatures under the root trust anchor or parent holder key, verify `par_hash`, validate `cnf` public-key authenticity or holder proof-of-possession, enforce algorithm/key compatibility, or authorize the root issuer. Per the June 2026 AAT Internet-Draft -01, derived issuer linkage (I1), parent-signing-input linkage (`par_hash)), and leaf proof-of-possession are separate invariants and must all be verified before an invocation can be authorized. The draft is still an individual Internet-Draft and not an endorsed IETF standard; these checks are research/specification alignment only:
- RFC 7638, JSON Web Key (JWK) Thumbprint: https://www.rfc-editor.org/rfc/rfc7638
- RFC 9278, JWK Thumbprint URI: https://www.rfc-editor.org/rfc/rfc9278
- AAT draft -01, delegation authority and verification algorithm: https://datatracker.ietf.org/doc/draft-niyikiza-oauth-attenuating-agent-tokens/

The exact-head workflow compiles and runs the key-link checker and uploads `delegation-chain-key-linkage.json`. Qualification remains NOT_CLAIMED until hosted runs finish and exact receipts are inspected.


The JWK thumbprint checker also rejects non-canonical base64url encoding of the Ed25519 `x` coordinate: decoding to 32 bytes is not sufficient unless re-encoding without padding yields the exact supplied string. The independent corpus contains an otherwise length-correct but non-canonical coordinate to guard this boundary. Thumbprints use only the RFC 7638 required members, so optional metadata such as `kid` does not alter the thumbprint URI.


## Parent JWS signing-input linkage (`par_hash`)

A further separate checker, `delegation_chain_par_hash.py`, tests the AAT-style parent-token instance commitment. For each derived hop it validates the supplied exact signing-input fixture shape as canonical unpadded BASE64URL(protected-header).BASE64URL(payload), then checks that `par_hash` equals unpadded BASE64URL(SHA-256(parent signing-input ASCII bytes)). The root must omit `par_hash`, because it has no parent. The digest value itself must be canonical BASE64URL for exactly 32 bytes; a 43-character string alone is insufficient.

The independent corpus implements its own segment canonicalization and digest calculation. It covers a valid four-token chain, wrong and missing hashes, a changed-parent-token re-association case, a root with an invalid parent hash, malformed/non-ASCII signing inputs, non-canonical digest encoding, a duplicate hop ID, excessive chain size, and an unknown schema. Three omitted-check mutations must be detected independently.

This still does **not** parse a compact JWT/JWS or verify signatures. The fixture contains an explicit `signing_input` string; production must derive it from the exact protected-header and payload segments of the presented compact JWS, validate the signature under the appropriate key, and then compare the claim. This check does not replace the JWK thumbprint issuer relation or holder proof-of-possession. The AAT draft's Section 4.6 states that `par_hash` binds the child to the parent's JWS Signing Input, whereas RFC 7638 JWK thumbprints bind a key's public required members. That distinction prevents a chain from being silently re-associated with a different parent token held under a compatible key:
- AAT Internet-Draft -01, section 4.6 and verifier algorithm: https://datatracker.ietf.org/doc/draft-niyikiza-oauth-attenuating-agent-tokens/
- RFC 7515, JSON Web Signature signing input: https://www.rfc-editor.org/rfc/rfc7515
- RFC 7638, JWK Thumbprint: https://www.rfc-editor.org/rfc/rfc7638

The exact-head workflow compiles and runs this checker and uploads `delegation-chain-par-hash.json`. Qualification remains NOT_CLAIMED pending hosted execution and receipt inspection.


The `par_hash` fixture checker also applies independent resource bounds before hashing: at most 64 KiB per supplied signing-input string and at most 256 KiB across the chain, alongside the nine-token ceiling. The independent tests freeze both byte limits and include one oversized signing input plus a chain whose individual inputs fit but aggregate input bytes exceed the chain bound. These limits are applied to the supplied signing-input fixture strings only; they do not prove that compact JWT tokens were fully parsed or that encoded token/stack sizes were enforced by a production verifier.


## Exact-head evidence receipt aggregate

The final workflow step uses `aggregate_auth_v20_evidence.py` to require all eleven specialist receipt files, validate each receipt's expected schema and `status=PASS`, require every receipt's `source_head` to equal the exact checked-out SHA, and require `qualification=NOT_CLAIMED`. It hashes every receipt into a single aggregate, and checks frozen counts for the bounded policy controls, 16,384 ordered differential pairs / 524,288 pair-request combinations, 14 manifest mutations, plus the dedicated policy and chain mutation suites.

The aggregate hashes those eleven specialist receipts plus the aggregate mutation guard’s own receipt as a twelfth exact-head item. It freezes the self-test’s twenty-seven mutation IDs and detection markers as well as the specialist mutation inventories. It also recomputes every specialist receipt’s declared source digests against the actual checked-out files (or matrices) and embeds those verified source paths and SHA-256 values in the final aggregate; the synthetic aggregate tests freeze the source-field/path map independently and mutate two receipt source-hash fields. For the eleven base policy controls, the workflow now retains a raw `*.input.json` scenario and `*.json` result file for each control; the aggregate requires the exact 23-file inventory (receipt plus 22 scenario/result files), independently recomputes each canonical input hash and result hash, and records the raw artifact SHA-256 and byte length in the final aggregate.

The aggregate reports `PASS_BOUNDED_RESEARCH_EVIDENCE`, **not** a production or cryptographic qualification pass. It is an evidence-integrity and completeness gate over receipts generated by the specialist checks; it does not independently validate the semantic truth of those checks beyond the explicitly frozen fields and counts, does not establish proof soundness outside their bounded domains, and cannot turn the fixture claim checkers into a JWT verifier.


The receipt aggregator now has its own mutation harness, `test_aggregate_auth_v20_evidence.py`. It freezes the complete eleven-specialist-receipt inventory independently, accepts a synthetic valid all-pass fixture, and requires rejection of twenty-seven deliberate weakenings: missing receipt, mismatched source head, qualification laundering, failed receipt hidden as acceptable, schema substitution, reduced corpus count, reduced manifest-mutation count, reduced checker-mutant count, malformed expected head, substituted manifest-mutation identity, substituted checker-mutant identity, duplicated checker-mutant identity, and six attacks on the aggregator self-test receipt (omission, wrong head, weakened count, substituted mutation identity, and forged aggregator/test source hashes). This tests evidence-gate sensitivity; it does not prove the specialist receipts are semantically sound.


## Compact Ed25519 JWS chain verification candidate

The dedicated `delegation_chain_compact_jws.py` layer verifies actual three-segment compact JWS values rather than trusting a caller-supplied `signing_input` fixture. Under a deliberately narrow `EdDSA` / OKP Ed25519 profile it:
- verifies the root signature under exactly one configured root trust-anchor key and requires root `iss` to match that anchor's issuer URI;
- verifies each child signature under the previous token's `cnf.jwk`, and parses that token's payload claims only after signature verification succeeds;
- rejects malformed/duplicate JSON members, unsupported algorithms and critical headers, non-canonical BASE64URL segments, malformed/private holder JWKs, and wrong-size signatures;
- verifies the derived issuer JWK thumbprint and `par_hash` over the exact prior compact-JWS signing input, unique `jti`, depth/expiry/`iat` monotonicity, and root/child/leaf AAT-entry cardinality;
- applies 64 KiB per-token and 256 KiB chain limits and an eight-edge/nine-token depth ceiling.

The integration harness generates temporary Ed25519 keypairs using OpenSSL and signs three positive chains: a four-token chain, a root-as-leaf single-token chain, and a chain carrying consistent JWK `use`/`alg`/`key_ops` metadata plus an unknown optional JWK member. It covers 39 negative controls including missing and forged-clock controls, tampered signatures, wrong trust anchors, child signatures under the wrong key, issuer/hash mismatches, duplicate `jti`, invalid JWKs, contradictory or malformed `use`/`alg`/`key_ops`, unsupported algorithms and invalid `typ`, JOSE `b64`/`crit` headers, malformed JSON, and size/depth violations. Four mutation tests disable signature, issuer-thumbprint, parent-hash, or JWK-usage-metadata checks and verify that the corresponding intentionally bad chain would then be accepted.

**The compact-JWS layer alone is not full AAT enforcement.** It does not implement the full RFC 9396 `authorization_details` capability/constraint-subsumption lattice, runtime tool/argument authorization, revocation, or leaf invocation-time proof-of-possession. A separate `delegation_chain_pop.py` wrapper now runs the chain verifier first, evaluates the leaf capability against the actual invocation, verifies a leaf-key-signed PoP, binds the proof to `aat_id`/`aat_tool`/`hta`, checks the configured audience and a separately supplied trusted clock, then atomically consumes the PoP `jti`. That wrapper uses a local SQLite replay-store candidate; fleet-wide replay guarantees, side-effect dispatch, and operational recovery are not validated. Both layers remain research candidates, not production-qualified.

The current AAT Internet-Draft specifies this verification ordering: check token/stack bounds, verify the root using a configured trust anchor, verify each child signature using the parent's holder key, then check issuer thumbprint, depth/TTL, capability monotonicity and exact-parent `par_hash`; the leaf invocation also requires proof-of-possession. This prototype implements a subset of those steps, not all of them. See Section 7 of the draft: https://datatracker.ietf.org/doc/draft-niyikiza-oauth-attenuating-agent-tokens/ and RFC 7515's definition of JWS signing input: https://www.rfc-editor.org/rfc/rfc7515.

The exact-head workflow now compiles and executes this integration harness and archives `compact-jws-chain.json`. The aggregate gate requires its receipt and checks 39 negative controls, three positive chains, nine verified token signatures across those chains, and four compact-verifier mutation checks. Qualification remains `NOT_CLAIMED` pending hosted execution, artifact inspection, and implementation of the excluded enforcement requirements.


Trust-anchor configuration is now a distinct required API parameter, separate from untrusted chain input. The chain-input schema rejects an embedded `trust_anchors` property rather than allowing a token-bundle caller to select its own root trust. A dedicated control attempts to smuggle an alternative trust-anchor list in the chain object and requires fail-closed rejection; production configuration must still authenticate and securely manage the external trust-anchor set.


The compact profile now uses the AAT media type `aat+jwt` (also accepting its RFC 7515 equivalent `application/aat+jwt`) rather than the generic `JWT` type; the negative corpus rejects generic `JWT`, a non-string `typ`, unsupported `b64`, and any `crit` extensions. The success code is `COMPACT_JWS_CRYPTO_LINKAGE_PASS` to distinguish signature/linkage checks from full capability authorization. RFC 7515 §4.1.9 says media-type values are case-insensitive and recipients must treat a `typ` value without a slash as having the `application/` prefix; see https://www.rfc-editor.org/rfc/rfc7515.html. The AAT document remains an individual Internet-Draft and is not endorsed by IETF (https://datatracker.ietf.org/doc/draft-niyikiza-oauth-attenuating-agent-tokens/).


The compact-JWS result status is `COMPACT_JWS_CRYPTO_LINKAGE_PASS`, deliberately narrower than complete invocation authorization: that layer verifies signatures, issuer, parent-hash, depth, lifetime and JWK usage metadata. Capability subsumption and proof-of-possession are separate layers, and neither the layers nor their aggregate imply production enforcement, revocation support, or deployment-wide replay-store correctness.


The aggregator mutation suite explicitly deletes a control input file, corrupts a control result, corrupts the raw input, and adds an unexpected file. Each attack must fail exact inventory, digest replay, or source-hash comparison; the positive synthetic fixture must expose all 22 control artifacts in its final hash inventory.


## JWK verification-use metadata consistency

The actual compact-JWS verifier now checks optional JWK usage metadata before using an Ed25519 key for signature verification. If present, `use` must be `sig`, `alg` must be `EdDSA`, and `key_ops` must be exactly the supported public verification operation `["verify"]`. Private members remain forbidden. Unrecognized optional JWK members remain ignored, and the JWK thumbprint continues to hash only RFC 7638 required members, not optional usage metadata.

The adversarial compact-JWS suite now includes three signed negative fixtures for conflicting `use`, `alg`, and `key_ops`, plus a positive signed chain with consistent usage metadata and an unknown extra optional member. Negative fixtures cover encryption-only `use`, a mismatched `alg`, absent `verify`, duplicate `key_ops`, wrong `key_ops` type, and an unrelated encryption operation. A fourth mutation disables the metadata checks and must cause all six bad fixtures to be accepted by the mutant; the unmodified verifier must reject them. Aggregate expectations are frozen at three signed positive chains, nine verified signatures across those chains, 36 negative controls, and four detected compact-verifier mutants.

This aligns the candidate's explicit verification profile with the semantics of JWK `use`, `key_ops`, and `alg` in RFC 7517 while keeping the profile intentionally narrower than arbitrary public JWK use. Unknown-member tolerance follows RFC 7517's instruction that additional JWK members unknown to a consumer are ignored:
- RFC 7517 sections 4.2–4.4: https://www.rfc-editor.org/rfc/rfc7517
- RFC 7638: optional JWK members are excluded from thumbprint computation: https://www.rfc-editor.org/rfc/rfc7638

This remains a bounded candidate. The exact-head hosted run and aggregate receipt must pass before these controls are considered executed evidence; no production qualification is claimed.


## Invocation-bound proof-of-possession candidate

The new `delegation_chain_pop.py` entry point calls the compact-AAT chain verifier first and refuses to evaluate PoP if that chain does not pass. After successful chain verification it re-reads the authenticated leaf claims, validates the leaf capability against the requested invocation, verifies a separate Ed25519 compact PoP JWT under the leaf `cnf.jwk`, and checks the PoP's `aat_id`, exact `aat_tool`, canonical `hta` versus actual invocation arguments, timestamp window, and configured audience policy.

The proof's `jti` is then consumed inside a SQLite `BEGIN IMMEDIATE` transaction with a unique key, so concurrent consumers of the same proof in that database/scope cannot both insert it. At rest, the replay key stores SHA-256 of the PoP `jti`, not the raw identifier. Consumed identifiers are not automatically purged by this candidate: deleting an expired row could allow the same `jti` to be reused later. Retention/compaction must therefore be managed without making identifiers reusable. Deployments that require fleet-wide replay protection must provide a durable transactional replay store shared by every enforcement point and configure a stable deployment-wide `replay_scope`; this candidate has only been designed and tested against a local SQLite file.

The harness generates temporary Ed25519 keys, signed AAT chains and signed PoP JWTs via OpenSSL. It covers two positive profiles (audience required and audience omitted when not configured), 25 denial/replay controls, including a fresh re-signed proof that reuses an already-consumed `jti` after the original timestamp window, a concurrent two-submission race that must yield exactly one pass and one replay denial, a replay-store outage and a group/world-writable replay-store parent and symlinked database path that must fail closed, and a forged caller-supplied clock that must not resurrect an expired chain. Five deliberate omissions must be detected: signature validation, invocation binding, canonical-payload validation, replay consumption, and trusted-clock enforcement. The aggregate now requires this receipt, validates its checker/test/fixture source hashes, freezes all five mutation IDs and the expected control counts, and includes two additional aggregate-mutation tests for a missing PoP receipt and weakened PoP mutation count.

### Explicit PoP and canonicalization boundaries

The AAT Internet-Draft requires a separate signed PoP JWT bound to the leaf token's `jti`, exact tool identifier and invocation argument map; it requires the payload to be JCS-canonical, an optional-but-profile-enforceable audience, a clock window, and stateful `jti` tracking for side-effecting invocations (draft -01, §§5.2–5.3, 7). This candidate is intentionally stricter/narrower in several local-profile choices: it permits only EdDSA/Ed25519, accepts a PoP `typ` only when absent or one of the two documented local profile values, rejects unrecognized PoP claims, and limits canonicalizable JSON to BMP Unicode and safe integers with **no floating-point values**. It is not a full RFC 8785 JCS implementation; any invocation outside that subset fails closed rather than being claimed interoperable.

The function returns `INVOCATION_POP_VERIFIED_CANDIDATE_PASS`, not a dispatch command or a production authorization certificate. It does not perform a tool side effect, roll back a side effect, prove multi-host database semantics, or establish that the enforcement point's replay scope is configured correctly. The implementation and its signed fixtures are research/specification evidence only:
- AAT draft -01 PoP structure and verification: https://datatracker.ietf.org/doc/html/draft-niyikiza-oauth-attenuating-agent-tokens-01
- RFC 8785, JSON Canonicalization Scheme: https://www.rfc-editor.org/rfc/rfc8785
- RFC 7515, JSON Web Signature compact serialization/signing input: https://www.rfc-editor.org/rfc/rfc7515

Hosted exact-head execution and receipt review are still required before reporting any test as passed; production qualification remains NOT_CLAIMED.


The invocation API requires `trusted_now` as a separate, explicit input. It overwrites the `now` field in the provided chain bundle before running chain verification and uses the same trusted value for PoP timestamp checks and replay bookkeeping. The signed regression fixture sets an attacker-controlled bundle time 30 seconds in the past (which keeps the AAT's `iat` within the profile's future-skew allowance) while passing a trusted enforcement time after the leaf's expiry; this must fail at chain verification before PoP evaluation. A caller must supply this value from the enforcement point's clock, not from token data, tool arguments, or an agent-controlled request.


## AAT capability resource bounds and aggregate inventory consistency

The capability checker now enforces finite limits for constraint-tree depth (32), constraint nodes (512), composite fan-out (128), tools per token (256), constrained argument keys per tool (64), embedded constraint-value depth (32) and nodes (512), tool-name length (256 UTF-8 bytes), and actual invocation-argument depth (32) and nodes (4,096). It uses iterative traversal for nested values and iterative JSON equality, avoiding recursive descent on attacker-controlled nested values. The first five resource limits align with recommended AAT implementation defaults where applicable; the value-tree and invocation bounds are explicit local-profile limits, not universal protocol constants.

The independent capability harness freezes all ten resource bounds and tests seven resource-specific denials: 257 tools, 65 constraints on one tool, an overlong tool identifier, over-deep and over-node-budget constraint values, and over-deep and over-node-budget invocation arguments. The evidence aggregator verifies the exact ordered inventory of all 15 malformed/bound controls and all 23 runtime/invocation controls, plus the serialized resource-limit record, instead of relying only on headline counts. The aggregate's own mutation harness also mutates the bound-control inventory and resource-limit record, including the invocation-control count and invocation-value node limit.

During this pass I found several CI evidence-contract defects and repaired them: the aggregate initially expected nine malformed/bound capability controls while the harness generated ten; the current frozen expectation is 15 after adding five constraint-metadata/value-bound denials. Separately, the capability mutant-count mutation was writing the correct value (four) instead of weakening it; it now writes three. The harness also independently rejects oversized invocation JSON even when the tool permits arbitrary values, and the aggregate now verifies the exact 23-control identity inventory plus the invocation-bound receipt values. These are source-level fixes; hosted execution and receipt inspection remain necessary to verify the harnesses themselves.


## Pre-signature token-instance cycle rejection

The compact-JWS verifier now implements the AAT draft's required pre-signature `jti` cycle check. A bounded structural JSON scanner extracts only the top-level `jti` string from each encoded payload, rejects missing/empty/non-string or duplicate `jti` members, and checks uniqueness across the presented chain before invoking any signature-verification operation. The scanner validates enough JSON structure to locate the top-level claim but does not materialize arbitrary nested payload values. Its input is bounded by existing per-token and whole-chain byte limits and a 64-level scan nesting ceiling.

These pre-signature identifiers remain **untrusted**: they are used only to reject duplicate token-instance IDs, never to authorize. After each signature succeeds, normal claim parsing and validation still run, and the authenticated `jti` must equal the value extracted by the pre-scan. A targeted regression control replaces the signature verifier with a function that fails if called for a chain containing duplicate `jti`; the duplicate must be rejected before any cryptographic operation. This matches the June 2026 AAT draft's chain verification algorithm, step 2(c), which permits and requires minimal `jti` extraction for cycle detection while deferring full application-claim deserialization until after signature verification: https://datatracker.ietf.org/doc/html/draft-niyikiza-oauth-attenuating-agent-tokens-01.

This remains a bounded implementation candidate. The pre-scan does not authenticate any token, and its result cannot be used as an authorization decision before the corresponding JWS signature succeeds.
