# SYM-CIVIC-014 — coverage-universe proof boundary preflight v1

Status: design/research preflight only; not qualified

Parent subject:
`2e1c7ef4e2d0ced913834681e11318104d02de75`
(SYM-CIVIC-013 exact head; authoritative run `37194455368` succeeded)

## Boundary

SYM-CIVIC-014 asks when a finality basis establishes not merely that a signed evidence object is authentic, but that the evidence covers a complete decision-relevant universe at a declared cut.

The target claim is conditional:

`declared universe + scope + anchored continuity + non-equivocation + freshness + complete non-inclusion => completeness relative to the declared universe`

It does not establish legal, judicial, institutional, execution, authorization, or civic authority.

## Research findings

RFC 9942 defines COSE Receipts as signed proofs about VDS states at issuance and distinguishes inclusion, consistency, and non-inclusion proof families.

RFC 9943 defines transparency-service properties including append-only and non-equivocation.

Certificate Transparency's consistency model demonstrates that consistency between two presented checkpoints proves an append-only relationship between those checkpoints, but auditing is also concerned with whether the same log view is presented to all query sources.

Current SCITT checkpointed-log work makes the fork boundary even more explicit: an offline verifier can verify internal consistency while still being unable to rule out a divergent fork without state retained by a checkpoint-aware witness.

Therefore 014 treats a stateful checkpoint-aware witness as a separate constructive non-equivocation requirement. A coherent chain without such a witness is insufficient for the stronger completeness claim.

References:
- https://www.rfc-editor.org/rfc/rfc9942
- https://www.rfc-editor.org/rfc/rfc9943
- https://www.rfc-editor.org/rfc/rfc9162
- https://datatracker.ietf.org/doc/draft-mih-scitt-checkpointed-local-log/

## Constructive witness model

### 1. Universe identity

A `UniverseDescriptor` binds:
- universe identifier;
- semantic version;
- namespace/domain;
- Decision identity;
- scope digest;
- canonicalization version;
- closure basis;
- manifest or enumerator identity.

Changing identity-critical universe semantics changes the descriptor identity.

### 2. Scope binding

A `ScopeBinding` binds the exact Decision identity, universe identity, universe semantic version, and decision-relevant evidence classes.

Scope changes are semantic changes, not presentation changes.

### 3. Anchored checkpoint continuity

The proof contains an authenticated genesis/anchor and a terminal frontier.

For each presented predecessor relation:

`checkpoint[n].predecessor_digest == checkpoint[n-1].identity_digest`

Array order is not semantic. The verifier reconstructs continuity from predecessor identifiers/digests.

A middle segment without its authenticated anchor is insufficient even when the terminal checkpoint itself is valid.

### 4. Stateful non-equivocation witness

Internal checkpoint consistency does not by itself rule out a producer presenting two coherent but divergent histories.

The model therefore requires a checkpoint-aware witness with retained state:
- same VDS producer/log identity;
- accepted anchor checkpoint;
- accepted terminal checkpoint;
- last accepted checkpoint;
- countersignature identity;
- witness time at or after the claimed frontier cut;
- explicit fork-free result.

A witness controlled by the producer would not become independent merely by being labeled a witness; independence is a deployment-layer property outside this synthetic qualifier.

### 5. Freshness

Freshness is separate from identity. The frontier, proof, and witness must each be temporally applicable to the qualification cut.

A stale but correctly bound proof is `COVERAGE_UNIVERSE_INSUFFICIENT`, not provenance rejection.

### 6. Proof-to-frontier binding

The non-inclusion proof itself commits to the exact universe identity, scope identity, terminal checkpoint identity/digest, coverage cut, and closure-manifest identity. A frontier certificate cannot relabel a valid proof from one domain as proof for another.

A mismatch in those identity-critical proof bindings is provenance rejection; a correctly bound proof whose cut is stale is substantive insufficiency.

### 7. Closure semantics

014 distinguishes three closure bases:

**FINITE_MANIFEST**

Completeness is relative to an exact finite manifest commitment. The manifest digest and enumerated count must match the declared universe.

**CANONICAL_ENUMERATOR**

Completeness is relative to an exact enumerator identity and output commitment. This preflight additionally requires an independently witnessed closure attestation binding the enumerated output and count.

An enumerator identifier alone is not treated as proof that the enumerator's output is complete.

**VDS_NATIVE**

Non-inclusion is interpreted only according to the VDS's own declared universe semantics. An open-world namespace is not silently promoted into a closed-world completeness theorem.

This is the key anti-vacuity boundary: the qualifier does not confuse an empty query result, a finite-looking enumerator, or a valid VDS proof with proof that every relevant element was covered.

### 8. Complete non-inclusion

A non-inclusion proof is sufficient only when:
- universe identity is exact;
- scope identity is exact;
- anchor and terminal frontier are identified;
- continuity and consistency are proven;
- a stateful non-equivocation witness closes the fork boundary;
- the proof type is supported;
- the proof and frontier reach the qualification cut;
- closure semantics actually close the declared universe;
- no decisive item is observed.

## Extension identity policy

The proof schema explicitly separates identity-critical from non-critical extensions.

In-toto's current parsing rules say unrecognized fields are ignored unless the predicate specification says otherwise, and extension fields must not change the meaning of other fields under the monotonic principle. 014 therefore defines its own predicate-local criticality rule rather than attributing this policy to in-toto itself.

Identity-critical extension namespaces are version-bound and canonicalized. Unknown or missing critical extensions cause provenance rejection. Non-critical extensions are ignored for semantic identity and cannot, by themselves, upgrade or downgrade a qualifying result.

The metamorphic corpus exercises:
- non-critical insertion;
- critical mutation/removal;
- unknown critical namespaces;
- extension schema-version changes.

## Corpus

The executable corpus contains exactly 32 cases and stores no expected verdicts. The positive census contains one canonical finite-manifest baseline; alternate positive closure constructions are exercised as metamorphic witnesses rather than duplicated corpus baselines.

The reference model derives the disposition from semantic predicates and independently recomputes the same disposition through a second qualifier path.

Canonical derived census:

`REJECT_COVERAGE_UNIVERSE_PROVENANCE = 15`
`COVERAGE_UNIVERSE_SUFFICIENT = 1`
`COVERAGE_UNIVERSE_INSUFFICIENT = 16`
`COVERAGE_UNIVERSE_UNRESOLVED = 0`

## Qualification ceiling

A GREEN result would establish only the constructive evidence boundary for this synthetic reference model.

It would not establish:
- global DHT completeness outside the declared universe semantics;
- correctness of any real-world universe enumerator;
- authenticity of Justice runtime state;
- legal or judicial finality;
- institutional legitimacy;
- execution authority;
- civic authorization.

Design rule:

`Do not infer completeness from absence; prove the universe over which absence is meaningful.`


The non-equivocation result is explicitly scoped to the named witness history. It does not prove the absence of forks outside that witness set; stronger deployment claims require broader witness coverage or another independently checkable cross-view mechanism.


The extension envelope carries an explicit identity digest over schema version and identity-critical namespaces; non-critical extensions are excluded from that semantic commitment.
