# CORE-LINEAGE Stage 1 v0.1 — Stable Rooted Causal Lineage

Status: **pure structural kernel**

CORE-LINEAGE Stage 1 proves stable rooted causal structure from already domain-qualified semantic facts. It deliberately does not authenticate domain evidence and does not establish currentness.

## Governing theorem

```text
root anchor facts
+ already-qualified semantic transition facts
        -> ProjectedRootedLineage
```

and explicitly:

```text
stable rooted lineage
!= verifier invocation evidence
!= closed-world source coverage
!= current head
!= actor authority
!= effect authority
```

## Stable semantics, dynamic evidence outside

Verifier timestamps, proof references and evidence leases are intentionally absent from Stage 1.

A consuming domain must qualify its transition evidence before adapting it into `TransitionFacts`. Re-verifying the same exact semantic edge later must not rewrite stable causal history merely because a verifier timestamp/reference changed.

Stage 2, not Stage 1, owns dynamic covered-head evidence and evidence-lease composition.

## Domain-neutral identities

Every security-relevant identity is a profiled non-zero 32-byte digest:

```text
(profile, digest[32])
```

Profiles are exact UTF-8 with no ASCII controls or leading/trailing ASCII space and are bounded to 256 bytes.

CORE-LINEAGE does not interpret the digest algorithm used by a consuming domain. A Root-A SHA-256 identity, an authority-state BLAKE3 identity or another registered profile remains opaque to this kernel.

## Root and transition coordinates

A root commits:

```text
lineage_domain_identity
generation
node_identity
source_descriptor_identity
effective_at_ms
```

Each transition commits:

```text
lineage_domain_identity
predecessor_generation
predecessor_node_identity
predecessor_source_descriptor_identity
successor_generation
successor_node_identity
successor_source_descriptor_identity
transition_semantic_identity
effective_at_ms
```

The kernel requires exact predecessor equality with the current endpoint and checked `successor_generation = predecessor_generation + 1`.

## Explicit source-descriptor evolution

Predecessor and successor source descriptors are separate fields because a domain may explicitly authorize source-verifier rotation.

The generic kernel does not decide whether that change was legal. It only proves exact structural continuity after the domain has already qualified the edge.

## Conflict semantics

The kernel distinguishes:

- exact semantic duplicate — harmless normalization;
- same predecessor/same successor but different transition semantic identity — `ParallelTransitionConflict`;
- same predecessor with a distinct successor coordinate — `ForkConflict`;
- same transition semantic identity attached to different structural facts — `TransitionIdentityCollision`.

No conflict is resolved by timestamp, input order, lexical digest order, verifier identity, reputation, stake or advisory scores.

## Stable commitment

Profile:

`mycelix-core-lineage-v1-sha256-framed-semantic`

Unframed domain separator:

`mycelix/core-lineage/rooted-lineage/v1`

For bytes `x`:

```text
frame(x) = u64_le(len(x)) || x
```

For `u64 n`:

```text
frame_u64(n) = frame(u64_le(n))
```

For one profiled digest:

```text
frame(profile) || frame(raw_32_byte_digest)
```

Canonical lineage bytes are exactly:

```text
DOMAIN_UNFRAMED
|| frame(LINEAGE_PROFILE)
|| profiled(lineage_domain_identity)
|| frame_u64(root_generation)
|| profiled(root_node_identity)
|| profiled(root_source_descriptor_identity)
|| frame_u64(root_effective_at_ms)
|| frame_u64(transition_count)
|| topology_ordered_transition_0
|| ...
```

Each topology-ordered transition is:

```text
frame_u64(predecessor_generation)
|| profiled(predecessor_node_identity)
|| profiled(predecessor_source_descriptor_identity)
|| frame_u64(successor_generation)
|| profiled(successor_node_identity)
|| profiled(successor_source_descriptor_identity)
|| profiled(transition_semantic_identity)
|| frame_u64(effective_at_ms)
```

Input ordering never participates. Exact duplicates are removed before topology projection.

Normative two-transition fixture digest:

`89a02c2002d5e95e2cb972d3726dd1be0b88b049f01351e4a31ac75872a7d5c0`

The fixture deliberately rotates the source descriptor on generation 1 -> 2 to prove that exact domain-qualified descriptor evolution is structurally representable.

## Resource bound

Stage 1 accepts at most 4,096 raw transition facts and checks this before sorting/canonicalization.

## No authority or currentness

`ProjectedRootedLineage` is a structural result only. It explicitly reports:

```text
grants_currentness() == false
grants_effect_authority() == false
```

A domain such as GOVSYS must retain its own Root-B/#839 qualified evidence around this structural theorem. Stage 1 cannot turn caller-created facts into constitutional authority.

## Stage 2 boundary

Stage 2 will compose an independently domain-qualified covered-head observation against the exact Stage-1 endpoint.

Stage 2 is where dynamic evidence freshness belongs and is intentionally not implemented by this tranche. If #181 `EvidenceLease` is reused, Stage 2 waits for #181 exact-head qualification rather than cloning a second lease algebra.

## Adversarial corpus

The crate freezes:

- stable golden vector;
- input permutation invariance;
- exact duplicate normalization;
- explicit source-descriptor rotation;
- parallel-transition conflict;
- successor fork conflict;
- transition-identity collision;
- skipped generation denial;
- predecessor node substitution denial;
- predecessor source-descriptor substitution denial;
- lineage-domain substitution denial;
- effective-time regression denial;
- unreachable skipped transition denial;
- root-only structural projection without currentness;
- malformed profile and zero-digest rejection; and
- resource-limit enforcement before canonicalization.

## Nonclaims

This kernel does not prove signature validity, verifier origin, legal legitimacy, governance approval, source authenticity, source coverage, currentness, administrative authority, execution authority or external effects.
