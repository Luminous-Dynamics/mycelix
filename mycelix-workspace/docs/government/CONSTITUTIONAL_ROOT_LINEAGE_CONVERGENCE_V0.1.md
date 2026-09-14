# GOVSYS-003C Root-C Convergence Qualification v0.1

Status: **normative convergence-qualification contract**

This tranche qualifies the exact Git assembly that brings together three already-qualified theorem lineages:

```text
qualified Root-B bootstrap provenance
+ qualified CORE-LINEAGE Stage 1
+ qualified GOVSYS-003C-V predecessor authorization
        -> qualified Root-C convergence substrate
```

It deliberately does **not** yet claim constitutional rooted-lineage semantics.

```text
qualified convergence substrate
!= VerifiedRootTransitionEvidence -> TransitionFacts adapter
!= constitutional rooted lineage
!= replay-nonce set closure
!= fork-free constitutional history
!= current constitutional root
!= source coverage/currentness
!= actor authority
!= execution authority
!= external-effect authority
```

The network remains infrastructure for institutions. It is not the sovereign.

## Exact convergence subject

Convergence commit:

`120d31918763f18be910af5570089ff1eed1ce78`

Convergence tree:

`9f705542ac258e6d08a5b7b6744511dfd8ccb6a9`

The convergence commit has exactly three ordered parents:

1. Root-B: `65848e071bf5e4e6a4f07b09ad659ca6e831d6c7`
2. CORE-LINEAGE Stage 1: `6c90aa822309d5f85c43ef1a0a70b5f592085cbd`
3. GOVSYS-003C-V transition verifier: `883aed18e6df3c31dc8e74c780164533c39ede8f`

The first-parent Root-B tree is:

`8df058c2e7a507e9ac845a212ee6bf0ede961784`

This ordering is intentional: Root-B remains the civic bootstrap lineage, while the generic structural kernel and predecessor-transition verifier are selectively converged as independent theorem parents.

## Qualified source evidence

The source theorem heads were independently hosted-qualified before convergence:

- Root-B `65848e071...`: pinned-root provenance exact-head qualification;
- CORE-LINEAGE Stage 1 `6c90aa822...`: run `34880200562` PASS;
- GOVSYS-003C-V `883aed18...`: run `34879886701` PASS.

The convergence qualifier inherits no theorem merely because Git names a parent. It proves that the exact reviewed blobs from those qualified heads are the blobs present in the convergence tree, then re-executes their compatible executable surfaces in the converged tree.

## Selective tree theorem

Relative to exact Root-B, the convergence tree may add exactly these nine paths and no others:

```text
.github/workflows/core-lineage-stage1.yml
crates/mycelix-core-lineage/Cargo.toml
crates/mycelix-core-lineage/src/lib.rs
crates/mycelix-core-lineage/vectors/rooted_lineage_v1.json
docs/core/CORE_LINEAGE_STAGE1_V0.1.md
.github/workflows/govsys-003c-transition-verifier.yml
mycelix-workspace/docs/government/CONSTITUTIONAL_ROOT_TRANSITION_VERIFIER_V0.1.md
scripts/qualification/govsys_003c_transition_verifier_vector_v1.json
scripts/qualification/govsys_constitutional_transition_verifier_v0_1.py
```

For every CORE-LINEAGE path, the convergence blob MUST be byte-identical to the blob at `6c90aa822...`.

For every transition-verifier path, the convergence blob MUST be byte-identical to the blob at `883aed18...`.

No ordinary merge from `main`, no unrelated workflow, no runtime zome, no administrative-policy implementation and no effect-authority path may be imported by this convergence.

## Qualification tranche surface

The commit immediately above the convergence subject may add exactly two files:

```text
.github/workflows/govsys-003c-root-convergence.yml
mycelix-workspace/docs/government/CONSTITUTIONAL_ROOT_LINEAGE_CONVERGENCE_V0.1.md
```

It MUST be exactly one commit above `120d319187...`. Any later semantic implementation requires a new exact subject and new qualification lineage.

## Re-execution requirements

The dedicated hosted gate re-executes compatibility checks from all three theorem families without mutating their frozen files:

- exact Root-A identity oracle;
- Root-B pinned-provenance oracle;
- GOVSYS-003C-V transition-verifier golden/adversarial corpus;
- an OpenSSL Ed25519 verification using the frozen transition fixture;
- CORE-LINEAGE Rust 1.98.1 formatting;
- CORE-LINEAGE all-target tests;
- warnings-denied Clippy;
- wasm32 `no_std` check; and
- immutable checkout postflight.

Executable re-execution supplements, rather than replaces, the exact source-head PASS evidence. The decisive convergence property is exact ancestry plus exact blob identity.

## Why convergence is not Root-C yet

The converged tree contains two sides of a future composition:

```text
GOVSYS-003C-V
VerifiedRootTransitionEvidence
        |
        |  adapter not yet implemented
        v
CORE-LINEAGE Stage 1
TransitionFacts
```

Until that adapter is independently specified and qualified, software must not project arbitrary transition evidence into the generic lineage kernel and call the result constitutional history.

The adapter must later prove at minimum that every projected structural field is locally rebound from one exact positive `VerifiedRootTransitionEvidence`, including lineage-domain identity, predecessor/successor generations, Root-A identities, source-descriptor identities, stable transition semantic identity and effective time.

## Replay and fork boundary

GOVSYS-003C-V deliberately verifies one edge at a time. CORE-LINEAGE deliberately rejects parallel/forked structural histories but does not know GOVSYS replay nonces.

Therefore the future Root-C adapter/aggregator still owns the set-level theorem:

```text
same predecessor
+ same replay nonce
+ different stable transition identity
        -> ReplayNonceConflict
```

Exact duplicate evidence for the same semantic transition/nonce may normalize harmlessly. Two distinct fully authorized successors remain conflict evidence; timestamp, arrival order, lexical digest order, verifier identity, reputation, stake or advisory scores cannot choose a winner.

## Nonclaims

Passing this convergence gate establishes only that the exact three qualified theorem lineages were assembled without byte drift or unrelated tree widening and still execute compatibly together.

It proves no legal legitimacy, democratic legitimacy, current constitutional head, source coverage, policy authority, administrative authority, coercive authority, judicial authority, execution authority or external effects.