# Replayable qualification evidence bundle research v1

Status: research-only.

The preceding layers independently establish:

    witness authentication
        +
    append-only VDS
        +
    governed key rotation
        +
    authenticated tree heads
        +
    inclusion receipts
        +
    cross-observer consistency

This layer binds those separate artifacts into one replayable evidence object without collapsing their meanings.

## Evidence binding

The bundle records:

- exact PR/head topology for the research stack;
- exact Git blob identities for every consumed fixture;
- witness registry and trust-root identities;
- VDS identity;
- transparency-service identity;
- observer-gossip registry identity;
- the requirement that each upstream verifier remains an independent prerequisite;
- the explicit requirement that hosted PASS is not inferred by the bundle.

A bundle is evidence-ready only when all artifact paths resolve, every Git blob matches its pinned identity, JSON artifacts contain no private-key material, and the cross-artifact identities remain consistent.

## Why Git object identity is used

Filename equality is not sufficient evidence. A malicious or accidental replacement can keep the same path while changing the contents.

The bundle therefore binds:

    path + Git blob SHA

for each consumed artifact.

Changing a fixture without updating the bundle causes the bundle verifier to return unresolved.

The bundle also pins the research stack's exact PR head SHAs so that “the same branch” cannot silently advance underneath an evidence packet.

## Cross-artifact checks

The verifier checks, among other relationships:

    trust root -> witness registry

    witness checkpoints -> witness registry

    VDS / tree heads -> VDS identity

    receipt -> receipt transparency-service registry

    receipt -> VDS identity

    gossip -> VDS identity + gossip registry

    bundle -> exact stack topology

These are binding checks, not substitutes for the upstream cryptographic verifiers.

## Security semantics

The bundle deliberately does not define:

    evidence-ready == qualified

Instead:

    evidence-ready
        ->
    all upstream verifiers may now be replayed against
    the exact artifacts named by the bundle

The hosted execution state remains separately observed. A synthetic success value is rejected by the bundle verifier.

## Campaign

The 18-case campaign covers:

- valid bundle;
- witness-registry artifact substitution;
- trust-root substitution;
- VDS fixture substitution;
- tree-head substitution;
- receipt fixture substitution;
- gossip fixture substitution;
- witness-registry identity substitution;
- VDS identity substitution;
- hosted-PASS injection;
- topology-head substitution;
- decision-prerequisite weakening;
- bundle identity substitution;
- each individual upstream-verifier prerequisite being disabled.

Python and Node implementations independently produce the same deterministic report.

## Claim ceiling

Research-only.

This adds replayable evidence binding. It does not itself execute or certify the underlying witness, VDS, receipt, or gossip proof suites. It proves that a verifier was given the exact artifacts that the packet names.

Hosted execution remains a separate evidence channel. A queued run is not a PASS, and a bundle containing hosted_status=success is invalid unless an independently trusted execution record establishes that fact outside this bundle.

The next protocol boundary is not another static fixture: it is binding the replayable bundle to actual signed execution receipts and, separately, to a COSE/CBOR-compatible receipt representation.
