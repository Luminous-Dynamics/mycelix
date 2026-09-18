# mycelix-forge-authority

Portable authority primitives for Mycelix Forge.

This crate is intentionally separate from `mycelix-forge-core`. The core crate owns the smallest stable project-identity and digest vocabulary; this crate owns repository authority structure.

## What FORGE-003 establishes

- provider-neutral principals committed by algorithm-qualified digests;
- typed, stable capability codes;
- per-capability threshold rules;
- normalized authority epochs with exact predecessor commitments;
- cumulative principal revocation across epoch transitions;
- project identity continuity across authority epochs;
- project-independent root authority policy commitments;
- positive `BoundGenesisAuthority` evidence that epoch-zero authority matches the root-policy commitment embedded in the project identity seed;
- structural quorum evaluation over distinct, already-authenticated principals;
- deterministic canonical bytes for root policy and authority epochs.

## Critical claim boundary

This crate **does not verify signatures or authenticate principals**.

`PrincipalId` is a commitment to an adapter-defined identity object. A future Xenia/Mycelix identity adapter must prove that a signature, session, DID, key lineage, or delegated credential resolves to that exact principal.

Likewise:

```text
structurally valid AuthorityEpoch
!= authorized authority transition

StructuralQuorum::satisfied()
!= signatures were authentic

sequence == 0 AuthorityEpoch
!= project-identity-bound genesis authority
```

Consumers that need the third property must require `BoundGenesisAuthority` (or equivalent independently verified evidence). Later FORGE tranches bind signed transition/review evidence to these exact subjects.

## Relationship to existing Mycelix systems

- `mycelix-identity` remains the owner of DID, credential, recovery, trust, and reputation semantics.
- Xenia remains the authentication/signing/delegation boundary.
- `mycelix-institutional-core` may later receive an adapter from these repository-specific authority objects; Forge does not depend on that still-evolving generic institutional kernel.
- Holochain may replicate/discover authority evidence, but DHT presence is not itself authority.

## Validation

Run each standalone crate under the pinned Forge CI toolchain:

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-features
```

FORGE-003 remains a protocol-structure tranche. It does not yet establish gittuf enforcement, Xenia signature verification, source review authorization, SLSA provenance, release authorization, or distributed repository replication.
