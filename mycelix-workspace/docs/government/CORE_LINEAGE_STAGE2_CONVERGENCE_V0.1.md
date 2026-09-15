# CORE-LINEAGE Stage 2 convergence — rooted lineage + monotone evidence lease

## Purpose

This tranche qualifies the exact Git convergence required before implementing CORE-LINEAGE Stage 2 covered-current-head composition.

It combines two already-qualified theorem families without copying or rewriting either one:

- reproducible CORE-LINEAGE Stage 1 semantics and locked dependency graph; and
- monotone `EvidenceLease` dynamic-evidence algebra.

## Exact convergence

First parent:

`a0ae8538d84ec44f60e8dc5e796320579269581e`

This is #860, the qualified reproducible CORE-LINEAGE head.

Second parent:

`58fa357e53d7e529362c5f766965498ee557d6ce`

This is #181, the qualified evidence-lease head.

Selective merge:

`4f93183069437df0b2a7c1b85b5c6032ac1f83af`

Merge tree:

`8d02364ee494aa34fe5cbccc0bb5a1264091a78c`

The merge tree is exact #860 plus only these frozen #181 product files:

- `crates/mycelix-authority-evidence-lease/Cargo.toml`
- `crates/mycelix-authority-evidence-lease/INVARIANTS.md`
- `crates/mycelix-authority-evidence-lease/src/lib.rs`

The #181 workflow is not copied into the product tree; its exact qualified commit is retained as a real second parent.

## What this convergence proves

A hosted PASS establishes only that the exact qualified Stage-1 structural kernel and exact qualified no-widening lease primitive were assembled with real ancestry, byte-identical theorem files, and still execute together under the convergence qualifier.

It does not yet establish a covered current head.

```text
qualified Stage-1 lineage
+ qualified EvidenceLease
+ qualified selective convergence
!= qualified Stage-2 current head
```

## Stage-2 boundary after convergence

The future Stage-2 implementation may consume:

- an already-qualified `ProjectedRootedLineage`;
- an already-domain-qualified covered-head observation;
- an exact `EvidenceLease`; and
- optional exact next-known-transition effective time.

It may prove structural endpoint/head equality and a bounded live evidence horizon.

It must not authenticate a source, interpret a civic Root-A verification profile, choose among forks, infer completeness from omission, or grant external-effect authority.

For GOVSYS, source authentication and closed-world coverage remain #842. Root-D / #834 performs the final constitutional composition.

## Lease law

The imported lease theorem remains:

```text
verified_at(composition) = max(input verified_at)
valid_until(composition) = min(input valid_until)
```

A scheduled successor at T may only preserve or shorten the positive horizon to T; it can never widen a lease.

## Nonclaims

This convergence proves no source authenticity, current constitutional root, policy currentness, legal legitimacy, administrative authority, execution authority, or external-effect permission.
