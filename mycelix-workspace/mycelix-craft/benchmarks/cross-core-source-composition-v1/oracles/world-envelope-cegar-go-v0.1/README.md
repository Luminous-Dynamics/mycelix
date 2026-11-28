# Craft finite-domain uncertainty envelope and CEGAR oracle V0.1

A dependency-free Go reference oracle for checking whether an abstract uncertainty representation conservatively covers every concrete assignment in a declared finite domain, and for refining that abstraction using only declared forbidden-combination constraints.

## Run

```sh
go test -count=1 ./...
go test -race -count=1 ./...
go vet ./...
```

## Semantics

- Each dimension declares a finite value domain. The full Cartesian product is enumerated under a 100,000-assignment guard.
- A declared constraint is a forbidden partial assignment with explicit evidence/assumption references. It excludes matching assignments from the concrete compatible-world set.
- A concrete world is one that does not match any declared forbidden constraint.
- An abstract cell denotes all assignments matching its fixed pattern and none of its excluded patterns.
- The abstract union must contain every concrete compatible world. Any omitted concrete assignment is an under-approximation and is rejected.
- Abstract cells may contain assignments excluded by the concrete constraints. These are spurious abstract worlds and are conservative: they can create false ambiguity, but cannot justify a false-safe result if every concrete world is covered.
- A refinement adds a declared forbidden pattern to one cell. The operation is accepted only if the full concrete set and its decision semantics remain unchanged.
- Decision summaries use hard-safety exclusions, per-world optimal actions, robust-safe actions, minimax loss and minimax regret. No probabilities are assigned.
- The oracle pins the domain/constraint/world digest for its two frozen synthetic fixtures. Cell definitions are intentionally excluded from this digest because refinement changes cells while preserving the same concrete world universe.

## Frozen cases

- `CWE-001-spurious-decision-flip`: one impossible combination creates an artificial decision flip; applying its declared constraint to the coarse cell removes the spurious world and restores the invariant action.
- `CWE-002-genuine-decision-flip`: a real action disagreement remains after the same refinement; the system must preserve non-identification.

## Proof boundary

`abstract_coverage_sound=true` means only that the abstract union covers the concrete compatible assignments generated from this finite domain and declared constraint set. It does not establish that the dimensions, values, constraints or source universe contain every real-world possibility. `fixture_exact` is pinned to two synthetic content digests only; it is not a real-world completeness proof. `declared_assumptions` is conditional. `not_evaluated` never upgrades to a global claim.

The oracle does not authenticate evidence references, prove the semantic truth of constraints, validate loss functions, grant authority, or execute actions. It is a test/reference module, not an SMT solver or production uncertainty engine.
