# Integral Interop 1 — Metamorphic Conformance Contract

**Status:** ReferenceModelOnly  
**Profile:** `integral-interop-1`  
**Projection:** `integral-interop-1-design-semantic-v1`

## Purpose

The validation contract is necessary but not sufficient. The interoperability
boundary also needs to demonstrate which mutations preserve semantic identity
and which mutations must change it.

The metamorphic corpus therefore tests identity-preserving transformations,
identity-changing transformations, and fail-closed transformations.

## Invariants

### Identity-preserving

These transformations must leave the selected semantic commitment unchanged:

- JSON object-key order changes.
- Unselected certification/provenance metadata changes.
- Unknown top-level fields are ignored because they are outside the frozen
  projection.
- Production-step object-key order changes.

### Identity-changing

These transformations must change the selected semantic commitment:

- Production-step array order changes, because array order is semantic under
  D6S-CANON-1.
- A selected production semantic value changes.
- The projection/hash domain changes.

### Fail-closed

These transformations must produce no commitment:

- A selected numeric value becomes fractional.
- An unknown field is added inside a selected production-step object.

The final case is deliberately strict: production-step objects have an exact
frozen field set. This prevents a future upstream field from silently becoming
part of the semantic identity merely because the implementation serializes the
whole step object.

## Cross-layer boundary

The intended chain is:

`Integral OAD object -> selected semantic projection -> structural validation
-> D6S-CANON-1 -> D6X qualified closure -> D6W input`

The metamorphic tests cover the first three boundaries. D6X/D6W propagation
remains covered by the existing closure and mutation tests.

## Authority

This is a Mycelix interoperability profile derived from publicly available
Integral reference material. It does not assert that the public Integral
technical specifications are ratified wire schemas. Integral's public
specifications page currently labels the Certified Design core structure as
DRAFT and the OAD-to-COS interface contract as PENDING, while describing
ratification as the point at which independent implementations should depend
on a contract.

That distinction is preserved here: this tranche freezes a deterministic
ReferenceModelOnly boundary without presenting it as an external protocol
commitment.


## Declarative vector schema

The machine-readable corpus uses explicit mutation operations rather than prose
descriptions:

- `set`: replace the value at a dot/array-index path.
- `reorder-object`: rebuild an object at the path with reversed insertion order.
- `swap-array`: exchange the first two array elements.
- `reparse`: serialize and parse the supplied JSON value.
- `hash-domain`: replace the semantic hash domain for the same projection.

Each vector declares an `id`, `operation`, `path`, `value` where applicable,
and an expected outcome. The Rust conformance test executes these vectors
directly. This makes the corpus closer to an interoperable test specification
rather than a prose checklist.
