# Mobility Qualification Outcome Algebra V1

This vocabulary defines the **epistemic state of an evidence/qualification assertion**. It is deliberately separate from physical safety, engineering approval, certification, regulatory authority, and Holochain protocol validation.

## States

| State | Meaning | Does not mean |
|---|---|---|
| supported | The supplied evidence satisfies the declared semantic/structural predicate. | physically safe, certified, or approved |
| contradicted | Supplied evidence explicitly conflicts with the declared predicate or invariant. | physically unsafe in every sense |
| unresolved | A required dependency/evidence item is unavailable, absent, or not yet retrievable. | failure, rejection, or safety |
| indeterminate | Evidence exists but is insufficient to distinguish competing interpretations under the declared method. | either interpretation is true |
| superseded | Historical evidence remains preserved but has been replaced for the current configuration/state. | the historical evidence was false |
| disputed | Attributable evidence or interpretations conflict and no resolution has been established. | one side is correct |
| externally-authoritative | A disposition is attributable to an identified external authority and recorded as such. | the commons created that authority |

## Required invariants

1. unresolved MUST NOT be converted into contradicted merely because a dependency is missing.
2. contradicted requires positive contradictory evidence or an explicit invariant violation.
3. superseded preserves the historical record; it does not erase or rewrite it.
4. disputed preserves competing attributable evidence rather than collapsing it into a consensus score.
5. externally-authoritative records authority; it does not manufacture authority through graph consensus, reputation, or quorum.
6. None of these states is a physical safety determination.
7. Holochain Valid, Invalid, and Unresolved Dependencies remain protocol-validation states. They MUST NOT be treated as synonyms for this engineering-evidence vocabulary.

## Holochain boundary

Holochain validation is deterministic and distinguishes valid operations, invalid operations, and unresolved dependencies. The Mobility layer can use that protocol boundary to decide whether an operation is admissible to the distributed protocol, but the protocol result remains distinct from the engineering evidence state.

For example:

- missing engineering evidence may produce unresolved;
- a Holochain validation callback may separately return Unresolved Dependencies;
- neither result proves a physical failure or safety condition.

## Scope

This vocabulary is a semantic/structural substrate for the Mobility Commons. It does not constitute a physical test method, engineering analysis, safety case, certification basis, or regulatory determination.
