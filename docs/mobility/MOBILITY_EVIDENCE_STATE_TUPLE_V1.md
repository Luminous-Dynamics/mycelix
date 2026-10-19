# Mobility Evidence State Tuple V1

A flat outcome enum is not adequate for lifecycle evidence because independent facts can coexist. A record can be epistemically supported, historically retained, disputed, attributable to an external authority, and based on a measurement at the same time.

## Dimensions

### Epistemic disposition
- `supported`
- `contradicted`
- `unresolved`
- `indeterminate`

### Lifecycle disposition
- `current`
- `superseded`
- `retired`

Lifecycle state never deletes historical evidence.

### Conflict disposition
- `uncontested`
- `disputed`

Disputed does not mean false; competing attributable evidence or interpretations remain visible.

### Authority provenance
- `commons`
- `external`

`external` requires an explicit `ExternalAuthorityReference`. It is not inferred from consensus, reputation, signatures, or Holochain validation.

### Evidence modality
- `observation`
- `measurement`
- `prediction`
- `simulation`
- `interpretation`
- `attestation`

Modality remains distinct from epistemic disposition. Numerical agreement between a prediction and an observation does not turn the prediction into a measurement.

## Invariants

1. `unresolved` is not contradiction.
2. `contradicted` requires explicit conflicting evidence.
3. `indeterminate` is not failure.
4. `superseded` preserves history.
5. `retired` preserves history.
6. `disputed` preserves competing attributable evidence or interpretations.
7. `external` authority requires an explicit external-authority reference.
8. No tuple establishes physical safety or regulatory approval.
9. Holochain `Valid`, `Invalid`, and `UnresolvedDependencies` remain protocol-layer outcomes.
10. Evidence modality never silently changes.

## Examples

| Tuple | Meaning |
|---|---|
| measurement + supported + current + uncontested + commons | A measurement supports the declared predicate. |
| measurement + supported + current + disputed + commons | The measurement supports its own predicate while interpretation or competing claims are disputed. |
| prediction + supported + current + uncontested + commons | A prediction satisfies a semantic predicate but remains a prediction. |
| observation + unresolved + current + uncontested + commons | A required dependency is unavailable. |
| attestation + supported + current + uncontested + external | An identified external authority made an attestation; the tuple does not manufacture authority. |
| measurement + supported + superseded + uncontested + commons | Historical measurement is preserved while the referenced configuration is superseded. |

## Boundary

This is a semantic/provenance model. It does not determine structural integrity, seaworthiness, roadworthiness, flightworthiness, regulatory compliance, certification, fitness for use, or physical safety.

## Compatibility mapping

The earlier seven-state vocabulary maps into these dimensions:

- Supported, Contradicted, Unresolved, Indeterminate → epistemic disposition
- Superseded → lifecycle disposition
- Disputed → conflict disposition
- ExternallyAuthoritative → authority provenance

The tuple is the normative representation for future records because it avoids enum explosion and preserves independent dimensions.
