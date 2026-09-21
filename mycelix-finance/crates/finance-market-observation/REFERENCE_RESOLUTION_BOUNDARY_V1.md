# FIN-MKT-002A Fill Reference Resolution Boundary V1

## Purpose

FIN-MKT-002A stores append-only correction/bust relations by **fill semantic commitment**.

A fill semantic commitment already binds the fill identity commitment, and the fill identity binds the exact order/account/instrument/provider observation subject. Therefore a resolved fill semantic commitment is cryptographically subject-bound.

However, a bare digest appearing inside an adjustment input is only a reference claim until a trusted observation frontier actually resolves it.

```text
adjustment contains fill semantic commitment
!= referenced fill exists
!= referenced fill was admitted
!= referenced fill belongs to this adjustment subject
!= referenced fill is currently effective
```

## 002A responsibility

002A establishes deterministic append-only relation identity only.

For a correction:

```text
Correction {
    prior_fill_commitment,
    replacement_fill_commitment
}
```

For a bust:

```text
Bust {
    prior_fill_commitment
}
```

The commitments name **fill semantic commitments**, never raw evidence envelopes.

002A rejects only relation-local impossibilities it can prove without resolving the observation graph, such as self-correction where prior and replacement commitments are identical.

## Resolver / projection responsibility

Before an adjustment affects effective execution projection, FIN-MKT-002B or a separately qualified resolver must prove at least:

- every referenced fill commitment resolves to an admitted canonical fill observation;
- the resolved fill is bound to the exact intended order lineage/account/instrument/provider subject required by the projection profile;
- correction/bust topology is supported by the selected profile;
- conflicting or cyclic correction graphs fail closed;
- a fill is not silently moved between predecessor/successor order lineages;
- additional evidence bindings for the same semantic fill do not create a new fill semantic identity.

```text
reference resolution
!= provider truth
!= currentness
!= settlement
```

## Missing references

A missing referenced fill is not interpreted as zero quantity, already-busted, provider rejection, or harmless history.

It remains unresolved/non-admitted for projection.

## Cross-subject attempts

A correction/bust observation may syntactically contain any 32-byte fill semantic commitment because 002A does not have a global fill registry.

If resolution shows that the referenced fill belongs to an incompatible subject, projection must reject the relation rather than reinterpret or transplant the fill.

```text
same-width digest
!= compatible fill reference
```

## Claim boundary

A qualified 002A observation proves only that the exact adjustment relation was canonically represented under the selected profile.

It does not prove that the referenced fill exists, that the adjustment is provider-authoritative, or that the resulting effective quantity/position/settlement state has changed.
