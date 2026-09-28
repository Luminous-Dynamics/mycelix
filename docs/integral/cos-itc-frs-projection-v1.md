# COS -> ITC / FRS projection boundary v1

Status: ReferenceModelOnly / SourceRefined

Current public Integral material describes COS production records as inputs to ITC and FRS. The Development Guide says FRS reads COS labor events, material consumption, QA outcomes and throughput data; the public ITC page describes Labor Capture as receiving verified LaborEvents from COS. These statements are treated as source requirements, not evidence that a complete runtime implementation exists.

The invariant implemented here is:

COS source observation != ITC projection != FRS interpretation != authority

## Source-owned observation

A COS observation carries its own identity, observation time, source evidence, verification state, origin and the exact source values used by a downstream projection.

Only an explicitly verified, currently valid observation may cross the projection boundary.

## ITC projection

The reference projection binds back to the source observation ID and copies only explicitly supplied source values. It does not invent labor hours, material usage or participant identity.

ITC-specific weighting, decay and access policy remain downstream Integral semantics.

## FRS projection

The reference signal carries source observation IDs and explicitly declares itself derived. A finding or recommendation must not overwrite the source observation.

## Negative boundaries

- unverified labor -> no ITC projection;
- stale evidence -> no ITC/FRS projection;
- quality/throughput observation -> not automatically an ITC labor/material projection;
- foreign evidence -> origin retained;
- projection -> no authority;
- FRS interpretation -> cannot replace COS source fact.

## Traceability

| Requirement | Reference owner | Witness | Formal boundary | Claim ceiling |
|---|---|---|---|---|
| COS labor feeds ITC | cos_projections | labor projection test | COS-FV-001 | source/projection separation |
| COS material consumption feeds ITC | cos_projections | material projection test | COS-FV-001 | source/projection separation |
| COS QA/throughput feeds FRS | cos_projections | FRS signal test | COS-FV-008 | source/derived separation |
| current evidence required | Evidence::Validity | stale tests | COS-FV-005 | freshness semantics |
| federation provenance | Evidence::Origin | foreign-origin test | COS-FV-010 | provenance preservation |

## Nonclaims

This does not implement Integral ITC economics, FRS diagnostics, credit issuance, access allocation, governance authority, physical production, or Integral ratification.
