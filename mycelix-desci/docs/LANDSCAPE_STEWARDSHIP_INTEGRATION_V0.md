# Landscape Stewardship Integration v0

**Status:** application-level profile and schema; not yet a new signed event type.  
**Owner:** Mycelix-DeSci / existing scientific event kernel.  
**Tracked by:** [LAND-STEWARD-001 #5277](https://github.com/Luminous-Dynamics/mycelix/issues/5277).  
**Upstream editor:** [Sol-Atlas draft PR #191](https://github.com/Luminous-Dynamics/sol-atlas-leptos/pull/191).  
**Upstream scenario gate:** [Symtropy draft PR #2322](https://github.com/Luminous-Dynamics/symtropy/pull/2322).

## Core rule: preserve the distinction between governance and physical evidence

A person's decision to approve a project version can establish that a particular decision was made under a stated role/authority reference and consent context. It does not establish that a simulated ecological effect occurred, that the person legally owns land, or that a planting survived.

A stewardship record therefore has separate fields for:
- the exact versioned Sol-Atlas design reference;
- an optional exact Symtropy scenario-manifest/result reference;
- the actor's asserted role, authority basis and consent record;
- project access and coordinate-disclosure policy;
- evidence class, statement, and rationale;
- any maintenance commitment, observation, consent transition or withdrawal.

These are not interchangeable dimensions and must not be collapsed into a reputation or sustainability score.

## Fit with the existing Mycelix-DeSci kernel

This profile deliberately does **not** create a parallel event store, content-hash primitive, or identity registry.

The existing event kernel in `mycelix-desci/src/core/src/scientific_events.rs` already provides:
- a BLAKE3-backed `ContentHash` type for Mycelix-side content identity;
- typed `EvidenceArtifact` records, including availability states `Public`, `Controlled`, `Embargoed`, and `Unavailable`;
- a signed immutable event envelope with versioned canonical codec, event chaining, actor identity, time, idempotency and schema version;
- append-only evidence and attestation events plus explicit correction, withdrawal, supersession and retraction event kinds.

The profile schema can be used as an application payload and, once an adapter is designed, referenced from the existing event model. It is not itself a cryptographically signed envelope and does not grant signing authority.

**Digest distinction:** Sol-Atlas and Symtropy reference their exact input artifacts with SHA-256 fields. Mycelix may additionally compute its existing BLAKE3 `ContentHash` for Mycelix artifact/event identity. Keep both algorithm names explicit; never reinterpret one digest as the other or use the upstream SHA-256 string as if it were a Mycelix `ContentHash`.

## Machine-readable contract

The profile is defined by [`landscape-stewardship-profile-v0.schema.json`](../schemas/landscape-stewardship-profile-v0.schema.json).

It includes record types for project-version review, scenario review, field observation, stewardship commitment, consent change and decision withdrawal. Conditional rules require scenario references for scenario reviews, observations for field-observation records, acceptance references for accepted maintenance commitments, and explicit authorization for public publication. Sensitive coordinates are not embedded in the profile; access policy records whether coordinate disclosure is none, coarse or exact.

### Privacy and authority defaults

- Producers must default visibility to `private`; changing it to `public` requires both publication consent and a publication authority reference.
- This v0 profile always sets `sensitive_site_data_included=false`. Exact location data must live in a separately access-controlled object, if collected at all.
- Actor role and authority kind remain distinct. A designer, community participant, reviewer, landholder/rights-holder and maintainer are not interchangeable.
- A DID, signature or reputation score does not establish land title or statutory authority by itself. `authority_reference` must reference the relevant authorization record or the role must be explicitly `not-claimed`.
- Consent changes and withdrawals create new append-only records. Do not silently rewrite history or claim that withdrawn copies stored elsewhere were deleted.

## Evidence and scenario discipline

Allowed evidence classes are observed, curated, synthetic, modelled, assumed and missing. Each displayed observation must retain method/source reference, collection time, units and uncertainty. A missing or synthetic value must never become a favorable zero.

The current Symtropy v0 contract only qualifies schema inputs and synthetic benchmarks for the Sol-Atlas schematic coordinate frame. That cannot be represented as verified hydrology, shade, habitat, accessibility, ecological benefit or construction compliance. A scenario-review profile references the exact manifest/result digest and keeps its declared evidence class.

Maintenance commitments are promises with a responsible-party reference, cadence, trigger, review date and status. Status `accepted` requires an acceptance reference; it does not prove the maintenance task was carried out or that the landscape outcome succeeded. Track later observed completion separately.

## Acceptance gates for runtime integration

1. Validate this schema and validate the full source project/schema before recording references.
2. Compute source-specific digests from exact source bytes and preserve the algorithm label.
3. Store profile/evidence through the existing immutable event API once an adapter is implemented; never create a parallel mutable history.
4. Require explicit consent/publication authorization before generating a public projection.
5. Refuse a public record with exact coordinates or embedded sensitive site data.
6. Keep corrections, consent withdrawal, commitment changes and project supersession as new records linked to prior IDs.
7. Verify that synthetic/modelled evidence never renders as observed evidence in public UI.
8. Test permission denial, stale project digest, changed scenario result, withdrawn consent, unknown role authority, concurrent version decisions and public export with sensitive coordinates.

This profile is the interoperability boundary proposal for Mycelix. Runtime signing/event adapters and cross-client conformance tests remain separate work and must not be claimed as implemented by the schema alone.
