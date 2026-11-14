# Battery Passport Lifecycle Projection v0.1

**Status:** Domain model and validator in `manufacturing_common`; not a deployed Holochain zome, EU Registry integration or conformity-assessment system.
**Owner:** [BATTERY-PASSPORT-LINEAGE-001 #6052](https://github.com/Luminous-Dynamics/mycelix/issues/6052).
**Related:** [MOBILITY-COMMONS-002 #3709](https://github.com/Luminous-Dynamics/mycelix/issues/3709).

## Source of truth

The physical battery, engineering configuration and passport are separate identities. Mycelix Manufacturing owns component/configuration and maintenance lineage. The passport is a versioned projection over that lineage, using an applicable legal data-point registry and audience access policy. It is not a second source of truth for engineering evidence.

## Lifecycle semantics

- Initial market placement issues the initial passport.
- Inspection, routine service and simple repair retain the current passport identity.
- Remanufacture and repurpose create a new passport with an explicit predecessor link and changed configuration digest.
- Recycling closes the active passport projection but never deletes historical events or predecessor passports.
- Configuration changes require explicit revalidation-obligation references.
- A successor passport must be triggered by the corresponding lifecycle event; clients cannot invent a successor record independently.

The lifecycle event log is append-only and SHA-256 chained. Every event binds physical battery identity, sequence, actor, event time, record time, configuration digest, evidence references and the previous event digest. Validation recomputes event hashes and checks the passport succession state machine.

## Data-point registry and access projection

The registry is versioned and stores legal source references, category applicability, default access class and whether a data point is dynamic. It deliberately does not hard-code a permanent “71 fields” list: Commission guidance may be revised, and applicability differs by battery category.

Access classes:
- Public
- Authority restricted (notified bodies, market-surveillance authorities, European Commission)
- Legitimate interest
- Operator confidential

A point instance may be more restrictive than its registry default, never more permissive. The responsible operator can view the information for which it is responsible; public projections return only public-authorized points. Applicable points require a value and source digest. Unknown/not-applicable points require a reason reference and carry no fabricated value. Projection receipts bind passport ID, audience, registry version/source digest, policy version and sorted visible points.

## Regulatory context

Regulation (EU) 2023/1542, Article 77 requires Battery Passports from 18 February 2027 for relevant EV, LMT and industrial batteries above 2 kWh. Commission guidance currently identifies 71 data points and their legal sources/applicability. Commission FAQs clarify that simple repair does not require a new passport, while remanufacture or repurposing does; a successor must link to its predecessor. Dynamic information should be updated when status changes or when a change is being considered.

The registry must be refreshed from current official guidance and applicable delegated/implementing acts before production use. This model does not determine legal applicability or compliance.

## Unit-test scope

Tests cover initial issuance; simple repair identity retention; remanufacture/repurpose succession; missing predecessor rejection; event hash tampering; configuration-change and revalidation controls; recycling closure without history deletion; post-recycling event rejection; public-data projection filtering; no access broadening; unknown-value rejection; deterministic projection digest; registry duplicate IDs; and serialization round trips.

## Non-claims

No EU compliance, product conformity, battery safety, second-life suitability, legal authorization, BMS actuation or registry submission is established by this library. It does not authenticate the authority or truth of evidence references; those remain responsibilities of upstream identity/attestation and external validation systems.
