# Property Legal-Effect Boundary

**Status:** design constraint and current-code review; not a legal opinion  
**Scope:** land and natural-resource interests represented through Mycelix Property  
**Related work:** [LAND-RRR-INTEROP-001 #6315](https://github.com/Luminous-Dynamics/mycelix/issues/6315), [LAND-RRR reference PR #6316](https://github.com/Luminous-Dynamics/mycelix/pull/6316), [Sol Atlas TIB-001 #937](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/937)

## 1. Required distinction

Mycelix Property stores application-level state. Its internal records must never be silently presented as an authoritative external land registration or legal determination.

Keep these states distinct:

1. **Private agreement:** parties, terms, signatures and conditions for an agreement.
2. **Internal transfer workflow:** Mycelix's proposal, acceptance, conditions, escrow and app-local state.
3. **Internal property projection:** the Property/TitleDeed entries and ownership links stored by Mycelix.
4. **External application:** a request, filing or evidence package submitted to a competent registry, court, regulator or other applicable body.
5. **External receipt/decision:** the exact response returned by that institution.
6. **Qualified external status:** a competent reviewer or trusted, jurisdiction-specific verifier has established authenticity, issuer competence, territorial and subject-matter scope, effective time, currentness and revocation status.
7. **Legal interpretation:** any conclusion about title, consent, priority, transferability or extinguishment, made by the relevant competent institution or appropriately qualified legal actors.

A transition in one state does not imply a transition in the next. Escrow release, DHT consensus, an internal TitleDeed, a DID signature or a community vote does not itself establish national registration or extinguish another person's or community's rights.

## 2. Current implementation seam

The current transfer coordinator's complete_transfer path checks the transfer's declared conditions, calls the local registry.transfer_ownership zome, updates the internal transfer record to Completed, then attempts a best-effort bridge notification. The registry coordinator derives the committing seller DID, checks it against the internal Property owner, updates the internal Property owner projection, creates a new internal TitleDeed entry and links the internal deed chain.

These are meaningful application-level integrity steps. They are not an external deeds-office or other competent-authority transaction. The returned deed identifier and Holochain action hashes identify Mycelix records, not a national registration number unless a future, independently verified adapter establishes that relationship.

Implementation references:
- [Transfer coordinator — complete_transfer](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/mycelix-property/zomes/transfer/coordinator/src/lib.rs)
- [Registry coordinator — transfer_ownership](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/mycelix-property/zomes/registry/coordinator/src/lib.rs)
- [Registry integrity — TitleDeed validation](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/mycelix-property/zomes/registry/integrity/src/lib.rs)

The current code review did not find an external registry receipt or independent legal-effect verification in this transfer path. This document therefore avoids describing the internal transfer result as a legal title event.

## 3. Fail-closed host boundary required before land/resource use

Before any operation projects an internal land/resource transfer as externally registered or legally effective, a future host adapter must require all of the following:

- Exact source record and assertion references, schema versions and content digests.
- An authority profile that states issuer identity, jurisdiction, decision type, scope, delegation, valid interval and revocation/currentness source.
- An exact external receipt or decision, with captured representation, digest, source reference and capture time.
- Independent verification of the receipt and the authority chain; a valid signature alone is not sufficient proof of competence or legal effect.
- A jurisdiction-specific rule profile reviewed by appropriately qualified legal practitioners.
- Current dispute/challenge and appeal state, including unresolved or late-arriving decisions.
- Required community governance, mandate, consent, access and publication permissions, kept separate by purpose.
- Survey/geospatial qualification appropriate to the asserted surface or subsurface space.
- An auditable decision packet that records the evidence frontier, unavailable sources, conflicts, reviewer requirements, dissent and recheck triggers.

The host must hold the operation when the result is review_required, not_evaluable, source currentness is unknown, a source is unavailable, there is an unresolved conflict, or an external operation's outcome is ambiguous. An unknown outcome must be reconciled against the external authority before retrying a mutation. Do not treat timeout, retries, consensus or model confidence as success.

The current Mycelix-DeSci QualificationDisposition / QualificationDecisionPacketV0 code is a pure reference policy and audit-packet model. It is **not yet wired into this Property transaction coordinator**. It does not currently prevent the coordinator from updating internal state. That integration is a separate required work item tracked by [PROP-LEGAL-EFFECT-001 #6319](https://github.com/Luminous-Dynamics/mycelix/issues/6319).

## 4. Preserve plural rights and non-land asset behavior

Do not force every asset into a single-owner land-title model. A territorial record may include multiple rights, responsibilities and restrictions with separate parties, bases, authorities, temporal intervals, vertical extents and disputes. Surface land, subsurface mineral rights, water, access, grazing, conservation, cultural heritage and infrastructure must remain distinct where the jurisdictional source makes them distinct.

The boundary in this document applies to claims of external legal effect for land and natural-resource interests. It does not require non-land asset workflows (for example, equipment or digital assets) to pretend that they are national land-registration processes.

Use the [Mycelix Land RRR reference profile](../../mycelix-desci/docs/LAND_RRR_INTEROP_V0_1.md) as a non-authoritative source-reference contract. Use [Sol Atlas TERR-001](https://github.com/Luminous-Dynamics/sol-atlas-leptos/pull/936) for the territorial assertion and its spatial feature reference. Neither system replaces the competent external authority.

## 5. Acceptance tests for future integration

A Property host-level legal-effect gate is not ready until tests demonstrate:

| Scenario | Required outcome |
|---|---|
| Internal transfer completes, no external receipt exists | Internal state only; no registered/legal-effect promotion |
| Receipt digest is valid but issuer competence or jurisdiction is unknown | Hold for review |
| Authority has expired, been suspended or revoked | Reject or hold, according to the qualified jurisdiction profile |
| External registry is unavailable or returns an ambiguous timeout | No retry mutation until reconciliation |
| A current, relevant challenge or appeal is active | Hold dependent legal-effect projection |
| Surface parcel is used for subsurface rights without qualified vertical geometry | Hold; do not infer subsurface extent |
| Community testimony exists but mandate/decision evidence is unavailable | Preserve assertion; do not promote it to a validated community decision |
| Sensitive community data lacks purpose-specific publication permission | Deny that disclosure; retain the underlying restricted evidence |
| A valid external receipt is independently qualified for the specified operation | Permit only the explicitly scoped projection; preserve receipt and verifier lineage |

The integration must also prove replay safety, no double-mutation after timeout, exact-head source pinning, correction/revocation propagation, audit export and a safe recovery procedure.

## 6. Release and review boundary

No real-world land decision should rely on this software without jurisdiction-specific legal review, qualified surveying/geospatial review, community-approved governance and data-use processes, privacy/security assessment, and an independent appeal/reconciliation path. South African use additionally needs practitioners competent in the relevant land, customary, mining, water and environmental regimes. This document does not interpret those laws or determine any person's rights.
