# MFG-PROC-001B — Distributed Manufacturing Resource Offers v0.1

**Parent:** Mycelix #3085. **Child:** #6533.  
**Companion:** MFG-PROC-001A #6531 / draft PR #6532 (typed process/capability refs).  
**Status:** schema and synthetic contract proposal; no resource-offer runtime or scheduler is implemented.

## 1. Objective

Represent a provider's versioned declaration that an exact resource at an exact site can offer a canonical manufacturing process under a referenced capability profile and configuration.

The resource offer is the handoff between engineering semantics and a distributed production network:

`typed process requirement → provider/resource offer → exact evidence resolution → qualified capability → calendar/capacity matching → reservation → execution → observed and accepted output`

The offer is only one intermediate object. A provider statement, even when signed, is not proof that a machine is present, that a process is qualified, that the provider controls the resource, or that a requested time is available.

## 2. Paradigm shift: capability and availability are different facts

Do not use `machine_type == "CNC" && status == "Available"`.

Represent separately:

| State/layer | What it means | Owner/evidence |
|---|---|---|
| Offer published | Provider issued a versioned claim about a resource/process | Mycelix offer record |
| Provider signer resolved | Signer DID and signing authority are verified for the provider/resource | Mycelix identity/trust owners |
| External refs resolved | Exact process, profile, resource, site and configuration objects exist and their content digests match | Their canonical owner contracts and resolver |
| Capability qualified | Exact resource/configuration/process/material/site has passed the required physical qualification | Symthaea process semantics + competent qualification owner |
| Declared capacity | Provider says a maximum output rate under a declared basis | Provider assertion, not measurement |
| Calendar availability | A source declares time windows for the resource | Exact calendar owner/ref and freshness |
| Bookable capacity | A reservation protocol says capacity can be committed | Separate commitment/reservation owner |
| Reserved capacity | An accepted reservation receipt exists | Commitment owner |
| Executed | A specific work order/process run was performed | Mycelix execution evidence / Symthaea machine authority |
| Accepted output | Product-specific independent measurement and acceptance pass | Mycelix manufactured-unit evidence and competent product authority |

These are not synonyms:

`published != signed != signer-authorized != refs-resolved != capability-qualified != calendar-open != bookable != reserved != executed != accepted`

The v0.1 schema has no `qualified`, `available`, `reserved`, `executed` or `accepted` boolean. Those facts belong to separate owner contracts.

## 3. Exact ownership and identity references

### Provider

Use the existing Mycelix DID/identity contract as the provider identity source. The offer carries an exact owner-object reference to the provider identity document plus an exact signer reference and declaration signature/digest. It does not duplicate the DID document, keys, trust score or role policy.

A syntactically valid DID/reference does not mean the signer is authorized to speak for the provider or resource. Authority resolution is separate.

### Resource

For a Mycelix manufacturing machine, reference the existing immutable machine-record/action identity from the manufacturing machine registry. For a workcell, specialist bureau, mobile cell, lab or partner service, reference its canonical owner record. Do not mint a second global machine identifier in this contract.

A Mycelix MachineEntry today still includes legacy `machine_type` and string capability labels. MFG-PROC-001A adds typed process-capability references; neither the legacy string nor a typed pointer proves physical capability.

### Site/facility

Reference the source-owned site/facility/zone record. Do not copy a street address or invent geospatial authority in the offer schema. Sol-Atlas may project exact site references spatially through #606, but its map/location projection does not become a facility registry or qualification.

### Process and capability profile

Use the exact Symthaea MFG-PROC process definition and capability-profile references from MFG-PROC-001A. These are pinned to the canonical source commit/type/object IDs. Until Symthaea #5901 replays the source API on current main and an exact object resolver verifies the object payload, a pointer is `TypedReferencePresent`, not `OwnerObjectVerified`.

### Configuration, tooling, material and prerequisites

Use exact source-owned references for as-configured machine generation, fixtures/tooling, material applicability, environmental/utility prerequisites, operator competence, calibration and inspection evidence. The offer must not copy engineering quantities or replace the relevant material, process, equipment, competence, metrology or qualification owners.

## 4. Capacity model

v0.1 represents two different claims only:

1. **Declared output capacity** — a provider assertion with an explicit output unit, quantity, time window and basis reference. Its evidence class is provider-declared.
2. **Calendar source** — an exact reference to the provider's time-window/calendar owner and its revision.

No algorithm converts declared capacity into bookable capacity, and no time slot is considered reserved merely because the offer contains a calendar pointer. The schema deliberately does not embed a reservation engine, queue, scheduler, work order or payment instruction.

Missing capacity or calendar data is null/unknown. It is not zero, infinite availability or a positive promise.

## 5. Immutable offer and derived lifecycle

The offer is an immutable declaration revision. Its own `publication_state` is restricted to `Published` or `Withdrawn`; a withdrawal is another provider declaration, not a destructive edit to the prior offer.

The following are **derived projection states**, not values the provider may set:
- `Expired`: derived from the exact validity interval and evaluation time;
- `Disputed`: derived from a separately owned dispute/lifecycle event, with its own source and authority resolution;
- `Superseded`: derived when a newer immutable offer references this exact offer revision;
- `Revoked`: derived from owner-native revocation/withdrawal evidence where applicable.

The schema therefore has no `Expired`, `Disputed`, `Superseded`, `Qualified`, `Available`, `Bookable`, `Reserved`, `Executed` or `Accepted` provider-settable status. It carries only exact lifecycle-event references and the optional supersedes pointer. A resolver may produce a projection, but that projection must retain its source evidence, evaluation time and uncertainty.

Offers are immutable revisions with stable identity, monotonic revision, exact provider/resource/site/process/profile/configuration/evidence pointers, issue/validity interval, signed-declaration references, declared capacity, optional calendar source, optional supersedes pointer and optional commercial-terms pointer.

## 6. Adversarial synthetic corpus

The 16 fixtures cover:
- syntactically valid provider offer that remains DeclaredOnly;
- legacy resource/machine string without typed process references;
- process/profile process-ID mismatch;
- process owner commit mismatch or unavailable object;
- provider DID and signer refs present but not verified/authorized;
- unknown resource/site/configuration identity;
- stale/expired capability evidence;
- published capacity with no calendar source;
- calendar source with declared capacity unknown;
- offer expired by time; provider-withdrawn immutable declaration; dispute derived from a separately owned event; supersession derived from a newer offer pointing to the prior revision;
- declared capacity incorrectly interpreted as reservation;
- quote/price absent but engineering feasibility kept separate;
- duplicate physical-resource IDs across two offers (cross-record conflict, not a single-record JSON-schema check);
- provider assertion misrepresented as third-party verification;
- missing operator, utility, calibration or inspection evidence.

All cases are synthetic. Positive JSON-schema validation proves structural shape only. The corpus deliberately retains `DeclaredOnly`, `Unresolved`, `Disputed`, `Expired` and `NotBookable` states rather than emitting a positive production-capability or reservation decision.

## 7. What comes next (not part of v0.1)

- **001B2:** owner-reference resolution against Mycelix DID/machine records, Symthaea process/profile objects and source-owned facility/site records; verify exact bytes/digests, issuer and revocation.
- **001B3:** provider signer authority and offer supersession/revocation proof.
- **001C:** process-plan graph/hold/rework semantics.
- **001E:** capability-aware scheduling using qualified offer evidence, calendar and capacity constraints.
- **001G:** read-only MTConnect / OPC UA resource adapters with exact model/namespace version pins.
- Reservation/commitment protocol remains a separate owner and must distinguish unknown commit outcome from confirmed failure.

## 8. Claim ceiling

This tranche supplies only JSON schemas and synthetic examples. It does not create a provider registry runtime, authenticate a signer, verify resource existence, qualify any machine/process/provider/site, prove current calendar availability, reserve capacity, execute a work order, accept a part or establish supplier reliability.