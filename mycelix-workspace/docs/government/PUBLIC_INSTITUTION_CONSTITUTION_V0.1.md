# Public Institution Composition Constitution v0.1

Status: **normative composition contract**

GOVSYS-002 defines the cross-domain constitutional boundary for composing public institutions from Mycelix primitives.

It does not create a government, grant public authority, encode one jurisdiction's law, or authorize external effects. It defines what later administrative, registry, records, finance, procurement, regulatory, statistics, benefits, education, health, municipal, and justice profiles are not allowed to weaken.

## Governing theorem

> Mycelix may carry and verify institutional authority, evidence, procedure, review, and effect provenance, but legitimacy originates in an explicitly identified institution, jurisdiction, rulebook, and authority source — never from software presence, DHT visibility, popularity, model output, or Mycelix itself.

The network is infrastructure for institutions. It is not the sovereign.

## Five planes remain distinct

Every consequential public-system claim must preserve the separation between these planes.

### 1. Constitutional / legitimacy plane

Defines the institution, jurisdiction, rulebook, lawful authority sources, offices/roles, amendment process, and any required external constitutional root.

```text
Institution != Mycelix platform
DHT presence != constitutional legitimacy
software deployment != public mandate
```

### 2. Authority plane

Answers which principal or office may perform which class of act, over which subject matter, under which rulebook and jurisdiction, for which validity interval.

```text
AuthorityGrant != administrative decision
AuthorityGrant != execution capability
role name != authority
identity != office
reputation != authority
model recommendation != authority
```

Broad authority must be attenuated to the exact consequential subject before execution where policy requires it.

### 3. Evidence / epistemic plane

Carries observations, records, attestations, qualified domain results, reconciliations, statistics, model outputs, and provenance.

```text
record != truth by storage fiat
observation != decision
registry entry != universal legal truth
audit finding != sanction
inspection finding != penalty
statistic != causal conclusion
forecast != decision
simulation != authority
```

A conclusion must retain an exact evidence basis sufficient for its semantic profile. Later software versions must not silently reinterpret historical evidence.

### 4. Procedure plane

Defines the required path from an initiating event through notice, evidence, opportunity to respond, competent decision, reasons, finality, reconsideration, appeal, review, stay, correction, or closure.

```text
Application != entitlement
EvidenceReady != Granted
DecisionIssued != NoticeServed
AdministrativeFinality != JudicialFinality
```

A procedure can prove that required steps and competent authority were present. It cannot manufacture the substantive legal standard it evaluates.

### 5. Effect plane

Represents execution against another system or the physical world.

```text
Decision != effect
ExecutionCapability != effect
DispatchStarted != confirmed effect
OutcomeUnknown != ProvenNotApplied
```

Irreversible or externally consequential effects require fresh authority/currentness and the existing durable-effect/reconciliation discipline. Recovery from ambiguity must not silently mint another effect.

## Cross-domain invariants

### PI-001 — explicit institutional context

Every consequential public act must bind an exact institution and rulebook. When jurisdiction is relevant, jurisdiction must also be explicit.

No generic `government=true`, `authorized=true`, or `official=true` field may substitute for that context.

### PI-002 — competence is scoped

A valid principal identity does not imply authority to decide a case. A valid office does not imply authority outside its subject matter, jurisdiction, rulebook, delegation, or validity interval.

### PI-003 — authority and evidence cannot self-justify cyclically

A public conclusion may depend on other qualified domain results, but the dependency graph must remain acyclic. The conclusion being proved cannot directly or indirectly create the authority or evidence required to prove itself.

### PI-004 — one consequential decision uses one coherent evidence cut

A decision or qualification must identify the exact record versions, authority decisions, policies, generations, exceptions, and qualification time that formed its basis.

Old and new generations may not be opportunistically mixed merely because the combination produces a desired result.

### PI-005 — historical meaning is stable

A later schema, policy, software release, or model does not retroactively rewrite the meaning of an earlier qualified decision.

Correction, amendment, supersession, reconsideration, appeal, reversal, and expungement-like outcomes require explicit new lineage. They do not erase the historical fact that the earlier record or decision existed.

### PI-006 — conflict is first-class

Two individually valid but incompatible authority, registry, evidence, or review claims produce an explicit conflict unless a separately qualified rule resolves them.

```text
conflict != max(timestamp)
conflict != first-seen
conflict != highest reputation
conflict != largest stake
```

Arrival order is not public authority.

### PI-007 — review does not mutate the reviewed act

Reconsideration, appeal, audit, judicial review, or supervisory review must target the exact prior decision/evidence identity and produce a distinct review result.

A reversed decision remains historically observable as reversed. It is not silently rewritten into the later outcome.

### PI-008 — finality is typed

The system must distinguish, where applicable:

- procedurally pending;
- administratively final;
- stayed;
- under reconsideration;
- under appeal;
- judicially reviewed;
- superseded;
- revoked;
- expired; and
- externally unenforceable/unknown.

A generic `final=true` must not collapse materially different finality regimes.

### PI-009 — public records and public disclosure are separate

The existence of an institutional record does not imply that every principal may dereference or disclose it.

Disclosure must be purpose/scope/policy qualified. Redaction may hide protected content while preserving the existence and provenance required for accountability where lawful.

### PI-010 — once-only evidence is purpose-bound, not universally readable

Inter-institution reuse should prefer minimum qualified propositions over universal record replication.

```text
"age >= 18"
    may be sufficient
full birth record
    is not automatically justified
```

Possession of a reference does not imply dereference authority.

### PI-011 — administrative convenience cannot strengthen uncertainty

Missing, stale, conflicting, unavailable, indeterminate, or disputed evidence remains explicit. A service deadline, queue state, or UI convenience cannot convert uncertainty into positive truth.

### PI-012 — public money preserves distinct authority stages

```text
Budget != Appropriation
Appropriation != Allotment
Allotment != Commitment
Commitment != Obligation
Obligation != Acceptance
Acceptance != DisbursementAuthorization
DisbursementAuthorization != Settlement
Settlement != AccountingRecognition
```

No later public-finance profile may collapse these into one mutable `payment_status` field.

### PI-013 — procurement award does not create settlement authority

```text
Need != Solicitation
Solicitation != Bid
Bid != Evaluation
Evaluation != Award
Award != Contract
Contract != Acceptance
Acceptance != PaymentAuthority
```

Procurement must compose existing Business/Commerce/Supply-Chain/Finance/Accounting truth rather than minting those domain facts itself.

### PI-014 — oversight findings are not self-executing sanctions

Audit, inspector-general, ombuds, ethics, regulator, and similar findings remain evidence/results until an independently competent process turns them into a remedy, sanction, correction, referral, or other consequence.

### PI-015 — official statistics have a distinct epistemic identity

Administrative records, survey observations, statistical estimates, revisions, forecasts, causal analyses, and simulations remain distinct products with distinct provenance.

Symthaea or another model may assist analysis, but model output cannot silently become an official statistic or a public decision.

### PI-016 — AI is advisory unless separately authorized

A model may summarize, classify, simulate, detect anomalies, propose options, or explain evidence when policy permits.

It must not become competent authority merely because its confidence, reputation, benchmark score, or predicted utility is high.

For a consequential public decision, the system must preserve which parts were human/institutional authority, which were model-derived analysis, and which evidence the model consumed.

### PI-017 — emergency powers are explicit, narrow, and expiring

Emergency procedure may alter ordinary timing or authority only through an explicit emergency rulebook/profile with scope, triggering evidence, authority source, effective time, expiry, and review requirements.

`emergency=true` is never a universal bypass around evidence, audit, rights, expiry, or review.

### PI-018 — coercive physical acts are not ordinary software authority

Mycelix may carry warrants, orders, authorizations, evidence, audit trails, and execution observations for functions involving physical coercion.

It must not treat generic software execution authority as autonomous authority to detain, search, seize, use force, target, or employ lethal force.

### PI-019 — effect adapters do not own policy

An integration adapter may prove provider/profile/route/materialization/execution facts. It does not decide that the underlying public act was lawful, properly authorized, or substantively correct.

### PI-020 — frontend presentation cannot strengthen institutional truth

The frontend may compress or summarize state but must preserve Unknown, Pending, Stale, Conflict, Revoked, Stayed, Appealed, OutcomeUnknown, and other consequential distinctions.

Color, icon, animation, ranking, or simplified copy cannot be the only carrier of consequential public state.

## Layer ownership

The intended dependency direction is:

```text
existing generic primitives
  identity / authority / governance / business / integration / evidence
        ↓
public-institution constitution (this document)
        ↓
reusable public kernels
  ADMIN / REGISTRY / RECORDS / ACCOUNT / PFM / PROC / STATS / REGULATORY
        ↓
domain profiles
  benefits / education / health / tax / customs / housing / transport / environment / municipal ...
        ↓
runtime storage + Holochain integration
        ↓
frontends and external effect adapters
```

A lower layer must not import a higher layer merely to manufacture authority.

## Required composition contract for consequential public acts

Where applicable, a later public kernel should be able to identify:

```text
institution
jurisdiction
rulebook
subject / case / resource
actor or office
required capability
exact authority source/grant
authority validity/currentness evidence
exact semantic profile
exact evidence cut
procedure generation/state
qualification/decision time
review/finality state
external-effect identity, if any
```

Not every read-only or non-consequential artifact needs every field. Any omission for a consequential act must be justified by the owning semantic profile rather than left implicit.

## Deliberate non-features

GOVSYS-002 does not introduce:

- a universal government superuser;
- a global mutable `official` bit;
- a national master database;
- automatic latest-wins conflict resolution;
- model/reputation/stake-based public authority;
- a generic coercion actuator;
- a universal policy DSL;
- one giant public-sector Holochain DNA;
- a single government-wide disclosure role; or
- autonomous AI decision authority.

## Immediate implementation consequences

`ADMIN-001` should be the first executable child because permits, benefits, licensing, tax assessments, procurement protests, public employment disputes, regulatory enforcement, immigration-like decisions, education administration, and many municipal decisions share the same procedural skeleton.

`REGISTRY-001` and `RECORDS-001` should branch independently from this constitution. They should converge with ADMIN before public finance, procurement, regulatory administration, and benefits profiles rely on them.

## Qualification meaning

A green GOVSYS-002 check proves only that this constitutional corpus preserves the required separations and anti-shortcut statements.

It does not prove that any runtime currently enforces them, that any jurisdiction has adopted them, or that Mycelix is ready to administer public power.
