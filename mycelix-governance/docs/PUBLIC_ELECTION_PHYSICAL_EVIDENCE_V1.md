# Mycelix Public Election Physical Evidence v0.1

Status: **ELECT-012 foundation; physical evidence contracts, not an audit algorithm**

Parent stack includes ELECT-001 through ELECT-011.

Profile identifier: `mycelix-public-election-physical-evidence-v1`

## Purpose

A cryptographically elegant election can still fail if the physical evidence is incomplete, substituted, mishandled, or impossible to reconcile with the electronic record.

This tranche makes paper-ballot evidence part of the same typed evidence system as cryptographic voting rather than treating the audit as an informal postscript.

The target is:

```text
ballot stock conservation
+ exact batch identity
+ continuous custody
+ scanner/CVR reconciliation
+ governed audit evidence
+ transparency checkpoint binding
```

None of these substitutes for the others.

## Research basis

EAC guidance on risk-limiting audits and chain of custody emphasizes several recurring requirements:

- the audit must refer back to the original paper ballot/paper record;
- chain of custody is essential;
- containers should be identified, sealed, and accompanied by custody logs;
- ballot manifests can be derived from custody/accounting records;
- comparison audits require the physical ballot to be traceable to its CVR; and
- unique ballot identification must not reconnect the ballot to the voter.

The EAC's chain-of-custody guidance also calls for election identity, precinct/batch/container identifiers, seal/serial information, actors, ballot accounting, process timing, and attestations/signatures to be documented.

References:

- https://www.eac.gov/sites/default/files/eac_assets/1/6/Risk-Limiting_Audits_-_Practical_Application_Jerome_Lovato.pdf
- https://www.eac.gov/sites/default/files/bestpractices/Chain_of_Custody_Best_Practices.pdf
- https://www.eac.gov/election-officials/clearinghouse-resources-audits-recounts

The exact audit method remains jurisdiction-governed. As of 2026 the EAC continues to describe multiple audit approaches and has published draft voluntary national audit standards for public comment rather than one universal mandatory method.

## 1. Ballot stock conservation

`BallotStockAccountingV1` works over an exact homogeneous accounting scope bound to:

- election constitution;
- jurisdiction scope;
- location/batch scope;
- ballot style; and
- physical unit definition.

The core conservation theorem is:

```text
opening_stock + supplemental_stock_received
=
cast_regular
+ cast_provisional_sealed
+ spoiled
+ issued_not_cast
+ unused
+ quarantined
```

All arithmetic is checked for overflow.

If exceptional categories are non-zero—provisional sealed ballots, issued-but-not-cast ballots, or quarantined stock—an explicit exception-evidence digest is required.

This does not imply every jurisdiction uses these exact human labels. The important theorem is that every physical unit entering the scoped stock must have one explicit disposition; adapters can map local legal categories onto this canonical accounting boundary.

## 2. Physical ballot batches

`PhysicalBallotBatchV1` binds an exact batch to:

- election constitution;
- accounting scope;
- ballot style;
- physical unit definition;
- exact physical count;
- initial container state; and
- batch manifest.

An empty batch is invalid.

Batch identities are evidence identities, not voter identities.

## 3. Container and seal state

A container is never represented merely as `sealed: bool`.

`SealStateV1` distinguishes:

```text
Sealed {
  seal identifier,
  seal application evidence
}

OpenForAuthorizedProcess {
  governing authorization,
  opening evidence
}
```

This prevents “seal opened” from being silently treated as an uninteresting boolean transition.

## 4. Exact custody lineage

Every `CustodyEventV1` binds:

- exact election and ballot batch;
- monotonic custody sequence;
- exact predecessor event;
- event type;
- from/to control domains;
- complete before/after container states;
- governing authorization;
- witness evidence;
- temporal evidence; and
- event evidence.

`validate_custody_successor(...)` requires:

```text
same election
same ballot batch
next.sequence == previous.sequence + 1
next.previous == exact previous event digest
next.before_state == previous.after_state
```

A timestamp does not establish custody ordering. Exact predecessor lineage does.

Two different same-sequence children from one predecessor classify as a `SiblingFork`; the only v1 policy is `FreezePendingEvidenceResolution`.

Physical custody therefore follows the same anti-first-arrival principle as anonymous voting authority and the transparency log.

## 5. Custody completeness summary

`CustodyChainSummaryV1` binds the batch, genesis/final event identities, total event count, conflicting-event count, final container state, and complete transparency checkpoint.

Any observed custody conflict blocks a clean summary.

The summary does not prove the chain by itself. ELECT-011's offline verifier must replay the full event lineage and derive the summary independently.

## 6. Scanner / CVR reconciliation

`CaptureReconciliationV1` binds:

- exact election/accounting scope;
- a frozen physical-to-CVR cardinality profile;
- physical cast-unit count;
- expected unique CVR count;
- observed unique CVR count;
- duplicate CVR count;
- unresolved physical↔CVR mapping count;
- scanner manifest;
- CVR export;
- exact CVR interoperability profile; and
- reconciliation evidence.

A reconciliation is `Clean` only when:

```text
expected_unique_cvr_count == observed_unique_cvr_count
AND duplicate_cvr_count == 0
AND unresolved_mapping_count == 0
```

Otherwise it is explicitly `Discrepant`.

The separate cardinality-profile digest matters because “one paper ballot equals one CVR” is not universally safe across multi-card ballots, scanner/reporting conventions, or jurisdiction-specific systems.

For U.S. adapters, NIST SP 1500-103 provides a common CVR interchange model designed for scanner/EMS/audit interoperability. The Mycelix evidence layer should bind an exact NIST/profile version rather than inventing an opaque proprietary CVR format.

Reference:

- https://www.nist.gov/publications/cast-vote-records-common-data-format-specification-version-10

## 7. Audit method policy

`AuditMethodPolicyV1` separates method identity from result evidence.

Supported structural classes are:

- risk-limiting;
- traditional/statutory;
- full hand count; and
- another explicitly governed method.

A risk-limiting profile must carry an explicit non-zero risk limit below 100%, encoded as integer parts-per-million rather than floating point.

Non-RLA methods must not accidentally inherit an RLA risk-limit field.

This crate does not implement BRAVO, Kaplan-Markov, MACRO, SHANGRLA, or another RLA algorithm. A concrete method implementation must have its own frozen method/profile/parameter digest and qualification evidence.

## 8. Audit sample evidence

`AuditSampleV1` binds:

- frozen population checkpoint;
- exact ballot manifest;
- audit policy;
- sample-selection evidence;
- sampled-position digest;
- population size; and
- sample size.

The sample must be non-empty and cannot exceed the frozen population.

Randomness/seeding is represented by evidence rather than implicit runtime entropy. The actual approved random-selection procedure belongs to the audit method profile.

## 9. Audit observations and adjudication

Each `AuditObservationRefV1` uses an **anonymized physical-unit digest** and optional CVR digest.

Observation classes include:

- match;
- vote overstatement;
- vote understatement;
- missing physical record;
- missing CVR;
- uninterpretable physical record; and
- governed adjudication required.

If adjudication is required, adjudication evidence is mandatory.

The physical-unit identifier must be created/used only after the ballot is no longer associated with a voter. Election evidence must never recreate a voter→ballot mapping merely to make audits convenient.

## 10. Audit result evidence

`AuditResultRefV1` binds the exact audit policy/method, frozen population checkpoint, exact sample, observation root, audit computation evidence, typed disposition, and result digest.

Possible dispositions are:

```text
OutcomeConfirmed
EscalateSample
FullHandCountRequired
Indeterminate
```

The crate intentionally does not derive `OutcomeConfirmed` from statistics. That authority belongs to a separately qualified audit-method verifier.

## 11. Binding physical evidence into the public election record

`PhysicalEvidenceBundleRefV1` binds:

- accounting receipt;
- custody summary;
- capture/CVR reconciliation;
- audit result;
- physical-evidence root; and
- transparency checkpoint.

This is the subject consumed by ELECT-011's `PhysicalAudit` verification stage.

The intended final chain is:

```text
physical ballots
  -> accounting + batches
  -> custody lineage
  -> scanner/CVR reconciliation
  -> audit population + sample
  -> observations / adjudication
  -> audit result
  -> physical evidence root
  -> transparency checkpoint
  -> offline verifier PhysicalAudit stage
  -> certification policy
```

## Fail-closed distinctions

Mycelix must preserve these distinctions:

```text
balanced ballot stock     != correct tally
sealed container          != complete custody
complete custody          != scanner correctness
matching CVR counts       != ballot-level correctness
an audit result object    != a valid audit computation
OutcomeConfirmed receipt  != election certification
```

## Deliberate non-claims

This tranche does not establish:

- jurisdictional legal compliance;
- actual seal signatures or hardware authenticity;
- truthful custodian identities;
- a physical ballot's voter intent;
- exact physical↔CVR mapping algorithms;
- any particular RLA mathematics;
- audit randomness correctness;
- recount law;
- public disclosure legality for all custody records; or
- election certification.

It establishes the evidence subjects and conservation/lineage invariants those systems must later qualify.

## Next direction

With ELECT-001 through ELECT-012 structurally represented, the next phase should **stop adding broad architecture temporarily and begin protocol selection + executable composition**:

1. compose ELECT-007/008/009/010/011/012 into one offline-verifiable evidence graph;
2. select and threat-model candidate anonymous-credential/nullifier constructions;
3. select candidate E2E ballot/tally families (ElectionGuard-style homomorphic tally, Belenios/mixnet variants, etc.);
4. implement independent verifier test vectors before runtime integration;
5. add adversarial corpus cases for split view, duplicate authority, corrupted manifests, custody forks, CVR mismatch, malicious archives, and unresolved disputes; and
6. only then connect the qualified public-election profile to Holochain coordinator paths.
