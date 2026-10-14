# AeroCommons Identity Reconciliation V1

Status: design/reconciliation evidence only — **not qualified**

Related: AEROCOMMONS-006 / #3682, EPI-001 / #2613/#2614, EPI-FABRIC-001 / #3471, MYC-SEM-001C-r2 / #2661/#2662.

## 1. Decision

AeroCommons MUST NOT become a second universal Mycelix identity or commitment authority.

The engineering layer should bind to existing Mycelix semantic and epistemic identities where those profiles are qualified, and introduce a new engineering-specific identity only when an existing qualified profile cannot express the required distinction.

The intended stack is:

```
mycelix-semantic-core
  semantic environment + semantic subject coordinates + shared commitment primitives

Mycelix EPI
  epistemic role identities + epistemic semantics

AeroCommons
  engineering-domain bindings + configuration lineage + digital-thread foreign identities

Holochain
  authored action / entry / link identity + deterministic validation
```

A projection or mapping may preserve authority boundaries, but MUST NOT strengthen them.

## 2. Current qualification state

The critical finding is that the apparent substrate is not yet safe to treat as a qualified dependency.

| Layer | Current evidence | Status for AeroCommons |
|---|---|---|
| EPI role identity (#2614) | Role-bound SHA-256 framing, strict typed admission, independent bootstrap vectors | **Bootstrap / unqualified** |
| EPI transport boundary | Public `EvidenceIdWireV1` can currently be deserialized without typed admission | **Pre-final blocker** |
| semantic commitment (#2661) | Repaired environment/subject commitment profile with independently reconstructed vectors | **Open; dedicated qualifier #2662 was described as queued** |
| semantic-core integration | EPI design note explicitly calls for rebase onto qualified semantic primitives | **Not yet a qualified dependency** |
| AeroCommons identity profiles (#3678) | Engineering-domain design contract | **Design-only until substrate reconciliation** |
| Holochain representation | EntryHash/ActionHash are protocol identities, not engineering identities | **Do not bind graph to unqualified engineering profile** |

Therefore no AeroCommons identity profile in this document is promoted to protocol compatibility status.

## 3. Required identity mapping

### 3.1 Artifact

Semantic role: an identifiable engineering artifact.

Preferred owner: EPI `ArtifactId`, once its identity profile is qualified and reconciled with semantic-core.

AeroCommons binding MUST retain:

- native EPI role/profile;
- semantic subject reference where applicable;
- external engineering identifiers;
- disclosure state;
- migration/profile version.

An artifact is not automatically its bytes, a configuration, an execution, or an evidence claim.

### 3.2 Content

Semantic role: exact external payload/content commitment.

Preferred owner: an existing qualified Mycelix content/payload commitment profile.

AeroCommons MUST NOT introduce another generic SHA-256 content identity.

External payload identifiers remain foreign coordinates until explicitly bound.

```
content commitment
!= artifact identity
!= configuration identity
!= evidence identity
```

### 3.3 Configuration

This is the strongest candidate for a genuinely new AeroCommons identity.

A configuration is a frozen engineering state composed from identified artifacts, parameters, requirements, interfaces, and declared configuration metadata.

It MUST remain distinct from:

- an artifact;
- an artifact's content bytes;
- an execution/observation;
- a source-evidence event.

A future configuration commitment should therefore be composition-specific rather than a generic replacement for semantic-core identity.

Before implementing it, audit the qualified semantic substrate for an equivalent immutable composition identity.

### 3.4 Execution / Observation

Preferred owner: EPI `ObservationId` where its semantics are sufficient.

If engineering execution requires additional distinctions not expressible by EPI ObservationId, introduce an explicitly engineering-scoped execution profile rather than overloading ObservationId.

Repeated runs over identical inputs remain distinct observations/executions.

```
same inputs + same tool + same parameters
!= same execution
```

The execution identity binds the event/observation lineage; it does not establish that the observed result is true outside its measurement/test context.

### 3.5 SourceEvidence

Preferred owner: existing Mycelix source/evidence identity substrate.

AeroCommons should adapt engineering acquisition records into that substrate rather than creating another source-evidence digest.

```
source evidence identity
!= claim identity
!= relation identity
```

### 3.6 EpistemicRelation

Preferred owner: EPI `EvidenceRelationId` / qualified relation identity.

The AeroCommons `EpistemicRelationV1` is an engineering-domain semantic object. Its identity must ultimately bind to the qualified EPI role identity rather than creating an independent universal relation hash.

A Holochain link may index the relation, but the link itself is not the relation's epistemic evidence.

### 3.7 Foreign STEP/AP242/QIF identifiers

Foreign standards remain foreign identities.

Examples include:

- STEP/AP242 object identifiers;
- QIF measurement/inspection identifiers;
- manufacturing/MES event identifiers;
- CAE/OpenMDAO/Aviary run identifiers.

A foreign identifier MUST be represented as an explicit `ForeignIdentityRef`-style binding containing at minimum:

```
foreign system
foreign profile/version
foreign identifier
binding profile/version
native target identity
```

Never silently cast a foreign identifier into a native Mycelix identity.

## 4. Holochain boundary

Holochain identities are transport/protocol identities:

- EntryHash identifies entry content;
- ActionHash identifies an authored action instance;
- agent identity identifies the authoring agent.

They are not interchangeable with engineering identities.

Consequently:

```
EntryHash != ArtifactId
EntryHash != ConfigurationId
ActionHash != ExecutionId
ActionHash != EvidenceRelationId
```

Holochain validation should enforce structural invariants and dependency availability/retrievability where appropriate. It must not be treated as physical engineering validation or certification.

## 5. Reconciliation matrix

| AeroCommons concept | Native owner | External binding | Qualification required |
|---|---|---|---|
| Artifact | EPI ArtifactId | STEP/AP242/etc. | EPI + semantic-core reconciliation |
| Content | Existing content commitment | file/object store URI or digest | Existing qualified profile |
| Configuration | **Potential AeroCommons-specific composition** | CAD/configuration identifiers | New qualification after substrate audit |
| Execution/Observation | EPI ObservationId if sufficient | test-run / CAE-run ID | EPI qualification |
| SourceEvidence | Existing evidence/source identity | acquisition/source record | Existing evidence qualification |
| EpistemicRelation | EPI EvidenceRelationId | domain relation refs | EPI qualification |
| Foreign identity | Foreign-only | STEP/QIF/MES/etc. | Mapping profile qualification |

## 6. Adversarial corpus

The reconciliation qualifier must reject or preserve distinctions for all of these cases:

1. Same payload, different execution.
2. Same payload, different artifact.
3. Same artifact, forked configuration.
4. Same evidence bytes, different source events.
5. One claim, multiple evidence records.
6. One evidence record, multiple epistemic relations.
7. EPI profile substitution.
8. Semantic-core profile substitution.
9. Repository/head substitution.
10. Holochain EntryHash substituted for an engineering identity.
11. Holochain ActionHash substituted for execution identity.
12. External STEP/QIF identifier colliding textually with a native ID.
13. EPI profile migration.
14. Semantic-core profile migration.
15. Unavailable external payload.
16. Private payload with public commitment.
17. Same textual subject under different semantic environments.
18. Same role/local identifier under different identity profiles.

The expected result is not merely "different hash". The qualifier must verify that the *semantic role and authority ceiling* remain different.

## 7. Transport hardening blocker

The EPI-001 audit exposed a subtle but important boundary:

```
wire shape parsed
!=
epistemic identity admitted
```

The current EPI-001 source explicitly notes that `EvidenceIdWireV1` derives public `Deserialize`. That allows downstream code to obtain the wire DTO without passing through `EvidenceId<R>::from_wire`.

The preferred repair already identified in the EPI review is:

1. make the public wire surface serialization/read-only;
2. use a private unchecked transport DTO for deserialization;
3. construct typed `EvidenceId<R>` only through the validating admission path;
4. add a regression test proving malformed wire input cannot yield an admitted typed identity;
5. keep the authority ceiling on the typed identity.

AeroCommons should not duplicate this repair. It should depend on the repaired/qualified substrate.

## 8. Semantic-core reconciliation blocker

MYC-SEM-001C-r2 (#2661) is valuable because it fixes an earlier golden-vector error before qualification. Its published design separates:

- semantic environment commitment;
- semantic subject commitment;
- outer commitment profile;
- domain canonicalization profile.

It also explicitly states that a commitment proves deterministic binding only, not trust, currentness, authority, semantic equivalence, or migration correctness.

That is the correct epistemic ceiling for AeroCommons.

However, #2661 remains an open PR and its own description identifies #2662 as the dedicated qualifier. Therefore AeroCommons MUST NOT treat the profile as qualified merely because the vectors look internally coherent.

## 9. Qualification sequence

The safe order is:

```
1. qualify semantic-core commitment substrate
2. repair + qualify EPI transport admission
3. rebase/rebuild EPI role identities on qualified semantic primitives
4. qualify EPI role vectors and authority ceilings
5. reconcile AeroCommons mappings against those exact profiles
6. qualify the new Configuration identity, if still necessary
7. only then implement Holochain engineering graph entries/links
```

This prevents a common failure mode:

```
design identity
 -> implementation
 -> graph integration
 -> compatibility debt
 -> later discovery that the identity profile was never qualified
```

## 10. Engineering safety invariant

The identity system must preserve these distinctions all the way through the physical lifecycle:

```
design prediction != measured observation
measurement != interpretation
evidence != claim
claim != relation
relation != authorization
configuration != execution
provenance != certification
consensus != physical evidence
identity commitment != truth
```

The identity layer is successful when it makes these distinctions difficult to erase accidentally.

## 11. Exit condition

AeroCommons can leave this reconciliation stage only when an independent implementation can:

- construct each mapped reference;
- reproduce canonical identity bytes/commitments;
- distinguish every adversarial case above;
- reject profile/role substitution;
- preserve foreign identifiers without native-ID coercion;
- preserve disclosure boundaries;
- preserve authority ceilings;
- survive EPI and semantic-core version migration through explicit mapping;
- construct the future Holochain graph without treating EntryHash/ActionHash as engineering identities.

Until then, AeroCommons identity profiles remain **design contracts, not qualified protocol commitments**.
