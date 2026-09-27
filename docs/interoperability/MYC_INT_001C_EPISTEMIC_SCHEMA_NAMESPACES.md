# MYC-INT-001C — Epistemic Schema Namespaces and Translation Contract

Status: architecture contract; documentation-only

Issue: #3131
Parent: #3132 (`MYC-INT-001B`)
Parent exact head: `ba3f0e2ab291bca199c60e2bbe126ff51580d6d1`
Original census base: `main` at `4a190a9c6ad8d9f1e291f10916472a183f01eddc`

## 1. Purpose

Mycelix currently contains at least two E/N/M/H classification systems whose type and level names overlap while their semantics do not.

This contract prevents accidental semantic collapse. It does not choose a preferred epistemology, migrate either implementation, or create a new semantic-commitment authority. It defines the minimum rules an adapter must obey before one schema can be referenced, displayed, translated, or compared from the context of another.

Core rule:

> A shared label or ordinal is never sufficient evidence of schema equivalence.

## 2. Frozen source identities for this analysis

These are source identities, not qualified semantic commitments.

| Schema component | Repository path | Blob SHA at census base |
|---|---|---|
| Core epistemic model | `crates/mycelix-core-types/src/epistemic.rs` | `182d0bba698cb6af0e1a8a8e1fa3fc20a8f2708d` |
| Core harmonic model | `crates/mycelix-core-types/src/harmonic.rs` | `c09b7e0dca644be3117f41a6fda6cb4907d74601` |
| Knowledge claims epistemic/harmonic model | `mycelix-knowledge/zomes/claims/integrity/src/lib.rs` | `41637eba2d9267238bb4f24a14be78d3eb875c4c` |

The older MYC-SEM line is responsible for qualified semantic commitments. Its current exact qualifier #2662 is not PASS. This interoperability contract therefore uses source path/blob identity only as a descriptive freeze and must not claim the assurance level of a qualified semantic commitment.

## 3. Temporary descriptive namespace labels

Until a qualified canonical schema-ID mechanism exists, documentation and fixtures should use unambiguous descriptive labels rather than bare `E/N/M/H` names:

```text
CoreEpistemicVCurrent
KnowledgeClaimsEpistemicVCurrent
CoreHarmonics8VCurrent
KnowledgeHarmonics12VCurrent
```

Serialized production identifiers should not adopt these exact labels merely because this document names them. A future shared `SchemaRef` should bind namespace, schema identifier/version, and an appropriate immutable definition reference.

## 4. Empirical dimension: no ordinal conversion

### Core empirical model

Core asks approximately: **How can this assertion be verified?**

| Core level | Meaning |
|---|---|
| E0 | Unverifiable |
| E1 | Anecdotal |
| E2 | Observable |
| E3 | Measurable |
| E4 | Cryptographically verifiable |

### Knowledge empirical model

Knowledge asks approximately: **How mature is the empirical validation of this claim?**

| Knowledge level | Meaning |
|---|---|
| E0 | Unverified; no empirical testing |
| E1 | Preliminary; initial observations/anecdotal evidence |
| E2 | Tested; systematic testing with limited sample |
| E3 | Replicated; multiple independent replications |
| E4 | Established; robust empirical consensus |

### Result

There is no lossless ordinal mapping.

Examples:

- A cryptographically signed anecdotal report may be Core E4 for cryptographic verifiability while remaining Knowledge E1 as empirical maturity.
- A replicated scientific result may be Knowledge E3/E4 while its source artifacts are not represented by cryptographic proof and therefore are not Core E4 for that reason.
- Core E0 “unverifiable” is not Knowledge E0 “unverified”. The former can mean verification is unavailable in principle/representation; the latter can mean validation simply has not occurred yet.

Therefore this conversion is forbidden:

```text
Knowledge E<n> -> Core E<n>
Core E<n>      -> Knowledge E<n>
```

A consumer that needs both concepts should preserve both dimensions separately.

## 5. Normative dimension: orthogonal axes

### Core normative model

Core asks approximately: **What is the scope of affected values/stakeholders?**

```text
N0 Individual
N1 Group
N2 Network
N3 Sentient
```

### Knowledge normative model

Knowledge asks approximately: **What is the state of normative evaluation/endorsement?**

```text
N0 Raw
N1 Contested
N2 Emerging
N3 Endorsed
```

These are orthogonal.

A proposal can simultaneously be:

```text
Core:      N3 Sentient       # broad affected scope
Knowledge: N1 Contested      # substantial disagreement
```

or:

```text
Core:      N0 Individual
Knowledge: N3 Endorsed
```

Therefore no direct scalar conversion is valid. A future unified analytical view, if desired, would require two separately named dimensions such as `normative_scope` and `normative_endorsement`, not a conversion function.

## 6. Materiality dimension: orthogonal axes

### Core materiality model

Core asks approximately: **How persistent or irreversible is the effect?**

```text
M0 Ephemeral
M1 Temporary
M2 Persistent
M3 Permanent
```

### Knowledge materiality model

Knowledge asks approximately: **How materially applicable/significant is the claim?**

```text
M0 Abstract
M1 Potential
M2 Applicable
M3 Transformative
```

Again there is no ordinal equivalence.

A highly applicable intervention may have only temporary effects. A permanent conceptual or archival fact may have low practical applicability. Consumers that require both must preserve both axes.

Candidate future decomposition:

```text
impact_persistence
application_maturity_or_material_significance
```

This is a conceptual decomposition only; it does not authorize a current code migration.

## 7. Harmonic dimension: incompatible cardinality and representation

### Core

Core defines eight harmonies:

```text
ResonantCoherence
PanSentientFlourishing
IntegralWisdom
InfinitePlay
UniversalInterconnectedness
SacredReciprocity
EvolutionaryProgression
SacredStillness
```

`HarmonicImpact` is an eight-component numeric impact vector, one value per harmony.

### Knowledge

Knowledge defines twelve harmonies:

```text
PanSentientFlourishing
IntegralWisdom
ResonantCoherence
EmergentJustice
CreativeExpression
EmbodiedPresence
RelationalDepth
EcologicalSymbiosis
TemporalWisdom
MysteryEmbracing
PlayfulBecoming
UnifiedDiversity
```

Knowledge `HarmonicImpact` instead stores:

```text
primary_harmony
secondary_harmonies[]
resonance
coherence_contribution
love_alignment
```

### Result

Even though three names occur in both schemas, the surrounding ontologies and representations differ. The repository currently also describes both as GIS v4.0, despite one being an eight-harmony system and the other a twelve-harmony system.

Therefore:

- common names may be presented as candidate correspondences only;
- no whole-object lossless mapping exists;
- absent harmonies cannot be invented by an adapter;
- an eight-value vector cannot be reconstructed from Knowledge primary/secondary fields without adding assumptions;
- Knowledge resonance/coherence/love-alignment cannot be recovered from the core vector without adding assumptions.

Any future mapping must explicitly state which semantics are preserved and which are lost.

## 8. Translation classes

Every cross-schema operation must classify itself as one of the following:

### Identity

Same schema identity/version and representation. No semantic transformation.

### Projection

A subset can be represented without inventing meaning. Source remains authoritative and the result records omitted dimensions.

### LossyTranslation

A deliberate policy maps one representation to another while declaring information loss and assumptions.

### DerivedInterpretation

A local analytical object is produced using source data plus a model/policy. It is a new claim, not a translated source fact.

### NoLosslessConversion

No faithful conversion exists. The source should remain opaque or be carried alongside an explicitly separate local interpretation.

For current Core ↔ Knowledge E/N/M/H translation, the default is `NoLosslessConversion` unless a narrower operation can prove a valid projection.

## 9. Proposed translation receipt semantics

A future cross-schema translation should produce an immutable receipt with at least:

```text
source_object_ref
source_schema_ref
source_definition_ref
source_digest

destination_schema_ref
destination_object_ref

translator_or_adapter_ref
translator_version
translation_class
policy_or_mapping_ref

preserved_fields_or_semantics
omitted_fields_or_semantics
assumptions
warnings

timestamp
```

A `LossyTranslation` with an empty loss declaration is invalid.

A `DerivedInterpretation` must never impersonate the source object or retain the source author's identity as if they authored the interpretation.

## 10. Comparison is distinct from conversion

It may be useful to compare different schemas without converting them.

For example, an interface may show:

```text
Core verification mode: Measurable
Knowledge empirical maturity: Replicated
```

This is safe because both assertions remain intact.

It is not safe to calculate an apparently universal scalar such as:

```text
(epistemic_score = core.E + knowledge.E) / 2
```

unless a separately versioned analytical model explicitly defines and justifies that operation. The result would then be a derived interpretation with its own provenance, not an intrinsic property of either source schema.

## 11. EpistemicKind remains orthogonal

The proposed interoperability `EpistemicKind` from MYC-INT-001B is intentionally not another quality score.

It classifies production mode:

```text
DirectObservation
InstrumentMeasurement
ActorReport
DerivedInference
Simulation
ForecastPrediction
NormativeAssertion
ExternalImport
```

That means an object could carry, for example:

```text
EpistemicKind: InstrumentMeasurement
Core classification: Measurable
Knowledge classification: Preliminary
```

without contradiction. These fields answer different questions.

This separation is valuable because it prevents one schema from being used as a universal container for source type, empirical maturity, normative scope, normative endorsement, material persistence, practical applicability, and verification state simultaneously.

## 12. Verification status is not an empirical level

Core already contains verification states such as verified, unverified, contested, refuted, and superseded. Knowledge also has evidence/challenge/fact-check machinery.

A verification state must remain separate from empirical classification.

Examples:

```text
cryptographically verified signature
!=
claim empirically established

fact-check result says supported
!=
source observation occurred

claim contested
!=
claim false
```

Future interoperability code should avoid encoding these as one scalar “truth score”.

## 13. Serialization rules

Before any shared serializer/adaptor is introduced, the following must hold:

1. every serialized epistemic payload names its schema identity/version;
2. bare `E4`, `N2`, `M3`, or harmony names are invalid at an interoperability boundary unless schema context is already cryptographically/structurally bound;
3. unknown schemas remain opaque rather than being guessed from field shape;
4. unknown future versions fail closed for conversion but may remain transportable as opaque verified bytes/objects;
5. deserialization does not choose a schema based solely on matching enum spelling;
6. source bytes/hash or source object reference survive local interpretation;
7. translation produces a new object/receipt rather than mutating source provenance.

## 14. Required negative tests

A later executable tranche should include at least these adversarial cases:

```text
Knowledge E4 -> deserialize as Core E4                     REJECT
Core N3 -> infer Knowledge N3                              REJECT
Knowledge M2 -> infer Core M2                              REJECT
Knowledge 12-harmony payload -> Core HarmonicImpact        REJECT
Core 8-vector -> fabricate Knowledge resonance metadata    REJECT
bare {"empirical":"E4"} at federation boundary           REJECT / AMBIGUOUS
unknown schema version -> silently use current mapping     REJECT
lossy translation without loss declaration                 REJECT
derived interpretation presented as source-authored        REJECT
```

Positive controls should include:

```text
same-schema round-trip preserves exact identity             PASS
opaque transport preserves unknown schema bytes/ref         PASS
side-by-side multi-schema comparison preserves both         PASS
explicit lossy mapping emits loss declaration/receipt       PASS
```

## 15. Migration posture

No existing E/N/M/H implementation should be mechanically rewritten as part of this tranche.

The safe order is:

```text
identify
  -> namespace
  -> freeze schema refs
  -> add collision tests
  -> define explicit projections/translations if justified
  -> only then consider domain migrations
```

A migration from Knowledge to core, core to Knowledge, or both to a future decomposed model would be a separate semantic change requiring domain-level review and evidence that information is preserved or intentionally deprecated.

## 16. Implications for Integral and other external systems

External systems must never be forced to adopt either Mycelix epistemic schema merely to exchange evidence.

An Integral CDS/FRS adapter, municipality, cooperative, scientific workflow, or conventional organization should be able to supply:

```text
source schema + source object + provenance
```

and optionally provide local projections/interpretations.

Mycelix can then distinguish:

```text
we can parse this
we can verify its integrity
we understand its source semantics
we have a local interpretation
we accept it as evidence for a particular purpose
we authorize action based on it
```

Those are separate states.

## 17. Result

The current Core and Knowledge E/N/M/H systems are **not mutually convertible scalar schemas**.

For the present interoperability architecture:

- E is different in meaning;
- N is orthogonal;
- M is orthogonal;
- H differs in ontology cardinality and representation;
- verification state is separate again;
- production mode (`EpistemicKind`) should remain a separate dimension.

The correct near-term architecture is therefore **namespaced coexistence with explicit translation receipts**, not premature unification.

The next executable step should be a very small namespace-safe `SemanticRef` / `SchemaRef` prototype that can refer to both schemas without importing or converting either payload.
