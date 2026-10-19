# MYC-SEM-000A — Mycelix Semantic Kernel Charter v1

Status: source architecture subject. This document defines boundaries only. It does not establish runtime qualification, compatibility, authority, or migration correctness.

## Purpose

Mycelix needs a small shared semantic substrate because independent domains are repeatedly rediscovering the same distinctions between evidence, currentness, authority, effects, and reconciliation.

The kernel standardizes **how meaning is identified and admitted**. It does not standardize every domain concept that can carry meaning.

The core pipeline is:

```text
domain state
    -> observation
    -> evidence
    -> appraisal / interpretation
    -> qualified semantic assertion
    -> currentness
    -> authority admission
    -> decision
    -> effect admission
    -> execution observation
    -> effect receipt
    -> reconciliation
```

Every arrow is an explicit semantic boundary. Later stages may depend on earlier receipts; they may not silently collapse them.

## Permanent distinctions

The kernel must preserve at least these non-equivalences:

```text
observation != truth
evidence != truth
attestation != trust

identity != attribution
attribution != mandate
mandate != current authority

policy match != authorization unless all required evidence is admitted

authorization != decision
decision != execution permission
execution permission != execution

provider invocation succeeded != domain mutation committed

receipt exists != desired effect occurred
receipt exists != obligation satisfied
receipt exists != legal discharge

historical validity != current validity

projection != observation
prediction != measurement

schema migration != semantic equivalence
semantic translation != authority transfer
```

No future convenience API may erase these distinctions.

## Crate topology

The intended dependency-light shared stack is:

```text
mycelix-semantic-core
        |
        +-----------------------+
        |                       |
        v                       v
mycelix-evidence-core   mycelix-semantic-migration
        |
        v
mycelix-authority-core
        |
        v
mycelix-effect-core
```

Initial home:

```text
mycelix-workspace/crates/
```

The kernel is not a replacement for the existing domain clusters.

## Dependency law

Kernel crates MUST NOT depend on:

- Holochain / HDK / HDI;
- Leptos or browser APIs;
- Xenia runtime code;
- Symthaea runtime or reasoning code;
- `mycelix-bridge-common`;
- a domain cluster such as Finance, Governance, Personal, Hearth, or Praxis;
- ambient network access;
- ambient process-global authority;
- an ambient wall clock.

Domains and adapters MAY depend on kernel crates.

Interop adapters MAY depend on both the external format/protocol and the relevant kernel crate, but the kernel MUST NOT depend on those adapters.

## Domain ownership rule

The kernel does not define universal domain objects such as:

```text
Property
Payment
Vote
MedicalRecord
LearningCredential
GitRef
MunicipalBudget
```

Those stay domain-owned.

A shared abstraction is admitted to the kernel only when at least one of the following is true:

1. it has been independently demonstrated by two or more domain implementations; or
2. it is required as an explicit interoperability primitive.

This is an anti-speculation rule.

## Semantic environments are load-bearing

Every authority-relevant or migration-relevant semantic object must be interpretable under an explicit semantic environment.

A semantic environment binds the profiles required to interpret a subject, including at minimum:

- schema;
- interpretation;
- identity;
- authority;
- temporal semantics;
- canonicalization.

Therefore:

```text
same textual identifier under environment A
!= same semantic subject under environment B
```

unless an explicit qualified translation or migration receipt establishes the required relationship.

There is no implicit `latest` environment.

A mutable registry may help locate profiles but may never redefine what a historical receipt meant.

## Existing crate boundaries

### `mycelix-claim-types`

`mycelix-claim-types` remains the ecosystem's epistemic/knowledge classification surface.

LEM coordinates, confidence values, and claim categories are useful descriptive metadata. They are not authority admissions.

```text
high confidence != authority
E4 empirical classification != authorization
broad normative agreement != mandate
```

The semantic kernel must not overload that crate.

### `mycelix-bridge-common`

`mycelix-bridge-common` remains the bridge/gating/runtime integration layer.

The semantic kernel must not become another bridge helper module and must not require HDK.

Existing civic gating may later be wrapped by compatibility receipts and migrated incrementally; it is not replaced by this charter.

### `Migratable`

The existing bridge `Migratable` trait concerns reading/upgrading versioned entry representations.

Semantic migration concerns preservation, loss, or transformation of meaning between explicit semantic environments.

```text
entry schema upgrade != semantic migration
```

Both mechanisms may coexist.

## Positive-type construction law

Authority-bearing positive receipts introduced after the semantic foundation must:

- have private fields;
- have no unchecked public constructor;
- be produced only through validating constructors/verifiers;
- bind their exact semantic environment and governing profiles;
- bind the evidence/receipts on which the positive result depends;
- state an explicit authority ceiling;
- avoid generic deserialization when deserialization would itself mint the theorem.

Input/wire types should reject unknown fields where compatibility permits and should use explicit bounded collections and text.

## Canonical identity law

Authority-relevant identity must not depend on:

- JSON object order;
- `serde` field order;
- `Debug` output;
- Rust memory layout;
- map iteration order;
- locale-specific formatting.

Canonical commitments must be:

- domain-separated;
- versioned;
- language-neutral;
- byte-order explicit;
- independently testable with frozen vectors.

## Time law

The kernel must not turn local wall-clock access into trusted currentness.

```text
signed timestamp != trusted now
source revision != trusted now
Holochain action timestamp != universal now
runner wall clock != evidence time
```

Chronology is evidence. Currentness is a separately admitted theorem under an explicit trusted-time/currentness profile.

## Conflict and completeness are independent

The kernel must preserve at least two independent axes:

```text
conflict disposition
coverage / completeness disposition
```

Therefore:

```text
no detected conflict != sufficient evidence
complete evidence != uncontested evidence
```

Arrival order, larger revision, higher reputation, or a newer-looking untrusted timestamp may not silently resolve semantic conflict.

## Delegation law

The generic kernel may verify that a domain-specific attenuation verifier produced a receipt establishing:

```text
child scope <= parent scope
under attenuation profile P
```

The generic kernel must not pretend it understands the partial ordering of every domain's scopes.

A municipal-spending scope, Git ref scope, medical-consent scope, and learning-credential scope may require unrelated domain verifiers.

## Effects law

The effect pipeline must preserve:

```text
effect intent
!= effect admission
!= dispatch attempt
!= invocation success
!= effect receipt
!= reconciled effect
```

A successful provider call is not automatically a successful domain mutation.

Reconciliation establishes only the profiled relationship it actually checked; it does not automatically establish commercial satisfaction, human acceptance, legal discharge, or obligation discharge.

## Federation, translation, and migration

Cross-environment translation is loss-aware and explicit.

A translation receipt must identify at least:

- source environment and subject;
- target environment and subject;
- translation profile;
- preserved properties;
- unavailable, lossy, or opaque properties.

Default rule:

```text
semantic translation does not transfer authority
```

Authority transfer requires a separate profiled authority admission.

Migration similarly binds source and target environments plus the transformation and provenance required to explain the relationship.

## Qualification baseline

Kernel crates should default to:

```text
#![forbid(unsafe_code)]
```

and remain:

```text
no Holochain
no Leptos
no ambient system clock
no network
no mutable process-global authority
```

The adversarial corpus should eventually include:

- cross-environment substitution;
- cross-subject substitution;
- cross-policy substitution;
- profile-revision substitution;
- evidence replay;
- same-ID/different-content conflict;
- semantic contradiction;
- incomplete evidence presented as complete;
- untrusted timestamp presented as currentness;
- delegation amplification;
- revoked delegation reuse;
- authority for A replayed on B;
- authorized intent with altered payload;
- provider-target mismatch;
- invocation success promoted to effect success;
- receipt promoted to satisfaction;
- migration promoted to equivalence;
- translation promoted to authority;
- unknown future profile/enum values;
- oversized collections.

## First implementation line

This charter authorizes only the first dependency-light semantic foundation:

```text
MYC-SEM-001A  primitive semantic references
MYC-SEM-001B  semantic environment commitment
MYC-SEM-001C  canonical commitment profile + frozen vectors
```

Evidence, currentness, authority, effects, migration, federation, and external interoperability remain later subjects.

## Nonclaims

Adopting this charter does not establish:

- truth;
- trusted currentness;
- authorization;
- governance legitimacy;
- settlement;
- legal validity;
- interoperability;
- semantic equivalence across versions;
- safe migration;
- runtime execution correctness.

It establishes the architectural boundary under which those later theorems can be proved without conflating them.
