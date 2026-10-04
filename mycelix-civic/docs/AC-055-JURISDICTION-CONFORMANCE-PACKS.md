# AC-055 — Jurisdiction Conformance Packs

## Purpose

AC-055 makes the Economic OS deployable across jurisdictions without requiring
a kernel fork.

An `EconomicPolicyProfile` describes the policy context. An
`EconomicJurisdictionPack` now declares what that profile actually implements,
which external representations it supports, what authority references apply,
and where interoperability is lossy.

This distinction is important:

**profile = what policy context exists**

**pack = what this implementation actually conforms to**

## Why this matters

The international economic-data ecosystem already has multiple standards with
different scopes. The 2025 SNA is the international standard for national
accounts; BPM7 covers external-sector statistics; SDMX standardizes statistical
data and metadata exchange; SEEA integrates environmental-economic information;
and ISO 20022 provides common financial-message semantics.

No single one of these should become the Economic OS kernel.

Instead, a jurisdiction pack can explicitly declare the mappings and guarantees
available for its implementation.

## Pack guarantees

A pack must identify:

- the exact policy profile;
- Economic OS kernel compatibility version;
- every supported operation;
- exactly one implementation capability per supported operation;
- authority references for capabilities that require authority;
- interoperability references;
- semantic preservation guarantee;
- deterministic conformance-test references;
- evidence supporting the declaration;
- explicit semantic-loss declarations for lossy mappings;
- optional predecessor/supersession identity.

### Interoperability guarantees

`Lossless` means the mapping preserves the semantics represented by the
capability.

`LossyDeclared` means some semantics cannot be represented and that loss is
declared explicitly.

`HumanReadableOnly` means the representation is for human review rather than
machine round-tripping.

`Unsupported` cannot be advertised as a supported Economic OS capability.

## Historical interpretation

A historical event should bind to:

1. its policy-profile reference;
2. the exact policy-profile fingerprint;
3. its event/payload identity;
4. the authority and evidence relevant at the time.

A later policy change should therefore create a new profile version rather than
mutating the interpretation of already-finalized history.

## No country forks

The preferred deployment model is:

**Economic OS kernel**
+
**jurisdiction profile**
+
**jurisdiction conformance pack**
+
**payment/statistical/legal adapters**

rather than a fork of the kernel for every country.

A kernel change should be justified by a global semantic requirement, not by a
single country's policy.

## Cross-border semantics

A cross-border transaction may involve two or more policy profiles.

The event envelope therefore needs to retain the semantic event identity and
the profile context rather than attempting to flatten multiple jurisdictions
into one policy namespace.

Cross-border settlement may use an external payment system while the Economic
OS preserves the higher-level action, evidence, reconciliation, and finality
history.

This is aligned with the direction of BIS Project Agorá, which is testing a
multi-currency shared programmable platform for wholesale cross-border
payments while retaining central-bank and commercial-bank money and
jurisdictional arrangements.

## Current reference implementation

AC-055 adds:

- `EconomicOsCapability`
- `InteroperabilityGuarantee`
- `EconomicJurisdictionPack`
- exact profile-fingerprint binding for governed monetary-policy application
- declared-loss validation
- operation/capability completeness checks.

## Important non-claims

A conformance pack does not prove:

- legal compliance;
- validity of the referenced authority;
- correctness of an economic policy;
- truthfulness of all underlying observations;
- universal interoperability with standards not explicitly mapped;
- that every country should use the same monetary regime.

It is an explicit technical conformance declaration.

## Research references

- UN, System of National Accounts 2025:
  https://unstats.un.org/unsd/nationalaccount/sna2025.asp
- IMF, BPM7 release and implementation:
  https://www.imf.org/en/publications/policy-papers/issues/2025/07/10/release-of-new-standards-for-macroeconomic-statistics-bpm7-568453
- SDMX 3.1 Technical Specifications:
  https://sdmx.org/standards-2/
- BIS Project Agorá:
  https://www.bis.org/project/agora
