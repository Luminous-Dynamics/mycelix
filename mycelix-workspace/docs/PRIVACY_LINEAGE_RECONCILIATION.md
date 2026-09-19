# MYC-DP-000R — Differential Privacy Lineage Reconciliation

## Purpose

Reconcile the currently visible Mycelix differential-privacy implementations
before creating another privacy crate or promoting any one implementation into a
canonical authority surface.

This document is a code-lineage and semantic reconciliation contract. It is not
a privacy proof, not a claim of differential privacy, and not a production
qualification.

## Governing theorem

```text
implemented noise + budget code
    !=
qualified differential privacy
```

and:

```text
same words (epsilon, delta, budget, Gaussian)
    !=
same privacy theorem
```

The ecosystem already contains several independent DP implementations. The next
step is convergence and qualification, not another parallel implementation.

## Known implementation lineages

### 1. Rust: `mycelix-differential-privacy`

Path:

```text
mycelix-workspace/mycelix-core/libs/differential-privacy/
```

Current source includes dedicated modules for:

- mechanisms;
- clipping;
- privacy budgets;
- composition;
- Python bindings.

The composition module includes:

- advanced composition;
- a moments accountant;
- Rényi DP accounting;
- zCDP accounting.

This is the strongest current **canonical candidate** by breadth, but breadth is
not qualification. Some formulas are documented as simplified/conservative
bounds and need independent validation before authority-bearing use.

Initial disposition:

```text
CanonicalCandidate
NeedsIndependentValidation
NeedsRngProfile
NeedsCanonicalReceiptModel
```

### 2. Rust: `mycelix-fl-core::privacy`

Path:

```text
mycelix-workspace/crates/mycelix-fl-core/src/privacy.rs
```

Implements:

- L2 gradient clipping;
- Gaussian noise via Box-Muller;
- a simple RDP budget tracker;
- DP pipeline/report helpers.

Important issue: the non-`std` path derives a deterministic RNG seed from the
sensitive gradient contents for reproducibility.

That may be useful for tests but must never qualify as production DP randomness.

Freeze:

```text
DeterministicTestNoise != DifferentialPrivacyNoise
```

Initial disposition:

```text
AbsorbOrAdapterCandidate
UnsafeForAuthorityOnDeterministicRngPath
NeedsAccountingCrossCheck
```

### 3. Rust: legacy/current `mycelix-fl::privacy`

Path:

```text
mycelix-workspace/crates/mycelix-fl/src/privacy.rs
```

The source itself labels the implementation a stub/scaffold for formal
accounting. It contains:

- clipping;
- seeded xorshift + Box-Muller noise;
- a simple epsilon budget;
- a one-round Gaussian epsilon estimate.

Because callers supply a deterministic seed and the implementation uses xorshift,
this must not be treated as a cryptographically appropriate production privacy
randomness source.

Initial disposition:

```text
HistoricalOrPrototype
UsefulForCompatibilityVectors
DoNotPromoteAsCanonical
```

### 4. TypeScript: agentic/trust analytics DP

Path:

```text
mycelix-workspace/sdk-ts/src/agentic/differential-privacy.ts
```

Implements:

- Laplace and Gaussian mechanisms;
- simple privacy budget tracking;
- private mean/count/histogram helpers;
- trust-distribution analytics.

Important issues to reconcile:

- randomness uses `Math.random()`;
- budget composition is simple additive accounting;
- `reset()` clears consumed privacy state without a typed privacy-epoch lineage;
- the code exposes high-level private analytics APIs that can look stronger than
  the underlying accounting theorem.

Initial disposition:

```text
CompatibilitySurfaceCandidate
PrototypeSemantics
DoNotUseAsCanonicalAuthority
```

### 5. TypeScript: FL Hub privacy

Known paths include:

```text
mycelix-workspace/sdk-ts/src/fl-hub/privacy.ts
mycelix-workspace/sdk-ts/src/fl-hub/types.ts
```

The SDK defines additional privacy-budget/accounting concepts independently of
the Rust lineages.

Initial disposition:

```text
NeedsSemanticDiffAgainstRustCanonicalCandidate
FutureFacadeOrAdapter
```

### 6. Other repository/sibling DP implementations

The wider Luminous/Mycelix source history also contains DP code in Health/HDC and
legacy archives. These must be included in the full census before declaring
source closure.

A sibling implementation is not automatically safe to copy merely because it is
Rust or domain-specific.

Initial disposition:

```text
NeedsCensus
DomainSpecificUntilProvenGeneric
```

## Reconciliation table

Every discovered implementation should receive a machine-reviewable disposition:

```text
CanonicalCandidate
Absorb
Adapter
CompatibilitySurface
DomainSpecific
Historical
Prototype
UnsafeForAuthority
NeedsIndependentValidation
Superseded
```

More than one disposition may apply during migration.

No implementation may become canonical merely because it has the most features.

## Canonical semantic contract required before promotion

A privacy claim must bind more than `epsilon` and `delta`.

At minimum a future qualified release profile should bind:

```text
privacy unit
neighboring-dataset / adjacency relation
query/release subject
contribution bounds
clipping/sensitivity theorem
sampling scheme
sampling rate where applicable
mechanism
noise calibration
randomness profile
accountant/composition profile
epsilon
delta
number/order of releases or equivalent accounting state
privacy epoch identity
implementation/profile version
```

Without these, an epsilon value is not independently interpretable.

## Mechanism config vs privacy guarantee

The implementation must separate configuration like:

```text
clip_norm
noise_multiplier
```

from a qualified claim like:

```text
(epsilon, delta)-DP under exact adjacency/contribution/sampling/accounting profile
```

Preset names such as `high_privacy`, `moderate_privacy`, or `low_privacy` must
not act as authority-bearing privacy claims unless they resolve to a complete
qualified profile.

## Randomness theorem

Production mechanisms requiring randomness must identify a qualified entropy
profile.

Candidate classes:

```text
DeterministicTestRng
OsCsprng
BrowserWebCryptoCsprng
QualifiedExternalRng(profile)
```

Rules:

1. deterministic test RNGs are allowed for fixtures/oracles only;
2. xorshift is not a production DP entropy claim;
3. `Math.random()` is not a production DP entropy claim;
4. RNG provenance belongs in the release/implementation profile;
5. reproducibility of test noise must never be represented as production privacy.

## Budget and epoch theorem

Privacy loss is historical state.

A software method named `reset()` must not semantically mean:

```text
previous privacy loss disappeared
```

A future canonical API should model an explicit epoch transition:

```text
PrivacyEpoch N --closed--> PrivacyEpoch N+1
```

with the policy/evidence that justifies why the new epoch has a separate budget.

The old epoch remains reconstructible audit evidence.

## Accountant theorem

Different accountants are different semantic profiles.

Examples:

```text
SimpleComposition
AdvancedComposition
RdpAccountant
MomentsAccountant
ZcdpAccountant
```

Do not silently convert between them or compare epsilon values as if they were
produced under the same assumptions.

Simplified subsampling formulas or approximations must be explicitly profiled and
validated against an independent reference before they become authoritative.

## Independent reference strategy

The strongest next step is not internal self-consistency alone.

Use at least one independent DP implementation/oracle for frozen vectors.

A suitable external library may be used as:

```text
reference oracle
cross-check backend
future qualified backend adapter
```

but external-library presence is not itself qualification. Version, enabled
features, numerical assumptions, mechanism selection and exact profile identity
must be frozen.

## Canonical crate decision

Do **not** create `mycelix-privacy-core` until the lineage census is complete.

After reconciliation, choose one of two paths deliberately:

### Option A — promote/restructure existing Rust library

```text
mycelix-differential-privacy
```

becomes the canonical core, with FL/domain adapters depending on it.

### Option B — create `mycelix-privacy-core`

Only if the existing crate's dependency/API/history boundaries make a new narrow
core materially cleaner. In that case, migrate/absorb proven semantics rather
than reimplementing them from memory.

The choice must be evidence-driven.

## Required future API boundaries

A canonical Rust privacy layer should eventually separate:

```text
mechanism primitives
privacy definitions/profiles
accountants
RNG/entropy providers
release planning
release execution
receipts/reports
compatibility adapters
```

FL should consume privacy APIs; privacy code should not depend on FL-specific
application state unless it is in an adapter crate.

## Receipt direction

A future `DpReleaseReceiptV1` should bind at minimum:

```text
release subject/input commitment
privacy profile
adjacency/privacy-unit profile
contribution/sensitivity profile
sampling profile
mechanism profile
randomness profile
accountant identity/state before + after
privacy epoch
epsilon/delta disposition
output commitment
implementation/qualification identity
explicit nonclaims
```

A receipt is evidence of the implemented release theorem, not proof that the raw
data was truthful, the analysis was scientifically appropriate, or every
external copy was protected.

## Adversarial qualification corpus

The first canonical executable tranche should include at minimum:

1. invalid epsilon;
2. invalid delta;
3. zero/negative noise scale;
4. invalid clipping bound;
5. NaN/Infinity inputs;
6. contribution-bound violation;
7. wrong privacy unit;
8. wrong adjacency profile;
9. wrong sampling rate/profile;
10. accountant/profile substitution;
11. deterministic test RNG presented as production -> reject;
12. xorshift/`Math.random` production profile -> reject;
13. privacy epoch reset without explicit transition -> reject;
14. budget monotonicity within one epoch;
15. composition monotonicity;
16. frozen independent reference vectors;
17. finite-arithmetic edge cases;
18. cross-language canonical profile vectors where SDK compatibility remains;
19. stale receipt cannot mint fresh budget;
20. exact-head locked qualification and immutable checkout.

## Migration direction

The intended convergence is:

```text
existing Rust canonical candidate
          |
          +-- validated/absorbed FL privacy semantics
          +-- domain-specific adapters
          +-- independent reference vectors
          |
          v
canonical Rust privacy theorem
          |
          +-- native consumers
          +-- WASM/browser compute adapter
          +-- Holochain/domain adapters
          +-- TypeScript compatibility facade
```

TypeScript should remain an excellent SDK boundary where useful, but first-party
authority-bearing semantics should not remain independently reimplemented in both
Rust and TypeScript.

## Immediate next tranche

`MYC-DP-001A` should begin only after this reconciliation is reviewed.

It should be deliberately narrow:

- choose/promote the canonical Rust owner;
- add explicit RNG profile injection;
- remove authority from deterministic/test randomness paths;
- define privacy epoch/accountant state types;
- define one qualified Gaussian mechanism/accounting profile;
- add frozen independent vectors;
- introduce typed release receipts;
- keep compatibility adapters separate.

Do not attempt to support every DP mechanism in the first qualification.

## Nonclaims

MYC-DP-000R does not establish differential privacy, legal privacy compliance,
anonymity, unlinkability, secure deletion, scientific validity, production
security, or fitness of any current implementation.

It establishes only the migration/reconciliation contract needed before one
canonical privacy line can be qualified.
