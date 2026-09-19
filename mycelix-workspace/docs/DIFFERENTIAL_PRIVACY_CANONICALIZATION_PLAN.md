# MYC-DP-001A — Canonical Differential Privacy Implementation Plan

## Status

Planning child of `MYC-DP-000R` / draft PR #1780.

This document does not establish differential privacy. It freezes the smallest
implementation tranche that may begin converting the current Rust candidate into
a canonical Mycelix privacy mechanism/accounting substrate.

## Candidate owner

The first candidate to promote is the existing Rust crate:

```text
mycelix-workspace/mycelix-core/libs/differential-privacy
package: mycelix-differential-privacy
```

It already contains dedicated mechanism, clipping, budget, composition and
Python-binding modules. MYC-DP-001A should improve this implementation in place
before considering a new `mycelix-privacy-core` crate.

A rename or physical move is explicitly deferred until semantic qualification
shows what the canonical surface actually is.

## Governing theorem

```text
noise sampled
+ budget object updated
!=
qualified differential privacy
```

A qualified privacy claim requires one exact profile binding at least:

```text
privacy unit
adjacency relation
contribution/sensitivity bounds
sampling semantics
mechanism theorem/profile
randomness profile
accountant/composition profile
epsilon/delta
privacy epoch
implementation/profile identity
release subject
```

None may be inferred from a friendly preset name.

## Current candidate strengths

The existing crate already provides useful foundations:

- Gaussian, Laplace and exponential mechanism implementations;
- clipping primitives;
- sequential budget tracking;
- composition/accountant code;
- RDP/zCDP-related machinery in the existing composition lineage;
- Rust-native implementation;
- optional Python bindings;
- focused tests and benchmarks.

The correct migration is therefore **harden and qualify**, not reimplement.

## Current hardening findings

### Non-finite inputs

Current mechanism constructors reject ordinary negative/zero invalid values but
comparison checks such as `epsilon <= 0.0` do not reject `NaN`.

MYC-DP-001A must reject all non-finite authoritative parameters before any
mechanism/accountant construction:

```text
NaN
+Infinity
-Infinity
```

This applies to epsilon, delta, sensitivity, clipping bounds, noise scales,
sampling rates and accountant parameters.

### Ambient randomness

Current mechanisms internally create an RNG. That makes the randomness source
implicit and prevents the caller/receipt from binding an exact entropy profile.

Production-capable APIs should receive a randomness provider through an explicit
boundary rather than constructing one invisibly inside `add_noise()`.

### Budget reset semantics

The current budget type contains `reset()` which zeroes counters and clears
history with a warning that it should only be used at epoch boundaries.

Canonical semantics must instead model epoch rollover explicitly:

```text
PrivacyEpoch A --closed--> immutable historical state
PrivacyEpoch B --opened--> fresh budget state
```

No API should imply that clearing process memory erases cumulative privacy loss.

### Serialization gap

Current expenditure history uses process-local `Instant` and is skipped during
serialization. That is reasonable for local diagnostics but insufficient as the
canonical durable privacy lineage.

The qualified path needs stable epoch/release identities and typed receipts.
Process-local timing may remain non-semantic diagnostic metadata.

## Proposed type surface

MYC-DP-001A should introduce or freeze equivalents of the following without
forcing every downstream domain to use identical UI/domain DTOs.

### Privacy unit

```rust
pub enum PrivacyUnitV1 {
    Record,
    User,
    Participant,
    Household,
    Device,
    Session,
    DomainDefined(ProfileRef),
}
```

The exact canonical form may differ, but the privacy unit must be explicit.

### Adjacency

```rust
pub enum AdjacencyProfileV1 {
    AddRemoveOne,
    SubstituteOne,
    BoundedContribution,
    DomainDefined(ProfileRef),
}
```

`same epsilon` under different adjacency models is not the same theorem.

### Contribution bounds

Conceptually:

```rust
ContributionBoundsV1 {
    max_records_per_unit,
    value_lower,
    value_upper,
    norm_bound,
    profile,
}
```

Fields should be mechanism/query specific rather than forcing meaningless values.

### Sampling

```rust
SamplingProfileV1 {
    scheme,
    probability_or_rate,
    replacement_semantics,
    population_definition,
}
```

Do not infer sampling amplification from a generic `sample_rate` field.

### Randomness

```rust
pub enum RandomnessProfileV1 {
    DeterministicTestOnly { fixture_id: String },
    NativeCsprng { profile: ProfileRef },
    BrowserCsprng { profile: ProfileRef },
    ExternalQualified { profile: ProfileRef },
}
```

Hard rule:

```text
DeterministicTestOnly
    cannot produce
QualifiedPrivacyRelease
```

The core mechanism API should use an injected RNG trait/provider. Platform
adapters own native/browser entropy acquisition.

### Mechanism profile

```rust
MechanismProfileV1 {
    mechanism_id,
    theorem_profile,
    sensitivity_profile,
    numeric_profile,
    parameters,
}
```

Do not make the enum name `Gaussian` by itself mean every Gaussian theorem or
calibration formula is qualified.

### Accountant profile

```rust
AccountantProfileV1 {
    accountant_id,
    composition_rule,
    orders_or_parameters,
    theorem_profile,
}
```

Sequential composition may be the first simple qualified profile. RDP/zCDP or
subsampled profiles qualify separately.

### Privacy epoch

```rust
PrivacyEpochV1 {
    epoch_id,
    subject_scope,
    profile,
    opened_at_evidence,
    predecessor_epoch,
}
```

An epoch close/open is lineage, not reset.

### Release receipt

```rust
PrivacyReleaseReceiptV1 {
    release_id,
    epoch_id,
    subject_commitment,
    privacy_profile,
    mechanism_profile,
    accountant_profile,
    randomness_profile,
    prior_accountant_state_commitment,
    resulting_accountant_state_commitment,
    claimed_epsilon,
    claimed_delta,
    output_commitment,
    implementation_qualification_ref,
    terminal_disposition,
}
```

The receipt is evidence. It is not consent, purpose authorization, anonymity or
permission to disclose the result.

## API split

Prefer a two-level API.

### Mechanism primitive

A low-level mechanism can perform a mathematically specified randomized
transformation with explicit parameters and injected randomness.

It should **not** return `dp_guaranteed = true`.

### Qualified release

A higher-level release builder verifies the full privacy profile, accountant
state, entropy classification and release context before producing a typed
receipt.

Conceptually:

```text
Mechanism primitive
        +
Privacy profile
        +
Accountant state
        +
Qualified randomness
        +
Release subject
        ->
Qualified release attempt + receipt
```

## Numeric validity

Every authoritative floating-point parameter must pass explicit finite/domain
validation before arithmetic.

At minimum reject:

- NaN;
- positive/negative infinity;
- epsilon <= 0;
- invalid delta for the selected theorem;
- sensitivity/bounds outside selected profile;
- sampling rate outside selected profile;
- zero-round division and invalid count/budget state;
- arithmetic results that become non-finite.

A constructor succeeding must mean the object satisfies its local structural
invariants; it does not by itself mean the overall privacy theorem is qualified.

## Gaussian profile discipline

Do not expose one global `GaussianMechanism == (epsilon, delta)-DP` theorem.

The first qualified Gaussian profile should name the exact calibration formula,
its parameter preconditions, adjacency/sensitivity semantics and numeric model.

If a stronger analytic Gaussian implementation or external reference backend is
adopted later, give it a distinct profile identity rather than silently changing
v1 semantics.

## External reference / oracle

Use an independent implementation as a qualification oracle where practical.

The oracle is not automatically runtime authority and its optional/experimental
features must be recorded rather than hidden.

Reference vectors should bind:

```text
profile
inputs
expected calibration/accounting result
tolerance / exact comparison rule
oracle implementation/version/features
```

Random output samples themselves are not expected to match across independent
RNG streams; calibration/accounting and deterministic statistical fixtures can.

## Migration of existing lineages

### `mycelix-fl-core`

Keep FL-specific clipping/DP adapters only where they express FL semantics.
Mechanism/accounting ownership should migrate to or wrap the canonical privacy
crate after qualification.

The deterministic non-production RNG path must remain visibly test-only.

### `mycelix-fl`

The caller-seeded xorshift implementation is historical/prototype code. Preserve
fixtures if valuable, but do not retain it as a second production privacy owner.

### TypeScript agentic DP

Move algorithmic authority out of TypeScript. The long-term SDK may expose a
compatibility API backed by Rust/WASM or a remote qualified service, but
`Math.random()` must not remain a production DP entropy path.

### FL Hub / Health / HDC

Classify each discovered implementation as:

```text
Adapter
DomainSpecificProfile
Absorb
Historical
Prototype
UnsafeForAuthority
```

Do not delete before differential fixtures and consumers are identified.

## Persistence and rollback

Privacy accounting is consequential durable state.

A process restart, browser refresh, worker restart or local cache deletion must
not allow a previously consumed privacy budget to become unused again on the
strongest path.

MYC-DP-001A may initially implement only the pure state types and transitions;
durable storage integration can be a child. The theorem must already make
rollback/replay semantics explicit.

Required distinction:

```text
accountant state unavailable
!=
zero privacy expenditure
```

If required prior state is unavailable, fail closed or return an explicit
indeterminate/unavailable disposition for authoritative release.

## Browser relationship

The canonical DP core remains UI/runtime independent.

A later browser adapter may obtain qualified entropy and execute through
`mycelix-browser-worker`, but:

```text
worker PASS
!=
DP PASS
```

The browser compute receipt and DP release receipt compose as separate evidence.

## MYC-DP-001AQ qualification corpus

The first executable qualification must include at least:

1. NaN epsilon rejected;
2. infinite epsilon rejected;
3. NaN/infinite delta rejected;
4. NaN/infinite sensitivity rejected;
5. invalid clipping/contribution bounds rejected;
6. invalid sampling parameters rejected;
7. deterministic test RNG cannot produce qualified-release disposition;
8. production profile requires an admitted randomness profile;
9. same deterministic fixture reproduces deterministic calibration/accounting;
10. accountant state never decreases inside one epoch;
11. negative expenditure rejected;
12. non-finite expenditure rejected;
13. budget overrun rejected;
14. epoch rollover preserves predecessor identity;
15. old epoch history cannot be erased by `reset()` semantics;
16. missing predecessor accountant state is not interpreted as zero;
17. profile substitution changes/rejects the release identity;
18. adjacency substitution changes/rejects the release identity;
19. privacy-unit substitution changes/rejects the release identity;
20. randomness-profile substitution changes/rejects the release identity;
21. accountant substitution changes/rejects the release identity;
22. independent calibration/accounting vectors pass for the selected theorem;
23. finite-arithmetic adversarial fixtures are bounded and fail closed;
24. serialization round trip preserves stable privacy lineage fields;
25. process-local diagnostic timestamps are not semantic identity;
26. stale release receipt cannot be replayed as fresh budget authority;
27. exact-head locked qualification and checkout immutability.

## Suggested first code diff

Keep MYC-DP-001A deliberately narrow. Prefer roughly:

```text
existing crate
  + finite parameter validators
  + explicit RNG abstraction/profile
  + privacy profile identifiers
  + epoch/accountant state identity
  + typed release receipt
  + migration/deprecation annotations
  + adversarial tests
```

Do not simultaneously rewrite every accountant, integrate every domain, add
browser WASM, and retire every legacy implementation.

## Deferred children

Potential later tranches:

```text
MYC-DP-002A  independently qualify advanced/RDP accounting profiles
MYC-DP-003A  durable accountant state + rollback/replay protection
MYC-DP-WASM  browser adapter / CSPRNG + worker integration
MYC-DP-SDK   TypeScript/Python compatibility adapters
MYC-DP-FL    mycelix-fl-core canonical privacy integration
MYC-DP-FED   federated analytics release composition
```

Names are provisional; keep each theorem narrow.

## Nonclaims

Passing MYC-DP-001A would not establish legal/privacy compliance, anonymity,
unlinkability, secure deletion, consent, purpose legitimacy, representativeness,
statistical validity, fairness, protection against arbitrary side channels, or
correctness of every future accountant/mechanism profile.

It would establish only the frozen canonical privacy semantics and qualified
behavior for the exact selected v1 profile(s).