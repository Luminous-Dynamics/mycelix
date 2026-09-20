# FHE-001A — Backend-Neutral FHE Capability Contract v1

Status: architectural contract only

Tracks: #2122, #2111, #2109, #2110

Parent semantic waist: PEC-001A / #2116

## Purpose

Freeze the semantics a real fully homomorphic encryption backend must declare before Mycelix can treat it as anything stronger than an experimental implementation.

The governing rule is:

```text
FHE API exists
    != ciphertext confidentiality established
    != selected parameters adequate
    != requested function supported
    != threshold decryption secure
    != metadata/access-pattern privacy
    != production admission
```

FHE-001A selects no library and qualifies no backend.

## Historical simulation containment

The historical TypeScript FHE surface remains `SimulationOnly` under PEC-000A / FHE-000.

```text
historical FHEClient
    != real FHE backend
```

No compatibility adapter may erase that disposition.

## Scheme-family semantics

A backend profile must bind an exact scheme family and numeric/function model. The semantic categories include, without implying universal equivalence:

```text
ExactModularInteger
ApproximateReal
BooleanOrGate
LookupTableFunctional
```

Typical families such as BFV/BGV, CKKS, and TFHE/FHEW-like systems have materially different correctness, precision, operation, and bootstrapping semantics.

Therefore:

```text
FHE supported != arbitrary application computation supported
```

## Exact backend identity

Every profile must bind at least:

- backend/project identity;
- exact version/commit or release identity;
- enabled feature set;
- target architecture;
- CPU/GPU/WASM execution mode;
- parameter/profile identity;
- serialization format/version;
- compiler/toolchain identity where relevant to retained qualification evidence.

A backend family name alone is not a security-bearing identity.

## Parameter identity

Security and correctness depend on exact cryptographic parameters. A profile must bind all theorem-bearing parameters through an exact structured identity or digest.

Depending on scheme, this can include concepts such as:

- polynomial/ring dimension;
- ciphertext/plaintext modulus parameters;
- decomposition/base parameters;
- error/noise distribution profile;
- security estimator/profile/version;
- key-switch/relinearization parameters;
- bootstrapping parameters;
- packing/slot configuration.

```text
same scheme + different parameters != same security profile
```

## Operation capability

A backend profile must declare the exact operation/function class it supports, including as applicable:

```text
AddCiphertext
AddPlaintext
MultiplyCiphertext
MultiplyPlaintext
RotateOrPermute
Compare
LookupTable
Bootstrap
KeySwitch
Relinearize
```

Availability of an API method is not enough: the admitted profile must state parameter-specific depth/noise/precision constraints.

## Noise, depth, and bootstrapping

For leveled/noise-bearing profiles, the planner must track whether a requested computation fits the admitted multiplicative depth/noise/precision budget.

```text
operation sequence type-checks
    != ciphertext remains correctly decryptable
```

Where bootstrapping is used, the profile must bind the exact bootstrapping mechanism and parameters rather than treating `supports_bootstrap=true` as universal evidence.

## Approximate arithmetic boundary

Approximate-real schemes require explicit numeric semantics.

A profile must bind, as applicable:

- encoding scale/precision policy;
- rescaling policy;
- expected error/tolerance model;
- overflow/range assumptions;
- deterministic comparison policy for application thresholds.

```text
decrypts near expected value
    != exact arithmetic equality
```

An application requiring exact integer/accounting semantics must not silently receive an approximate-real profile.

## Key-role model

The profile must name which principals hold or receive:

- secret/decryption key material;
- public/encryption material;
- evaluation keys;
- relinearization keys;
- rotation/Galois keys;
- bootstrapping keys;
- threshold shares where applicable.

It must also state generation, distribution, retention, rotation, revocation/currentness, and destruction assumptions relevant to the use case.

```text
ciphertext secure under key K
    != K handled securely by deployment
```

## Decryption-recipient semantics

Every plan must bind who is permitted and technically able to decrypt outputs.

Examples of semantic roles may include:

```text
SingleClient
NamedRecipient
ThresholdRecipientSet
NoApplicationDecryptionUntilLaterStage
```

These are architectural roles, not authorization by themselves.

## Threshold FHE separation

Threshold FHE is a separate profile, not a boolean option on single-key FHE.

```text
single-key FHE != threshold FHE
secret sharing != threshold FHE
DKG available != threshold FHE qualified
```

A threshold profile must additionally bind:

- distributed/threshold key-generation protocol;
- corruption/adversary model;
- decryption threshold;
- collusion assumptions;
- share refresh/currentness if used;
- distributed decryption protocol;
- robustness/abort behavior;
- exact compatibility with the underlying FHE parameter profile.

## Multi-key FHE separation

If future backends support multi-key FHE, that capability must remain distinct from threshold FHE.

```text
MultiKeyFHE != ThresholdFHE
```

The input-key ownership and output-decryption semantics differ and require separate profiles.

## Function privacy boundary

Ordinary FHE protects plaintext data under the stated model; it does not automatically hide the evaluated function/program from the evaluator or other parties.

```text
input confidentiality != function privacy
```

Function privacy, if required, must be represented as a separate requirement/mechanism.

## Verifiable computation boundary

FHE evaluation alone does not necessarily prove the evaluator executed the requested computation correctly.

```text
ciphertext output produced
    != requested function correctly evaluated
```

Where malicious evaluator correctness matters, compose an independently qualified proof/verifiable-computation mechanism or another profile that establishes the required integrity theorem.

## Metadata/access-pattern boundary

FHE does not automatically hide:

- client/server identities;
- ciphertext sizes;
- number/timing of requests;
- circuit/function shape;
- database access patterns outside the encrypted computation model;
- traffic volume.

These must flow into the PEC leakage model.

## Serialization and parameter-confusion safety

Ciphertext/key envelopes must bind scheme/backend/parameter identity strongly enough to reject cross-profile confusion.

```text
bytes deserialize != ciphertext admitted for this profile
```

Unknown versions/parameters must fail closed.

## Resource and denial-of-service boundary

FHE can have large key/ciphertext sizes and substantial CPU/GPU/memory requirements. A backend profile must therefore expose measured resource ceilings and admission limits rather than accepting attacker-controlled dimensions indefinitely.

Measure, as applicable:

- key generation latency/memory;
- public/evaluation key size;
- ciphertext expansion;
- operation latency;
- bootstrapping latency;
- GPU memory;
- CPU memory;
- serialization/deserialization cost;
- WASM/client cost;
- network transfer cost.

## Licensing/patent/deployment metadata

Backend selection must carry deployment metadata for license and patent constraints separately from technical security.

```text
open-source license != patent grant
license compatible != cryptographically secure
```

The planner may treat deployment constraints as feasibility inputs, but legal interpretation remains outside the cryptographic theorem.

## Current research context

FHE remains an active standardization/reference-material area. NIST's 2026 Threshold Call includes FHE as a special category, reinforcing that exact specifications, implementation evidence, evaluation, and patent information matter for threshold/FHE assessment.

This context does not qualify any backend for Mycelix.

## First backend-evaluation tranche

Before selection, evaluate at least two materially different candidate implementations/profiles where practical. Record exact versions and compare:

- supported scheme/function families;
- security/parameter evidence;
- CPU/GPU/WASM portability;
- threshold capabilities;
- performance and memory;
- serialization stability;
- maintenance maturity;
- license/patent constraints;
- independent audit/qualification evidence.

A benchmark winner is not automatically the production choice.

## Planner fail-closed examples

```text
requirement: ExactIntegerAccounting
candidate: ApproximateReal
=> Incompatible(NumericSemanticsMismatch)
```

```text
requirement: ThresholdOutputDecryption
candidate: SingleKeyOnly
=> Incompatible(KeyModelMismatch)
```

```text
requirement: SequenceAccessPatternPrivacy
candidate: FHE only
=> Incompatible(AccessPatternPrivacyUnavailable)
```

## Relationship to PEC

FHE-001A consumes the shared PEC requirement, leakage, adversary, backend identity, qualification, and composition semantics. It must not create a second `secure: bool` authority path.

## Nonclaims

FHE-001A establishes no FHE security level, parameter adequacy, numeric correctness for an application, threshold-FHE security, function privacy, malicious-evaluator correctness, metadata privacy, patent/license conclusion, production admission, application authority, or deployment readiness.
