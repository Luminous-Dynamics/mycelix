# MYC-FL-SA-000R — Robust / Secure Aggregation Composition Contract

## Status

Cross-cutting reconciliation contract between the canonical federated-learning and
secure-aggregation lineages.

Relevant current lineages include:

```text
MYC-FL-001A
  #1842  typed aggregation admission
  #1844  theorem-bound KrumV1
  #1859  robustness-scope contract
  #1861  initial independent Krum fixtures

MYC-SA
  #1800  secure-aggregation constitution
  #1809  crypto-independent protocol-core implementation plan
```

This document is based on `main` so neither unqualified child lineage becomes an
accidental prerequisite of the other.

## Governing theorem

```text
secure aggregation
    +
Byzantine-robust aggregation
    !=
automatically composable system
```

A secure-sum protocol may intentionally hide every individual participant update
from the server.

A robust selector such as Krum may require pairwise distances between individual
participant updates.

Those two requirements conflict unless an additional protocol computes or proves
the robustness predicate without exposing prohibited information.

Therefore Mycelix must never advertise:

```text
Secure Aggregation: ON
Krum: ON
```

as a valid combined profile merely because both modules exist.

## 1. Distinct theorems

Keep these claims separate:

```text
confidentiality of individual updates
integrity/authenticity of participant submissions
input-domain validity
Byzantine robustness
robust selector correctness
aggregate correctness
privacy accounting
model convergence
model promotion authority
```

No one theorem implies another.

Examples:

```text
encrypted contribution
    !=
valid bounded contribution

valid bounded contribution
    !=
honest contribution

secure sum
    !=
Byzantine-robust sum

robust aggregation
    !=
differential privacy

correct aggregate
    !=
safe model
```

## 2. Why ordinary secure sum and Krum conflict

Canonical KrumV1 requires, conceptually:

```text
for each admitted update i:
    compute distance(i, j) for every other admitted update j
    sort distances
    sum n-f-2 nearest squared distances
select minimum-score participant
```

A conventional secure-sum protocol instead aims to reveal only something like:

```text
sum_i update_i
```

while hiding each `update_i`.

If the aggregator receives only the sum, it cannot reconstruct the pairwise
geometry required by Krum.

Required distinction:

```text
secure sum primitive
    !=
secure evaluation of arbitrary aggregation rule
```

## 3. CompositionProfileV1

Every combined privacy/robustness deployment should name an explicit composition
profile.

Conceptually:

```rust
pub struct FlSecureAggregationCompositionProfileV1 {
    pub profile_id: ProfileId,
    pub confidentiality_profile: ConfidentialityProfileRef,
    pub input_validity_profile: InputValidityProfileRef,
    pub robustness_profile: RobustnessProfileRef,
    pub compute_profile: SecureComputeProfileV1,
    pub leakage_profile: LeakageProfileV1,
    pub trust_profile: TrustProfileV1,
    pub dropout_profile: DropoutProfileV1,
    pub qualification_profile: QualificationProfileRef,
}
```

The exact Rust representation may differ. The semantic separation must remain.

## 4. SecureComputeProfileV1

Reserve distinct profiles for genuinely different trust/computation theorems.

At minimum:

```rust
pub enum SecureComputeProfileV1 {
    PlaintextRobustAggregator,
    SecureSumWithValidatedInputs,
    SecretSharedRobustAggregation { profile: ProfileRef },
    MpcRobustAggregation { profile: ProfileRef },
    HomomorphicRobustAggregation { profile: ProfileRef },
    TeeRobustAggregation { profile: ProfileRef },
    DomainDefined { profile: ProfileRef },
}
```

These names do not imply qualification merely by existing.

## 5. PlaintextRobustAggregator

This is the simplest robust path:

```text
participant update
    -> authenticated channel
    -> server sees update
    -> Krum / Multi-Krum / other robust rule
```

It can support full pairwise geometry but does not provide secure-aggregation
confidentiality from the aggregator.

Possible protections such as TLS, access control, disk encryption, retention
policy or isolated execution are valuable but are different from cryptographic
secure aggregation.

Receipt wording must not say `secure aggregation` for this profile.

## 6. SecureSumWithValidatedInputs

This family keeps additive secure aggregation but constrains contributions before
they enter the sum.

Examples of possible validity predicates:

```text
L2 norm <= bound
L-infinity norm <= bound
coordinate range
shape/schema correct
finite values
model subject matches
participant authenticated
one contribution per participant/round
```

Privacy-preserving proofs or commitments can establish some of these predicates
without revealing the whole vector.

This profile can mitigate important classes of model-replacement/scaling attacks.
It does **not** become Krum merely because the input is bounded.

Required distinction:

```text
bounded secure sum
    !=
distance-based robust selection
```

## 7. SecretSharedRobustAggregation

Secret sharing can permit multiple non-colluding or threshold parties to evaluate
functions over hidden participant inputs.

A specific profile must freeze:

```text
sharing scheme
field/ring and encoding
number of compute parties
privacy threshold
Byzantine threshold
collusion assumption
malicious/semi-honest security model
verification/commitment scheme
dropout handling
reconstruction rules
robust aggregation function
numeric quantization/error bounds
```

Do not summarize all of those as:

```text
MPC enabled
```

Secret-shared Krum-like distance evaluation is a separate protocol theorem from
ordinary secret-shared summation.

## 8. MpcRobustAggregation

General MPC can in principle evaluate rich robust selectors while hiding inputs,
but cost and leakage depend heavily on the exact function.

For a Krum-family profile, qualification must include at least:

```text
secure pairwise squared distance
secure sorting / nearest-neighbor selection
secure score comparison
secure deterministic tie resolution
secure selected-vector extraction
output reconstruction
```

The protocol must say who learns:

```text
selected participant identity
candidate scores
pairwise distances
aggregate only
failure reason
```

Those are different leakage profiles.

## 9. HomomorphicRobustAggregation

Homomorphic encryption may support some robust-compute profiles, but:

```text
FHE available
    !=
Krum efficiently/easily available
```

Pairwise distances, comparisons, sorting and argmin can be much more expensive
than additions.

A homomorphic profile must bind:

```text
scheme
parameter set
encoding / fixed-point semantics
supported operations
comparison strategy
bootstrapping/noise profile
ciphertext packing
key ownership / threshold model
numeric error bounds
result-verification profile
```

A homomorphic filtering classifier is also not semantically equivalent to Krum.
It gets its own robustness profile.

## 10. TeeRobustAggregation

A TEE profile may allow plaintext robust computation inside an isolated execution
environment while hiding inputs from the surrounding host.

Its trust theorem differs materially from cryptographic MPC/FHE.

Freeze at least:

```text
TEE technology/profile
attestation profile
measured code identity
key-release policy
side-channel assumptions
rollback protection
host threat model
I/O binding
receipt/attestation linkage
```

A TEE may be a practical deployment profile, but must never be presented as
trust-equivalent to threshold cryptography.

## 11. InputValidityProfileV1

Secure aggregation should distinguish contribution validity from contribution
honesty.

Conceptually:

```rust
pub enum InputValidityProfileV1 {
    StructuralOnly,
    NormBound { profile: ProfileRef },
    RangeBound { profile: ProfileRef },
    NormAndRange { profile: ProfileRef },
    ProofCarrying { profile: ProfileRef },
    DomainSpecific { profile: ProfileRef },
}
```

Hard rule:

```text
proof that x is bounded
    !=
proof that x was honestly trained
```

## 12. Proof-carrying contributions

A future contribution may bind:

```text
ciphertext commitment
round
participant
model subject
shape
range/norm predicate
key epoch
proof profile
signature
```

Conceptually:

```rust
ContributionValidityProofV1 {
    contribution_commitment,
    predicate_profile,
    proof_system_profile,
    verification_key_profile,
    public_inputs_commitment,
}
```

This composes naturally with Mycelix's ZKP foundation, but proof-system
qualification remains separate from secure aggregation and FL robustness.

## 13. ConfidentialityProfileV1

Do not use one `encrypted=true` flag.

Possible distinctions include:

```text
server sees individual plaintext updates
server sees bounded metadata but not vectors
server sees distances/scores but not vectors
server sees selected IDs but not vectors
compute parties see shares
TEE sees plaintext inside enclave
only aggregate revealed
aggregate + validity failures revealed
```

The exact leakage is part of the theorem.

## 14. LeakageProfileV1

Conceptually:

```rust
pub struct LeakageProfileV1 {
    pub participant_identity_visible: bool,
    pub vector_visible_to_aggregator: bool,
    pub vector_visible_to_compute_party: LeakageCardinalityV1,
    pub norms_visible: bool,
    pub distances_visible: bool,
    pub scores_visible: bool,
    pub selected_ids_visible: bool,
    pub aggregate_visible: bool,
    pub rejection_reason_visibility: RejectionVisibilityV1,
}
```

A secure protocol can still leak sensitive metadata. Receipt/UI language should be
based on this explicit profile rather than the presence of ciphertext.

## 15. TrustProfileV1

At least distinguish:

```text
single trusted aggregator
honest-but-curious aggregator
malicious aggregator
non-colluding multi-server
threshold honest-party assumption
TEE hardware/vendor trust
fully malicious MPC profile
domain-defined trust
```

`decentralized` is not a trust profile.

## 16. DropoutProfileV1

Federated rounds are not static laboratory sets.

A secure-aggregation profile must specify:

```text
minimum collection threshold
when roster freezes
whether dropouts before/after contribution are tolerated
whether recovery reveals extra information
whether Byzantine f is defined before or after dropout
whether robustness preconditions are re-evaluated after dropout
```

Hard rule:

```text
Krum feasible at round start
    !=
Krum feasible after dropouts
```

The final admitted set determines algorithm feasibility.

## 17. Byzantine count semantics

Secure aggregation and robust aggregation may use different thresholds.

Keep separate:

```text
collection_threshold
privacy/reconstruction_threshold
compute-party corruption threshold
client Byzantine bound f
client dropout bound d
```

One integer called `threshold` must not own all of them.

## 18. Robustness under hidden inputs

If individual vectors are hidden, the system needs an explicit answer to:

```text
where does the robustness decision happen?
```

Valid answers might include:

```text
before encryption at participant          // weak unless independently verified
after commitment using proofs             // predicate-limited
inside MPC                                 // protocol-specific
inside FHE                                 // operation/cost-specific
inside TEE                                 // hardware-trust-specific
at multiple secret-sharing compute nodes  // threshold/collusion-specific
not performed                              // secure sum only
```

`somewhere in the secure layer` is not sufficient.

## 19. CompositionDispositionV1

Never expose `secure_and_robust: bool`.

Use a typed disposition.

Conceptually:

```rust
pub enum CompositionDispositionV1 {
    StructurallyCompatible,
    QualifiedForProfile { qualification: QualificationRef },
    Incompatible { reason: CompatibilityFailureV1 },
    OutsideQualifiedScope { reason: ScopeMismatchV1 },
    Indeterminate { missing: Vec<MissingEvidenceV1> },
}
```

Example incompatibility:

```text
requested robustness: KrumV1
secure compute profile: aggregate-only secure sum
result: Incompatible(PairwiseGeometryUnavailable)
```

## 20. Compatibility matrix

The first registry should explicitly encode capabilities rather than relying on
naming conventions.

Illustrative matrix:

| Secure-compute profile | Secure sum | Norm/range validation | Pairwise geometry | Arbitrary robust selector |
|---|---:|---:|---:|---:|
| PlaintextRobustAggregator | yes | yes | yes | yes |
| SecureSumWithValidatedInputs | yes | yes | no | no |
| SecretSharedRobustAggregation | profile-specific | profile-specific | profile-specific | profile-specific |
| MpcRobustAggregation | yes | yes | yes if implemented | yes if implemented |
| HomomorphicRobustAggregation | yes | profile-specific | possible/expensive | profile-specific |
| TeeRobustAggregation | yes | yes | yes | yes within attested code |

This table is descriptive, not qualification evidence.

## 21. KrumV1 compatibility

`KrumV1` specifically needs:

```text
admitted individual vectors
pairwise squared Euclidean distance
nearest-neighbor selection
score ordering
argmin/tie-break
selected-vector output
```

Therefore:

```text
PlaintextRobustAggregator
    -> semantically compatible

SecureSumWithValidatedInputs
    -> incompatible with KrumV1 itself
       unless another secure-compute phase exposes/evaluates required geometry

MPC/FHE/TEE robust profiles
    -> potentially compatible only if their exact qualified profile implements
       the KrumV1 arithmetic and tie semantics
```

## 22. MultiKrumV1 compatibility

Multi-Krum adds:

```text
m selected contributions
secure top-m score selection
selected-set extraction
specified output weighting
```

Do not assume a Krum-capable secure-compute profile automatically supports
Multi-Krum.

## 23. Differential privacy composition

DP may occur at different points:

```text
local DP before secure aggregation
central DP after secure aggregation
DP inside secure computation
DP after robust filtering
```

These have different utility and threat semantics.

For distance-based robust rules, adding local noise before selection changes
geometry and can change qualification scope.

Hard rule:

```text
KrumV1 qualified on raw admitted updates
    !=
KrumV1 qualified on locally DP-noised updates
```

Combined profiles must qualify the exact ordering.

## 24. Research patterns informing the registry

Mycelix should treat outside systems as architectural evidence, not copy targets.

### Privacy-preserving input validity

RoFL demonstrates one important family: secure aggregation plus privacy-preserving
validation of norm constraints on encrypted/committed client updates.

Architectural lesson:

```text
secure sum + proof of bounded input
```

is useful and meaningfully stronger than unconstrained secure sum, but is not a
substitute for arbitrary robust selection.

### Secure robust selection / computation

Published privacy-preserving Byzantine-robust FL systems use additional secure
computation such as multi-party computation, secret sharing, encrypted filtering,
or other specialized protocols to combine confidentiality with malicious-update
handling.

Architectural lesson:

```text
privacy + robustness requires an explicitly qualified composition protocol
```

not two independent checkboxes.

### TEE robust aggregation

TEE-based work demonstrates another practical trust point: plaintext robust
computation can occur inside attested confidential execution.

Architectural lesson:

TEE composition should be supported as a separate trust profile rather than
pretending it has the same theorem as MPC/FHE.

## 25. SecureAggregationPlan relationship

The future `SecureAggregationPlanV1` should be able to reference a composition
profile when used for FL:

```rust
SecureAggregationPlanV1 {
    ...,
    application_profile: Some(
        ApplicationProfile::FederatedLearning {
            composition_profile: FlSecureAggregationCompositionProfileRef,
        }
    ),
}
```

The secure-aggregation core should not import FL implementation code. It should
bind an opaque typed application profile/reference.

## 26. FL AggregationContext relationship

Likewise the FL context may reference confidentiality/secure-compute evidence:

```rust
AggregationContextV1 {
    ...,
    confidentiality_profile: Option<ProfileRef>,
    secure_compute_receipt: Option<ReceiptRef>,
}
```

This is evidence composition, not crate-level circular dependency.

## 27. Receipt composition

A combined round should retain separate receipts:

```text
Contribution admission receipts
        |
Secure-compute / secure-aggregation receipt
        |
Aggregation receipt
        |
Privacy release receipt (if any)
        |
Model-update / promotion decision
```

A higher-level round receipt may bind all of them by commitment/reference.

Do not flatten them into one `verified=true` object.

## 28. Failure semantics

Examples of typed failures:

```text
PairwiseGeometryUnavailable
UnsupportedRobustnessProfile
UnsupportedNumericProfile
LeakageProfileMismatch
TrustProfileMismatch
ByzantineBoundInfeasibleAfterDropout
InputValidityProofMissing
InputValidityProofRejected
SecureComputeQualificationMissing
RobustnessQualificationMissing
CompositionQualificationMissing
```

The UI may summarize them, but authoritative state should remain typed.

## 29. Browser / Leptos presentation

A first-party Leptos UI should present composition truth explicitly.

Example:

```text
Federated round
---------------
Update confidentiality     Aggregate-only secure sum
Input bounds               L2 proof verified
Robust selector            Not available in this privacy profile
Byzantine filtering        None
Differential privacy       Pending
Secure-aggregation receipt Bound
Robustness qualification   Not applicable
```

versus:

```text
Update confidentiality     TEE-confidential
Robust selector            KrumV1
TEE attestation            Verified profile
Krum qualification         Profile Q17
Deployment data regime     Outside Q17 (non-IID severity unknown)
Robustness disposition     Indeterminate
```

The UI must not strengthen `encrypted` into `robust` or `qualified`.

## 30. Qualification direction — MYC-FL-SA-001Q

A future compatibility/receipt qualification should include at least:

1. secure-sum-only + Krum request rejects as incompatible;
2. secure-sum-only + FedAvg sum profile is structurally compatible;
3. norm-proof secure sum does not claim Krum;
4. missing input-validity proof fails the profile requiring it;
5. invalid proof fails closed;
6. plaintext Krum profile reports confidentiality limitation;
7. MPC profile cannot claim Krum unless pairwise geometry/selection capability is admitted;
8. FHE profile cannot claim Krum from additive capability alone;
9. TEE profile requires attestation binding to exact robust-compute code;
10. trust-profile substitution changes/rejects composition identity;
11. leakage-profile substitution changes/rejects composition identity;
12. numeric-profile substitution changes/rejects Krum qualification;
13. dropout can invalidate final Krum feasibility;
14. collection threshold and client Byzantine f remain separate;
15. reconstruction threshold and client Byzantine f remain separate;
16. secure-compute-party corruption threshold remains separate from client f;
17. local-DP-before-Krum requires a distinct qualification profile;
18. central-DP-after-aggregation does not retroactively prove input robustness;
19. selected-ID leakage matches declared leakage profile;
20. score leakage matches declared leakage profile;
21. combined receipt binds exact secure-aggregation receipt;
22. combined receipt binds exact aggregation receipt;
23. receipt substitution is rejected;
24. unsupported profile remains unsupported rather than silently downgraded;
25. exact-head qualification and immutable checkout.

## 31. Proposed implementation order

Do not build a giant privacy-preserving robust-FL crate.

Prefer:

```text
MYC-FL-SA-000R
  this compatibility constitution

MYC-FL-SA-001A
  pure profile/capability/compatibility types

MYC-FL-SA-001Q
  compatibility-matrix qualification

MYC-SA-002A
  one real secure aggregation backend

MYC-FL-SA-002A
  one explicitly compatible FL composition profile
```

The first real combined backend should be selected based on the deployment theorem,
not novelty.

A practical sequence could begin with either:

```text
A. SecureSumWithValidatedInputs
   useful for bounded FedAvg-style aggregation
   simpler theorem

or

B. TeeRobustAggregation
   practical Krum-capable confidentiality profile
   explicit hardware trust
```

A full malicious-secure MPC/FHE Krum profile can follow as a stronger but more
expensive theorem.

## 32. Nonclaims

MYC-FL-SA-000R does not establish:

- cryptographic security;
- a real secure-aggregation backend;
- Krum over ciphertexts;
- malicious-secure MPC;
- TEE side-channel security;
- threshold FHE;
- Byzantine robustness;
- non-IID robustness;
- dropout tolerance;
- differential privacy;
- anonymity;
- participant authenticity;
- model convergence;
- deployment safety;
- scientific validity;
- legal compliance.

Its purpose is to prevent a much more dangerous architectural error:

```text
independently valid security/robustness modules
    -> incorrectly composed
    -> stronger claim than either module actually supports
```

Mycelix should make that category of mistake structurally difficult.