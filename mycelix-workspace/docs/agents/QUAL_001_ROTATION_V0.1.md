# QUAL-001R Rotation-Capable Verifier Genesis v0.2 — Fixed Contract

Status: **candidate; not adopted; independent topology inactive**

Issues: #866, #938, #947, #1107, #1114

## Purpose

QUAL-001R extends qualified QUAL-001A with a rotation-capable verifier while preserving:

```text
TheoremSubject
!= QualificationVerifier
!= QualificationExecutionEnvironment
!= QualificationReceipt
```

Exact parent: `9f707a765631e1dd3416d5a32249c2f190a6ccdf`.

## Semantic identity

```text
profile  mycelix.qual.static-subject-independence.v0.2-monotonic
bundle   1360eff37924a6ce580e7363ab094e9f76caf3b36887b57202fafa628b76f028
pointer  fe1af51c278f67459a96b1a1405645bc047443eae7de3b0b93a734432ee04e97
gate     69264268de3383b22ea0ac9392e76c30a7d94f333e2955f068050ed238112644
core     a93af3422785695b601bd873f316d1119a34cd41a4c8c5bb18b93adde7ca9c5f
wrapper  3541fd050d4eb4c76db324b664481b7d3ac9b98fbc4b15b409ac050222005030
policy   72db15ea4d1f335e0553d1b9a30e49a45f2195ce6b7bed151e367ed0ad198a12
base self-test
         39d7aa3e67b4bd92f7ab8bb4dbab11c58b25beb2c2998529d0f20124e36f0ece
fixed-contract self-test
         50853049e235676f4f45ce032983e1bb4cab567c0b8dc4ae51f1f5d50b1d6532
```

Predecessor bundle:
`b0281549127d0aa892a347ebffd32c5d9e59f8b6c26b3a478db852dc4366a964`

Predecessor bootstrap receipt:
`e13ea1d651f1a5a2cfb19ceba483136488a2ff4275d25273b9935e18e5614fc2`

## Ordinary rotation theorem

Successor verifier code is hostile **data** during pre-adoption authorization and is not executed.

Ordinary V0.2 rotation may replace verifier implementation bytes while preserving the complete declared security/continuation contract. It authorizes exactly one registered successor bundle profile:

`mycelix.qual.static-subject-independence.v0.3`

The successor must also preserve:

- gate profile and rotation-policy profile exactly;
- receipt schema exactly;
- gate and rotation path/byte bounds exactly;
- forbidden exact-path and prefix censuses set-wise;
- base and rotation receipt-field censuses set-wise;
- current-pointer path exactly;
- rotation exact/prefix allowlists set-wise;
- immutable-launcher-path census set-wise;
- launcher and successor component censuses set-wise;
- `subject_execution=false`;
- same-repository subject requirement;
- repository-protection requirement;
- `admin_bypass_qualifies=false`;
- candidate/subject execution prohibition; and
- predecessor-bundle continuity.

```text
ordinary verifier implementation rotation
!= verifier security-contract transition
```

Any contract or profile transition outside this named next profile requires a separately qualified semantic-transition theorem.

## Hostile evidence

The focused fixed-contract suite proves rejection of:

- unregistered successor bundle-profile substitution;
- gate-profile substitution;
- rotation-policy-profile substitution;
- gate/policy path and byte bounds widened or shrunk;
- receipt obligations dropped or impossible obligations added;
- forbidden-path coverage weakened or over-restricted;
- rotation allowlists broadened or narrowed;
- launcher immutability removed or expanded onto the current pointer; and
- successor component obligations added or removed.

Negative cases count only `RuntimeError`-class verifier rejection. Fixture/programming exceptions do not qualify as security evidence.

The base rotation suite separately preserves verifier-shadow, wrong-predecessor, mixed-product, component-tamper, launcher-mutation, and control/newline-path rejection.

## Lifecycle

`.github/workflows/qual-001-bootstrap.yml` is retired in this subject.

```text
QUAL-001R exact-head PASS
+ #938 P0 protection PASS
+ #1114 exact ancestry/byte adoption PASS
+ #1107 hostile-subject independence PASS
-> #938 P1 verifier enforcement eligibility
```

## Non-claims

```text
QUAL-001R bootstrap PASS != P0 repository protection
QUAL-001R bootstrap PASS != #1114 adoption PASS
QUAL-001R bootstrap PASS != #1107 hostile-subject independence PASS
QUAL-001R bootstrap PASS != P1 verifier enforcement
QUAL-001R bootstrap PASS != active independent topology
QUAL-001R PASS != arbitrary successor-verifier correctness
admin break-glass != qualified rotation
QUAL-001 receipt != runtime authority
```
