# IG-007C1 — Governance-config authorization counterexamples

## Parent

IG-007C1 is a child of IG-007C0 / draft #946 and is evidence for P0 #943.

Bound observed profile:

```text
mycelix-governance-config-observed-fca2c107-v1
de4435a69356557b1812f8beb46d654c66b9c957be9d18c64bd0431f92546d5a
```

Authority of the profile remains `ObservedSourceBound`.

The counterexample corpus has `MeasurementOnly` authority.

## Corpus identity

```text
schema mycelix-governance-config-counterexamples-v1
SHA-256 3009e97529934fa8f470769de5615dcfda17bde0e942683e939d2b733430a216
```

## CE-CFG-01 — existence-only proposal predicate

Fixture abstraction:

```text
proposal_id             MIP-DRAFT-FIXTURE
modeled proposal state  Draft
get_proposal             SomeRecord
status inspected         false
type inspected           false
```

Frozen result:

`AuthorizationContinuesAfterExistenceOnly`

The `Draft` label demonstrates why record existence and governance authorization are distinct. The fixture does not execute a config mutation and does not claim a live Draft proposal was used to change production state.

## CE-CFG-02 — policy flexibility under shape validation

The corpus freezes a pure threshold vector:

```text
Default:
basic          0.2
proposal       0.3
voting         0.4
constitutional 0.6

Candidate:
basic          0.1
proposal       0.3
voting         0.4
constitutional 0.6
```

Both vectors are finite, inside `[0,1]`, and nondecreasing.

Frozen result:

`StructurallyValidLowerRuntimeGate`

This is deliberately **not** classified as a defect by itself. Governance may legitimately choose a lower or different threshold under an authorized versioned policy. The defect under #943 is that the observed mutation path does not establish the declared authorization theorem before exercising that flexibility.

A successor should therefore preserve legitimate policy flexibility while closing unauthorized mutation.

## CE-CFG-03 — DHT shape validity is not authority

Fixture abstraction:

```text
changed_by_proposal                MIP-FIXTURE
config shape                       StructurallyValid
integrity validator                check_consciousness_config
proposal authority reconstruction  NoneObserved
entry author authorization         NoneObserved
```

Frozen result:

`IntegrityAcceptsShapeWithoutObservedProposalAuthorityPredicate`

This models the source-visible validation contract. It does not create a live DHT entry.

## Why three fixtures

The three counterexamples separate questions that are easy to conflate:

```text
Does a proposal record exist?
!=
Is this actor authorized to make this exact change?

Is the config numerically valid?
!=
Is this config change authorized?

Can policy thresholds change?
!=
Should policy thresholds be immutable?
```

The future fix should answer the authorization question without accidentally collapsing legitimate mechanism-design freedom.

## Successor test

A corrected profile should stop reproducing CE-CFG-01 and CE-CFG-03 because proposal/action/caller authority becomes explicit.

CE-CFG-02 may continue to be mathematically true under an authorized policy. What must disappear is the implication that shape validity plus proposal existence is sufficient authority.

## Relationship to #944

#944 remains a propagation/contract issue. An eventual constitution-to-runtime sync should consume the corrected authorized config-mutation surface and produce a reconciliation receipt.

## Non-claims

No live config mutation, exploit success, deployment currentness, or normative claim about the correct consciousness thresholds.