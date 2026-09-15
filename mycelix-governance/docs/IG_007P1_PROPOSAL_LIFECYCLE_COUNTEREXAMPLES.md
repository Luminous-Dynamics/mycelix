# IG-007P1 — Legacy proposal lifecycle counterexamples

Issue: #994

Parent: IG-007P0 / draft #993

Underlying authority finding: #66

## Purpose

IG-007P1 turns the frozen legacy Proposal lifecycle semantics into five deterministic source-contract counterexamples so future #66 repairs can demonstrate an exact before/after improvement without rewriting historical evidence.

No live Proposal update is published by this tranche.

## Bound profile

```text
profile id     mycelix-proposal-lifecycle-observed-fca2c107-v1
profile SHA    7f42e2a8df25df94112d23f261d1f3ffe299d46d37cb3a5a6fe02aca0aa6c108
authority      ObservedSourceBound
production     fca2c107a1ea5108823ce617ba4111b6f7f77230
same-tree main 31ede2365b81365bb119cd9351b2739119974130
```

## Corpus identity

```text
schema         mycelix-proposal-lifecycle-counterexamples-v1
authority      MeasurementOnly
SHA-256        13eaaa988c73d29d67bccf7381f6f72cabd4eb7be090f36f0b444978cc708324
counterexamples 5
issue          #66
```

SHA-256 is deterministic corpus identity only.

## CE-PROP-01 — stale ProposalById projection

Fixture:

```text
creation status       Draft
existing update child Active
ProposalById target   creation action
linked record present true
```

The frozen source profile says the primary lookup returns the linked record without traversing its updates and only reaches the local-chain fallback if linked lookup does not return a record.

The fixture therefore records:

```text
observed read status       Draft
authoritative currentness  NotEstablished
```

This is a source-semantics example, not a claim that every deployed proposal has an Active child behind a stale link.

## CE-PROP-02 — Draft→Active can carry semantic-content mutation through the pure structural check

Fixture:

```text
original status          Draft
updated status           Active
id/author                unchanged
version                  original + 1
semantic content         changed
```

The frozen content-freeze predicate is:

`OriginalStatusNotDraft`.

For a Draft parent that predicate is false, so the pure update check does not reject the simultaneous title/description/actions/proposal-type mutation.

Frozen result:

`ContentMutationNotRejectedByObservedUpdateCheck`.

## CE-PROP-03 — update action author is not an integrity predicate

Fixture conceptually separates:

```text
Proposal.author      Alice
Update action author Bob
```

with structurally valid Proposal fields.

The frozen profile records:

`update_action_author_binding = NoneObserved`.

Result:

`IntegrityDoesNotEstablishUpdateAuthorAuthority`.

No live unauthorized publication is performed.

## CE-PROP-04 — temporal fields are not update-bound

The fixture starts from a valid creation interval, then models an update with:

- reversed voting start/end ordering;
- changed `created` metadata;
- arbitrary `updated` metadata.

The source-bound profile records no update-level:

- voting-start immutability;
- voting-end immutability;
- `created` immutability;
- `updated`↔Holochain action timestamp binding;
- voting-period ordering recheck.

Frozen result:

`TemporalMutationNotRejectedByObservedUpdateCheck`.

This does not claim the modeled record was published to a live DHT.

## CE-PROP-05 — sibling update ambiguity

The fixture models one Draft parent with two individually shape-valid version-2 children:

```text
child A -> Active
child B -> Cancelled
```

Both transition shapes exist in the allowed transition set.

The legacy observed profile has:

```text
explicit competing-update fork rule = NoneObserved
ProposalById update refresh          = NoneObserved
```

Frozen result:

`NoObservedDeterministicAuthoritativeChildSelection`.

No DHT arrival/timestamp winner is inferred.

## Successor comparison boundary

A repaired or migrated Proposal lifecycle should create a new profile/corpus where the old fixtures intentionally stop reproducing—for example because:

- linked/current reads use an authority-safe projector;
- activation content is frozen;
- lifecycle update authority is explicit;
- temporal fields are bound;
- sibling forks fail closed deterministically.

The stronger draft successor stack (#44/#59/#63+) remains a separate lineage until qualified/deployed. P1 does not silently substitute that design for current production.

## Qualification

The exact-head workflow:

1. binds the exact Proposal coordinator/integrity blobs;
2. syntax-compiles P0/P1 scripts;
3. revalidates P0 twice byte-identically;
4. runs P1 self-test twice byte-identically;
5. emits the P1 corpus twice byte-identically;
6. asserts exact profile/corpus commitments and CE-PROP-01..05 outcomes;
7. verifies checkout immutability.

A hosted PASS would establish deterministic source-contract measurement only.

## Non-claims

No live unauthorized update, deployment exploit, authoritative-currentness, successor-stack deployment, or governance-safety claim is made.
