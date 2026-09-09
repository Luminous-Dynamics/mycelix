# Mycelix Authority Historical Operational Set v0.1 — Normative Invariants

Status: **pure closed-set composition of already-qualified historical operational authority**

This layer answers one narrow question:

> Does the caller possess exactly one valid #481 historical operational authority capability for every exact required `(subject, generation, transition digest)` coordinate, with no missing, extra, duplicate, cross-root, or stale proof?

It does not discover history, qualify source coverage, select a causal coordinate, or create historical authority itself.

## 1. Requirements are transportable, not authority

`HistoricalOperationalAuthorityRequirement` may be serialized/deserialized because it is only a statement of what must be proven.

It contains exactly:

- `AuthoritySubjectRef`;
- target generation; and
- target transition digest.

Deserializing a requirement establishes no authority.

## 2. Positive inputs are opaque #481 capabilities only

The proof side accepts only `QualifiedHistoricalOperationalAuthority` references.

Loose caller fields such as authority digest, evidence digest, root digest, generation or verification time cannot substitute for the opaque positive capability.

## 3. Exact closed-set bijection

The number of qualified proofs must equal the number of requirements.

Each requirement is keyed by:

`subject.identity_digest + generation + transition_digest`

and each qualified proof must match exactly one such key and then pass full subject equality plus exact generation/digest equality.

The theorem rejects:

- duplicate requirements;
- duplicate qualified proofs;
- missing proofs;
- unexpected/extra proofs; and
- any binding mismatch.

## 4. Input order is never authority

Requirements and proofs are canonicalized with a `BTreeMap` over the exact key above.

The stable set identity and evidence identity are derived in that canonical order. Reordering caller arrays cannot change the qualified set.

## 5. Requirement fan-in is bounded

The set is bounded to 4,096 exact requirements, matching the Identity #454 lineage bound.

Zero requirements or a larger set fail before composition.

## 6. One current verification root

Every #481 capability in one set must carry the same bootstrap-root qualification digest and profile.

This does not mean the current root retroactively created each historical decision. It means one closed verification operation may not silently combine historical proofs accepted under different current verification-root contexts.

## 7. Historical authority remains exact per coordinate

The set never reduces multiple coordinates to “this signer is generally authorized.”

If the same subject appears at two different signed authority-state coordinates, both requirements remain distinct and both proofs are required.

## 8. Stable authority vs dynamic evidence

The stable set `qualification_digest` commits:

- exact verification-root identity;
- exact number of requirements;
- each canonical subject identity;
- each exact generation + transition digest; and
- each #481 stable authority identity/profile.

The separate `evidence_digest` commits the stable set identity plus each #481 fresh evidence identity/profile in the same canonical order.

Refreshing evidence for the exact same closed authority set may change evidence identity without changing stable set identity.

## 9. Conservative shared evidence horizon

Every supplied #481 capability must be usable at `now_ms`.

The final verification time is the maximum of all member verification times. The final lease is the minimum of all member leases.

If the combined set is stale, qualification fails even if some individual members remain usable.

## 10. Positive set is non-deserializable

`QualifiedHistoricalOperationalAuthoritySet` derives `Serialize` but not `Deserialize`.

A runtime cannot manufacture a positive closed set from transport bytes.

## 11. No live currentness or effect authority

The set exposes no conversion to current freshness/current operational authority and enables no external effect.

It proves only that an exact closed requirement set has corresponding currently-verifiable historical operational authority.

## 12. Identity convergence target

After ancestry convergence, Identity #454 can mechanically project one requirement per policy transition using:

- #464 exact generic signer subject mapping;
- #454 signed authority-state generation; and
- #454 signed authority-state transition digest.

This generic set theorem can then require an exact #481 proof for every transition without learning any Identity-specific policy semantics.
