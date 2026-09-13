# Mycelix Public Election Adversary Coverage v0.1

Status: **ADV-002 threat-accountability layer; not a claim that all adversaries are mitigated**

Parent: ADV-001 structural adversarial corpus.

Profile identifier: `mycelix-public-election-adversary-coverage-v1`

## Purpose

ELECT-001 defines twelve mandatory adversary classes. ADV-001 already gives us theorem-aware attack coverage, but theorem coverage is not the same thing as threat-model coverage. A project can have many excellent tests and still quietly omit the attacker that matters most.

ADV-002 closes that bookkeeping gap.

Its target theorem is:

> Every adversary class required by the public-election threat model is represented exactly once by executable adversarial evidence, an explicit protocol blocker, a residual research gap, or a combination of those routes.

This is deliberately an **accountability theorem**, not a mitigation theorem.

## The twelve required adversaries

The registry imports the exact ELECT-001 census:

1. nation-state attacker;
2. malicious election official;
3. colluding trustee subset;
4. malicious voter;
5. compromised voting device;
6. compromised scanner;
7. supply-chain compromise;
8. compromised Mycelix nodes;
9. malicious verifier;
10. network partition;
11. coercer; and
12. stolen credential.

The registry validator rejects missing adversaries, duplicate adversaries, references to non-existent executable attacks, references to non-blocking protocol gaps, and adversaries with no route at all.

## Accounted for is not mitigated

ADV-002 intentionally distinguishes three useful states:

```text
ExecutableCoveragePresent
ExplicitlyBlocked
ExecutableCoverageAndBlocker
```

A structural attack being executable means we can falsify a concrete contract today. It does **not** prove that every capability of that adversary has been defeated.

For example:

- compromised scanners already have executable CVR/reconciliation and physical-audit failure paths;
- compromised voting devices remain blocked on real cast-as-intended and endpoint-compromise defenses;
- colluding trustees remain blocked on the concrete threshold tally/decryption protocol;
- coercers remain blocked on coercion resistance;
- stolen credentials remain blocked on credential-theft/recovery semantics.

## Residual research gaps

Some threat classes need blockers more specific than the seven protocol gaps in ADV-001. ADV-002 therefore records the following residual gaps without pretending they are solved:

- `CompositeNationStateCampaigns`;
- `TrusteeThresholdRobustness`;
- `EndpointCompromiseResistance`;
- `VotingDeviceSupplyChainIntegrity`;
- `ElectionLivenessUnderPartition`; and
- `CredentialTheftRecovery`.

These are not new security features. They are explicit reasons **not** to make a complete-threat-model security claim yet.

## Current representative routing

Examples include:

```text
NationState
  -> SplitViewEquivocation
  + BallotSecrecyAgainstConcreteTranscript
  + CompositeNationStateCampaigns

ColludingTrusteeSubset
  -> TalliedAsRecorded protocol blocker
  + TrusteeThresholdRobustness residual gap

CompromisedVotingDevice
  -> CastAsIntended protocol blocker
  + EndpointCompromiseResistance residual gap

CompromisedScanner
  -> CvrDuplicateOrMissing executable attack

SupplyChainCompromise
  -> VerifierBuilderCollapse executable attack
  + VotingDeviceSupplyChainIntegrity residual gap

NetworkPartition
  -> OfflineNetworkDependency executable attack
  + ElectionLivenessUnderPartition residual gap

Coercer
  -> CoercionResistance protocol blocker

StolenCredential
  -> CryptographicEligibilitySoundness protocol blocker
  + CredentialTheftRecovery residual gap
```

A later tranche may add multiple executable cases per adversary or more granular capability trees. ADV-002's immediate job is to make omission impossible to hide.

## Strong negative theorem

The crate deliberately exposes:

```text
full_threat_model_security_claim_is_blocked() == true
```

for the current program.

That function should remain true until every protocol-dependent and residual threat blocker has been retired by qualified evidence. Passing ADV-002 therefore means **we know what remains unsolved**, not that the election system is ready for governmental deployment.

## Deliberate non-claims

ADV-002 does not establish:

- nation-state resistance;
- trustee-collusion resistance;
- compromised-endpoint resistance;
- voting-device supply-chain security;
- election liveness during network partitions;
- coercion resistance;
- stolen-credential recovery;
- legal election certification; or
- completeness of every possible adversary capability.

It establishes a smaller and important property: the currently required adversaries cannot silently disappear from the assurance program.

## Next hardening

The next structural target should be **ELECT-013 — bounded verifier resource envelope**. The offline verifier currently bounds artifact count, path length, and interoperability profile count but does not yet bound declared or expanded evidence byte volume. ELECT-013 should separate the election's frozen resource policy from each verifier implementation's safe capability and fail closed when the workload exceeds that capability.
