# Regenerative Asset Provenance Contract v0.1

## Purpose

Define the Mycelix-side evidence model for long-lived cyber-physical assets whose components can be inspected, repaired, replaced, or regenerated.

This is a provenance and coordination contract, not a safety certification system.

## Asset identity

Every tracked asset and critical component should have a stable identity independent of its current operator or network location.

A lifecycle record should be able to associate:
- asset/component identity;
- manufacturer or origin claim;
- installation event;
- configuration digest;
- operating interval;
- intervention authorization;
- intervention event;
- post-intervention evidence;
- verification result.

## Intervention record

A repair or healing event should distinguish:
1. who/what authorized the action;
2. which qualified action was requested;
3. which component/configuration it targeted;
4. when it began and ended;
5. what evidence existed before the action;
6. what evidence was produced afterward.

An intervention record must never itself assert that the physical asset recovered.

## Recovery attestation

Recovery is a separate claim with explicit evidence references.

Minimum semantic fields:
- asset/component identity;
- intervention identifier;
- pre-intervention evidence reference;
- post-intervention evidence reference;
- verification method;
- verifier identity/authority;
- verification timestamp;
- configuration/qualification digest;
- schema version.

A recovery claim with missing or stale evidence is incomplete rather than accepted by inference.

## Offline-first operation

The vehicle must be able to make local deterministic safety decisions without Mycelix connectivity.

Mycelix is responsible for durable lifecycle provenance, synchronization, authorization and multi-party evidence exchange. Network availability must not be a prerequisite for safe local restriction or quarantine.

## Relationship to Symthaea

Symthaea supplies observations, predictions, residual reports and intervention candidates.

Mycelix supplies identity, provenance, authorization and lifecycle evidence.

Neither layer alone establishes regulated operational permission.

## Compatibility

The initial contract is intentionally generic enough for:
- road vehicles;
- marine vehicles;
- rotorcraft;
- fixed-wing aircraft;
- remote/space assets;
- industrial machines.

Platform-specific schemas should specialize the evidence vocabulary without changing the core distinction between observation, intervention and verified recovery.

## Proposed integration path

1. Define schema and entry semantics.
2. Add single-agent validation tests.
3. Add multi-agent/DHT validation for conflicting or replayed lifecycle claims.
4. Add authorization checks for intervention records.
5. Add deterministic reconciliation rules for offline-created records.
6. Add transport-domain integration only after the generic semantics stabilize.
7. Connect Symthaea evidence references using explicit schema/version digests.

## Threat cases

The test matrix should include:
- replayed repair events;
- future-dated recovery claims;
- component identity substitution;
- configuration mismatch;
- unauthorized intervention;
- conflicting verifiers;
- missing post-repair evidence;
- valid repair followed by contradictory telemetry;
- offline concurrent repair records;
- stale synchronization;
- compromised peer attempting to promote an unverified repair.

The protocol should preserve the conservative state under all ambiguous cases.
