# Integral seam scenario DSL v1

Status: **ReferenceModelOnly**

This scenario layer deliberately models semantic events instead of network mechanics. Its purpose is to make the same adversarial workload replayable against different runtime adapters.

## Required property

`scenario + profile + envelope -> deterministic semantic outcomes`.

The scenario must not change when the transport changes. Only the adapter that realizes delivery changes.

## Current mutation set

- provider acceptance
- semantic admission
- stale schema
- delivery identity mismatch
- retry with same logical delivery
- retry with payload mutation
- unknown delivery
- foreign recognition

## Why this matters

Distributed systems can receive duplicate mutations after a lost response; stable request identity lets an idempotent receiver safely recognize repeats. The scenario therefore treats logical delivery identity as distinct from attempt identity. citeturn0search0

Recent work on semantic models for distributed systems similarly argues that deterministic scenario vocabularies reduce accidental variation in transport/timing mechanics and make failure interleavings easier to test. citeturn0search2

## Qualification boundary

A scenario PASS proves only that the reference model produced its specified semantic result. It does not prove transport reliability, exactly-once external execution, authorization correctness, or physical/economic/ecological outcomes.