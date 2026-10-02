# CIV-RES-004 — executable option-space qualification corpus v1

Status: synthetic qualification only
Parent: CIV-RES-001D / `a923352445db955c8c4cbe7c52d570506e6be0b0`
Tracking issue: #3832
Program: #2006

## Purpose

Exercise the reachable option-space boundary as a deterministic synthetic corpus. The fixture is not a real-world mental-health, violence, policing, or resident-risk system.

## Evaluation model

Each case is candidate input only:

`generation -> pathway availability -> attempt -> outcome -> feedback`

The evaluator keeps expected dispositions in a separate oracle artifact. The oracle is not passed as candidate input.

The corpus preserves dimensions rather than calculating a single resilience, capability, risk, or safety scalar.

## Case families

OS-01 through OS-18 cover reachability, usability, conflicting/stale capacity, dependency concentration, temporal availability, operational resolution, economic observations, buffering, feedback, aggregate projection, prohibited inference/authority transformations, provenance preservation, missingness, ordering, and path dependence.

## Deterministic evidence receipt

The evaluator binds:

`program + fixture_sha256 + oracle_sha256 + evaluator_identity + case_id + seed`

and emits a canonical JSON receipt. The receipt contains no composite resilience/risk/safety score.

## Qualification ceiling

PASS means only that this synthetic corpus/evaluator preserves the declared semantic, provenance, privacy, temporal, missingness, and authority boundaries.

It does not establish service effectiveness, community resilience, clinical judgment, suicide prediction, violence prediction, causal effect, public safety, municipal authority, or deployment readiness.
