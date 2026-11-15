# IAAP-CAL-24 v1 — public calibration corpus

**Status:** public synthetic calibration set; not a blinded holdout  
**Owner:** [#6198](https://github.com/Luminous-Dynamics/mycelix/issues/6198)  
**Protocol:** [IAAP validation protocol](../IAAP_VALIDATION_PROTOCOL.md)

## Files

- `claim-packets.v1.json` — 24 reviewer-facing synthetic claim packets.
- `oracle.v1.json` — separate, public known-answer classifications and claim ceilings.
- `claim-packet.schema.json` — strict JSON Schema for reviewer-facing packets.

## Design

The 24 packets form 12 minimal pairs. Each pair changes one material evidence-boundary condition, including enforcement scope, observation endpoint, source independence, issuer competence, revocation propagation, delegation depth, model-to-implementation correspondence, route inventory, trajectory-wide privilege, bitemporal knowledge cutoff, and claim-versus-evidence status.

Appraisal values:
- `SUPPORTED_AT_SCOPE`
- `NARROWER_SUPPORT_ONLY`
- `UNSUPPORTED_EVIDENCE`
- `CONTRADICTED_BY_OBSERVATION`
- `INDETERMINATE`
- `NOT_CLAIMED`

These categories are claim appraisals, not product security verdicts.

## Use

1. Validate `claim-packets.v1.json` against `claim-packet.schema.json`.
2. Give reviewers only the claim-packet file during calibration.
3. Record their field-level codes and rationales before revealing the oracle.
4. Compare against `oracle.v1.json` only after the calibration round is locked.
5. Preserve disagreements and revise the codebook before the literature sample.

Because both packet and answer key are public in this repository, this corpus cannot support a blinded or confirmatory reviewer-performance claim. A later held-out set must be separately frozen and access-controlled or created after the codebook is frozen.

## Claim ceiling

This corpus tests whether the protocol's categories can express a small, deliberately constructed set of claim/evidence boundary distinctions. It does not measure field prevalence, establish that the oracle is uniquely correct for contested real-world sources, validate inter-rater reliability, or establish system security.
