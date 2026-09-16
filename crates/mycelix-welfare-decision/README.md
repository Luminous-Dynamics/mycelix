# mycelix-welfare-decision

Pure `#![no_std]` AMSAP-003 decision substrate separating artificial-status evidence from welfare policy.

The crate consumes a structurally well-formed AMSAP-002 status observation only as context. It does **not** infer a protection bundle from `C/V/A/I/R`, and high consciousness/welfare evidence alone does not change an otherwise low-risk decision.

Decisions are drawn from the closed AMSAP-001 vocabulary:

- `Allow`
- `AllowWithSafeguards`
- `RequireIndependentReview`
- `TemporarilyPreserveAndReview`
- `Deny`
- `EmergencyContain`

The policy separately considers the explicitly supplied protection bundle, welfare-impact class, intervention risk, human-rights impact, third-party risk, protection cost, and safety urgency.

Protection hysteresis prevents established P2/P3 protections from being removed for administrative convenience and prevents large upward jumps from one striking result without exceptional convergent evidence.

This crate grants no legal standing, governance authority, currentness, personhood, deployment authorization, or external-effect authority. A recommendation is a bounded policy output for later competent review; it is not an actuator capability.
