# Threshold Qualification Runtime Adapter Contract

The pure `mycelix-governance-authority` model predates the canonical-profile registry work and stores `actions_digest` as 32 bytes without an adjacent digest-profile field.

Therefore `QualifiedThresholdAuthorization.actions_digest` is **not self-describing**.

A runtime `governance_threshold_authority_verifier` MUST additionally bind the exact action-digest profile from the verified proposal-authority / execution-preflight path.

For the current stack that profile is:

`mycelix-governance-execution-authority-v1-blake3-exact-json`

The runtime provider must require:

- request `actions_digest` == qualified threshold authorization `actions_digest`;
- request `actions_digest_profile` == the exact profile returned by current execution preflight;
- the current verified `ProposalAuthorityBinding.actions_digest_profile` == that same profile; and
- the cryptographic signature verifier actually verified the content commitment using that exact profile/byte contract.

A matching 32-byte digest under a different profile is not equivalent authority.

## Future protocol cleanup

A later version should move `ProfiledDigest` semantics into the governance-authority types themselves so signing policy/evidence/context carry digest profile next to digest bytes. That is a protocol-version change and should not be silently retrofitted into the v0.1 wire types.

Until then the runtime adapter is responsible for the explicit profile join.
