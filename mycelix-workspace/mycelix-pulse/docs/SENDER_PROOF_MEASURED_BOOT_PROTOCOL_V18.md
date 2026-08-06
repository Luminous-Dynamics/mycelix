# Sender-proof measured-boot boundary — runtime protocol v18

Protocol v18 adds a trust layer above fresh device attestations and approved
firmware measurements. A custody device is not promotion-ready until the runtime
has verified the complete ordered boot event log, reconstructed the required PCR
values, checked the active root and measurement revocation epoch, resolved any
measurement incident through an exact successor policy, and pinned the resulting
state externally.

## Runtime claims

`ProofVerifierKeyCustodyManifest` now publishes independently:

- measured-boot policy identity,
- active evidence hash and sequence,
- ordered event-log verification,
- trust-revocation set hash and epoch,
- measurement-recovery state,
- measured-boot checkpoint.

Every claim is absent or false in the shipped DNA. `custody_ready`, promotion
readiness, release readiness, and proof acceptance therefore remain false.

## DHT evidence

The mail-bridge integrity zome accepts append-only, hash-only records for the
policy, event-log evidence, revocation state, recovery linkage, and checkpoint.
Raw event logs, TPM certificates, signatures, and verifier binaries are not
stored in these entries.

## Acceptance boundary

A reservation challenge binds the exact measured-boot state but does not treat
it as an acceptance credential. Production enablement still requires the exact
verifier, authenticated proof and envelope signature, globally safe replay
reservation, release quorum, operator activation, and every prior custody and
rollout prerequisite.
