# Sender-Proof Device Attestation — Runtime Protocol v16

Protocol v16 separates four questions that must never be collapsed into a
single `custody_ready` bit:

1. Is the active custody device still in a fresh, approved measured state?
2. Is that measured-state history monotonic and externally checkpointed?
3. Has any conflicting clone evidence been resolved?
4. Was the retiring device actually zeroized and revoked after succession?

## Runtime claims

For each sender-proof contract Pulse publishes the attestation policy and record
protocols, active attestation sequence, bounded validity interval, monotonic boot
counter, checkpoint state, clone-evidence state, and decommission evidence.
Every unavailable fact is represented directly and contributes a stable
promotion blocker.

## DHT evidence

The bridge integrity zome defines append-only records for measured attestations,
canonical clone evidence, device decommissioning, and attestation checkpoints.
Raw endorsement keys, signatures, firmware images, and destruction artifacts are
not placed on the DHT; records retain hashes and governance metadata only.

## Browser evidence

The live browser proof requires protocol v16 and verifies that the supplied DNA
does not claim fresh attestation, clone resolution, checkpoint pinning, or secure
decommissioning. A future release must change those claims only together with
real authenticated evidence and corresponding lifecycle tests.

## Acceptance status

Protocol v16 does not enable sender-proof acceptance. Fresh attestation is one
additional conjunct alongside exact verifier provenance, rollout safety, release
quorum, replay governance, activation provenance, and custody continuity.
