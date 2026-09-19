# FORGE-004D3B2C2B — Guest Evidence Envelope

This crate defines the evidence object that may cross from the sandboxed Forge guest back to the host.

The child does **not** serialize a positive authority object. `GuestEvidenceEnvelopeV1` contains raw/portable evidence only:

- exact guest-plan digest;
- exact guest-tool-map digest;
- raw `InsideIsolationEvidence`;
- typed `PolicyTrustInventory`;
- exact `GittufLocalReceipt`;
- raw `GuestTranscriptV1`.

The parent calls `qualify_guest_evidence_envelope` and independently re-derives:

1. the guest-plan and tool-map subjects;
2. the policy-state binding of the trust inventory;
3. `LocalEmbeddedKeysV1` qualification from that typed inventory;
4. gittuf receipt internal commitments and exact request binding;
5. the expected replay-receipt commitment;
6. the inside-isolation evidence digest;
7. every guest-transcript phase through `GuestTranscriptBindings`.

The transcript's policy-inventory phase binds the canonical `PolicyTrustInventory` digest, while its local-trust phase binds the separately re-derived local-key qualification evidence. This preserves the distinction between observed verifier methods and the policy conclusion that those methods are permitted.

The richer metadata-scan receipt remains collector evidence and does not need to become a trusted serialized child assertion.

## Claim boundary

This tranche performs no process execution and does not qualify the guest tool map against the enclosing ExecutionSpec/Nix closure. The host must still perform that theorem separately, qualify final parent+inside isolation, bind the same live process, perform NAR pre/post checks, and run the M0 composition gate before `OfflineEvidence` can exist.
