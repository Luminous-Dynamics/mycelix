# Prism Hardening Wave 11

## Purpose

Wave 11 makes release admission independently re-auditable after verifier-key
rotation and recoverable across the narrow crash window between journal append
and external-head publication. It does not claim live cryptographic, Cargo,
Nix, Bubblewrap, or Chromium validation in this environment.

## Ordered patch sets

1. **Type canonical build evidence receipts**
   - Replaces loose release-side JSON indexing with exact Rust schemas.
   - Binds source, tools, commands, streams, environment, and platform fields.

2. **Verify typed build evidence at release**
   - Makes independent release verification consume the same typed contract as
     the canonical evidence producer.

3. **Pin verifier-agent key material**
   - Advances the local verifier protocol to v2.
   - Binds request ID, algorithm, verification-key digest, message digest, and
     signature digest in every response.

4. **Expose validated release evidence bytes**
   - Preserves the exact admitted policy and signed-attestation bytes for
     durable checkpoint publication.

5. **Chain exact release evidence in journal checkpoints**
   - Stores exact signed attestations and anchored policies.
   - Adds a local checkpoint chain independent of the signed statement chain.

6. **Publish idempotent external release heads**
   - Supports crash recovery when journal admission succeeds before head
     publication.
   - Repeating the same release republishes the same head without advancing the
     journal twice.

7. **Add cryptographic release-journal audit**
   - Replays hybrid verification over every admitted checkpoint.
   - Requires an independently anchored external head.

8. **Use typed build receipts during preparation**
   - Removes remaining untyped evidence interpretation from the signing-request
     preparation path.

9. **Bind verifier-key epochs into release policy**
   - Advances release-policy schema and domain to v2.
   - Every policy binds exact Ed25519 and ML-DSA-65 verification-key digests.
   - Historical audits select keys from each checkpoint's embedded policy.

10. **Publish deterministic journal-audit receipts**
    - Binds every statement, checkpoint, policy, attestation, signature, and
      historical verifier-key epoch.
    - Publishes create-once, idempotent audit evidence.

11. **Test crash-safe release-head recovery**
    - Exercises the append-then-retry recovery path.
    - Proves the second admission is `AlreadyCurrent` and preserves the head.

12. **Promote canonical Wave 11 release gates**
    - Adds a Wave 11 static verifier and live gate.
    - Promotes Cargo, Nix, and Python evidence contracts to `wave11-static`.
    - Preserves Wave 10 receipts and scripts as historical evidence.

13. **Require live journal audit and external-head artifacts**
    - The live ceremony requires create-once head and audit outputs.
    - It re-audits the complete journal after release verification and records
      both artifacts in the live evidence bundle.

14. **Define the Wave 11 recovery and audit ceremony**
    - Documents verifier-key digests, head publication, complete history audit,
      external digest anchoring, and interrupted-publication recovery.

15. **Publish the reproducible Wave 11 static receipt**
    - Regenerates the machine-readable static evidence from repository bytes.

## Trust invariants

- Build evidence is interpreted through one exact typed contract.
- A verifier response cannot substitute another key, message, signature,
  algorithm, request, process, or executable.
- Every release checkpoint preserves the exact signed evidence and anchored
  policy that justified admission.
- Release policies identify the exact classical and post-quantum verification
  keys authoritative for that release epoch.
- Journal audit uses historical policy keys rather than current global keys.
- The external release head binds both the signed lineage and local checkpoint
  lineage.
- Retrying after interrupted head publication is idempotent.
- The deterministic audit receipt commits to the full admitted history.

## Canonical static validation

```sh
python3 scripts/verify-wave11-static.py \
  --check security/wave11-static-evidence.json
python3 scripts/verify-supply-chain.py \
  --check security/supply-chain-evidence.json
```

## Canonical executable validation

On the admitted Rust/Nix/Linux release host:

```sh
scripts/verify-ci.sh
python3 scripts/capture-build-evidence.py \
  --lane nix --release \
  --output target/prism-evidence/canonical-nix
python3 scripts/run-wave11-live-gates.py --help
```

The complete operator sequence is documented in
`security/RELEASE_CEREMONY.md`.

## Remaining blocker

Release admission remains intentionally blocked until `flake.lock` is generated
on a Nix-enabled host, committed, semantically reviewed, and hybrid-signed.
