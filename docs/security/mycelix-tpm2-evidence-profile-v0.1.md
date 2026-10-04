# Mycelix TPM 2.0 Evidence Profile v0.1

**Status:** executable reference profile  
**Claim ceiling:** `ReferenceModelOnly`

This is the first concrete TPM Evidence adapter profile for the Mycelix enclave program.

It deliberately chooses a **software TPM (swtpm/libtpms)** so the quote/PCR/nonce semantics can be exercised reproducibly before hardware capture is available.

## Exact stack

The reference profile freezes:

- swtpm **0.10.1**;
- libtpms **0.10.2**;
- tpm2-tools **5.8**;
- SHA-256;
- RSA-2048 Attestation Key;
- RSASSA-SHA256 quote signatures;
- TPM 2.0 endorsement hierarchy for AK creation;
- SHA-256 PCR bank, PCR 16 only;
- local Unix-domain TCTI transport;
- state locking, mode 0600, and fsync-oriented state handling.

Current upstream material identifies TPM 2.0 Library Specification Version 185 (March 2026). TCG also publishes a platform-specific PC Client TPM Profile, with Version 1.07 dated March 23, 2026. The latter is deliberately **not** claimed as the semantics of this vTPM workload-PCR fixture. citeturn412647search2turn567190search2

The selected software stack is security-patched at the libtpms layer: libtpms 0.10.0 and 0.10.1 had a published vulnerability and 0.10.2 is the patched release. citeturn440612search4turn440612search1

## Exact evidence chain

```
workload-manifest bytes
        ↓ SHA-256
workload digest
        ↓ TPM2_PCR_Extend
PCR 16
        ↓ TPM2_Quote
quote message + signature + PCR selection/value
        ↓ TPM2_CheckQuote
cryptographic Evidence verification
        ↓
RATS Evidence envelope
        ↓
RATS Verifier
        ↓
bounded Attestation Result
        ↓
Relying Party local authorization
```

The qualifying data is a 32-byte challenge nonce. tpm2_quote places caller-provided qualifying data into the quote to support freshness, and tpm2_checkquote can verify that qualifying data together with the signature and PCR values. citeturn955390search6turn955390search2

## What this proves

A green execution can establish, for this exact vTPM profile:

- the selected TPM tooling can create an Attestation Key;
- the TPM can extend a workload digest into PCR 16;
- the resulting PCR can be independently reconstructed;
- the TPM can quote PCR 16;
- the quote signature verifies under the AK public key;
- the quote is bound to the challenge nonce;
- tampered quote/PCR material is rejected;
- a second PCR extension invalidates a quote made before the extension;
- the generated evidence envelope records exact tool/profile/provenance data.

## What this does not prove

PCR 16 here is an application-controlled fixture measurement. It is **not** a measured-boot proof.

A vTPM is not a hardware root of trust. It does not establish:

- physical TPM resistance;
- firmware integrity;
- UEFI measured boot;
- PC-client PCR semantics;
- manufacturer EK trust;
- kernel integrity;
- supply-chain correctness;
- CMMC/classified authorization;
- FIPS 140-3 validation;
- resource authorization.

The profile therefore forbids the semantic collapse:

```
TPM quote valid
    -> machine trusted
    -> workload authorized
```

The actual RATS architecture remains:

```
Attester
  -> Evidence
  -> Verifier + appraisal policy/reference values
  -> Attestation Result
  -> Relying Party
  -> local authorization
```

RFC 9334 explicitly separates those roles and says Reference Values are inputs to Evidence appraisal, while Attestation Results are consumed by the Relying Party for its own decision. citeturn412647search0

## Workload PCR model

The fixture uses PCR 16 because it gives us an isolated application-controlled measurement slot.

For an initial PCR value `P0` and workload digest `D`:

```
P1 = SHA256(P0 || D)
```

The qualifier independently computes this value and compares it with `tpm2_pcrread`.

This is a useful integrity exercise, not a statement that PCR 16 has the same meaning as any production PC-client boot PCR.

## Negative tests

The executable qualifier must fail closed for:

1. wrong nonce;
2. quote signature tampering;
3. quoted-PCR substitution;
4. workload digest substitution;
5. a second PCR extension after quote generation;
6. missing/unavailable verifier tooling;
7. any later cross-domain authorization mismatch.

The first five are tested at the TPM Evidence layer. The final authorization conditions remain under the RATS/Relying-Party contract and must not be inferred from a successful TPM quote.

## Execution

Run:

```bash
python3 scripts/security/qualify_mycelix_tpm2_v0_1.py
```

The qualifier exits with:

- `0` only when all executable checks pass;
- `2` when the required TPM/vTPM toolchain is unavailable, which is **NOT EXECUTED** rather than PASS;
- `1` for an executed qualification failure.

The current development environment does not contain `swtpm` or `tpm2-tools`, so no physical or vTPM execution claim is made from this environment.

## Next hardware transition

Once the vTPM semantic chain is green, the next profile should replace only the Evidence adapter:

```
swtpm/libtpms fixture
        ↓
real TPM 2.0 device
        ↓
real PC-client measured-boot profile
        ↓
real event-log reconstruction
        ↓
reference-value appraisal
```

The RATS Result and Relying-Party authorization boundary should remain unchanged.
