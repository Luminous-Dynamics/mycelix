# Sender-proof endorsement-chain boundary — runtime protocol v19

Runtime protocol v19 adds a fail-closed manufacturer-endorsement boundary to the sender-proof verifier lifecycle. A fresh device attestation, approved platform measurement, and reconstructed measured-boot log are necessary but not sufficient: the endorsement key used by the custody device must also chain to an active, externally governed manufacturer root under the exact policy advertised by the runtime.

## Required production evidence

A production verifier release must bind all of the following exact values:

- endorsement-policy hash;
- manufacturer-root-set hash and monotonic epoch;
- verified endorsement-chain evidence hash and sequence;
- endorsement-revocation-set hash and monotonic epoch;
- endorsement checkpoint hash;
- firmware security version satisfying the policy floor.

The chain verifier is fail-closed. It requires a canonical leaf-to-root certificate path, exact issuer linkage, supported signature algorithms, active roots, unrevoked certificates and keys, approved device class and manufacturer, bounded validity windows, and firmware-security anti-rollback.

## Root governance and revocation

Manufacturer-root sets and endorsement revocations are monotonic, hash-linked records. Successors may not silently remove roots, lower epochs, erase revocations, or reduce the firmware-security floor. Root replacement and policy changes must produce new externally pinned evidence rather than mutating historical records.

## Current runtime truth

The present Pulse DNA does not install a production endorsement parser, manufacturer-root set, chain authenticator, revocation feed, or checkpoint verifier. It therefore advertises:

- no pinned endorsement policy;
- no active manufacturer-root set;
- no verified endorsement chain;
- no verified endorsement revocation state;
- no verified firmware anti-rollback state;
- no pinned endorsement checkpoint;
- proof acceptance disabled.

The source contract defines canonical evidence and promotion blockers. It does **not** claim production X.509, TPM EK-certificate, vendor-root, OCSP, CRL, or firmware-verification integration. Those integrations must be implemented, independently tested, and pinned before any verifier can enter the production catalog.
