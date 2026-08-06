# Legacy Claim Migration

Legacy migration preserves history without upgrading its epistemic authority.

A legacy `DesciClaim` may contain an E0–E4 tier, opaque verification signatures, provenance entries, aggregate fingerprint values, trust values, and a reproducibility score. Those values were produced under the mutable compatibility model. Migration therefore records them as historical metadata only.

## Guarantees

For each source record, migration:

- reuses the legacy claim UUID as the canonical stream ID;
- derives a deterministic research-object ID;
- hashes the exact original JSON bytes with BLAKE3;
- records a non-sensitive logical source locator and source-system identifier;
- preserves the old creator string, tier, verification count, and provenance count as non-evidentiary metadata;
- lists materially omitted legacy fields;
- creates no evidence artifacts, reviews, reproductions, or replications;
- produces a signed schema-v3 `legacy_claim_imported` genesis event;
- produces a separately signed receipt-time authority record for the migration decision;
- is idempotent when the same source bytes are imported again;
- fails closed if the stream ID already exists with another origin or source hash.

The resulting assessment is `proposed` until new canonical evidence and attestations are submitted.

## Offline command

Store the migration signing seed in a file with restrictive permissions. The file may contain exactly 32 raw bytes or 64 hexadecimal characters.

```bash
chmod 600 /secure/migration-ed25519.seed
chmod 600 /secure/authority-receipt-ed25519.seed

cargo run --release --package mycelix-desci-core --bin mycelix-desci -- \
  migrate-legacy .mycelix/claims \
  --event-log ./data/scientific-events \
  --actor did:key:migration-service \
  --signing-key-file /secure/migration-ed25519.seed \
  --credential-registry ./data/scientific-credentials.json \
  --credential-bootstrap-trust-file ./config/credential-bootstrap-trust.json \
  --authority-audit ./data/scientific-authority \
  --receipt-signing-key-file /secure/authority-receipt-ed25519.seed \
  --source-system mycelix-desci-legacy-json \
  --report ./data/legacy-migration-report.json
```

Before migration, register the migration actor in the credential registry. During initial bootstrap, before threshold governance exists, `credential-actor-register` may be used once through the direct administrator path. After threshold governance has been initialized, create an `append_credential_event` governance action containing the signed `actor_registered` event, collect the required approvals, wait through the activation delay, and execute it. The durable cutover marker prevents older CLI binaries from restoring the direct path after restart.

The command rejects symbolic-link key files and, on Unix, requires both signing seeds to have mode `0600` or stricter. The migration actor must already exist in the append-only credential registry, hold the `migration_service` role, possess the supplied active signing key, and belong to the requested acting organization when one is supplied. The command no longer constructs a temporary self-asserted identity profile. A cryptographically distinct receipt-service key signs the authority decision, and the receipt binds the exact credential-registry revision used. `--receipt-trust-file` carries historical public keys when the receipt key has rotated; the same trust set verifies historical credential acceptance records. The command reconciles the audit journal before and after migration and fails if any migrated event remains pending, legacy-unattested, or unsafe-unattested.

## Ownership after import

The signer of an imported genesis event is the migration service, not the unverified creator string found in the old record. The old creator string remains visible only as historical attribution metadata. This is intentional. A legacy creator string is not sufficient proof of current key control.

Until a separate governed ownership-adoption protocol is implemented, imported claim content should be treated as historical and only a governed editor should attach author-controlled evidence. External actors may still submit qualified reviews, reproductions, replications, critiques, and conflict disclosures under the normal authorization rules.

## Source retention

Do not delete the original legacy JSON after migration. The canonical event stores its hash and locator, not every mutable field. Retain the source in a read-only archive so operators and auditors can reproduce the recorded hash and inspect omitted material.
