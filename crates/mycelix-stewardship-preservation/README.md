# mycelix-stewardship-preservation

STEW-007 provides evidence-oriented preservation primitives for exact STEW-001 representations.

It deliberately does **not** define a universal `is_preserved` boolean. Preservation is represented through exact identities plus observations, replica attestations, migration records, recovery-test evidence, and a manifest that references those records.

## Core theorem

```text
preservation evidence exists
!= durable forever
!= authentic content
!= factual truth
!= public access
!= ownership
```

## Fixity

A `FixityObservationV1` compares one observed digest against the exact digest committed by the STEW-001 target and produces a structural `Match` or `Mismatch`. It does not prove that the observation process, observer, storage device, or supplied digest is trustworthy.

## Replication

A `ReplicaAttestationV1` states that an exact representation is asserted to exist in an opaque storage domain under an asserted custodian. It does not prove independence, geographic diversity, availability, or durability.

## Migration

A `MigrationRecordV1` keeps exact original and migrated representations plus process/evidence references.

```text
migration != replacement
```

The original identity remains first-class.

## Recovery tests

Recovery outcomes are explicitly `ReportedSuccess`, `ReportedFailure`, or `Indeterminate`; a report is not self-verifying evidence that future recovery will succeed.

## Manifest

`PreservationManifestV1` groups typed references to fixity observations, replica attestations, migration records, and recovery tests. Empty categories are representable and do not imply success.

## Non-claims

No archival durability, geographic independence, content authenticity, preservation-policy compliance, legal custody, access permission, deletion resistance, future readability, or recovery guarantee is established.
