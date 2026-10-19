# AeroCommons Identity Qualification V1

These vectors are semantic qualification cases for AEROCOMMONS-005. They are intentionally expressed without prescribing a second universal digest algorithm.

## Canonical fixtures

- Artifact: part:bracket-A
- Content profile: content-bytes-v1
- Configuration profile: configuration-manifest-v1
- Evidence profile: myc-source-evidence-v1
- Execution profile: execution-v1
- Relation profile: aero-relation-v1

## Vector 1 — same payload, different executions

Input: two execution records reference byte-identical content and the same artifact/configuration, but have distinct execution identities.

Expected:

- content identities MAY be equal;
- execution identities MUST differ;
- evidence identities MAY differ;
- no execution equality may be inferred from content equality.

## Vector 2 — same payload, different artifacts

Input: byte-identical payload is published once as a CAD representation of artifact A and once as a payload associated with artifact B.

Expected:

- content identity MAY be equal;
- artifact identities MUST remain distinct;
- configuration identity follows the artifact membership, not raw byte equality.

## Vector 3 — configuration fork

Input: configuration C1 contains artifact A. C2 changes one parameter and declares C1 as predecessor.

Expected:

- C1 != C2;
- C2 retains explicit predecessor C1;
- evidence bound to C1 is not silently rebound to C2.

## Vector 4 — duplicate evidence bytes, different source events

Input: two agents produce byte-identical evidence payloads from independent observations.

Expected:

- content equality does not collapse source-evidence identity;
- both source events remain independently addressable;
- a later relation MAY state that one reproduces/supports/contradicts the other.

## Vector 5 — one claim, multiple evidence records

Input: claim Q has evidence E1 and E2 from independent producers.

Expected:

- Q does not become the identity of E1 or E2;
- each evidence record retains its own identity;
- a relation can explicitly connect Q to each basis record.

## Vector 6 — relation supersession

Input: relation R2 corrects or supersedes R1.

Expected:

- R2 has a distinct relation identity;
- R2.predecessor = R1;
- R1 remains addressable with lifecycle state;
- no in-place reinterpretation of R1.

## Vector 7 — private payload, public commitment

Input: inspection payload is controlled, but its commitment and public evidence metadata are disclosed.

Expected:

- public identity can bind the exact controlled payload;
- absence of public payload access does not make the commitment meaningless;
- disclosure policy remains separate from content identity.

## Vector 8 — unavailable external artifact

Input: relation references content/profile that cannot currently be retrieved.

Expected:

- identity reference remains syntactically addressable;
- validation does not invent content;
- Holochain-level validation may remain unresolved when a required dependency is unavailable;
- domain code reports the missing dependency rather than converting it to valid/invalid physical evidence.

## Vector 9 — unknown profile

Input: IdentityRef has a non-empty ID but an unsupported profile.

Expected:

- structural parsing may succeed;
- consequential verification MUST NOT silently proceed as though the profile were known;
- result is quarantine/unknown until the profile is resolved.

## Vector 10 — Holochain hashes are transport identities

Input: two authored records have ActionHashes AH1/AH2 and an entry hash EH.

Expected:

- AH1/AH2 identify authored actions;
- EH identifies entry content;
- neither is substituted for Artifact/Configuration/Execution/Relation identity.

## Vector 11 — semantic manifest reorder

Input: configuration manifest entries are reordered without changing the declared semantic set.

Expected:

- canonical configuration identity remains equal only if the configuration profile defines deterministic ordering;
- non-canonical input order must not create accidental configuration forks.

## Vector 12 — one parameter changes

Input: C1 and C2 differ in one engineering parameter.

Expected:

- C2 receives a distinct configuration identity;
- a ChangeSet explicitly identifies the changed parameter;
- impact analysis decides what evidence requires review/revalidation;
- no evidence is automatically promoted from C1 to C2.

## Conformance rule

An implementation passes this qualification set only if it preserves the distinctions above. Passing these tests demonstrates identity-semantics conformance; it does not demonstrate engineering correctness, certification, or airworthiness.
