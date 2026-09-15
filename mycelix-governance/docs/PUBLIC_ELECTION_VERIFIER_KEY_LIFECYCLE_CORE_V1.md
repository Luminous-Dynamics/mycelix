# Public Election Verifier-Key Lifecycle Core V1 (ELECT-018A)

ELECT-018A defines the **pure, authority-free transition theorem** for verifier authentication keys after the ELECT-017/017B/017C trust material has been frozen.

Qualified parent boundary:

- ELECT-017B public-key binding: #864, exact head `5c52d2b2e1a5afdfaa602849e17313ab73396650`;
- ELECT-017C offline authenticated-verifier package: #907, exact head `5fb4e89cde9ac2ed2314ebf287348f6db34247ec`;
- #907 hosted qualification: run `34954238915`, job `104332299892`;
- Xenia authentication profile V1 SHA-256: `e8b6ac90028c3064880bfb6b59ac7fbd1a1db04182bd3841e656bba2736d2b90`.

## Claim boundary

ELECT-018A proves only that one lifecycle request is structurally admissible from one exact predecessor and yields one exact successor projection.

It does **not** prove that the transition has been approved, that a materialized state is globally current, or that any receipt signature is valid.

The intended composition is:

```text
ELECT-018A valid proposal + exact successor projection
        +
future independently qualified lifecycle approval
        ↓
future authorized successor composition
        +
ELECT-009/010 transparency + witness current-head evidence
        ↓
current authoritative lifecycle head
        ↓
ELECT-019 receipt authentication
```

`valid successor != authorized successor != current authoritative head`.

## Frozen lifecycle policy

The V1 policy commits to:

- the exact ELECT-017 authorization-root digest;
- the exact frozen authorization-policy digest;
- the exact ELECT-017B public-key-bundle digest;
- the exact qualified Xenia authentication-profile digest;
- the exact Xenia suite-registry digest;
- lifecycle profile/version;
- a fixed maximum of 256 transitions.

Changing any of those creates a different policy lineage. Profile migration inside an election is forbidden in V1.

## Deterministic sequence-0 state

There is no caller-defined genesis.

Sequence 0 is derived from the exact qualified ELECT-017 root plus ELECT-017B public-key bundle:

- identical verifier-release universe;
- identical implementation-lineage identities;
- identical builder/control-domain identities;
- exact Ed25519 and ML-DSA-65 public keys whose signer IDs were qualified by ELECT-017B;
- every verifier active;
- zero predecessor digest;
- zero authorization-event digest;
- empty retired-key sets.

Input ordering of the parent bundle is non-semantic.

## Transition language

V1 exposes only two operations.

### `RotateKeyPair`

A rotation:

- targets an already-frozen verifier release;
- requires the exact current Ed25519 and ML-DSA-65 signer IDs;
- replaces **both** suites together;
- recomputes each replacement signer ID from the complete replacement public key under the frozen Xenia profile;
- rejects wrong suite-specific public-key lengths;
- rejects any current-key reuse;
- rejects every signer ID retired anywhere in the lifecycle history;
- preserves the verifier release, implementation lineage and builder/control-domain identity.

There is no one-suite rotation operation.

### `DisableVerifier`

Disablement:

- targets an active frozen verifier;
- retires both outgoing signer IDs;
- is terminal in V1;
- cannot be reversed or rotated afterward.

There is no `AddVerifier`, `ReenableVerifier`, `ChangeRelease`, `ChangeLineage`, `ChangeBuilderDomain`, or `LowerThreshold` operation.

## Acyclic proposal construction

ELECT-018A deliberately separates a request, a successor projection and the proposal digest:

```text
exact current-state digest
        +
transition request
        ↓
deterministic successor projection
        ↓
successor-projection digest
        +
canonical request bytes
        ↓
valid proposal digest
```

The projection does not contain an authorization-event digest. A later authority tranche can therefore approve the proposal digest without creating a self-hash cycle.

`ValidLifecycleProposalV1` and `ProjectedLifecycleSuccessorV1` have private fields and are only produced by the validator. They are evidence of structural validity, not approval.

## Historical non-reuse

The current state carries explicit, sorted cumulative retired signer-ID sets for both suites.

Every successful rotation or disablement adds exactly one outgoing Ed25519 signer ID and one outgoing ML-DSA-65 signer ID. Therefore, for sequence `N`, each retired set has exactly `N` entries.

Replacement keys are rejected if their signer IDs appear in:

- any currently active signer ID for the same suite; or
- any retired signer ID for the same suite.

This prevents a retired key from reappearing on the same verifier or a different verifier.

The V1 transition cap is 256. At 32 bytes per signer ID, the two retired-ID sets are bounded to 16 KiB total before container/object framing. When the cap is reached, further mutation blocks rather than dropping historical evidence.

## Certification capacity is separate from state validity

A security-motivated disablement may leave too few active independent verifiers for certification.

ELECT-018A therefore returns:

```text
CertificationCapacityV1::Sufficient
```

or

```text
CertificationCapacityV1::Insufficient { observed..., required... }
```

without weakening the frozen ELECT-017 thresholds.

A disablement can be structurally valid while certification capacity becomes insufficient. Later authorization/currentness layers must preserve that distinction.

## Fork evidence

Two different valid proposals from the same predecessor and sequence are explicit `LifecycleForkEvidenceV1`.

ELECT-018A does not choose a winner using wall clock, DHT arrival order, database order, author identity or administrator preference.

Durable current-head selection belongs to the later transparency/witness-qualified currentness theorem.

## Canonical vectors

The checked corpus and independent Python oracle reproduce:

```text
lifecycle policy
a016dab477257e83147087e8787944ea38fdd08fb141b076a7db6cca0783ddac

sequence-0 lifecycle state
3f63f50ff814e7d440ce108c981a7616cc1cab78e53d8f0b24950d2e994ad8d1

rotation successor projection
ca7107c5e629a5994b4cfd4a49413a92814041d480696e4bcd58607a5495b19c

rotation proposal
948e8bac9e77af3df6fc76668a2f8d1e9a8c630b708a7816678ea05dbaa49d61
```

## Deliberate non-claims

ELECT-018A does not establish:

- governance or threshold approval;
- cryptographic validity of governance approval signatures;
- private-key possession or HSM integrity;
- durable/global latest-state currentness;
- resolution of competing lifecycle forks;
- Ed25519 or ML-DSA-65 receipt-signature validity;
- independent-verifier certification quorum;
- ballot or tally cryptography;
- legal election certification.

Those remain separate qualification layers.
