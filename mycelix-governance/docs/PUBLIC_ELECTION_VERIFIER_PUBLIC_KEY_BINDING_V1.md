# Public Election Verifier Public-Key Binding V1 (ELECT-017B)

ELECT-017B closes the identity gap between ELECT-017's **authorized signer identifiers** and the concrete public keys an offline verifier will later use for cryptographic authentication.

## Qualified dependencies

- ELECT-017 exact head: `3ea28d33dc6103b5445c68d10d1dcdddd666612b`
- ELECT-017 hosted qualification: run `34834362468`, job `103944693833`
- Xenia generic-auth exact head: `e589b65b11f54590a3e5f1883c0581a2b272af0a`
- Xenia hosted qualification: run `34893112622`, job `104140381678`
- Xenia authentication profile V1 SHA-256: `e8b6ac90028c3064880bfb6b59ac7fbd1a1db04182bd3841e656bba2736d2b90`
- Xenia authentication-suite registry SHA-256: `0255c2b3070e579d52e41ab6a9d767d1700b61fd68787bbae78bd41aa868945f`

The Xenia profile freezes the exact signer-key-ID transcript, suite IDs, byte order, public-key lengths and suite verification profiles. ELECT-017B consumes that profile as a qualified external semantic dependency; it does not vendor Xenia or duplicate Xenia's signature implementation.

## Public-key identity theorem

For every verifier release already authorized by ELECT-017, the evidence bundle must contain exactly one Ed25519 public key and exactly one ML-DSA-65 public key. For each key:

```text
qualified Xenia profile
+ exact suite ID
+ complete public-key bytes
        ↓
independently recomputed Xenia signer-key ID
        =
signer-key ID frozen by ELECT-017
```

No caller-supplied signer ID is authoritative. The ID is recomputed from the complete public key using the qualified Xenia V1 transcript.

## Public evidence only

The bundle contains only public verification material:

- verifier-release digest;
- suite ID;
- qualified Xenia profile digest;
- complete public-key bytes;
- recomputed signer-key ID.

No private key, seed, signing API, HSM handle or custody assertion is accepted by this tranche.

## Canonical bundle

The bundle is bound to:

- ELECT-017 authorization-root digest;
- election-definition digest;
- jurisdiction-snapshot digest;
- qualified Xenia authentication-profile digest;
- Xenia authentication-suite registry digest;
- the complete public-key evidence population.

Evidence records are canonically ordered by verifier release, suite ID and signer ID before hashing. Input order is therefore non-semantic, while duplicates and omissions fail closed.

Reference vectors:

```text
ELECT-017 authorization root
4e52e8b427611e11b36b0a22b8c8ab7b62bf808d4223db1955f50ca9b02e65b5

ELECT-017 certification requirements
6aa17c36fa6fda24756157c3c50a7b746852771a26e4cbdfc95b0782da18d9c4

ELECT-017B public-key bundle
97565d554ffaddbcc35d2a22d209ec1fb2646c0f4303146bb73c3d9640fdbefc

ELECT-017B certification requirements
5885c0782e5fcddc7d96f4dea04a08aeec53e3f78934a1ac390a7467ed0cb293
```

## Fail-closed cases

The executable corpus rejects, among other cases:

- changed public key with copied old signer ID;
- recomputed signer ID for a key that ELECT-017 never authorized;
- wrong suite;
- wrong Xenia profile;
- missing Ed25519 evidence;
- missing ML-DSA-65 evidence;
- duplicate release/suite evidence;
- unexpected verifier release;
- changed ELECT-017 root;
- malformed suite-specific public-key lengths;
- profile substitution;
- detached ELECT-017 root substitution in the certification binding;
- detached ELECT-017B bundle-digest substitution in the certification binding.

## Certification binding is recomputed, not self-asserted

ELECT-017B does **not** accept either an opaque parent requirements digest or an opaque bundle digest as authoritative. The requirements theorem consumes:

- the typed ELECT-017 requirements binding;
- the actual ELECT-017 authorization root and policy;
- the actual ELECT-017B public-key bundle;
- the exact qualified Xenia profile digest.

It independently recomputes both the parent and child commitments before extending the chain:

```text
typed ELECT-017 requirements binding
+ actual ELECT-017 authorization root/policy
        ↓ recompute and compare
ELECT-017 certification-requirements digest
ELECT-017 authorization-root digest

actual ELECT-017B public-key bundle
+ same authorization root/policy
        ↓ validate + recompute
ELECT-017B public-key-bundle digest

all recomputed values
+ qualified Xenia profile digest
        ↓
next certification-requirements digest
```

A caller therefore cannot splice a detached parent root, an unrelated parent requirements digest, or a digest for another public-key bundle into an otherwise valid certification chain.

## Offline-verification contract

The complete public keys and Xenia profile identity are public election evidence. A verifier must not need Holochain, DNS, a key server, Symthaea, a vendor service or another network dependency to determine which concrete public keys correspond to ELECT-017's authorized signer IDs.

ELECT-011 does not yet list this bundle as a mandatory logical package artifact. That gap is tracked explicitly as **ELECT-017C / #865**; ELECT-017B does not silently reinterpret an existing ELECT-011 artifact slot.

## Deliberate non-claims

ELECT-017B proves **public-key/profile identity binding only**. It does not prove:

- possession or custody of corresponding private keys;
- signature validity;
- HSM or secure-element integrity;
- key rotation/revocation authority;
- reproducible-build equality;
- remote attestation;
- ballot or tally cryptography;
- legal election certification.

ELECT-018 must inherit this exact profile/key identity theorem before any key rotation is admitted. ELECT-019 can then perform actual Xenia signature verification against these exact authorized public keys.
