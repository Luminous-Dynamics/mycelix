# Witness key rotation ceremony research v1

Status: research-only.

This layer adds a distinct key-transition evidence boundary above authenticated witness observations.

A rotation is qualified only when all three channels hold:

    predecessor key authorization
        +
    successor proof of possession
        +
    governance quorum from non-target witnesses
        ->
    rotation-qualified

The predecessor and successor signatures bind the complete rotation record. The governance approvals bind the same record to a proposed next registry. The target witness is explicitly excluded from the governance quorum in this research profile.

## Version safety

The ceremony requires:

    predecessor_valid_until = activation_version - 1
    successor_valid_from   = activation_version

The proposed registry must reflect the same non-overlapping transition. This prevents a cryptographically valid old key from being treated as current merely because its signature still verifies.

## Governance separation

The sample registry has four witnesses with a three-of-four quorum. For this ceremony, governance approval is supplied by w02, w03, and w04 while the target is w01.

This demonstrates quorum-separated authorization of a target witness's rotation. It does not establish organizational independence among those witnesses, secure hardware custody, or resistance to compromise of the governance quorum.

## Proof of possession

The successor signs a domain-separated rotation payload with the new private key. Only the public key and resulting signature are stored in the repository. The private key used to construct the research fixture is not committed.

The predecessor similarly signs the same rotation record with a distinct predecessor-approval domain. This provides cryptographic continuity at the ceremony boundary rather than simply changing a registry field.

## Campaign

15 deterministic cases cover:

- valid dual-sided rotation;
- missing predecessor authorization;
- predecessor domain and algorithm substitution;
- missing successor proof of possession;
- successor algorithm substitution;
- governance below threshold;
- wrong governance signature;
- injecting the target witness into governance;
- successor-key substitution;
- activation rollback;
- predecessor/successor overlap;
- proposed-registry substitution;
- successor registry mutation;
- and cryptographic envelope tampering.

Python and Node implementations independently verify the ceremony and emit byte-identical reports.

## Claim ceiling

This demonstrates a research proof of key control at rotation time and governance authorization relative to the current witness registry.

It does not demonstrate continuous secure key custody, HSM/TEE protection, independent legal/organizational governance, availability, revocation dissemination, or production key-management interoperability.

This is complementary to RFC 9943's requirement that trust anchors and registration policy be made transparent and auditable; the ceremony makes a key-transition event itself explicit and replayable, without pretending that the event establishes the real-world governance behind the keys. (RFC 9943, Section 5.1.1.2)
