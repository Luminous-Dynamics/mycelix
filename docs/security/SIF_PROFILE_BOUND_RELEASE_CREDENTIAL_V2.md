# SIF Profile-Bound Release Credential v2

Status: draft / qualification pending.

This profile is additive to `sif-release-credential-v1`. Historical v1 credentials remain representable and verifiable, but they do not authorize a high-assurance release that requires an exact SIF protected-transfer profile.

## Authorization theorem

For a protected release to reach Xenia's output boundary, the exact profile commitment authorized by Mycelix must be the same commitment authenticated by Xenia negotiation and later signed by receiver custody evidence:

```text
Mycelix credential-v2 required_sif_profile_digest
        ==
Xenia durable disclosure authority required profile
        ==
Xenia authenticated negotiated profile
        ==
receiver SifDeliveryReceiptV2 profile
```

No component may replace a missing profile with its local current profile.

## Credential identity

The v2 credential identifier is derived from the complete canonical v1 statement and the exact required profile digest. Therefore the same witnessed evidence under a different required SIF profile is a different credential, not a reinterpretation of the old credential.

## Fail-closed rules

- `required_sif_profile_digest == 0` is invalid.
- A v1 authority signature cannot qualify a v2 statement.
- Changing the profile after signing invalidates authority signatures and changes the v2 credential ID.
- High-assurance Xenia release must reject legacy/profile-less credentials rather than infer a profile.
- Any future acceptable-profile-set scheme requires a new committed schema; v2 authorizes one exact profile only.

## Privacy boundary

The profile digest is protocol metadata. The v2 statement remains commitment-only and does not add subject identity, case/matter identifiers, purpose text, query text, or protected record bytes.
