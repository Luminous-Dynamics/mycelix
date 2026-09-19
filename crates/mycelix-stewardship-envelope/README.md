# mycelix-stewardship-envelope

STEW-002 is the authority-neutral public envelope theorem for stewarded knowledge and creative artifacts.

It composes STEW-001 structural identity with a deliberately small set of public metadata and typed references without placing protected payload bytes in the envelope and without inferring any authority from the presence of a reference.

## Core separation

```text
identity
+ public descriptor
+ disclosure class
+ typed references
!= permission
!= ownership
!= cultural authority
!= factual truth
!= preservation success
!= reciprocity satisfaction
```

The envelope can say that an access-policy reference, cultural-protocol reference, provenance reference, stewardship claim, preservation manifest, or reciprocity policy exists. It cannot decide whether that referenced object is authentic, current, applicable, satisfied, or authoritative.

## Protected content

`PayloadDisclosureClassV1::ProtectedRepresentation` means only that the envelope declares the associated representation to be protected. It does **not** establish encryption, confidentiality, key custody, community consent, or successful access control.

The envelope contains no field for protected plaintext bytes.

## Reference discipline

Reference kinds are closed-world in v1, while the reference identifier itself is an opaque canonical protocol identifier from STEW-001. Duplicate exact `(kind, reference)` pairs are rejected. Reusing the same identifier under different semantic kinds is representable and does not merge those kinds.

## Non-claims

This crate establishes no authorship, copyright, title, stewardship legitimacy, cultural authority, access right, disclosure permission, provenance validity, signature validity, archival durability, reciprocity duty, legal compliance, or AI-training permission.
