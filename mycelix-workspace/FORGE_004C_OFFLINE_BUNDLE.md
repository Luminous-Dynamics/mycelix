# Mycelix Forge FORGE-004C — Portable Repository Closure

**Status:** implementation candidate  
**Depends on:** FORGE-004B cache-free gittuf verifier  
**Artifact format:** Git bundle v3

## Purpose

FORGE-004C proves that the exact protected source and required gittuf metadata can be packaged into a standard self-contained Git bundle, carried elsewhere, replayed into an empty repository, and reproduce the same cache-free FORGE-004B verification subject.

It deliberately does **not** satisfy the protocol-level `OfflineEvidence` capability. Repository closure and verifier-environment closure are different claims. gittuf verification can depend on verifier-side material—notably Sigstore/TUF trust roots—so FORGE-004D closes the remaining hermetic verifier boundary.

## Why Git bundle

Git bundle already represents refs plus reachable Git objects. A bundle created from explicit positive refs without negative revision ranges is intended to be self-contained; bundle v3 also carries object-format capability information.

Mycelix therefore does not invent another repository archive format.

## Creation protocol

Given one exact `RepositoryVerificationRequest` and translated `RepositoryPolicyState`:

1. reject shallow source repositories;
2. reject partial clones and configured promisor remotes;
3. force `GIT_NO_LAZY_FETCH=1` and non-interactive Git operation;
4. run FORGE-004B locally and obtain receipt `R1`;
5. bind `R1` back to the exact request;
6. derive the canonical bundle ref set:
   - exact protected ref → exact requested target;
   - `refs/gittuf/reference-state-log` → receipt RSL root;
   - `refs/gittuf/policy` → receipt policy root;
   - `refs/gittuf/attestations` → receipt root when present;
7. execute `git bundle create --version=3` using only those positive refs;
8. require `git bundle list-heads` to advertise exactly the expected ref set/tips;
9. require a v3 bundle header;
10. SHA-256 the exact artifact bytes and record byte length;
11. run FORGE-004B again as receipt `R2`;
12. require `commitment(R1) == commitment(R2)`;
13. create the canonical `OfflineBundleManifest`.

Step 12 forms a race barrier around materialization. It does not claim immunity to an adversary capable of perfectly timed mutate/restore attacks; immutable source snapshots are a later hardening layer.

## Manifest subject

The manifest binds:

- schema version;
- required bundle format v3;
- gittuf semantic target;
- exact Forge request digest;
- complete translated `RepositoryPolicyState`;
- exact repository-policy-state digest;
- exact FORGE-004B local receipt commitment;
- SHA-256 and byte length of the bundle artifact;
- Git object format;
- canonical sorted ref name/tip set.

Filesystem paths, wall-clock timestamps, hostnames, and hosting-provider identifiers are excluded from the canonical subject.

## Replay protocol

Replay requires the manifest, bundle bytes, exact Forge request, Git, and a compatible gittuf verifier component.

1. re-bind manifest to the exact request;
2. verify exact bundle SHA-256 and byte length;
3. require the v3 header;
4. create an empty bare repository with the declared object format;
5. run `git bundle verify` in that empty repository;
6. require `git bundle list-heads` to equal the manifest exactly;
7. fetch every manifest ref from the bundle into the same ref name;
8. re-read every imported ref and require exact equality;
9. reconstruct the FORGE-004B invocation from the manifest's complete policy state;
10. run cache-free FORGE-004B against the replay repository;
11. require the replay receipt commitment to equal the manifest's source receipt commitment;
12. require replay receipt roots to reconstruct the same manifest ref set;
13. return `PortableRepositoryReplay`.

No protocol-level `OfflineEvidence` is emitted here.

## Why the empty repository matters

Bundle prerequisites are objects the recipient must already possess. Running `git bundle verify` in an empty repository makes a prerequisite-bearing bundle fail instead of accidentally relying on unrelated local Git state.

This turns “self-contained repository closure” into an observed property of the replay lane.

## Why this is not yet hermetic offline verification

gittuf v0.16 supports verifier types whose trust inputs are not necessarily contained in the Git repository. Its Sigstore verifier reconstructs verification material from the signature object, but obtains trusted material through cosign/Sigstore TUF machinery.

Therefore:

```text
self-contained Git closure
    !=
self-contained verifier environment
```

FORGE-004D must additionally bind the exact Git/gittuf verifier executables, local trust material where needed, verifier configuration, and network-denied execution before `OfflineEvidence` can be claimed.

## Adversarial gates

At minimum:

- one-byte bundle mutation → artifact digest mismatch;
- truncation/extension → digest or size mismatch;
- bundle v2 → reject;
- bundle with prerequisites → empty-repo `git bundle verify` fails;
- missing or extra protected/RSL/policy/attestation ref → exact ref-set mismatch;
- ref-tip substitution → exact tip mismatch;
- wrong Git object format → reject;
- manifest policy-state mutation → validating deserialization/request binding fails;
- different Forge request → manifest/request binding fails;
- replayed gittuf verification failure → no positive replay type;
- replay receipt differs from source receipt → reject;
- local persistent gittuf cache → inherited FORGE-004B fail-closed behavior;
- shallow source → reject;
- partial clone → reject;
- promisor remote → reject;
- implicit lazy object fetch → disabled by environment.

## Claim boundary

FORGE-004C establishes that the exact carried bundle can reconstruct the exact protected/gittuf repository state and reproduce the same cache-free gittuf verification subject from a self-contained Git object closure.

It does **not** establish:

- global/network currentness at capture time;
- provenance of the `git` executable;
- provenance of the `gittuf` executable;
- hermetic or network-denied verifier execution;
- pinned/local Sigstore trusted roots;
- resistance to every hostile local-filesystem race;
- Xenia/DID principal authenticity;
- exact review/merge authorization;
- SLSA source/build provenance;
- reproducible build output;
- release authorization;
- SCITT/transparency witnessing.

Those remain independent evidence claims.
