# mycelix-forge-gittuf-bundle

FORGE-004C turns FORGE-004B's local cache-free gittuf verification into a **portable, self-contained repository closure** using standard Git bundle v3 artifacts.

It deliberately does **not** claim protocol-level `OfflineEvidence`. Repository closure and verifier-environment closure are separate properties: gittuf signature verification may still depend on verifier-side trust material such as Sigstore/TUF roots. FORGE-004D closes that boundary.

## Creation

The bundle creator:

1. rejects shallow, partial-clone, or promisor-backed source repositories;
2. disables Git lazy fetching for all subprocesses;
3. runs FORGE-004B against the exact request;
4. derives the exact protected, RSL, policy, and optional attestations refs;
5. creates a self-contained Git bundle v3 from those explicit positive refs;
6. requires `git bundle list-heads` to equal that exact ref set and tips;
7. SHA-256 hashes the actual bundle bytes and records byte length;
8. runs FORGE-004B again on the source repository;
9. rejects the artifact if the before/after local receipt changed.

The canonical manifest embeds the complete `RepositoryPolicyState`, request digest, source receipt commitment, artifact digest/size, object format, and canonical ref set. Filesystem paths and timestamps are not part of the subject.

## Replay

The replay verifier:

1. validates manifest ↔ exact Forge request;
2. validates bundle SHA-256, size, and v3 header;
3. initializes an empty bare repository with the declared object format;
4. runs `git bundle verify` there, so prerequisite-bearing bundles fail;
5. requires bundle-advertised refs/tips to exactly match the manifest;
6. fetches only those refs into their exact names;
7. independently re-reads every imported ref;
8. runs the same cache-free FORGE-004B verifier against the replayed repository;
9. requires the replay receipt commitment to equal the source receipt commitment;
10. returns `PortableRepositoryReplay`.

`PortableRepositoryReplay` proves that the carried Git artifact reconstructs the exact protected/gittuf repository state and reproduces the same local verifier subject. It is not an `OfflineEvidence` qualification.

## Why `OfflineEvidence` is deferred

A self-contained Git bundle closes the repository-object dependency, but not every verifier dependency. In particular, gittuf's Sigstore path obtains trusted material through cosign/Sigstore TUF machinery. A genuinely offline capability therefore also needs pinned verifier binaries, pinned trust material where required, and network-denied execution.

That becomes FORGE-004D.

## Claim boundary

FORGE-004C establishes portable repository closure and replay. It does not establish network currentness at capture time, provenance of `git`/`gittuf`, hermetic verifier execution, fully local Sigstore trust material, Xenia identity authenticity, review authorization, SLSA provenance, release authority, or transparency witnessing.

## Validation

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-features
```
