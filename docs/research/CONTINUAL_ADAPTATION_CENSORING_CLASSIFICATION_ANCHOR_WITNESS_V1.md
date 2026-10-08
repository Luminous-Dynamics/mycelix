# Anchor witness non-equivocation research v1

Status: research-only.

This layer models the missing consistency property above #4837:

    independently identified witnesses
      -> attest to the same ordered anchor checkpoint
      -> threshold agreement
      -> non-equivocation decision

The current implementation intentionally uses deterministic semantic attestation commitments rather than real public-key signatures. Therefore it proves quorum and consistency relative to a pinned witness registry, but it does not prove cryptographic witness authentication or independent operational custody.

## Security properties

A candidate is qualified only when:

- the witness registry matches the externally pinned trust-root commitment;
- the configured threshold is not weakened;
- each witness is uniquely registered;
- each attestation commitment matches its exact semantic fields;
- all quorum witnesses agree on authority, manifest version, manifest commitment, predecessor and trust-root reference;
- same-version forked views remain unresolved;
- a manifest can advance only one version at a time and must name the observed predecessor;
- duplicate/replayed witness identities cannot increase quorum;
- representation order does not change the verdict.

## Claim ceiling

This proves consistency of independently named witness observations under the frozen research registry. It does not prove the witnesses are independent entities, honest, uncompromised, cryptographically authentic, or externally hosted.

The next closure should replace semantic witness commitments with actual signature verification and independently governed witness keys, then add cross-witness consistency proofs or a VDS/SCITT-style append-only witness log.