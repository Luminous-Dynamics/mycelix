# mycelix-forge-runtime-closure-evidence

FORGE-004D2B3A defines the evidence contract for re-deriving the contents of the exact Nix closure committed by FORGE-004D2A.

The contract intentionally does not trust Nix store metadata as proof of the bytes currently mounted at a store path. A producer must serialize each actual store root as a canonical Nix Archive (NAR), hash that byte stream with the algorithm committed by the closure manifest, and report the resulting digest plus observed NAR byte length.

## Exact-set qualification

`RuntimeClosureObservation` must contain exactly one observation for every committed closure entry and no extras. Qualification fails for missing paths, additional paths, duplicate paths, digest mismatches, closure-commitment mismatches, or empty NAR streams.

`QualifiedRuntimeClosureEvidence` commits to the expected closure manifest and the canonical observation set.

## Bootstrap boundary

This tranche does **not** claim that the NAR serializer/auditor is trusted. Using the same unqualified Nix binary to attest the closure containing itself would be circular.

A later producer tranche must bind an independently committed NAR-auditor executable and its exact invocation through the FORGE-004D1 execution/tool contract. The preferred path is a small independently reviewable NAR implementation or an otherwise independently qualified serializer, rather than `nix path-info` or store-database metadata.

## Why NAR

NAR is Nix's canonical filesystem serialization. Hashing the NAR stream fingerprints the contents Nix considers semantically relevant while excluding unstable filesystem metadata such as timestamps.

## Claim boundary

A positive value means only that the supplied NAR observations exactly reproduce the committed closure digests. It does not establish auditor provenance, kernel isolation, verifier trust, source provenance, or repository `OfflineEvidence` by itself.
