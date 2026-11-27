# Mycelix Craft

Decentralized talent marketplace with living credentials, guild-based mastery, and
employer-funded apprenticeship pathways. This DNA stores opt-in profile metadata,
publicly indexed credential pointers with Ebbinghaus vitality decay, and peer
attestations. Verifiable credentials remain in the Identity/Praxis DNAs and are
referenced via hashes.

## Scope
- Craft profiles (opt-in, user-authored professional identity)
- Living credentials with Ebbinghaus forgetting curve decay
- Skill endorsements (peer attestations)
- Job postings with apprenticeship stakes
- Work history with peer verification
- Connection graph and recommendations
- Application lifecycle (Draft -> Submitted -> Interview -> Offered -> Accepted)

## Build
```sh
cd mycelix-craft
cargo build --release --target wasm32-unknown-unknown
```

Then package the DNA/hApp:
```sh
hc dna pack -o dna/mycelix_craft.dna dna/dna.yaml
hc app pack -o mycelix-craft.happ happ.yaml
```


## Work evidence contract (draft)

The additive `crates/work-evidence-v1` package is the first Rust reference implementation for
`CRAFT-WORK-EVIDENCE-001`. It defines versioned work claims, artifact digests, scoped
assessor authorities, signed attestations, an independent verifier, and synthetic adversarial
fixtures.

Run the isolated reference tests from this workspace:

```sh
cargo test -p craft-work-evidence-v1
```

**Status:** draft reference protocol only. It is not yet called by the Holochain work-history
coordinator and is not a deployed or production-qualified evidence system. Identity trust-store
integration, atomic host-side replay reservation, append-only dispute/correction/revocation
events, Praxis and Manufacturing adapters, cryptographic review, and hosted qualification
remain outstanding.

Craft vitality/freshness displays are advisory UI projections. They must never rewrite
issuer-defined credential validity or silently transform a historical achievement into
an invalid credential.
