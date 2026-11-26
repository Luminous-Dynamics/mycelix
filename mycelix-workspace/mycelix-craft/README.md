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

## Community need intake (reference core)

The standalone `craft-need-intake-v1` crate validates evidence-traceable draft
outcomes, capability requirements and work-package dependencies. It keeps
sources, assumptions, inferences, contradictions and open questions distinct.
Community, domain, safety and provenance reviews bind to an exact proposal
digest; only a separate locally authorized ratification can advance a proposal.
The crate has no authority to generate truth, ratify community intent, authorize
hazardous work, reserve resources or execute payments.

See `docs/NEED-INTAKE-V1.md` and
`docs/NEED-INTAKE-BENCHMARKS-V1.md`.
