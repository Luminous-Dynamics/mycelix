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

## Counterfactual repair guidance and dissent preservation

The standalone `craft-counterfactual-repair-dissent-v1` reference core validates read-only, owner-proposed next-step certificates and attributed community positions. It binds every blocker and guidance requirement to exact decision/context and owner-evaluation references, rejects incomplete or cyclic blocker/step graphs, and prevents material positions from disappearing from summaries or being marked resolved without the exact resolution artifact.

It does not evaluate evidence truth, authenticate authority, decide consent, authorize work, or execute actions. Hosts must authenticate and replay each domain owner's output; newly acquired evidence returns to its owning evaluator. See:
- `docs/COUNTERFACTUAL-REPAIR-CERTIFICATE-V1.md`
- `docs/DISSENT-PRESERVATION-V1.md`
- Benchmark relations MR-017 and MR-018 in issue/PR #6776.

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
