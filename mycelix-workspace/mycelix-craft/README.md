# Mycelix Craft

Mycelix Craft is an opt-in professional identity and opportunity-coordination application.
The current DNA stores profile metadata, credential pointers, peer attestations,
job postings, work history and connection records. Verifiable credentials remain
owned by the Identity/Praxis domains and are referenced rather than duplicated.

The architecture is evolving toward a **Federated Productive Capability Network**:
communities can form useful projects, develop capability, coordinate consenting
teams and cooperate across autonomous guilds and plural economic institutions.
This is an architectural direction, not a claim that the full Capability Commons
runtime is implemented.

The pure-Rust reference crates for guild governance, professional assessment
authority, economic agreements and project formation are isolated policy
contracts. They are not yet Holochain-integrated or production-qualified. See
[the Craft design and qualification documents](docs/README.md).

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
