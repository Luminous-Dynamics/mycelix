# Living Credentials: Ebbinghaus Decay in Consciousness-Coupled Identity Systems

## Target Venue
ACM CCS, USENIX Security, or IEEE S&P (identity/credentials track)

## Status
OUTLINE (April 2026)

## Abstract (Draft)

Traditional credential systems issue attestations that remain valid indefinitely---a university degree from 1985 carries the same authority as one from 2025, despite decades of potential skill decay. We present Living Credentials, a system where attestations of competence decay according to Ebbinghaus's forgetting curve (R(t) = e^(-t/S) where S is a function of mastery level and review count) unless refreshed through demonstrated practice. Implemented in the Mycelix craft cluster, living credentials measure Domain Competence (D7 in the 8D sovereign profile), ensuring that governance power derived from expertise reflects *current* rather than *historical* qualification. The system supports peer-verified endorsements, guild federations with consciousness-gated tiers (Observer through Elder), and cross-DNA verification via bridge zomes. Credential vitality is visible in the frontend as SVG decay curves with heatmap visualization. The Ebbinghaus model prevents credential fossilization---the accumulation of stale qualifications that no longer reflect actual capability---while respecting long-term mastery through stability-dependent decay rates (well-reviewed credentials decay more slowly).

## Paper Structure

### 1. Introduction
- The credential fossilization problem
- Static vs living credentials
- Ebbinghaus forgetting curve as credential model

### 2. Ebbinghaus Decay Model
- R(t) = e^(-t/S) where S = f(mastery, review_count)
- Stability increases with successful reviews (spaced repetition)
- Constitutional bounds on minimum/maximum decay rates
- Comparison to certificate expiration (binary) vs continuous decay

### 3. Guild Architecture
- 5 consciousness-gated roles: Observer → Apprentice → Journeyman → Master → Elder
- CertificationPath with vitality requirements
- GuildFederationLink for cross-bioregion standards
- Peer-verified endorsements weighted by endorser's own credential vitality

### 4. Cross-DNA Verification
- Praxis issues credentials (Proof of Learning + BKT mastery)
- Craft publishes living credentials (guild context + epistemic code)
- craft-bridge → praxis-bridge for cross-DNA verification
- Privacy: ZKP proof of "credential above vitality threshold"

### 5. Frontend Visualization
- SVG Ebbinghaus decay curves in CredentialsPage
- Vitality heatmap across credential portfolio
- Review countdown timers
- TierGate component for guild-gated features

### 6. Evaluation
- 42 tests across craft zomes (11 applications + 9 job-postings + 8 craft-graph + 6 connection-graph + 4 work-history + 4 guild)
- Credential decay simulation over 1-year timeline
- Comparison to static credentials (university degrees, professional certifications)

### 7. Related Work
- Verifiable Credentials (W3C)
- Open Badges (Mozilla)
- European Digital Identity Wallet
- Soulbound Tokens (static, no decay)
- Skills-based hiring platforms (LinkedIn, Degreed)

### 8. Conclusion
- Living credentials as anti-fossilization primitive
- Integration with 8D sovereign profile (D7)
- Open-source (AGPL-3.0)

## Key Code References
- `mycelix-craft/zomes/craft-graph/coordinator/src/lib.rs` — Living credentials
- `mycelix-craft/zomes/guild/coordinator/src/lib.rs` — Guild architecture
- `mycelix-craft/zomes/craft-bridge/coordinator/src/lib.rs` — Cross-DNA dispatch
- `mycelix-praxis/zomes/praxis-bridge/coordinator/src/lib.rs` — Credential issuance
- `crates/mycelix-leptos-core/src/sovereign_radar.rs` — Frontend visualization
