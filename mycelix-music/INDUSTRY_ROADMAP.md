# Mycelix-Music: Industry Transformation Roadmap

## Vision: Become the Default Choice for the Entire Music Industry

**Mission:** Build an open, fair and community-governed music ecosystem that gives artists clear economic choices, portable data, and auditable royalty statements.

**Evidence discipline:** This is a strategic vision and roadmap, not a claim that all listed features are shipped or commercially validated. Artist payout multiples, platform fees, settlement speed, privacy properties and resilience claims must be measured on a real pilot before they are advertised as outcomes. Streaming services do not share one universal per-stream rate; payout structures and rights-holder agreements differ. See [Industry Comparison and Evidence](docs/INDUSTRY_COMPARISON_AND_EVIDENCE.md) for the current benchmark and qualification gates.

**Roadmap freshness:** As of 13 October 2026, Q1 and Q2 2026 milestones are historical. Their checkboxes are not qualification or deployment evidence. Reconfirm every completed item against a dated commit, reproducible build/test result, deployed environment or external partner receipt before carrying it forward.

---

## Industry problems we aim to address

| Area | Established industry baseline | Mycelix target / evidence required |
|---|---|---|
| **Creator economics** | Subscription/ad-supported streaming commonly allocates revenue through rights-holder pools and contractual splits; there is no universal per-stream rate. | Offer multiple economic models, and publish gross-to-net receipts. Any payout comparison must name the model, funding source, currency, sample and period. No 10x payout result is yet asserted. |
| **Fees** | DSPs, distributors, labels, payment rails and collecting societies have different cost bases and contract terms; a generic 30–40% “platform cut” is not an apples-to-apples benchmark. | Treat low fees as a target. Measure all-in cost including processor, gas, storage/CDN, FX, fraud, refunds and operations before quoting an effective rate. |
| **Settlement** | Timing depends on reporting cycles, agreements, thresholds, dispute/hold rules and payment rails. | Measure usage-to-statement, accrued-to-authorized and authorized-to-final-funds separately. “Instant” is not a current end-to-end claim. |
| **Rights metadata** | DDEX and CISAC standards define established message formats for releases, recording/rights revenue, contributors, work data and distribution. | Build version-pinned adapters and schema validation; mapping objects alone are not standards conformance. |
| **Transparency** | Reporting detail varies by participant and format. | Use evidence-linked accounting and correction history, then independently verify it against real partner reports. |
| **Privacy and resilience** | Data practices and decentralization vary across services. Hash/Merkle commitments alone do not provide zero-knowledge privacy or make a service unstoppable. | Minimize data, define disclosure policy, threat-model the system, test failure recovery and only claim specific properties that are implemented and reviewed. |

---

## Architecture: Progressive Decentralization

```
                    MYCELIX-MUSIC STACK
    ┌─────────────────────────────────────────────────┐
    │                                                  │
    │   PHASE 1: HYBRID (Current)                     │
    │   ├── Centralized API (Express → Rust)          │
    │   ├── Decentralized Contracts (Gnosis)          │
    │   └── Distributed Storage (IPFS)                │
    │                                                  │
    │   PHASE 2: FEDERATED                            │
    │   ├── Multiple API providers                    │
    │   ├── Community CDN nodes                       │
    │   └── Graph Protocol indexing                   │
    │                                                  │
    │   PHASE 3: FULLY DECENTRALIZED                  │
    │   ├── Holochain P2P streaming                   │
    │   ├── Community-owned infrastructure            │
    │   └── DAO governance                            │
    │                                                  │
    └─────────────────────────────────────────────────┘
```

---

## Phase 1: Foundation (Q1 2026)

### 1.1 Core Platform

- [x] Economic Strategy Router (Solidity)
- [x] Pay-Per-Stream Strategy
- [x] Gift Economy Strategy
- [x] Frontend (Next.js)
- [ ] Verify production build on the exact release head (record CI evidence)
- [ ] Real IPFS integration (Web3.Storage)
- [ ] Production deployment (Gnosis mainnet)

### 1.2 Artist Onboarding

- [ ] One-click artist registration
- [ ] Automatic royalty splits
- [ ] Economic strategy wizard
- [ ] Dashboard with real-time earnings

### 1.3 Listener Experience

- [ ] Web player with queue
- [ ] Mobile-responsive design
- [ ] Wallet connection (Privy)
- [ ] Discovery/recommendations

**Milestone:** 100 artists, 1,000 listeners, $10K in artist earnings

---

## Phase 2: Ecosystem Growth (Q2 2026)

### 2.1 Additional Economic Models

```solidity
// Expand from 2 to 11 economic strategies
- [ ] SubscriptionStrategy       // Monthly/yearly plans
- [ ] NFTGatedStrategy          // Token-gated exclusives
- [ ] PatronageStrategy         // Recurring support
- [ ] PayWhatYouWantStrategy    // Listener-set pricing
- [ ] AuctionStrategy           // Time-limited bidding
- [ ] StakingGatedStrategy      // Stake-to-listen
- [ ] TimeBarter (TEND)         // Skill exchange
- [ ] FreemiumStrategy          // Free tier + premium
- [ ] DownloadStrategy          // Pay-per-download
```

### 2.2 Community CDN

```
┌─────────────────────────────────────────────────┐
│              COMMUNITY CDN NETWORK               │
├─────────────────────────────────────────────────┤
│                                                  │
│   Node Operators earn TEND for:                  │
│   ├── Bandwidth served                          │
│   ├── Uptime percentage                         │
│   ├── Geographic diversity                      │
│   └── PoGQ quality score                        │
│                                                  │
│   Incentive Pool:                               │
│   ├── 0.1% of all payments → CDN rewards        │
│   ├── Artist-funded priority pinning            │
│   └── Genre DAO treasury allocations            │
│                                                  │
└─────────────────────────────────────────────────┘
```

### 2.3 Cross-Project Integration

- [ ] **PoGQ from Mycelix-Core** - CDN node quality scoring
- [ ] **MATL Trust Layer** - Artist verification
- [ ] **Living Treasury** - Shared FLOW token
- [ ] **CGC Registry** - Cross-ecosystem rewards

**Milestone:** 1,000 artists, 50,000 listeners, 10 CDN nodes, $500K in artist earnings

---

## Phase 3: Holochain Integration (Q3-Q4 2026)

### 3.1 Low-cost play recording (design goal)

```rust
// Holochain DNA for P2P streaming
dnas/mycelix-music/
├── zomes/
│   ├── catalog/        // Song metadata, IPFS hashes
│   ├── plays/          // Avoid a chain transaction per play where feasible; infrastructure still costs
│   ├── balances/       // Credit/debit tracking
│   ├── trust/          // MATL integration
│   └── discovery/      // Collaborative filtering
```

**Design goal:** Avoid a blockchain transaction for every play where feasible. Holochain actions, storage, replication, bandwidth, indexers, abuse prevention and cashout still carry costs; measure them before describing streaming as free. Cashout and bridging costs must be disclosed separately.

### 3.2 Hybrid Bridge

**Illustrative scenario only:** the $50 deposit, 5,000 plays, $0.01 per-play allocation and 1% fee in this diagram are assumed parameters for one possible economic strategy, not an industry benchmark, shipped payout rate or guarantee. The example excludes payment processing, gas, FX, refunds, reserves, tax/compliance costs and infrastructure costs; production terms must disclose them.

```
┌─────────────────────────────────────────────────┐
│           HOLOCHAIN ←→ GNOSIS BRIDGE            │
├─────────────────────────────────────────────────┤
│                                                  │
│   Listener deposits $50 FLOW on Gnosis          │
│   ↓                                              │
│   Credits appear on Holochain agent             │
│   ↓                                              │
│   Listener plays 5,000 tracks in this example   │
│   ↓                                              │
│   Example: $0.01/play (illustrative only)          │
│   ↓                                              │
│   Artist requests cashout                       │
│   ↓                                              │
│   Bridge validators attest balance              │
│   ↓                                              │
│   Net payout depends on disclosed all-in costs    │
│                                                  │
└─────────────────────────────────────────────────┘
```

### 3.3 P2P Discovery

- [ ] Holochain-based recommendation engine
- [ ] Privacy-preserving collaborative filtering
- [ ] No centralized algorithm control
- [ ] Community curation DAOs

**Milestone:** 10,000 artists, 500,000 listeners, $5M in artist earnings

---

## Phase 4: DAO Governance (2027)

### 4.1 Protocol DAO

```
┌─────────────────────────────────────────────────┐
│              MYCELIX-MUSIC DAO                   │
├─────────────────────────────────────────────────┤
│                                                  │
│   Voting Power:                                  │
│   ├── Artists (weighted by streams)              │
│   ├── Listeners (weighted by spend)              │
│   ├── CDN operators (weighted by bandwidth)      │
│   └── Developers (weighted by contributions)     │
│                                                  │
│   Governance Scope:                              │
│   ├── Protocol fee adjustments                   │
│   ├── New strategy approvals                     │
│   ├── Treasury allocations                       │
│   ├── Infrastructure decisions                   │
│   └── Dispute resolution                         │
│                                                  │
└─────────────────────────────────────────────────┘
```

### 4.2 Genre DAOs

- Hip-Hop DAO, Electronic DAO, Classical DAO, etc.
- Genre-specific curation
- Local treasury for genre promotion
- Artist collectives and label alternatives

### 4.3 Artist Agency (design objectives, not unconditional guarantees)

Any artist control promise depends on shipped access controls, contract semantics, dispute processes, governance and applicable law. The platform must publish a moderation/appeal policy and an emergency/legal-compliance path rather than claiming immunity from takedowns or unilateral capture.

```
┌─────────────────────────────────────────────────┐
│              ARTIST SOVEREIGNTY                  │
├─────────────────────────────────────────────────┤
│                                                  │
│   Artists Control:                               │
│   ├── Their economic model (any of 11)          │
│   ├── Their pricing (min $0.001)                │
│   ├── Their splits (collaborators, labels)      │
│   ├── Their content (no platform censorship)    │
│   ├── Their data (portable, exportable)         │
│   └── Their community (direct relationship)     │
│                                                  │
│   Platform Cannot:                               │
│   ├── Change artist terms unilaterally          │
│   ├── Remove content without DAO vote           │
│   ├── Adjust payouts retroactively              │
│   └── Sell artist data to third parties         │
│                                                  │
└─────────────────────────────────────────────────┘
```

**Milestone:** 100,000 artists, 5M listeners, $50M in artist earnings, fully community-owned

---

## Industry Disruption Strategy

### Target Segments

| Segment | Existing category | Mycelix opportunity (target; not a proven advantage) |
|---|---|---|
| **Independent Artists** | Streaming and direct-sales services | Offer multiple economic models, itemized gross-to-net statements and exportable evidence. Validate with a rights-cleared pilot. |
| **Labels and collectives** | Contract, rights and split-management tools | Versioned split agreements and attributable settlement receipts, subject to legal and partner integration requirements. |
| **Podcasters** | Hosting and distribution services | Portability and transparent participant allocations if built and tested; no censorship-immunity claim. |
| **Music educators** | Membership and digital-commerce tools | Combine subscriptions, patronage or licensing where those models are actually released. |
| **Live performers** | Ticketing and event platforms | Optional event-linked exclusives; evaluate accessibility, refund handling and local regulation before launch. |
| **Sample makers** | Sample catalogues and licensing tools | Explicit licence terms and usage-to-payment records; do not claim better royalties until measured. |

### Potential differentiators (hypotheses to validate)

1. **Accounting provenance:** Explicit obligation, deduction, settlement receipt, allocation-lineage and statement commitments may make royalty decisions easier to reconstruct. This advantage exists only when exact-head qualification, source integrity and independent verification succeed.
2. **Economic choice:** Subscription, direct support, pay-per-play and other models can serve different audiences. No model guarantees higher creator income independent of funding, engagement, rights splits and costs.
3. **Federation and portability:** Multiple operators and community infrastructure could reduce reliance on one provider. The resilience claim requires tested multi-node operation, key recovery and incident handling.
4. **Artist governance:** Artists can have more control if contract rights, deployment permissions and dispute processes enforce it in practice. A roadmap statement or DAO alone is not a guarantee against capture.

### Growth Strategy

```
Year 1: Pilot (Electronic, Experimental, Indie)
        └── Validate retention, payout comprehension and economics with a rights-cleared cohort

Year 2: Expansion (additional genres and territories)
        └── Expand only when support, rights metadata and settlement metrics remain within published bounds

Year 3: Larger-scale distribution
        └── Establish repeatable onboarding, dispute handling, partner interoperability and incident response

Year 5: Interoperable creator infrastructure
        └── Aspirational direction, not a forecast or a claim that legacy platforms will decline
```

---

## Technical Roadmap

### Q1 2026: Stabilization
- [x] Fix TypeScript build failures
- [ ] Real Web3.Storage integration
- [ ] Production deployment to Gnosis mainnet
- [ ] Artist onboarding wizard
- [ ] Mobile-responsive player

### Q2 2026: Rust Migration
- [ ] Axum API server (replace Express)
- [ ] Shared crates with Mycelix-Core
- [ ] Event indexer (Graph Protocol or custom)
- [ ] CDN node software v1

### Q3 2026: Holochain MVP
- [ ] Catalog zome (song metadata)
- [ ] Plays zome (avoid per-play chain transactions; benchmark storage, replication and network costs)
- [ ] Balances zome (credit tracking)
- [ ] Bridge contracts (cashout)

### Q4 2026: Full P2P
- [ ] WebRTC streaming integration
- [ ] P2P discovery/recommendations
- [ ] Mobile apps (iOS/Android via Capacitor)
- [ ] Offline mode with sync

### 2027: DAO Launch
- [ ] Protocol DAO deployment
- [ ] Genre DAO framework
- [ ] On-chain governance
- [ ] Full community ownership transition

---

## Success Metrics

The figures below are **aspirational planning targets, not forecasts or current user/earnings counts**. Report actuals separately, with date range, cohort definition, rights-cleared status and source evidence.

| Metric | Year 1 target | Year 2 target | Year 3 target | Year 5 target |
|--------|--------|--------|--------|--------|
| **Artists** | 1K | 10K | 100K | 1M |
| **Listeners** | 50K | 500K | 5M | 50M |
| **Artist Earnings** | $500K | $5M | $50M | $500M |
| **CDN Nodes** | 10 | 100 | 1K | 10K |
| **Royalty transparency** | Publish a real sample statement and gross-to-net method | Report correction/dispute rates | Add independently verified reporting partners | Publish multi-territory audit results |
| **All-in payout cost** | Establish measured baseline by model and rail | Publish median / P95 and all included fees | Set targets from the observed baseline | Reassess against real operating costs |

---

## Call to Action

**For Artists:**
- Join a pilot with plainly disclosed economic terms
- Choose among the models actually available in the released product
- Export your statements and verify gross-to-net calculations
- No payout multiple or settlement time is guaranteed before pilot evidence is published

**For Developers:**
- Contribute to open source
- Build on the Mycelix SDK
- Run a CDN node

**For Listeners:**
- Support artists directly where the selected economic model permits it
- Privacy-minimizing design and clear disclosure of data collection; advertising-free listening is a product goal
- Participate in community governance if and when the relevant mechanism ships

---

## Resources

- **Website:** mycelix.net
- **GitHub:** github.com/Luminous-Dynamics/Mycelix-Music
- **Docs:** docs.mycelix.net/music
- **Discord:** discord.gg/mycelix

---

*"The music industry was built on exploitation. We're building the alternative."*

**Mycelix-Music: Fairer economics, verifiable accounting, and artist agency—measured in practice, not just promised.**
