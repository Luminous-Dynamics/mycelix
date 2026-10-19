# 🎵 Mycelix Music: Modular Music-Economy Prototype

> **Research and prototype source—not a production release.**

**Vision:** Every artist should be able to choose transparent, interoperable economic terms for their music.
**Design hypothesis:** Pluggable economic strategies and evidence-linked accounting can improve creator agency and statement auditability.
**Goals:** Portable data, explicit gross-to-net reporting, privacy-conscious operation and resilient/federated infrastructure. No payout multiple, universal per-play rate, zero-cost operation or censorship-immunity claim is made.

---

## ⚡ Verified Status Reference (13 October 2026)

| Component | Status | Notes |
|-----------|--------|-------|
| **Smart Contracts** | ⚠️ Source present | Not audited or verified in this snapshot |
| **Leptos Frontend** | ⚠️ Integration work | Fixture-free by default; external path dependencies required |
| **Backend APIs** | ⚠️ Multiple prototypes | TypeScript and Rust implementations are not established as equivalent |
| **IPFS Storage** | ⚠️ Dependency | Credentials and end-to-end verification required |
| **Holochain** | ⚠️ Source present | Economic mutations fail closed pending real authorization and atomic settlement |
| **CDN** | 📋 Design/prototype | Production operation is not demonstrated here |

See [IMPLEMENTATION_STATUS.md](./IMPLEMENTATION_STATUS.md) for verified behavior, blockers, and release gates. Roadmaps and phase reports describe intent, not current evidence. See [Industry Comparison and Evidence](docs/INDUSTRY_COMPARISON_AND_EVIDENCE.md) for the current market benchmark, standards roadmap and evidence gates.

---

## 🌟 The Revolutionary Concept

Music reaches creators through several business models: streaming subscription/ad pools, direct sales, memberships, licensing and collective-rights reporting. Their economics and settlement terms are not interchangeable; streaming royalties do not use one universal per-stream rate. See [Spotify's royalties explanation](https://artists.spotify.com/en/royalties-guide) and the [industry comparison](docs/INDUSTRY_COMPARISON_AND_EVIDENCE.md).

**Mycelix Music's design direction:** let artists select from configurable economic strategies while keeping terms, accounting evidence and net payouts inspectable. The repository is a research/prototype source; production artist payouts and cross-platform conformance are not established.

### Illustrative Product Sketches (not live artist case studies)

The following profiles show intended choices. They do not imply deployed onboarding, real users, actual payouts or verified performance.

**DJ Nova** (electronic artist):
- **Model:** Gift Economy
- **Listening:** FREE
- **Monetization:** Optional tips + listener rewards with CGC tokens
- **Why:** Building community first, monetize later

**The Echoes** (indie rock band):
- **Model:** Configurable pay-per-play strategy
- **Listening price:** Defined by the creator's disclosed model and funding terms; no universal rate is implied
- **Monetization:** Contract-configured split among band members and producer after transaction finality; production cash-out is not established
- **Why:** Established fanbase, want immediate revenue

**Symphony Orchestra** (concept sketch):
- **Model:** Patronage/subscription
- **Listening:** Example membership price configured by the orchestra
- **Monetization:** Split among musicians according to an explicit agreement
- **Status:** Patronage and subscription paths require implementation and qualification before being presented as available

These sketches are intended to share a common strategy interface. End-to-end integration across all three workflows has not been demonstrated.

---

## 📁 Repository Structure

```
mycelix-music/
├── contracts/                          # Smart contracts (Solidity)
│   ├── EconomicStrategyRouter.sol     # Core routing logic
│   ├── strategies/
│   │   ├── PayPerStreamStrategy.sol   # configurable pay-per-play prototype
│   │   └── GiftEconomyStrategy.sol    # Free + tips model
│   └── test/                           # Comprehensive tests
│
├── packages/
│   └── sdk/                            # TypeScript SDK
│       └── src/
│           └── economic-strategies.ts  # High-level API
│
├── apps/
│   └── web/                            # Next.js frontend
│       ├── components/
│       │   └── EconomicStrategyWizard.tsx  # Artist UI
│       └── pages/
│           ├── upload/                 # Upload + config flow
│           └── dashboard/              # Analytics
│
├── docs/
│   ├── ECONOMIC_MODULES_ARCHITECTURE.md   # Design philosophy
│   ├── IMPLEMENTATION_EXAMPLE.md          # Complete walkthrough
│   ├── DEPLOYMENT_GUIDE.md                # Deploy to production
│   └── Business Plan v1.0.md              # Business context
│
└── scripts/                            # Deployment & testing
    ├── deploy-*.js
    └── test-*.js
```

---

## 🎯 Quick Start

### For Developers: Run Locally (30 Minutes)

**Follow the development guide:** [**docs/guides/QUICKSTART.md**](./docs/guides/QUICKSTART.md)

#### Option A: Nix Development Shell (recommended)

If you're on NixOS (or have Nix installed) the repo already provides a flake:

```bash
# 0. Drop into the dev shell (installs Node 20, npm, git, TypeScript tooling, etc.)
nix develop

# 1. Install JavaScript dependencies inside the shell
npm install

# 2. Continue with the steps below (anvil, deploy, services, etc.)
```

The shell works on any system supported by `nixos-unstable`, so teammates get identical toolchains without touching global Node installs.

#### Option B: Manual setup (Node/npm on host)

> **Note:** Contract scripts (`npm run contracts:*`, `forge fmt`, etc.) require [Foundry](https://book.getfoundry.sh/getting-started/installation). If Foundry isn't installed the contract lint step simply logs a warning and continues.

```bash
# 1. Navigate to project
cd mycelix-music

# 2. Install dependencies
npm install

# 3. Set up environment
cp .env.example .env

# 4. Start blockchain (Terminal 1)
anvil --block-time 1

# 5. Deploy contracts (Terminal 2)
npm run contracts:deploy:local

# 6. Start services (Terminal 3)
npm run services:up

# 7. Seed test data
npm run seed:local

# 8. Start frontend
cd apps/web && npm run dev

# 9. Visit http://localhost:3000 🎉
```

The guide uses development services and test data. It does not establish working production payment flows.

**Detailed instructions, troubleshooting, and verification:** See [QUICKSTART.md](./QUICKSTART.md)

### Developer Workflows & Quality Gates

- `npm run lint` &mdash; runs Turbo-powered linting across API, SDK, frontend, and contract packages. If Foundry (`forge`) isn’t installed, the contracts lint step logs `[contracts] forge not found; skipping lint`.
- `npm run lint --workspace=<pkg>` &mdash; lint an individual workspace (e.g., `apps/api`, `packages/sdk`, `apps/web`) when iterating locally.
- `npm run test --workspace=packages/sdk` &mdash; runs the Jest suite for the TypeScript SDK (watchman disabled via local Jest config, safe for sandboxed CI).
- `nix flake check` &mdash; once run outside sandboxed environments, this executes the flake-defined checks; for now the dev shell (`nix develop`) is the primary Nix entrypoint.

### For Artists: Intended onboarding flow (not yet a public workflow)

The following is a product sketch. The README marks the public site as coming soon; do not treat these steps as an available or payment-qualified flow.

1. **Visit** https://mycelix.music (coming soon)
2. **Connect Wallet** (we'll create one for you!)
3. **Upload Song** (FLAC, MP3, or WAV)
4. **Choose Economics:**
   - Independent Artist (pay-per-stream)
   - Community Collective (gift economy)
   - Custom (build your own)
5. **Set Revenue Splits** (you + collaborators)
6. **Deploy** (one click, ~$0.50 gas fee)
7. **Share Link** with your fans!

---

## 🏗️ Architecture Highlights

### 1. Smart-contract router pattern (prototype design)

```solidity
// Intended pattern: route a payment to a configured strategy
songStrategy[songId] => PayPerStreamStrategy | GiftEconomyStrategy | CustomStrategy
router.processPayment(songId, amount)
  → strategy logic validates and allocates the transaction
  → chain confirmation/finality still applies
```

**Design benefits to validate:**
- Add strategies without changing the router's core interface.
- Give each release a disclosed economic configuration.
- Make on-chain transaction paths inspectable where they actually execute.
- Do not equate an on-chain allocation with a bank payout: production cash-out, reversals, compliance, source accounting and off-chain services need separate proof.

### 2. TypeScript SDK example (illustrative configuration)

```typescript
import { EconomicStrategySDK } from '@mycelix/sdk';

// Artist registers song
await sdk.registerSong('my-song-id', {
  strategyId: 'pay-per-stream-v1',
  paymentModel: PaymentModel.PAY_PER_STREAM,
  distributionSplits: [
    { recipient: artistAddress, basisPoints: 9500, role: 'artist' },
    { recipient: protocolAddress, basisPoints: 500, role: 'protocol' }
  ],
  minimumPayment: 0.01, // example token-unit threshold; not a USD quote
});

// Example amount in the configured token unit; not a universal price or USD quote.
await sdk.streamSong('my-song-id', '0.01'); // illustrative configured token amount

// Transaction success/finality and subsequent cash-out must be verified separately.
```

### 3. React UI Wizard

```tsx
<EconomicStrategyWizard songId="my-song" />
```

**Result:** Beautiful 5-step wizard that guides artists through:
1. Choose preset or custom model
2. Configure payment model
3. Set revenue splits
4. Add listener incentives
5. Review & deploy

---

## 📊 Economic strategy prototypes in source

The contracts below are source-level prototypes, not a production-qualified payment service. Production use requires current-head tests, security review, token/treasury authority, anti-abuse controls, payment finality handling, terms disclosure and a tested cash-out path.

### Model 1: Pay-per-play strategy

**Prototype behavior and limits:**
- The strategy can be configured with a per-play amount denominated in its token/accounting unit; that amount is not an industry-wide rate and must not be described as USD unless a supported conversion authority says so.
- Contract logic can allocate a confirmed transaction among configured recipients. It does not establish instant bank settlement or eliminate blockchain, processor, storage, FX, fraud, dispute or operating costs.
- The amount, funding source, split basis, failure/reversal policy and all-in fee must be disclosed before a creator or listener opts in.

**Smart contract:** `PayPerStreamStrategy.sol`  
**Potential use case:** Artists with a fanbase who prefer a direct, explicitly funded per-play model.

### Model 2: Gift-economy strategy

**Prototype concept:**
- Free listening plus optional tips and token incentives.
- Any listener-reward mechanism requires a documented token source, anti-fraud/sybil controls, budget limits and a disclosure of the token's actual value and redemption rights.
- Incentive amounts and multipliers in code examples are not evidence of funded or paid rewards.

**Smart contract:** `GiftEconomyStrategy.sol`  
**Potential use case:** Community-building and experimental music.

### Model 3+: Planned strategies

- **Patronage:** Monthly support.
- **NFT-gated:** Token-gated releases, subject to rights/accessibility requirements.
- **Pay what you want:** Listener-selected amount.
- **Auction:** Time-limited release pricing.
- **Time barter:** Exchange access for a defined contribution or unit.

These are planned/optional models, not promises that all are implemented. Each model needs its own conservation tests, funding and reversal rules, dispute semantics and disclosure.

---

## 💡 What the prototype is trying to differentiate

This is a design comparison, not an audited ranking of competitors. Features and commercial terms change, and a fair comparison needs the same date, geography, rights type, currency, cost basis and production evidence.

| Dimension | Mycelix design direction | Evidence still needed |
|---|---|---|
| Economic choice | Pluggable per-release strategies and alternative funding models | Which models actually ship, creator adoption, disclosed terms and net payouts |
| Accounting | Typed obligations, deductions, settlement receipts, statement projections and lineage commitments are being developed in the accounting layer | Exact-head qualification, real-source reconciliation, correction/dispute rates and an independent verifier |
| Interoperability | DDEX/CRD-oriented field mappings are a starting point | Official schema/profile validation, licence and partner acceptance; mapping objects are not conformant messages |
| Privacy | Minimize data and support carefully scoped disclosure | Full data-flow review and validation of each privacy property; no production zero-knowledge claim |
| Resilience/governance | Hybrid/federated operation is a direction | Multiple live operators, key recovery, moderation/appeals, upgrade controls and failure-recovery evidence |

See [Industry Comparison and Evidence](docs/INDUSTRY_COMPARISON_AND_EVIDENCE.md) for the dated benchmark and the evidence required before making comparative claims.

---

## 🚀 Implementation Status

### Prototype component inventory (not release qualification)

**Smart Contracts (630 lines)**
- [x] EconomicStrategyRouter.sol with pluggable strategies
- [x] PayPerStreamStrategy.sol with contract-level split logic in the prototype source (not production payout qualification)
- [x] GiftEconomyStrategy.sol with CGC rewards
- [x] Comprehensive Foundry test suite (400 lines, 12 tests)
- [x] DeployLocal.s.sol with mock tokens

**TypeScript SDK (475 lines)**
- [x] Complete high-level API for frontend integration
- [x] 4 preset configurations (Independent, Band, Gift, Producer)
- [x] Batch operations for gas optimization
- [x] Full TypeScript type safety

**Frontend Application (1,450 lines)**
- [x] Next.js 14 with Tailwind CSS
- [x] Landing page with glass morphism design
- [x] Upload wizard (file → metadata → strategy → deploy)
- [x] Discover page with filters and streaming
- [x] Privy wallet authentication (ready)
- [x] Smooth animations with Framer Motion

**Backend API (300 lines)**
- [x] Express REST API with 9 endpoints
- [x] PostgreSQL database with schema
- [x] Redis caching layer
- [x] IPFS upload integration (structure ready)
- [x] DKG claim creation (structure ready)

**Infrastructure & DevOps**
- [x] Docker Compose for all services
- [x] Automated deployment scripts
- [x] Test data seeding script
- [x] Complete .env.example template
- [x] Turborepo monorepo structure

**Documentation (37,000 words)**
- [x] QUICKSTART.md (30-minute setup)
- [x] ECONOMIC_MODULES_ARCHITECTURE.md (7K words)
- [x] MYCELIX_PROTOCOL_INTEGRATION.md (8K words)
- [x] IMPLEMENTATION_EXAMPLE.md (5K words)
- [x] DEPLOYMENT_GUIDE.md (4K words)
- [x] INTEGRATION_COMPLETE.md (10K words)
- [x] IMPLEMENTATION_STATUS.md (current status)
- [x] SESSION_CONTINUATION_SUMMARY.md (this session)

**Current State:** 🟡 **Prototype components are present; testnet readiness is not established.**

### Production-readiness gates (no date promised)

- [ ] Get real service keys (Privy, Web3.Storage, Ceramic)
- [ ] Replace mocked IPFS/DKG with real implementations
- [ ] Add audio player component (Howler.js)
- [ ] Deploy contracts to Gnosis Chiado testnet
- [ ] Invite 10 beta testers

### Pilot gates (after production-readiness checks)

- [ ] Recruit 50 founding artists
- [ ] Launch "Independent Electronic Producers" DAO (first Hearth)
- [ ] 1000+ real streams
- [ ] Gather feedback on economics
- [ ] Build artist dashboard with analytics

### Mainnet gates (only after explicit sign-off)

- [ ] Security audit ($15-25K)
- [ ] Deploy to Gnosis Chain mainnet
- [ ] Launch 3+ genre DAOs
- [ ] 10K+ artists, 100K+ listeners
- [ ] Add 2-3 more economic strategies based on feedback

---

## 🤝 Contributing

We welcome contributions in all areas:

### For Developers
- **Smart contracts:** Add new economic strategies
- **Frontend:** Improve artist/listener UX
- **Testing:** Write comprehensive test suites
- **Documentation:** Improve guides and examples

### For Artists
- **Beta testing:** Try the platform and provide feedback
- **Economic design:** Propose new payment models
- **Community:** Help onboard other artists

### For Researchers
- **Economic modeling:** Analyze strategy performance
- **Governance:** Design DAO mechanisms
- **Music theory:** How does economics affect creativity?

**How to contribute:**
1. Read [CONTRIBUTING.md](CONTRIBUTING.md)
2. Pick an issue or propose a feature
3. Submit a PR with tests and documentation
4. Celebrate being part of the revolution! 🎉

---

## 📚 Key Documentation

**Start here:**
1. [Economic Modules Architecture](docs/architecture/ECONOMIC_MODULES_ARCHITECTURE.md) - Core design philosophy
2. [Implementation Example](docs/architecture/IMPLEMENTATION_EXAMPLE.md) - Illustrative walkthrough
3. [Deployment Guide](docs/guides/DEPLOYMENT_GUIDE.md) - Deployment design; observe the readiness gates first

**Deep dives:**
- [Business Plan v1.0](Business%20Plan%20v1.0.md) - Market strategy
- [Technical Design](Technical%20Design%20v1.0.md) - Detailed architecture
- [Smart Contracts](contracts/) - Solidity implementation
- [SDK Documentation](packages/sdk/README.md) - TypeScript API

---

## 🎤 The Vision

Music is the most universal form of human expression. Yet the industry that distributes it is broken:
- Artists earn pennies while platforms extract billions
- Listeners have no say in how artists are paid
- One economic model forced on everyone from classical to punk

**We're building something different:**

> A platform where every artist is sovereign, every listener is valued, and every community can design its own economy.

This is not just a music platform. It's an experiment in **economic pluralism**. It's proof that decentralization can be BETTER than centralization, not just more idealistic.

If we succeed, we won't just change music. We'll show that the future of the internet is not a few giant platforms, but millions of interconnected communities, each with their own values and economics.

**That's worth building.** 🚀

---

## 📞 Contact & Community

- **Website:** https://mycelix.music (coming soon)
- **Discord:** https://discord.gg/mycelix
- **Twitter:** [@MycelixMusic](https://twitter.com/MycelixMusic)
- **GitHub:** https://github.com/mycelix/mycelix-music
- **Email:** hello@mycelix.music

---

## 📜 License

This snapshot is licensed under the [GNU AGPL v3 or later](LICENSE), matching its package metadata and source headers.

The smart contracts have additional audit requirements before mainnet deployment (see [SECURITY.md](SECURITY.md)).

---

## 🙏 Acknowledgments

Built on the shoulders of giants:
- **Holochain** for agent-centric architecture
- **Ceramic Network** for decentralized knowledge graphs
- **IPFS** for distributed file storage
- **Ethereum/Gnosis** for economic rails
- **Mycelix Protocol** for trust infrastructure

And inspired by the vision that technology should amplify consciousness, not exploit attention.

---

**Status:** Prototype; see [IMPLEMENTATION_STATUS.md](./IMPLEMENTATION_STATUS.md) before deployment or economic use.

🎵 **Let's rebuild music, together.** 🎵

---

**Key Documents:**
- [**Quickstart**](./docs/guides/QUICKSTART.md) - Development setup
- [**IMPLEMENTATION_STATUS.md**](./IMPLEMENTATION_STATUS.md) - Complete status report
- [**Historical phase reports**](./docs/phases/) - Prior progress notes; not release evidence
