# Mycelix Music Federated Artist Networks Design Specification

**Status:** Proposed  
**Audience:** Mycelix Music product, protocol, Holochain, DKG, governance, economics, frontend, and community contributors  
**Primary goal:** Allow artists to form independent, interoperable streaming networks without surrendering ownership, identity, provenance, or exit rights  
**Suggested repository path:** `docs/design/MYCELIX_MUSIC_FEDERATED_ARTIST_NETWORKS_DESIGN_SPEC.md`  
**Related documents:**
- `README.md`
- `INDUSTRY_ROADMAP.md`
- `MYCELIX_PROTOCOL_INTEGRATION.md`
- `HOLOCHAIN_ECONOMICS_STRATEGY.md`
- `ECONOMIC_MODELS_IMPROVED.md`
- `MUSE_ARTIST_CAPABILITY_AND_PUBLISHING_INTEGRITY_DESIGN_SPEC.md`

---

## 1. Purpose

Mycelix Music should not become only one decentralized streaming service.

It should become shared infrastructure through which artists can create, govern, brand, and operate their own music services while remaining interoperable with the wider Mycelix ecosystem.

A federated artist network may operate its own:

- brand;
- domain;
- catalog;
- artist membership;
- editorial voice;
- listener pricing;
- economic model;
- treasury;
- moderation standards;
- discovery policy;
- community programs;
- streaming infrastructure;
- governance.

The shared Mycelix layer should preserve:

- portable identity;
- canonical works and releases;
- DKG provenance;
- contributor claims;
- rights and license records;
- artist-controlled catalog relationships;
- SAP settlement;
- transparent splits;
- listener portability;
- network interoperability;
- dispute and correction history.

The defining principle is:

> Networks own their service and culture. Artists retain their identity, works, rights, provenance, and ability to leave.

---

## 2. Why federation matters

A single community-owned platform can still become:

- culturally centralized;
- economically rigid;
- algorithmically dominant;
- geographically distant;
- difficult for niche communities to govern;
- vulnerable to institutional capture.

Federation allows many legitimate music economies to coexist.

Examples:

- a Cape Jazz cooperative;
- a global experimental-music network;
- a classical-orchestra federation;
- a regional African music service;
- an independent game-music network;
- a singer-songwriter cooperative;
- a public-domain educational archive;
- a label-owned service with transparent contracts;
- a worker-owned streaming platform;
- a network for adaptive and interactive music licensing.

Each may look like a distinct streaming platform to its users while sharing Mycelix identity, provenance, settlement, and catalog protocols.

---

## 3. Product terminology

### 3.1 User-facing terms

Use:

- **Music Network**
- **Artist Cooperative**
- **Streaming Collective**
- **Community**
- **Federation**

Avoid requiring ordinary users to understand:

- Sector DAO;
- Hearth DAO;
- DHT;
- DKG;
- protocol provider;
- settlement bridge.

Those terms may remain visible in technical and governance views.

### 3.2 Protocol terms

```rust
pub type NetworkId = String;
pub type FederationId = String;
pub type WorkId = String;
pub type ReleaseId = String;
pub type AgentId = String;
```

A **Music Network** is a branded listener-and-artist service.

A **Federation** is an interoperability agreement among one or more Music Networks.

A **Collective** is an artist group that may publish together without operating a complete streaming service.

A **Provider** operates network infrastructure such as indexing, streaming, storage, or settlement.

---

## 4. Core product principles

### 4.1 Artist portability

An artist must be able to leave a network without losing:

- identity;
- work and release identifiers;
- provenance;
- contributor history;
- follower relationships where followers consent to portability;
- earnings records;
- licenses;
- supporter relationships;
- public dispute history;
- the ability to publish elsewhere.

### 4.2 Non-exclusive by default

Joining a network grants only the distribution rights explicitly accepted.

Exclusivity must be:

- explicit;
- narrow;
- time-bounded;
- visible to collaborators;
- revocable according to agreed terms;
- recorded in the DKG.

### 4.3 Economic transparency

Before joining or subscribing, users must be able to understand:

- price;
- platform fee;
- artist share;
- treasury allocation;
- payout method;
- reserve policy;
- refund or cancellation terms;
- whether payouts are user-centric or pooled.

### 4.4 Interoperability before sameness

Networks may differ in:

- curation;
- culture;
- pricing;
- design;
- governance;
- moderation;
- discovery;
- access policy.

They must agree on shared protocol contracts for:

- identity;
- work references;
- provenance;
- rights;
- settlement;
- portability;
- disputes;
- federation discovery.

### 4.5 Shared infrastructure without shared ownership

A network may distribute or index a canonical release without owning the underlying work.

### 4.6 Exit without erasure

Leaving a network may end that network's right to distribute a work.

It must not erase:

- the canonical work;
- valid release history;
- contributor claims;
- public licenses;
- disputes;
- corrections;
- prior settlement records.

### 4.7 Listener dignity

Listeners should not need:

- a separate wallet for every network;
- a separate identity for every community;
- a complete rebuild of their library;
- knowledge of protocol internals.

---

## 5. Roles

### Artist

Publishes works, joins networks, configures economics, approves licenses, and receives settlement.

### Listener

Subscribes, pays per use, supports artists, follows networks, and maintains a portable private library.

### Network Founder

Creates a network, defines its initial charter, invites founding members, and configures infrastructure.

### Network Steward

Operates catalog, moderation, support, finances, and community programs under the network charter.

### Curator

Creates collections, programs releases, and explains editorial choices.

### Contributor

Performer, composer, arranger, producer, engineer, illustrator, writer, or other credited participant.

### Infrastructure Provider

Provides storage, streaming, indexing, availability, or settlement services.

### Federation Steward

Maintains interoperability agreements between networks.

### Auditor or Resolver

Investigates disputes, verifies process, and records findings without owning the underlying claims.

---

## 6. Core entities

```rust
pub struct MusicNetwork {
    pub id: NetworkId,
    pub name: String,
    pub slug: String,
    pub description: String,
    pub charter_id: String,
    pub manifest_id: String,
    pub governance_id: String,
    pub treasury_id: String,
    pub settlement_policy_id: String,
    pub discovery_policy_id: String,
    pub moderation_policy_id: String,
    pub branding_id: String,
    pub status: NetworkStatus,
}
```

```rust
pub struct NetworkMembership {
    pub network_id: NetworkId,
    pub agent_id: AgentId,
    pub role: NetworkRole,
    pub joined_at: i64,
    pub rights: Vec<NetworkPermission>,
    pub obligations: Vec<NetworkObligation>,
    pub status: MembershipStatus,
}
```

```rust
pub struct NetworkCatalogGrant {
    pub network_id: NetworkId,
    pub release_id: ReleaseId,
    pub grantor: AgentId,
    pub license_id: String,
    pub territories: Vec<String>,
    pub starts_at: i64,
    pub ends_at: Option<i64>,
    pub exclusivity: Exclusivity,
    pub revocation_policy: String,
}
```

```rust
pub struct ListenerSubscription {
    pub listener_id: AgentId,
    pub network_id: NetworkId,
    pub plan_id: String,
    pub price_sap: u64,
    pub started_at: i64,
    pub renewal: RenewalPolicy,
    pub status: SubscriptionStatus,
}
```

---

## 7. Federation manifest

Every network must publish a signed, machine-readable manifest and a human-readable summary.

### 7.1 Required manifest fields

- network identity;
- legal or cooperative operator where applicable;
- governance model;
- membership rules;
- pricing;
- platform fee;
- artist payout formula;
- treasury allocation;
- reserve policy;
- settlement frequency;
- accepted economic strategies;
- moderation policy;
- discovery policy;
- recommendation policy;
- data policy;
- portability policy;
- exclusivity policy;
- dispute process;
- artist exit rights;
- listener cancellation rights;
- infrastructure providers;
- protocol version;
- supported interoperability capabilities;
- current signatories;
- effective date;
- superseded manifest version.

### 7.2 Human-readable subscription summary

Example:

> **9 SAP per month**  
> 90% goes to artists you actually hear.  
> 5% operates the network.  
> 5% funds new recordings.  
> Your library and follows remain portable.  
> Cancel at any time.

### 7.3 Manifest versioning

A manifest change must:

- create a new signed version;
- preserve the prior version;
- state which terms changed;
- define the effective date;
- notify affected artists and listeners;
- prohibit retroactive economic changes.

Material changes may require renewed consent.

---

## 8. Economic models

Networks may compose economics from shared primitives.

### 8.1 Supported primitives

- free access;
- pay per stream;
- freemium;
- pay what you want;
- tips;
- patronage;
- network subscription;
- artist subscription;
- purchase to keep;
- download;
- institutional license;
- catalog license;
- adaptive-media license;
- commission;
- community-funded release;
- time exchange with TEND;
- limited release;
- public subsidy;
- shared treasury funding.

### 8.2 Mixed economic models

A network may support different models per release.

Example:

- new releases: patron-only for one month;
- primary catalog: network subscription;
- older catalog: free with tips;
- live recordings: pay what you want;
- educational scores: institutional license;
- adaptive assets: commercial license.

### 8.3 SAP

SAP is the shared spendable or transferable settlement unit.

User-facing presentation should describe purpose before symbol:

- Support balance: 120 SAP
- Subscribe for 9 SAP/month
- Tip 2 SAP
- Pay 0.01 SAP to listen

Do not present SAP as the meaning of participation itself.

### 8.4 MYCEL

MYCEL is non-transferable community standing.

It must not appear as a wallet balance beside SAP.

Use:

> Community standing: 742 MYCEL  
> Cannot be bought, sold, or transferred.

MYCEL may influence eligibility for stewardship or moderation roles, but it should not automatically determine artistic visibility.

### 8.5 CGC and TEND

CGC represents gratitude or community appreciation.

TEND represents reciprocal time and service exchange.

The UI must clearly separate:

- value;
- reciprocity;
- community standing;
- appreciation.

---

## 9. Subscription settlement

### 9.1 User-centric payout recommended

The default network subscription model should distribute the artist pool according to each listener's actual eligible listening.

Example:

- listener pays 10 SAP;
- 1 SAP funds network operations;
- 1 SAP funds the shared treasury;
- 8 SAP is divided among artists the listener heard.

This avoids routing a niche listener's payment primarily to globally dominant artists.

### 9.2 Alternative payout policies

Networks may choose:

- user-centric;
- pooled proportional;
- equal member allocation;
- weighted cooperative dividend;
- project funding pool;
- hybrid allocation.

The policy must be explicit and signed.

### 9.3 Cross-network bundles

Federations may offer bundles.

Example:

> Join any five participating networks for 20 SAP/month.

The clearing layer must record:

- bundle payment;
- selected networks;
- allocation formula;
- listening-based allocation;
- infrastructure fees;
- artist splits;
- settlement receipts.

### 9.4 Reserve and solvency

Listener deposits and artist earnings must not become invisible unsecured operating capital.

Networks should publish:

- reserve policy;
- pending artist obligations;
- settlement liabilities;
- cashout availability;
- bridge status;
- material incidents.

---

## 10. Catalog model

### 10.1 Canonical release

The canonical release exists independently of any one network.

A network receives a catalog grant that references it.

### 10.2 Multiple network carriage

A release may appear in multiple networks with:

- different editorial context;
- different listener access;
- different support options;
- territory-specific terms;
- network-specific collections.

The canonical provenance remains the same.

### 10.3 Preferred provider

When several networks carry the same release, a listener may select:

- preferred network;
- lowest cost;
- strongest direct artist support;
- local community;
- best audio availability;
- artist-preferred provider.

### 10.4 Catalog withdrawal

Withdrawal should:

- end future distribution according to the grant;
- preserve prior settlement;
- preserve release history;
- update availability;
- notify affected collections;
- not erase the canonical work.

---

## 11. DKG provenance

The DKG is the canonical claim and relationship layer.

### 11.1 Network-related entities

- `MusicNetwork`
- `Federation`
- `NetworkManifest`
- `NetworkCharter`
- `Membership`
- `CatalogGrant`
- `SubscriptionPlan`
- `SettlementPolicy`
- `DiscoveryPolicy`
- `ModerationPolicy`
- `TreasuryPolicy`
- `InfrastructureProvider`
- `FederationAgreement`
- `BundleAgreement`
- `Dispute`
- `Correction`
- `Revocation`

### 11.2 Relationships

- `member_of`
- `steward_of`
- `federates_with`
- `carries_release`
- `licensed_by`
- `settles_through`
- `served_by`
- `governed_by`
- `supersedes_manifest`
- `withdrawn_from`
- `disputes`
- `corrects`
- `revokes`
- `endorsed_by`
- `preferred_by_artist`

### 11.3 Claim states

- proposed;
- signed;
- active;
- superseded;
- disputed;
- corrected;
- revoked;
- expired.

The DKG proves:

- who made a claim;
- what was claimed;
- when it was claimed;
- what evidence was attached;
- which version is current.

It does not automatically prove legal ownership.

---

## 12. Holochain architecture

Holochain should provide agent-centric state, network discovery, catalog availability, listening records, and low-cost settlement events.

### 12.1 Suggested DNAs or zomes

```text
mycelix-music
├── identity
├── works
├── releases
├── networks
├── federation
├── memberships
├── catalog-grants
├── listening
├── subscriptions
├── settlement
├── treasury
├── discovery
├── moderation
├── disputes
└── infrastructure
```

### 12.2 Agent-centric ownership

Artists maintain:

- source-chain publication events;
- catalog grants;
- membership decisions;
- payout receipts;
- withdrawal events.

Listeners maintain:

- private follows;
- private library;
- subscriptions;
- payment authorizations;
- local recommendation preferences.

Networks maintain:

- manifest;
- membership state;
- catalog index;
- treasury events;
- moderation decisions;
- discovery policy.

### 12.3 Public versus private data

Public:

- network manifest;
- public catalog;
- public releases;
- public contributor claims;
- public license terms;
- public governance decisions;
- public disputes and corrections where appropriate.

Private:

- detailed listening history;
- private library;
- payment credentials;
- unreleased works;
- private annotations;
- private moderation evidence;
- private recommendation state.

### 12.4 Settlement bridge

High-frequency listening and support records may remain in Holochain.

External chain settlement should occur for:

- deposits;
- cashouts;
- treasury movements;
- high-value licenses;
- bridge reconciliation.

---

## 13. Governance and anti-capture safeguards

Federation should not recreate platform monopoly under cooperative branding.

### 13.1 Minimum protocol guarantees

Every conforming network must support:

- artist portability;
- listener portability;
- manifest transparency;
- non-retroactive economic terms;
- contributor provenance;
- dispute and correction records;
- catalog grant revocation;
- data export;
- clear moderation appeals;
- conformance testing.

### 13.2 Governance autonomy

Networks may choose:

- one-member-one-vote;
- elected council;
- artist cooperative;
- multi-stakeholder governance;
- foundation;
- stewarded commons;
- label governance;
- constitutional founder model.

The governance model must be visible.

### 13.3 Limits on MYCEL weighting

MYCEL may contribute to:

- eligibility;
- trust context;
- moderation selection;
- governance weighting.

It must not create an unchallengeable permanent aristocracy.

Recommended protections:

- maximum weight caps;
- recency or activity considerations;
- role-specific reputation;
- appeal;
- transparent computation;
- non-purchaseability.

### 13.4 Paid promotion

Paid promotion must be labeled.

Networks must distinguish:

- editorial selection;
- member voting;
- paid placement;
- algorithmic recommendation;
- artist self-promotion.

### 13.5 Moderation

Moderation policy should define:

- content boundaries;
- spam policy;
- duplicate policy;
- rights disputes;
- harassment policy;
- appeal process;
- emergency actions;
- restoration;
- transparency reports.

---

## 14. Discovery

### 14.1 Local discovery

Each network may define its own:

- homepage;
- charts;
- editorial collections;
- recommendation logic;
- new-release policy;
- community spotlights;
- event programming.

### 14.2 Global discovery

The Mycelix client may aggregate across networks using user-selected policies.

Examples:

- From networks I joined
- Nearby communities
- Artist-owned releases
- Complete provenance
- Local to my region
- Pay-what-you-want
- Available for adaptive licensing
- New cooperative releases

### 14.3 No forced universal ranking

The protocol should not define one mandatory global popularity score.

### 14.4 Discovery explanations

A recommendation should identify its source:

- chosen by a network curator;
- followed artist;
- related provenance;
- listener-selected preference;
- nearby network;
- community-funded release;
- paid placement.

---

## 15. Listener experience

### 15.1 One identity

The listener uses one Mycelix identity across networks.

### 15.2 One support balance

SAP should remain usable across participating networks unless a network explicitly chooses a separate model.

### 15.3 Portable library

The listener's private library may contain:

- artists;
- works;
- releases;
- network collections;
- external references;
- saved journeys;
- listening annotations.

A network cannot hold the library hostage.

### 15.4 Network switcher

The main client should include:

- All Mycelix
- My Networks
- Explore Networks
- Current Network
- Network settings

### 15.5 Subscription center

Shows:

- active networks;
- monthly SAP cost;
- next renewal;
- artist allocation;
- cancellation;
- portability;
- bundle eligibility;
- network term changes.

### 15.6 Support receipt

After a listening period, show meaningful impact:

> This month, 8.4 SAP went to 14 artists.  
> 1.0 SAP supported the Cape Jazz recording fund.  
> 0.6 SAP operated the network.

---

## 16. Artist experience

### 16.1 Network membership dashboard

Shows:

- networks joined;
- catalog grants;
- exclusivity;
- pricing;
- earnings by network;
- listener reach;
- pending votes;
- grant expiration;
- disputes;
- exit options.

### 16.2 Join network flow

1. Review network culture.
2. Review economics.
3. Review rights grant.
4. Review moderation and discovery policy.
5. Select catalog.
6. Invite contributor approval where required.
7. Sign membership and grants.
8. Publish availability.

### 16.3 Leave network flow

1. Select membership or catalog grant.
2. Review pending obligations.
3. Choose withdrawal timing.
4. Notify collaborators.
5. Sign revocation or expiration event.
6. Preserve settlement history.
7. Remove future network availability.

### 16.4 Start a network flow

1. Name and purpose.
2. Founding members.
3. Charter.
4. Membership model.
5. Economic model.
6. Listener plans.
7. Artist payout policy.
8. Treasury policy.
9. Catalog policy.
10. Moderation.
11. Discovery.
12. Infrastructure provider.
13. Federation options.
14. Review and sign manifest.
15. Launch privately or publicly.

---

## 17. UX information architecture

### 17.1 Mycelix global client

Primary navigation:

- Home
- Discover
- Networks
- Library
- Artists
- Communities
- Create & Publish
- Support

### 17.2 Network page

Tabs:

- Home
- Music
- Artists
- Collections
- Community
- Support
- About
- Governance

### 17.3 Artist network console

Tabs:

- Overview
- Catalog
- Members
- Listeners
- Economics
- Treasury
- Discovery
- Moderation
- Governance
- Federation
- Infrastructure
- Settings

### 17.4 Network directory

Filters:

- genre;
- region;
- language;
- governance;
- listener price;
- artist payout;
- economic model;
- provenance completeness;
- open membership;
- adaptive licensing;
- cooperative ownership.

### 17.5 Federation explorer

Shows:

- connected networks;
- shared agreements;
- catalog overlap;
- bundles;
- settlement relationships;
- shared infrastructure;
- cross-network collections;
- disputes or incompatibilities.

---

## 18. Visual design

### 18.1 Shared shell

The Mycelix protocol shell should remain recognizable through:

- navigation structure;
- provenance mark;
- accessibility;
- payment language;
- network identity indicator;
- trust and manifest actions.

### 18.2 Network theming

Networks may customize:

- logo;
- type pairing;
- accent palette;
- imagery;
- layout modules;
- editorial voice;
- homepage collections.

They may not alter:

- provenance disclosures;
- price and fee clarity;
- subscription cancellation;
- rights grant language;
- dispute status;
- paid-promotion labels;
- accessibility minimums.

### 18.3 Color semantics

Recommended shared semantics:

- teal: network and provenance;
- green: active and verified;
- copper/gold: artist support and value;
- violet: federation and governance;
- warm white: music and primary content;
- red: dispute, broken rights, or real error only.

### 18.4 SAP and MYCEL display

SAP belongs in:

- Support;
- pricing;
- settlement;
- treasury;
- wallet details.

MYCEL belongs in:

- Community standing;
- eligibility;
- stewardship history;
- trust context.

Never present them as equivalent balances.

### 18.5 Network identity

The current network should be visible but not overpower the music.

Example header:

> Cape Jazz Commons  
> A member-owned Mycelix Music Network

---

## 19. Key page designs

## 19.1 Explore Networks

Primary content:

- featured networks;
- local networks;
- networks followed by artists you support;
- transparent economics summary;
- governance summary;
- member and catalog scale;
- join action.

Each card should answer:

- What music lives here?
- Who owns it?
- What does it cost?
- Where does payment go?
- Can artists leave?
- What makes this network distinct?

## 19.2 Network Home

Primary content:

- editorial feature;
- new releases;
- artist spotlights;
- community-funded projects;
- upcoming events;
- listener support impact;
- transparent network economics.

Avoid making governance the first screen.

## 19.3 Network Economics

Show:

- listener plans;
- artist payout formula;
- operating fee;
- treasury allocation;
- current reserves;
- pending artist obligations;
- settlement schedule;
- historical changes;
- manifest signature.

## 19.4 Federation Map

Show:

- network nodes;
- active federation links;
- shared bundles;
- shared infrastructure;
- catalog exchange;
- regional or cultural relationships.

The map must remain explanatory rather than decorative.

## 19.5 Start a Music Network

Use a guided, saveable wizard.

The user should be able to create a draft manifest without immediately launching infrastructure.

## 19.6 Artist Portability

A dedicated screen should show:

- export identity;
- export catalog grants;
- revoke network access;
- transfer preferred network;
- notify supporters;
- preserve external links;
- inspect dependencies.

---

## 20. Frontend component map

The current web application may implement these components in Next.js while the protocol contracts remain frontend-agnostic.

```text
MycelixMusicApp
├── GlobalShell
│   ├── NetworkSwitcher
│   ├── GlobalSearch
│   ├── SupportDrawer
│   └── PersistentPlayer
├── NetworkDirectoryPage
│   ├── NetworkFilters
│   ├── FeaturedNetworkCard
│   └── NetworkCard
├── NetworkPage
│   ├── NetworkHero
│   ├── ManifestSummary
│   ├── NetworkNavigation
│   ├── EditorialCollections
│   ├── ArtistGrid
│   └── SupportImpact
├── SubscriptionCenter
├── ArtistNetworkDashboard
├── NetworkAdminConsole
├── FederationExplorer
├── StartNetworkWizard
├── JoinNetworkFlow
├── CatalogGrantFlow
└── PortabilityCenter
```

---

## 21. Protocol APIs

```rust
pub trait MusicNetworkRegistry {
    fn create_network(&self, draft: NetworkDraft) -> Result<MusicNetwork>;
    fn publish_manifest(&self, network: NetworkId, manifest: NetworkManifest) -> Result<String>;
    fn join_network(&self, membership: NetworkMembership) -> Result<String>;
    fn leave_network(&self, network: NetworkId, agent: AgentId) -> Result<String>;
    fn grant_catalog(&self, grant: NetworkCatalogGrant) -> Result<String>;
    fn revoke_catalog(&self, grant_id: String) -> Result<String>;
}
```

```rust
pub trait FederationRegistry {
    fn propose_federation(&self, proposal: FederationProposal) -> Result<String>;
    fn accept_federation(&self, proposal_id: String, network: NetworkId) -> Result<String>;
    fn publish_bundle(&self, bundle: FederationBundle) -> Result<String>;
    fn settle_bundle_period(&self, period: SettlementPeriod) -> Result<SettlementReceipt>;
}
```

```rust
pub trait PortableLibrary {
    fn export_library(&self, listener: AgentId) -> Result<EncryptedLibraryBundle>;
    fn import_library(&self, bundle: EncryptedLibraryBundle) -> Result<ImportReport>;
    fn set_preferred_provider(&self, release: ReleaseId, network: NetworkId) -> Result<()>;
}
```

---

## 22. Conformance

A network may be:

- experimental;
- compatible;
- conforming;
- verified conforming;
- suspended;
- incompatible.

Conformance tests should cover:

- identity portability;
- catalog portability;
- manifest validity;
- economic transparency;
- settlement correctness;
- rights grant handling;
- cancellation;
- dispute records;
- accessibility;
- protocol compatibility.

A network should not imply full Mycelix conformance without passing the corresponding test suite.

---

## 23. Failure and dispute states

### Network insolvent

- freeze new obligations;
- preserve listener access where funded;
- show liabilities;
- initiate settlement or recovery process;
- notify artists and listeners;
- prevent silent disappearance.

### Network offline

- use alternate providers where licenses allow;
- show availability state;
- retain library and canonical release references.

### Manifest conflict

- identify incompatible terms;
- require new consent;
- preserve prior rights.

### Catalog dispute

- show claim status;
- preserve evidence;
- temporarily constrain distribution only according to policy;
- provide appeal.

### Federation incompatibility

- identify which protocol capability failed;
- avoid silently degrading portability.

---

## 24. Implementation phases

## P0 — Network foundations

- network registry;
- signed manifest;
- artist membership;
- catalog grants;
- network-branded page;
- simple network subscription;
- SAP settlement;
- artist payout report;
- join and leave flows;
- portability export;
- one pilot artist cooperative.

## P1 — Network operations

- treasury;
- moderation;
- discovery configuration;
- listener subscription center;
- user-centric payout;
- network directory;
- public conformance status;
- MYCEL stewardship context;
- governance proposals;
- infrastructure provider selection.

## P2 — Federation

- network-to-network agreements;
- cross-network catalog carriage;
- federation directory;
- shared bundles;
- clearing and settlement;
- preferred provider;
- federation map;
- shared discovery collections;
- alternate streaming providers.

## P3 — Open ecosystem

- third-party network clients;
- provider marketplace;
- portable recommendation agents;
- regional federation alliances;
- institutional licensing;
- adaptive-media networks;
- external conformance certification;
- federation-of-federations.

---

## 25. Acceptance criteria

The feature is ready for alpha when:

1. A group can create a private Music Network and publish a signed manifest.
2. An artist can join without transferring ownership of identity or works.
3. A release can be licensed to a network through an explicit catalog grant.
4. A listener can subscribe in SAP and see the payout formula before confirming.
5. Artists can see earnings and settlement attributable to the network.
6. An artist can leave and revoke future catalog distribution without erasing provenance.
7. A listener can export their library and follows.
8. Network terms cannot change retroactively.
9. SAP and MYCEL are presented as distinct concepts.
10. The network may brand its service without hiding price, provenance, rights, or cancellation.
11. Public network status states whether conformance has been tested.
12. The canonical work remains independent from network-specific listings.
13. A dispute preserves evidence and supports appeal.
14. One network outage does not destroy artist identity or listener library.
15. Governance is visible but does not dominate the ordinary listening experience.

---

## 26. Open questions

- Should every network be required to accept SAP?
- Which settlement policies qualify for verified conformance?
- Should listeners select payout policy preferences?
- How are cross-network bundle fees bounded?
- Which rights grants require all contributors to co-sign?
- How should network insolvency be resolved?
- Should a network be allowed to require exclusivity at all?
- How should MYCEL influence stewardship without entrenching incumbents?
- Which moderation minimums are protocol-wide?
- How should private invitation-only networks appear in global discovery?
- What legal structure templates should be offered to cooperatives?
- How much custom theming can be allowed before trust disclosures become inconsistent?
- Should networks be able to issue local non-transferable recognition in addition to MYCEL?
- How should listener privacy work in user-centric settlement?
- Which parts of a recommendation policy must be inspectable?

---

## 27. Final design intent

Mycelix Music should make it possible for artists to create their own platforms without recreating isolated silos.

The platform should evolve from:

> One fairer streaming service.

to:

> A shared music protocol supporting many artist-owned services, each with its own culture and economics, while identity, provenance, rights, libraries, and settlement remain portable.

The defining rule is:

> Build many platforms. Preserve one interoperable musical commons.
