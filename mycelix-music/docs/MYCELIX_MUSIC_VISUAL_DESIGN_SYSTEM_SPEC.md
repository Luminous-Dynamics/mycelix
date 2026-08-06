# Mycelix Music Visual Design System & Product Experience Specification

**Status:** Proposed  
**Audience:** Mycelix Music product, frontend, protocol, artist tools, network/federation, accessibility, and brand contributors  
**Primary implementation:** Web application, initially Next.js/React; portable to future clients  
**Suggested repository path:** `docs/design/MYCELIX_MUSIC_VISUAL_DESIGN_SYSTEM_SPEC.md`  
**Visual reference:** `MYCELIX_MUSIC_VISUAL_REFERENCE.png`

**Related documents:**

- `MYCELIX_MUSIC_FEDERATED_ARTIST_NETWORKS_DESIGN_SPEC.md`
- `MUSE_ARTIST_CAPABILITY_AND_PUBLISHING_INTEGRITY_DESIGN_SPEC.md`
- `INDUSTRY_ROADMAP.md`
- `MYCELIX_PROTOCOL_INTEGRATION.md`
- `HOLOCHAIN_ECONOMICS_STRATEGY.md`
- `ECONOMIC_MODELS_IMPROVED.md`

---

## 1. Purpose

Mycelix Music should look and feel like a living, creator-owned musical commons.

It should not look primarily like:

- a cryptocurrency dashboard;
- a generic streaming clone;
- a DAO administration console;
- a Web3 marketplace;
- a wall of platform statistics;
- a system that places economics before music.

The experience should make the underlying advantages immediately felt:

- artists choose how support works;
- listeners can see where their value goes;
- works carry provenance and contributor history;
- communities can operate their own networks;
- identity, libraries, and releases remain portable;
- the protocol supports many platforms without fragmenting the musical commons.

The visual design should therefore prioritize this order:

1. **Music**
2. **Artist and listener relationship**
3. **Context and community**
4. **Provenance and rights**
5. **Economics**
6. **Protocol and governance**

The user should encounter the music first and discover the infrastructure as evidence of trust.

---

## 2. North-star experience

A first-time listener should understand Mycelix Music without knowing what Holochain, DKG, SAP, MYCEL, or federation mean.

Within a few moments they should be able to answer:

- What can I listen to?
- Who made this?
- Why is it here?
- How can I support the artist?
- What does this cost?
- Where does my support go?
- Is this release genuinely connected to its contributors?
- Which community or network is presenting it?

The experience should feel:

- intimate rather than transactional;
- alive rather than corporate;
- transparent rather than technical;
- plural rather than uniform;
- trustworthy rather than surveillant;
- musical rather than financial.

---

## 3. Design principles

### 3.1 Music first

The primary content on every listener-facing page is music, artists, releases, collections, or journeys.

Wallets, governance, balances, and protocol status must not dominate the home screen.

### 3.2 Relationship before transaction

Prefer:

> Support this artist

over:

> Execute payment

Prefer:

> 1 SAP goes directly to the artist and collaborators

over:

> Router settlement amount: 1 SAP

### 3.3 Transparency without cognitive overload

Detailed economic and provenance data should be available, but progressive.

Default views should present concise human summaries.

Expanded views may expose:

- contributor splits;
- signed claims;
- manifest versions;
- settlement receipts;
- license terms;
- network governance;
- DKG evidence.

### 3.4 Multiple economies, one visual grammar

Pay per stream, freemium, pay what you want, patronage, purchase, and licensing should each be understandable without creating a different app for each model.

### 3.5 Networks may express culture, not hide trust

Artist networks may customize:

- logo;
- accent colors;
- imagery;
- type pairing;
- editorial tone;
- homepage modules.

They may not obscure:

- price;
- artist share;
- provenance state;
- paid promotion;
- rights status;
- cancellation;
- dispute state;
- portability.

### 3.6 Organic, not ornamental

Mycelial forms, branching lines, and bioluminescent imagery should represent real relationships:

- provenance;
- artist networks;
- contributor graphs;
- catalog lineage;
- federation links;
- listener support paths.

Avoid decorative mushrooms and network lines that communicate nothing.

### 3.7 Calm confidence

Mycelix should not overstate itself.

Avoid constant claims such as:

- revolutionary;
- unstoppable;
- 50x better;
- owned by the people;
- verified truth.

Use evidence and precise language.

---

## 4. Brand relationship with Muse

Muse and Mycelix Music should feel related but distinct.

### Muse

- intimate;
- warm;
- compositional;
- precise;
- reflective;
- copper and violet;
- score and instrument metaphors.

### Mycelix Music

- social;
- organic;
- expansive;
- plural;
- connected;
- teal, green, copper, and restrained violet;
- network, archive, community, and living-catalog metaphors.

Shared family traits:

- dark foundations;
- warm typography;
- restrained glow;
- precise provenance language;
- persistent playback;
- careful motion;
- non-extractive product voice.

A Muse work moving into Mycelix should feel like moving from the studio into the commons.

---

## 5. Information architecture

## 5.1 Listener navigation

Recommended primary navigation:

- Home
- Discover
- Networks
- Library
- Journeys
- Artists
- Communities

Secondary navigation:

- Support
- Notifications
- Settings

Avoid a top-level “DAO Governance” item for ordinary listeners.

Governance lives inside a network or community.

## 5.2 Creator navigation

Use a separate **Create & Publish** workspace.

Recommended navigation:

- My Works
- Release Builder
- Contributors
- Rights & Provenance
- Economics
- Networks
- Earnings
- Licenses
- Distribution
- Settings

## 5.3 Network operator navigation

Recommended console:

- Overview
- Catalog
- Artists
- Listeners
- Economics
- Treasury
- Discovery
- Community
- Moderation
- Governance
- Federation
- Infrastructure
- Settings

## 5.4 Persistent player

The player remains available across listener-facing pages.

It should not obscure forms, dialogs, or creator workflows.

On narrow screens it becomes a compact bottom bar.

---

## 6. Global shell

## 6.1 Left navigation rail

Desktop behavior:

- 224–248 px wide;
- icon and label;
- collapsible to icon rail;
- network switcher above primary navigation;
- creator workspace link separated from listener navigation.

Recommended structure:

- current network identity;
- listener navigation;
- creator tools;
- account and support.

Do not place spendable balances permanently at the bottom of the rail.

Use a compact support indicator:

> 124 SAP available

Clicking opens the Support drawer.

## 6.2 Header

Contains:

- global search;
- network context;
- notifications;
- messages;
- support drawer;
- account.

Search placeholder:

> Search music, artists, networks, or collections

Search results should visually distinguish:

- artist;
- release;
- work;
- network;
- collection;
- external reference.

## 6.3 Right contextual rail

The current mock uses the right rail as a wallet panel. Replace that with a contextual rail.

Possible states:

### Home

- now playing;
- why recommended;
- support impact;
- followed-artist activity.

### Release

- support options;
- provenance summary;
- contributor split;
- license actions.

### Artist

- support;
- patronage;
- upcoming releases;
- network memberships.

### Network

- price;
- payout policy;
- ownership;
- join or subscribe.

### Creator workspace

- release readiness;
- unresolved contributor claims;
- earnings or settlement state.

The wallet is a drawer, not the page’s dominant permanent panel.

---

## 7. Home screen redesign

The current hero is visually strong but too platform-centered.

Replace the permanent slogan hero with a living editorial feature.

## 7.1 Featured release hero

Contents:

- artwork or artist-owned visual;
- release title;
- artist;
- one-line editorial context;
- network;
- economic model;
- provenance summary;
- primary action;
- support action.

Example:

> **Copper Meridian**  
> Tristan Stoltz  
> A patient folk sonata built around return and transformed memory.  
> Published through the Independent Composer Commons.

Actions:

- Play
- Support
- View lineage

Secondary metadata:

- Free · tips welcome
- Full contributor provenance
- Muse score available

## 7.2 Home sections

Recommended modules:

- Continue listening
- From your networks
- New from artists you support
- Community-funded releases
- Complete provenance
- Pay what you want
- Local and regional scenes
- Journeys chosen by curators
- Works with editable scores
- Available for game and adaptive-media licensing

The platform statistics row should be removed from the primary home experience.

A small transparent-network footer may show:

- artists paid this month;
- median network fee;
- active networks;
- public settlement status.

---

## 8. Discover

Discovery should support multiple paths without one universal popularity ranking.

Recommended tabs:

- For You
- Editorial
- Networks
- New
- Local
- Open Licensing
- Complete Provenance
- Pay What You Want

Filters:

- genre;
- region;
- language;
- mood;
- network;
- economic model;
- release type;
- provenance;
- license;
- duration;
- instrumental/vocal;
- adaptive-media availability.

Every recommendation should expose its origin:

- Chosen by Cape Jazz Commons
- New from an artist you support
- Related to a kept release
- Community-funded
- Paid placement
- New in your region

Paid placement must be visually explicit.

---

## 9. Music cards

Music cards should foreground music and identity, not price.

## 9.1 Required content

- artwork;
- release title;
- artist;
- play control;
- concise economic label;
- network or context;
- keep action;
- provenance mark where useful.

## 9.2 Economic labels

Use user-facing language:

- Free
- Free · tips welcome
- 3 free listens
- 0.01 SAP per listen
- Included for patrons
- Purchase to keep
- License available

Avoid badges such as:

- Strategy 0x03
- Gift Economy v2
- Premium Pool Settlement
- Smart Contract Active

## 9.3 Card states

- default;
- hover;
- playing;
- kept;
- unavailable;
- external reference;
- disputed;
- replaced by newer version;
- network-exclusive;
- patron-only.

## 9.4 Artwork

Prefer:

1. artist-supplied artwork;
2. release-derived visual identity;
3. tasteful platform placeholder.

Do not default every release to AI fantasy art.

---

## 10. Release page

The release page should be the strongest expression of the product.

## 10.1 Hero

Contains:

- artwork;
- title;
- artist;
- release type;
- network;
- playback;
- support;
- keep;
- share;
- release context.

## 10.2 Support card

Shows:

- current access model;
- exact cost;
- where payment goes;
- optional tip or patronage;
- contributor split summary.

Example:

> **Free · tips welcome**  
> 96% goes to the artist and collaborators.  
> 2% supports the network.  
> 2% supports shared infrastructure.

Action:

> Support with SAP

## 10.3 Provenance summary

Use a compact **Lineage Mark**.

Summary:

- provenance complete;
- contributors signed;
- composition and master linked;
- license available;
- disputes: none.

Expanded lineage:

- Work
- Arrangement
- Performance
- Recording
- Master
- Release
- Distribution listings

## 10.4 Credits

Display roles clearly:

- composer;
- lyricist;
- performer;
- arranger;
- producer;
- engineer;
- artwork;
- publisher;
- network.

## 10.5 Tabs

- About
- Credits
- Lineage
- Supporters
- Versions
- License
- Discussion

Do not make comments the primary secondary tab.

---

## 11. Artist page

The artist page should communicate body of work, personhood, and support.

## 11.1 Header

Contains:

- artist image or mark;
- name;
- concise statement;
- verified identity state;
- networks;
- support;
- follow;
- contact or commission where enabled.

## 11.2 Main modules

- Featured work
- Releases
- Works and compositions
- Journeys
- Support options
- Upcoming releases
- Networks and collectives
- Credits on other works
- Licensing availability
- Public provenance activity

## 11.3 Economic models

Do not show a donut chart of “economic models” as a major artist identity element.

Instead show practical options:

- Listen free
- Become a patron
- Support a recording fund
- Purchase scores
- Commission work
- License music
- Offer TEND exchange

---

## 12. Networks and federations

## 12.1 Network directory

Each network card answers:

- What music lives here?
- Who governs it?
- What does it cost?
- Where does payment go?
- Is membership open?
- Can artists leave?
- Is the network conforming?

Recommended card content:

- network mark;
- region or musical focus;
- ownership model;
- listener price;
- artist payout;
- member count;
- current feature;
- join or explore.

## 12.2 Network page

Tabs:

- Home
- Music
- Artists
- Collections
- Community
- Support
- About
- Governance

Default page should feel like a cultural publication, not an admin page.

## 12.3 Federation explorer

Use real relationships.

Node types:

- network;
- artist collective;
- infrastructure provider;
- shared bundle;
- federation agreement.

Edges:

- federates with;
- shares catalog;
- shares settlement;
- shares infrastructure;
- participates in bundle.

The map should provide list and table alternatives.

---

## 13. Support drawer

Replace the permanent wallet card with a support drawer.

## 13.1 Primary view

Heading:

> Your Support

Contents:

- available SAP;
- artists supported this month;
- active patronages;
- network subscriptions;
- next renewals;
- recent support.

Actions:

- Add SAP
- Send support
- Manage subscriptions
- View receipts
- Cash out, for eligible creators

## 13.2 Value categories

Separate clearly:

### Spendable value

- SAP

### Reciprocity

- TEND

### Appreciation

- CGC

### Community standing

- MYCEL

MYCEL must not look spendable.

Display:

> **742 MYCEL**  
> Community standing  
> Cannot be bought, sold, or transferred

## 13.3 Impact receipt

Monthly summary:

> 8.42 SAP reached 14 artists.  
> 1.00 SAP supported shared infrastructure.  
> 0.58 SAP funded community projects.

---

## 14. SAP visual and language rules

SAP is a settlement and support unit.

### User-facing labels

Prefer:

- Support balance
- Add SAP
- 0.01 SAP per listen
- Support with 2 SAP
- Network subscription: 9 SAP/month

Avoid unexplained:

- SAP Token
- Utility asset
- ERC-20 balance
- Bridge liquidity

### Visual treatment

- copper/green accent;
- droplet or seed icon only if officially adopted;
- never the dominant brand color;
- use tabular numerals;
- show fiat estimate only as secondary and with appropriate caveats.

---

## 15. MYCEL visual and language rules

MYCEL represents non-transferable community standing.

### User-facing labels

- Community standing
- Stewardship eligibility
- Contribution history
- Trust context

### Visual treatment

- mycelial node or branching mark;
- teal or pale green;
- never a wallet token icon;
- never paired with Send, Buy, Sell, Deposit, or Cash Out.

### Explanation

> MYCEL reflects trusted participation and contribution. It cannot be purchased or transferred.

Avoid turning MYCEL into a universal social score.

Where possible, show domain context:

- Publishing MYCEL
- Stewardship MYCEL
- Moderation MYCEL
- Infrastructure MYCEL

---

## 16. Provenance visual system

## 16.1 Lineage Mark

A small mark appears on releases with provenance data.

States:

- Complete
- Partial
- Creator asserted
- Disputed
- Corrected
- External reference
- Unknown

The icon must not imply legal ownership certainty.

## 16.2 Provenance language

Prefer:

- Contributors signed
- Recording linked to composition
- Release lineage complete
- Rights claim disputed
- Evidence attached

Avoid:

- Ownership proven
- Copyright guaranteed
- Truth verified
- Plagiarism confirmed, unless a formal adjudication has occurred

## 16.3 Expanded lineage

Use a horizontal or branching chain.

Each node includes:

- object type;
- title;
- contributor;
- date;
- signature state;
- evidence;
- version.

---

## 17. Typography

Use a restrained two-family system.

### Display and editorial

A warm serif for:

- release titles;
- artist names where appropriate;
- editorial headings;
- community statements;
- selected quotes.

### UI and body

A highly legible sans-serif for:

- navigation;
- metadata;
- controls;
- prices;
- forms;
- tables;
- accessibility.

### Monospace

Use sparingly for:

- hashes;
- identifiers;
- protocol versions;
- receipts;
- advanced provenance.

Recommended scale:

- Display: 48–64 px
- H1: 36–44 px
- H2: 28–32 px
- H3: 20–24 px
- Body large: 18 px
- Body: 16 px
- Metadata: 13–14 px
- Caption: 12 px minimum

Do not use 10–11 px text for meaningful metadata.

---

## 18. Color system

The current mock uses attractive color but too much neon competition.

Recommended palette roles:

### Foundation

- `--bg-root`: near-black blue
- `--bg-surface`: slightly raised navy
- `--bg-elevated`: elevated cool-charcoal
- `--border-subtle`: low-contrast blue-gray

### Brand

- `--mycelix-teal`: connection and provenance
- `--living-green`: active, available, verified
- `--support-copper`: artist support and economic action
- `--federation-violet`: network/federation and governance
- `--music-warm-white`: primary text and music identity

### Semantic

- success: green
- warning: amber
- dispute/error: red
- informational: blue
- proposal/pending: violet

Suggested CSS tokens:

```css
:root {
  --bg-root: #07101a;
  --bg-surface: #0b1521;
  --bg-elevated: #101c2a;
  --bg-soft: #142233;

  --text-primary: #f4f1ea;
  --text-secondary: #aeb9c6;
  --text-muted: #778596;

  --border-subtle: rgba(157, 181, 205, 0.16);
  --border-strong: rgba(157, 181, 205, 0.30);

  --mycelix-teal: #34d5b2;
  --living-green: #77d87b;
  --support-copper: #d9a05f;
  --federation-violet: #8d6cf2;
  --signal-blue: #5aa7e8;
  --danger: #e26d78;
}
```

Final values should be contrast-tested.

---

## 19. Surfaces and elevation

Avoid heavy glassmorphism.

Use four surface levels:

1. Root background
2. Standard panel
3. Elevated interactive panel
4. Modal or drawer

Surface distinction should come from:

- luminance;
- border;
- soft inner highlight;
- very restrained shadow.

Avoid:

- excessive blur;
- bright outer glows;
- every card appearing to float;
- thick neon borders.

---

## 20. Buttons and actions

### Primary

Use for one dominant action per region.

Examples:

- Play
- Support
- Publish
- Join Network

### Secondary

Examples:

- Follow
- Keep
- View lineage
- Compare terms

### Tertiary

Text or icon actions.

Examples:

- More
- Share
- View receipt

### Destructive

Use only for:

- revoke grant;
- leave network;
- archive;
- withdraw;
- delete local draft.

Do not use red for ordinary cancel actions.

---

## 21. Motion

Motion should communicate:

- playback;
- support transfer;
- network connection;
- provenance expansion;
- state transition.

Good motion:

- artwork-to-player continuity;
- gentle waveform movement;
- support receipt flowing toward contributors;
- lineage nodes unfolding;
- network transitions preserving context;
- cards settling into the library.

Avoid:

- constant particle fields;
- aggressive equalizers;
- looping decorative mycelial movement;
- glowing every interactive surface;
- motion that suggests payments before confirmation.

Respect reduced-motion preferences.

---

## 22. Player design

The persistent player should communicate:

- title;
- artist;
- artwork;
- current economic model;
- playback;
- support;
- progress;
- queue;
- output;
- provenance shortcut.

Do not put the full price on every moment of playback.

Examples:

- Free
- Patron access
- 0.01 SAP/play

Expanded player may show:

- support impact;
- queue;
- lyrics;
- credits;
- lineage;
- related journey.

---

## 23. Creator and publishing visual system

Creator tools should feel related to Muse Studio but remain release-oriented.

### Core status ribbon

- Sketch
- Work in Progress
- Release Candidate
- Published
- Corrected
- Withdrawn

### Release readiness

Show:

- master ready;
- contributors approved;
- rights reviewed;
- provenance complete;
- economics configured;
- publication review complete.

Avoid a single opaque readiness score.

### Contribution record

Use a timeline:

- generated;
- selected;
- edited;
- arranged;
- performed;
- mixed;
- mastered;
- approved;
- published.

This supports artist capability without reducing authorship to a percentage.

---

## 24. Responsive design

## Desktop

- left navigation;
- center content;
- contextual right rail where useful;
- persistent player;
- wide editorial layouts.

## Tablet

- collapsible left rail;
- contextual rail becomes drawer;
- player remains fixed;
- two-column release and artist layouts.

## Mobile

- bottom navigation;
- support drawer as bottom sheet;
- contextual information in tabs;
- artwork and playback prioritized;
- one primary action at a time;
- prices and support terms remain visible before confirmation.

Do not compress desktop dashboards into tiny mobile cards.

---

## 25. Accessibility

Requirements:

- WCAG AA contrast minimum;
- visible keyboard focus;
- complete keyboard navigation;
- semantic headings;
- text alternatives for artwork and graphs;
- reduced motion;
- chart summaries;
- no information communicated only by color;
- clear price and payment confirmation;
- screen-reader labels for provenance states;
- minimum 44 px interactive targets on touch devices;
- no autoplay with sound.

Economic confirmations must state:

- amount;
- currency;
- recipient;
- recurring status;
- cancellation policy;
- network fee;
- artist share where available.

---

## 26. Copy voice

The product voice should be:

- warm;
- precise;
- non-coercive;
- artist-respecting;
- evidence-oriented;
- plainspoken.

Prefer:

> Support this release

over:

> Fuel the revolution

Prefer:

> 96% goes to the artist and collaborators

over:

> Maximum creator sovereignty

Prefer:

> Provenance complete

over:

> Cryptographically proven ownership

Prefer:

> This recommendation comes from Cape Jazz Commons

over:

> The algorithm chose this for you

---

## 27. Key component inventory

### Global

- GlobalShell
- NetworkSwitcher
- GlobalSearch
- NotificationCenter
- SupportDrawer
- PersistentPlayer

### Music

- ReleaseCard
- TrackRow
- CollectionCard
- JourneyCard
- ArtistCard
- NetworkCard
- PlayButton
- KeepButton
- SupportButton
- EconomicLabel
- LineageMark

### Release

- ReleaseHero
- SupportSummary
- ContributorList
- LineageGraph
- LicensePanel
- VersionList
- DistributionLinks

### Artist

- ArtistHero
- SupportOptions
- NetworkMemberships
- CreditsGraph
- CommissionPanel
- ArtistCatalog

### Networks

- NetworkHero
- ManifestSummary
- EconomicsSummary
- GovernanceSummary
- FederationBadge
- ConformanceStatus
- JoinNetworkFlow
- NetworkSubscriptionCard

### Creator

- WorkStateRibbon
- ReleaseReadiness
- ContributionTimeline
- RightsChecklist
- EconomicsConfigurator
- ProvenanceReview
- DistributionPackage

---

## 28. Refined page set

The first coherent visual-design implementation should include:

1. Listener Home
2. Discover
3. Release Detail
4. Artist Profile
5. Library
6. Support Drawer
7. Network Directory
8. Network Home
9. Subscription Center
10. Create & Publish Dashboard
11. Release Builder
12. Rights & Provenance
13. Economics Configuration
14. Federation Explorer
15. Mobile Player and Release Detail

---

## 29. P0 implementation priorities

### P0.1 Design foundation

- tokenized color system;
- typography;
- spacing;
- surfaces;
- controls;
- responsive grid;
- accessibility baseline.

### P0.2 Listener core

- global shell;
- home;
- discover;
- release page;
- artist page;
- persistent player;
- library.

### P0.3 Support clarity

- support drawer;
- SAP payment confirmation;
- MYCEL non-transferable treatment;
- economic labels;
- support receipts.

### P0.4 Provenance

- Lineage Mark;
- provenance summary;
- expanded contributor and release chain;
- dispute and correction states.

### P0.5 Networks

- network switcher;
- network directory;
- network home;
- subscription summary;
- manifest disclosure.

---

## 30. Acceptance criteria

The visual system is ready for alpha when:

1. The home screen presents music before wallet or protocol data.
2. SAP is understandable as support value without protocol knowledge.
3. MYCEL is never presented as spendable currency.
4. A release clearly communicates cost before playback or purchase.
5. A listener can see where support goes.
6. Provenance state is visible without overstating legal certainty.
7. A network may express its identity without hiding shared trust disclosures.
8. Paid promotion is distinguishable from editorial selection.
9. Listener and creator navigation are clearly separated.
10. The persistent player remains usable across primary pages.
11. The full experience works with keyboard and reduced motion.
12. Mobile pages prioritize music and core actions over dashboard density.
13. The interface does not require the user to understand DAO, DKG, or Holochain vocabulary.
14. Public claims use evidence-oriented language rather than hype.
15. The product feels recognizably related to Muse while remaining socially and visually distinct.

---

## 31. Open questions

- What is the official long-form meaning of SAP?
- Should network-specific accents be allowed inside economic actions?
- Which parts of a network manifest must remain permanently visible?
- How should user-centric payout impact be summarized without exposing listening history?
- Should MYCEL be global, domain-specific, or both?
- How should external releases and native releases differ visually?
- Should provenance-complete releases receive discovery preference?
- Which visual elements should be shared with other Mycelix sector applications?
- What is the fallback visual identity for releases without artist artwork?
- How should network insolvency and settlement delays appear to listeners?

---

## 32. Final design intent

Mycelix Music should feel like a beautiful music service whose fairness is structurally visible.

The user should experience:

- music before machinery;
- relationships before balances;
- support before speculation;
- evidence before claims;
- communities before governance;
- portability before lock-in.

The defining visual rule is:

> Make the commons visible through the music, not through a crypto dashboard.
