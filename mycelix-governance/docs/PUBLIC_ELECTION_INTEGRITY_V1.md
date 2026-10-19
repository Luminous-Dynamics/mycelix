# Mycelix Public Election Integrity Foundation v0.1

Status: **design + pure-kernel foundation; not a production election protocol**

Profile identifier: `mycelix-public-election-v1`

## Purpose

Mycelix already supports rich organizational governance: weighted voting, quadratic voting, delegation, trust/consciousness-derived influence, ZK eligibility proofs, threshold signing, constitutional rules, and jurisdictional governance. Those mechanisms are useful for ordinary Mycelix governance but are not automatically appropriate for a secret public election.

This profile establishes a separate, fail-closed authority surface for high-stakes public elections. It does **not** treat the existing `VerifiedVote` path as a public-election ballot format and does **not** claim that Holochain/DHT acceptance establishes election truth.

The governing security target is:

> A compromised component must not be sufficient to produce an undetectably incorrect certified election outcome.

## Research baseline

The foundation intentionally follows current election-assurance direction rather than inventing a blockchain-voting security model:

- EAC VVSG 2.0 requires software independence: software/hardware defects must not be able to cause an undetectable result change. Paper-based and cryptographic E2E architectures are recognized routes.
- The EAC E2E protocol evaluation process requires cast-as-intended, recorded-as-cast, tallied-as-recorded, and dispute-resolution procedures.
- NIST's 2026 E2EV expert study emphasizes that cryptographic strength alone is insufficient; usability, accessibility, process, and public comprehensibility materially affect adoption and trust.

References:

- https://www.eac.gov/vvsg-20
- https://www.eac.gov/voting-equipment/end-end-e2e-protocol-evaluation-process
- https://www.nist.gov/publications/us-election-expert-perspectives-end-end-verifiable-voting-systems

## Scope: ELECT-001 through ELECT-006

### ELECT-001 — Explicit adversary classes

`election-integrity-types` registers a minimum public-election threat census covering:

- nation-state attackers;
- malicious election officials;
- colluding trustee subsets;
- malicious voters;
- compromised voting devices;
- compromised scanners;
- software supply-chain compromise;
- compromised Mycelix nodes;
- malicious verifiers;
- network partitions;
- coercers; and
- stolen credentials.

A later threat-model evidence document must bind mitigations, residual risk, and test evidence to every required class. Merely listing an attacker is not qualification evidence.

### ELECT-002 — Frozen Election Constitution

Every public election must bind a non-zero digest for the exact:

1. election definition;
2. jurisdiction/electorate snapshot;
3. eligibility rules;
4. ballot definition;
5. trustee policy;
6. audit policy;
7. dispute policy; and
8. certification policy.

The constitution also binds the public-election authority firewall described in ELECT-006. The initial kernel rejects zero digests, an empty election ID, a wrong profile ID, and disallowed authority semantics.

Later tranches should bind explicit canonicalization/version identifiers and crypto-suite identifiers before these digests are promoted to cross-implementation interoperability contracts.

### ELECT-003 — Fail-closed election lifecycle

The initial lifecycle is deliberately linear:

```text
Draft
  -> Reviewed
  -> Frozen
  -> TrusteeCeremony
  -> CredentialCeremony
  -> Voting
  -> PollsClosed
  -> CryptographicTally
  -> PhysicalAudit
  -> ChallengeWindow
  -> Certified
  -> Archived
```

No generic `set_status()` authority exists in the pure kernel. Every accepted edge has an evidence gate and all shortcuts fail closed.

Important certification rules already encoded:

- cryptographic tally cannot advance to physical audit without verified tally proof evidence;
- physical audit cannot advance to challenge resolution without a passing audit result;
- unresolved qualifying challenges block certification;
- certification requires an independent-verifier quorum; and
- certification requires complete certification evidence.

This is a lifecycle theorem only. It does not yet define who may author each transition or how distributed agreement over transition evidence is established.

### ELECT-004 — Election evidence vocabulary

The initial evidence vocabulary distinguishes:

- election definition;
- jurisdiction snapshot;
- eligibility rules;
- ballot definition;
- trustee policy and ceremony;
- credential ceremony;
- transparency checkpoints;
- encrypted ballot records;
- physical ballot manifests;
- custody transfers;
- tally proofs;
- audit samples/results;
- challenges and resolutions;
- certification evidence; and
- archival packages.

The DHT may replicate these objects, but DHT presence is not itself proof of correctness, uniqueness, completeness, or certification authority.

Later work should make checkpoint lineage append-only, witnessable by independent parties, exportable to offline archives, and verifiable without a running Mycelix network.

### ELECT-005 — Election theorem registry

The initial registry contains twelve certification requirements:

1. **Eligibility** — every counted ballot corresponds to authority held by an eligible elector.
2. **Uniqueness** — no elector contributes more authority than the frozen rules permit.
3. **Ballot secrecy** — published evidence does not reveal voter-to-choice linkage.
4. **Receipt freeness** — the protocol does not intentionally provide transferable proof of a voter's selections.
5. **Cast as intended** — evidence exists that selections were encoded as intended.
6. **Recorded as cast** — accepted ballots can be checked for inclusion without revealing choice.
7. **Tallied as recorded** — the tally is independently derivable from accepted election records.
8. **Software independence** — software faults cannot silently create an undetectably wrong certified result.
9. **Administrative non-authority** — no single administrator can unilaterally determine/certify the outcome.
10. **Evidence continuity** — required evidence cannot be silently deleted, substituted, or history-rewritten.
11. **Recoverable verification** — verification remains possible from preserved evidence when normal Mycelix infrastructure is unavailable.
12. **Evidence before certification** — certification authority exists only after frozen evidence requirements are satisfied.

These are requirements, not accomplished claims. Each theorem needs separately identified qualification evidence before Mycelix can claim it for a real protocol.

### ELECT-006 — Public-election authority firewall

The v1 public-election profile currently requires:

- equal voting authority per authorized elector;
- no reputation weighting;
- no consciousness/phi weighting;
- no quadratic influence;
- no generic delegation;
- no identity-bearing ballot objects;
- no voter-written reason text attached to the ballot;
- no unilateral administrator finalization;
- no remote marked-ballot transmission in the flagship profile;
- voter-verifiable paper evidence; and
- end-to-end verifiability.

This firewall is intentionally stricter than ordinary Mycelix governance. Existing rich governance mechanisms remain available outside `mycelix-public-election-v1`.

## Separation from the current ZK governance workflow

The current governance ZK workflow stores `voter_did`, voter commitments, effective trust-derived weight, and voter-indexed links. That is useful design work for private eligibility within ordinary governance, but it is **not** an acceptable public-election ballot representation as-is.

A future public-election credential design must separate:

```text
civil/eligibility identity
        |
        | proves eligibility
        v
unlinkable election-specific authorization
        |
        | single-use/nullified without identity disclosure
        v
anonymous ballot protocol
```

The public election transcript must not contain a stable Mycelix DID, K-vector, reputation score, consciousness score, social graph identity, or ordinary account key that enables voter-to-ballot linkage.

## Deliberate non-claims

This foundation does **not** yet provide:

- an encrypted ballot construction;
- coercion resistance;
- cryptographic receipt freeness;
- anonymous credentials;
- a nullifier design;
- cast-as-intended verification;
- a tally proof system;
- a mixnet or homomorphic tally;
- trustee DKG for election decryption;
- risk-limiting-audit mathematics;
- physical ballot chain-of-custody implementation;
- an election transparency log;
- independent verifier implementations;
- election certification; or
- evidence sufficient to deploy Mycelix in a governmental election.

The existing Mycelix threshold-signing work is a reuse candidate for institutional authorization, but election decryption/tally key management must be reviewed separately; governance finality keys and ballot-decryption keys must not be conflated by convenience.

## Next tranche

The next narrow engineering sequence should be:

1. **ELECT-007** — anonymous eligibility statement and privacy boundary;
2. **ELECT-008** — election-scoped single-use/nullifier contract;
3. **ELECT-009** — append-only transparency/checkpoint lineage;
4. **ELECT-010** — independent witness/checkpoint quorum;
5. **ELECT-011** — minimal offline verifier input/output contract; and
6. **ELECT-012** — physical ballot, manifest, custody, and audit evidence types.

Ballot cryptography should be selected only after those boundaries exist. Candidate protocols should be evaluated against the theorem registry rather than chosen because they are fashionable or decentralized.
