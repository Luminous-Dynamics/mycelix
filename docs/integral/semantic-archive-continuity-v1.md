# Semantic archive continuity and historical evidence integrity v1

Status: **ReferenceModelOnly**

D6J defined when historical material may be reclaimed. D6K defines what happens to the material that remains archived, and how it may be used without confusing historical evidence with present semantic authority.

The governing distinction is:

\`\`\`
historically evidenced
!=
currently normative
\`\`\`

An archive is a representation of prior semantic state. Its usefulness does not grant it present authority.

## Historical evidence profile

\`HistoricalEvidenceProfileV1\` binds:

- semantic environment;
- permitted historical claim classes;
- a currentness ceiling;
- whether the profile may seed reconstruction;
- a profile commitment.

The currentness ceiling is deliberately bounded to historical evidence or reconstruction input. There is no archive-native \`CurrentNormativeAuthority\` ceiling.

The permitted claim classes are explicit. A profile may permit historical state and lineage analysis without permitting historical authority, capacity, consent, actuation, or policy interpretation.

## Archive manifest

\`SemanticArchiveManifestV1\` binds:

- archive identity;
- source snapshot root;
- source frontier root;
- semantic environment;
- membership epoch;
- evidence profile;
- content and inventory roots;
- retained tombstone identifiers;
- completeness status;
- manifest commitment.

Completeness is profile-relative:

- \`CompleteForProfile\` can support qualified historical analysis and reconstruction input;
- \`Partial\` can support only explicitly partial historical analysis;
- \`Unknown\` and \`Unavailable\` cannot be presented as complete;
- \`Conflicting\` remains contested.

This prevents a convenient but incomplete archive from silently becoming a complete historical record.

## Archive continuity

\`ArchiveContinuityCertificateV1\` makes succession explicit.

A successor archive must identify:

- the exact predecessor archive;
- predecessor frontier;
- exact successor archive/frontier;
- semantic environment;
- predecessor and successor membership epochs;
- predecessor and successor evidence profiles;
- the transition kind;
- explicit membership/profile transition evidence when either changes.

Therefore:

\`\`\`
archive-A + archive-B
!=
continuity
\`\`\`

Continuity is a semantic relation that must itself be evidenced.

Wall-clock adjacency, monotonically named files, increasing database IDs, or matching visible values are not sufficient.

### Membership transitions

A membership epoch change requires an explicit membership transition root.

This prevents an archive generated under one authority set from silently becoming the basis for a different authority set.

### Profile transitions

A profile change requires explicit profile transition evidence.

This prevents a historical representation from being reinterpreted under a new evidence policy merely because the bytes still parse.

## Archive use

The model distinguishes:

- historical analysis;
- historical audit;
- reconstruction input;
- current authorization;
- current actuation;
- current policy interpretation.

The last three are blocked by construction.

Even a complete archive whose frontier happens to equal a current frontier does not become current authority merely because the values match.

This preserves:

\`\`\`
current value equality
!=
current authority
\`\`\`

## Historical claims

\`HistoricalClaimReceiptV1\` binds a claim to:

- an archive;
- an evidence profile;
- a semantic environment;
- an explicit historical temporal scope;
- a source frontier;
- a subject;
- a permitted claim class;
- an evidence-completeness status;
- a non-currentness ceiling.

A historical authority claim can therefore be useful as evidence about what authority existed at a historical frontier without becoming authority now.

Likewise, historical capacity and consent claims remain historical facts rather than automatically becoming live capacity or consent.

## Archive conflicts

\`ArchiveConflictSetV1\` keeps divergent archives first-class.

Possible conflict classes include:

- divergent content;
- divergent frontier;
- lifecycle conflict;
- profile conflict.

The model deliberately does not select one archive as authoritative.

A contested archive set remains contested until an explicit semantic transition resolves it.

This preserves the D6F/D6G/D6H/D6I/D6J progression:

\`\`\`
divergence
  -> identity distinction
  -> causal ordering
  -> lifecycle/no-resurrection
  -> stable reclamation
  -> archive continuity
\`\`\`

## Rehydration

D6J's \`ColdStartManifestV1\` remains the normative reconstruction contract.

D6K only establishes that an archive may serve as reconstruction input when:

1. its environment matches;
2. its evidence profile matches;
3. the profile permits reconstruction;
4. the archive is complete for that profile;
5. required history is available;
6. required tombstone anchors are available;
7. D6J reconstruction checks succeed.

A self-consistent archive is therefore not enough.

\`\`\`
parseable archive
!=
normative reconstruction
\`\`\`

The archive supplies evidence. The cold-start manifest and semantic qualification supply the reconstruction boundary.

## Tombstone continuity

Retained tombstone identifiers are part of the archive manifest.

A required tombstone missing from the archive or from the available reconstruction set blocks normative reconstruction.

This directly carries D6I's no-resurrection invariant across storage-tier boundaries.

The archive cannot turn:

\`\`\`
retired generation
\`\`\`

into:

\`\`\`
available generation
\`\`\`

merely by omitting the tombstone.

## Conservation

Archive use cannot create:

- authority;
- capacity;
- consent.

This is intentionally stronger than merely saying "archives are read-only."

The invariant is semantic:

\`\`\`
claims_after_archive_use
subseteq
claims_before_archive_use
\`\`\`

Historical representation may be incomplete or unavailable, but missing representation cannot manufacture a new right or resource.

## Symthaea boundary

Symthaea can:

- discover relevant archives;
- classify completeness;
- compare historical representations;
- identify likely continuity;
- detect archive conflicts;
- recommend reconstruction inputs;
- propose evidence-retention policies.

Symthaea cannot:

- promote an archive to current authority;
- silently reinterpret an archive under another profile;
- resolve archive conflicts by selection;
- clear lifecycle tombstones;
- mint current capacity or consent from historical evidence;
- authorize current actuation;
- declare continuity without the required transition evidence.

Mycelix remains the semantic root.

## Qualification boundary

The model is deterministic reference semantics only.

It does not establish:

- durable archival storage;
- cryptographic authenticity of roots;
- Byzantine consensus;
- production distributed-system safety;
- legal retention or deletion;
- privacy compliance;
- physical durability;
- real-world availability.

Execution evidence is still required before raising the claim ceiling.
