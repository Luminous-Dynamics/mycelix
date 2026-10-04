# SYM-CIVIC-016 — effective witness-set intersection preflight v1

Status: design/research preflight only; **not qualified**

Parent subject: `ed66bf43ac498316da53386752e9db7efb07a0ab`

## Boundary

016 moves one boundary above the qualified single-agreement witness set:

`multiple bilateral agreements + deterministic witness-entry identity + effective-set intersection + effective-quorum resolution => effective witness-set satisfiability`

The September 2026 SCITT Attestation Reconciliation Protocol draft (ARP-04) states that when a deployment holds more than one Bilateral Register Agreement, the effective Witness Set is the intersection of the Witness Sets declared by every Agreement, while the effective Witness Quorum is the largest quorum declared by any Agreement. Two Witness Entries are equal for this intersection when their normalized Operating-Party Identifiers and Verification Method References are equal. The effective set must still contain at least the effective quorum with pairwise-distinct Operating-Party Identifiers; otherwise the deployment is non-conforming and must not serve reads.

ARP-04 also makes normalization mechanically relevant: Operating-Party Identifier equality is evaluated after normalization under RFC 3986 sections 6.2.2 and 6.2.3. The intersection therefore cannot treat syntactically different spellings of one origin as distinct or collapse different verification methods merely because the operating party matches.

## Model

Two or more synthetic bilateral agreements each declare:
- an agreement hash;
- a witness set;
- a declared witness quorum.

Each witness binds:
- Audience Member Identifier;
- Verification Method Reference;
- Operating-Party Identifier;
- public-key identity digest.

The effective Witness Set is computed over the normalized pair:

`(normalized Operating-Party Identifier, Verification Method Reference)`

For every key present in **every** agreement, the selected entry is the entry declared by the lexicographically least Agreement Hash. Agreement ordering in the input is non-authoritative.

The effective Witness Quorum is the maximum quorum declared by the agreements.

The verifier rejects source-agreement provenance failures before intersection, including duplicate witness identity, duplicate Verification Method Reference, duplicate key identity, missing identity fields, and a source quorum that cannot be satisfied by that agreement's own distinct operating-party witness entries.

A structurally valid intersection is **unsatisfiable** when its cardinality, distinct normalized Operating-Party Identifier count, or distinct Verification Method Reference count cannot satisfy the effective quorum.

The qualifier does not infer real-world independence, common-control absence, witness honesty, cryptographic validity, register truth, or legal authority.

## Corpus

Exactly 18 cases; no expected-verdict fields.

The corpus covers:
- identical multi-agreement witness sets;
- disjoint witness sets;
- same-party/different-key separation;
- same operating party + same verification method with different Audience Member Identifier and deterministic least-hash selection;
- normalized Operating-Party Identifier equivalence;
- maximum-quorum escalation above the effective intersection;
- independently conforming Agreements whose intersection is unsatisfiable;
- duplicate operating party inside one Agreement;
- duplicate verification method inside one Agreement;
- duplicate resolved key identity;
- malformed witness identity;
- unknown critical extension;
- non-critical extension mutation;
- Agreement input-order permutation;
- witness-entry order permutation;
- Agreement-hash selection change;
- zero-quorum effective set;
- effective-set quorum preserved at the boundary.

Metamorphic probes separately verify that reordering Agreements and witnesses does not change the effective result, while changing the Agreement Hash ordering deterministically changes which equal witness entry supplies the effective representation.

## Ceiling

A GREEN result establishes only this synthetic effective-witness-set intersection boundary.

It does not establish global non-equivocation, real-world independence, common-control detection, witness honesty, completeness of all agreements held by a deployment, correctness of register contents, legal/judicial finality, authorization, civic authority, or operational safety.

The CLL line remains narrower still: checkpoint witnessing proves continuity against a witness's retained checkpoint and does not prove content truth or prove that no parallel uncommitted history exists.

Design rule:

`Intersect by the standards-defined witness identity tuple, resolve quorum deterministically, and reject an effective set that cannot actually satisfy the effective quorum.`

References:
- https://datatracker.ietf.org/doc/draft-hillier-scitt-arp/ (draft-hillier-scitt-arp-04, September 2026)
- https://www.rfc-editor.org/rfc/rfc3986
- https://datatracker.ietf.org/doc/draft-mih-scitt-checkpointed-local-log/ (draft-mih-scitt-checkpointed-local-log-01, September 2026)
- https://www.rfc-editor.org/rfc/rfc9943
