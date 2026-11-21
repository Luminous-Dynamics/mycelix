# Directory Fabric v0.1 — Request-scoped causal cuts and decision/effect receipts

**Status:** executable pure-Rust reference contracts authored; not compiler-, cryptographic-, runtime- or protocol-qualified.  
**Programme:** [MYC-DIR-FABRIC-001 / #6428](https://github.com/Luminous-Dynamics/mycelix/issues/6428)  
**Parent design/reference:** [#6430](https://github.com/Luminous-Dynamics/mycelix/pull/6430) and [#6434](https://github.com/Luminous-Dynamics/mycelix/pull/6434)  
**Implementation:** `tools/myc-dir-fabric-reference/src/causal_cut.rs` and `src/receipts.rs`.  
**Adversarial contract:** [MYC_DIR_CAUSAL_CUT_RECEIPTS_V0_1.json](../benchmarks/MYC_DIR_CAUSAL_CUT_RECEIPTS_V0_1.json) plus its Draft 2020-12 schema.  
**Claim ceiling:** finite synthetic reference semantics only. No signature verification, complete dependency-closure proof, independent verifier deployment, SCITT integration, Holochain runtime, real effect-owner integration, performance result or production enforcement is established.

## 1. Architectural decision: one request, one complete causal cut

A decentralized authorization request should not require every peer, realm, issuer or unrelated authority in the network to be globally current. It must, however, have sufficient evidence for every authority dependency that could alter the outcome of this exact request.

The proposed **Request-Scoped Causal Authority Cut (RSCAC)** is the exact, versioned set of authority positions needed to evaluate one request under one policy and one dependency-graph snapshot.

For request:

`Q = (realm, subject, subject-key-epoch, workload, resource, action, audience, purpose, context, nonce, evaluation-profile)`

the closure resolver identifies the relevant authority dependencies, including where applicable:

- realm policy and realm-root epoch;
- subject membership and membership lifecycle;
- issuer admission, key status and credential status;
- relevant group and role relationships;
- delegation ancestors, attenuation constraints and revocation;
- federation contract and foreign-to-local mapping;
- device posture and workload trust-domain/audience evidence;
- resource policy, explicit denies, obligations and revocation state.

The cut contains exact identifiers, dependency kinds and minimum authority positions. The authority positions are per authority (or per source-chain lineage), not one summed height. A higher sequence does not prove ancestry by itself.

**Completeness and minimality are different properties.**

- Completeness is a safety requirement: every dependency capable of changing the result must be represented.
- Minimality is an efficiency and availability objective: avoid requiring unrelated authorities to be current.

A conservative superset can be safe but may increase latency and false denial during partitions. An incomplete cut can be unsafe and must never produce Allow. The v0.1 reference models exact binding and completeness-attestation consumption; it does not prove a resolver returns the mathematically minimal cut.

## 2. Two-stage closure verification

A resolver-produced list is not self-authenticating. The requester and resolver must not be the only parties deciding that the dependency list is complete.

The target protocol has two separate artifacts:

### 2.1 Dependency-graph snapshot

The resolver operates on an authenticated, versioned dependency graph snapshot, identified by an exact digest. It must include the policy/entity relationships from which dependencies are derived, including deny-relevant and revocation-relevant edges.

If the graph snapshot is unavailable, stale under the required realm profile, or cannot be authenticated, closure cannot be claimed complete.

### 2.2 Dependency closure receipt

An independently pinned verifier evaluates the exact tuple:

`(request_digest, realm, policy_digest, dependency_graph_digest, cut_digest, resolver_profile_digest, proof_digest, verifier_id, verifier_build_digest)`

The receipt states one of:

- `VerifiedComplete`
- `Incomplete`
- `Rejected`
- `Unavailable`
- `Unverified`

Only `VerifiedComplete` is eligible to continue. The verifier must be independent of the resolver's ability to omit or alter inputs; verifier identity/build and trust roots must be supplied out of band under the relying realm's policy. The closure proof format, verifier signature, anti-rollback and key-rotation protocol remain future work.

The reference crate accepts the receipt as an external verifier result and checks that its request, realm, policy, graph, cut and resolver-profile bindings match. It does not itself verify the proof digest, signature, authority of the verifier, or graph authenticity. In particular, a caller must never set `VerifiedComplete` merely because it constructed a non-empty dependency array.

### 2.3 Local frontier comparison

After closure verification, the PEP checks every required authority position against its authenticated local snapshot:

- exact authority position: covered;
- older local position: behind;
- higher local sequence with a verified exact ancestry witness: covered;
- higher sequence without that witness: incomparable;
- same sequence with a different digest: incomparable;
- missing non-genesis authority position: behind/incomplete.

An unrelated authority's progress cannot compensate for one required dependency being behind. If any component is behind or incomparable, no Allow is possible. A known explicit forbid may still produce Deny when its exact witness is independently valid and bound to the full request.

This is deliberately narrower than global consistency. The requirement is a complete, authenticated causal lower bound for the dependencies that matter to this request.

## 3. Compose with existing authorization instead of duplicating it

Directory Fabric must not create another policy language, capability algebra or host-call lifecycle model.

The intended composition is:

`Complete dependency closure`
`+`
`Sufficient local authority frontier`
`+`
`Existing full-context policy result`
`+`
`Exact decision receipt`

Existing work remains authoritative for its domain:

| Existing work | Ownership retained |
|---|---|
| [#4743 / PR #4752](https://github.com/Luminous-Dynamics/mycelix/pull/4752) | One exact effective capability atom must match the full request context (resource/action/audience/time) with exact contributor provenance; contextual laundering is denied |
| [#4854](https://github.com/Luminous-Dynamics/mycelix/issues/4854) | Bounded constraint satisfiability/subsumption, normalization and deny preservation |
| [Pulse #5771](https://github.com/Luminous-Dynamics/mycelix/pull/5771) | Causal frontier, grant identity/lifecycle, host-call admission, revocation, retry identity and bounded formal models |
| [Pulse #4903](https://github.com/Luminous-Dynamics/mycelix/issues/4903) | Incomplete DHT visibility is unknown; stale application projection is never host permission |
| [Pulse #5924](https://github.com/Luminous-Dynamics/mycelix/issues/5924) | Revocation linearization at actual host admission; pre-admitted in-flight execution is distinguished from new post-revocation admission |
| [LISE #6236](https://github.com/Luminous-Dynamics/mycelix/issues/6236) | Trajectory-wide revocation propagation, route/session inventory and stale-capability exposure across PEPs, tokens, workers and connectors |
| [LISE #6235](https://github.com/Luminous-Dynamics/mycelix/pull/6235) | Candidate-blind trajectory scorecard and separate safety metrics |
| [#6050](https://github.com/Luminous-Dynamics/mycelix/issues/6050) | Machine subject, key epoch, instance, artifact digest and workload identity remain distinct |
| [#889](https://github.com/Luminous-Dynamics/mycelix/issues/889) | Non-authoritative enterprise protocol TCK; protocol evidence does not become Mycelix authority |
| [#1149](https://github.com/Luminous-Dynamics/mycelix/issues/1149) and [#1191](https://github.com/Luminous-Dynamics/mycelix/issues/1191) | Bounded local capability leases and runtime branch-enforcement boundaries |

The Directory Fabric reference module supplies the composition boundary and evidence binding; it does not replace these owners.

## 4. Policy evaluation: completeness is not the same as policy syntax

A critical interoperability seam exists around policy evaluation errors.

Cedar's authorization algorithm has explicit semantics for `permit`, `forbid` and evaluation errors. A matching `permit` can produce an Allow even when another policy evaluation returns an error, because error policies are skipped; a matching `forbid` takes precedence. That behaviour is a property of the Cedar policy algorithm, not proof that an external identity/issuer/realm dependency set is complete.

Directory Fabric must maintain two separate domains:

1. **Authority/dependency completeness:** did the verifier establish all mandatory identity, issuer, membership, revocation, delegation and policy-graph inputs for this request?
2. **Policy evaluation:** given the complete, authenticated inputs, what does the selected policy engine decide?

A missing required authority dependency is not a harmless policy error to skip. The PEP must stop before treating a Cedar permit as sufficient. An adapter may retain Cedar's documented error behaviour for policy statements that its profile explicitly declares non-mandatory, but it cannot classify missing mandatory authority evidence as an ignorable statement error.

v0.1 decision-receipt rules:

| Cut assessment | Policy result | Reference decision |
|---|---|---|
| Sufficient | exact full-context Permit witness | Allow candidate |
| Not sufficient / unknown | Permit witness | Indeterminate |
| any cut state | exact, request-bound explicit Forbid witness | Deny |
| Sufficient | complete-policy-set DefaultDeny | Deny |
| Not sufficient / unknown | DefaultDeny | Indeterminate |
| any state | mandatory policy evaluation unavailable | Indeterminate |

The word “candidate” matters: production still needs signature verification, subject/issuer admission, runtime enforcement, and exact PEP binding. A receipt is evidence of a decision claim, not the decision's truth by itself.

## 5. Two receipts, not one: authorization versus effect

A positive authorization is not evidence that a protected operation happened. Conversely, a successful downstream operation does not prove it was properly authorized.

### 5.1 Authorization decision receipt

The v0.1 canonical decision record binds:

- decision ID and realm;
- exact request digest;
- policy digest and epoch (epoch is represented through the bound policy/snapshot profile);
- exact dependency cut digest;
- dependency-closure receipt digest;
- policy snapshot digest;
- evaluator build digest;
- request nonce/challenge digest;
- named clock profile;
- issued-at and expiry;
- cut assessment;
- outcome (Allow/Deny/Indeterminate);
- outcome basis kind and exact permit/forbid/default-deny/cause digest.

These bytes are versioned and domain-separated. The relying party must hash and sign them using a pinned signature/hash profile. The current Rust reference emits canonical bytes but does not compute the digest, sign, verify signer identity, provide durable nonce consumption, or prove trusted time.

### 5.2 Effect-attempt and effect-reconciliation receipts

Keep two different record types.

**Physical attempt receipt** is emitted for each actual dispatch/attempt and binds:

- same request digest;
- exact authorization decision receipt digest;
- stable logical operation ID;
- unique physical attempt ID and consecutive attempt number;
- stable idempotency-key commitment;
- effect-owner identity;
- outcome: Committed, RejectedNoEffect, Unknown or NotObserved;
- exact observation digest.

**Reconciliation receipt** is a separate owner-side observation. It binds the exact prior attempt ID and number, same logical operation/request/idempotency commitment, same effect owner, and one outcome: ConfirmedCommitted or ConfirmedNoEffect. A reconciliation does not consume a physical attempt number and cannot be used as a new execution event.

The verifier must check every physical attempt for the same logical operation preserves request, idempotency commitment and effect owner. There may be at most one effective commit for the logical operation, considering both direct attempt observations and reconciliations of Unknown attempts. Unknown does not mean no effect and blocks a new physical attempt until a reconciliation receipt explicitly targets that exact Unknown/NotObserved attempt. ConfirmedNoEffect may permit a retry of the same logical operation with the next consecutive physical attempt number; if that retry becomes Unknown, unresolved status returns. ConfirmedCommitted closes the logical operation and makes any later physical attempt invalid.

A reconciliation must not target an attempt already reported as Committed/RejectedNoEffect, must reference the exact decision receipt used for that attempt, and must not be duplicated. The reference crate checks these relationships structurally; it does not authenticate the effect owner or prove its observation inventory complete.

The local reference history checker enforces these structural rules. It does not authenticate effect-owner identity or prove that the owner enumerated every route/session/worker capable of producing an effect. That stronger inventory and observation obligation belongs with LISE #6236.

### 5.3 Transparency is a third, separate plane

SCITT (RFC 9943) can make signed decision statements transparently registrable and auditable. A transparency-service receipt proves registration/inclusion under that service's profile; it does not prove that the statement is true, that the evaluator was authorized, that the permission remains current, or that a downstream action happened.

The correct chain is therefore:

`Authority and closure evidence`
`→ Authorization decision receipt`
`→ Actual effect-owner observation receipt`
`→ Optional transparency registration and receipt`

SCITT receipt != authorization grant.  
SCITT receipt != effect receipt.  
Hash commitment != issuer identity.  
Signature validity != organizational authority.

## 6. Paradigm shifts and research questions

### A. From global freshness to request-scoped causal sufficiency

The main research question is whether a verifier can independently prove that a request's dependency cut is complete and that the PEP's snapshot dominates the minimum authority positions for each dependency.

This extends, rather than replaces, the per-request consistency work in SpiceDB and OpenFGA. SpiceDB's ZedToken `at_least_as_fresh` mode gives a useful read-after-write comparator; OpenFGA exposes higher-consistency versus latency-oriented query modes. Mycelix's proposed difference is a cut across independently governed authority histories, not just another consistency token.

### B. From roles to an evidence graph with explicit deny edges

The dependency resolver must include not only positive relationships but also deny-relevant constraints, revoked delegations, issuer/key status, exclusions and policy obligations. Omitting a deny dependency must be a specific negative control. Deny preservation continues to be owned by #4854.

### C. From “one Allow” to an accountable action lifecycle

The decision/effect receipt separation allows measurement of what happened after an authorization decision: whether an effect committed, was rejected, remained unknown or required reconciliation. This allows LISE to measure stale capability exposure and false claims of successful execution without creating a competing revocation engine.

### D. From protocol adapters to evidence-preserving translation

SCIM, OIDC/OAuth, SPIFFE/SPIRE and LDAP adapters should translate external identity evidence into bounded, source-attributed inputs. They must preserve unknown/unsupported fields and never promote protocol success into authorization.

- SCIM is provisioning and lifecycle exchange, not an authorization decision.
- OIDC tokens must be audience/purpose bound and validated under the configured issuer profile; sender-constrained proof-of-possession should be evaluated for sensitive APIs.
- SPIFFE identity and SVIDs are workload identity evidence in an explicit trust domain, not human membership or unlimited service authority.
- LDAP read-only projections do not implement Kerberos, Windows domain join, DNS, PKI, Group Policy or endpoint management.

The v1 migration path remains read-only import → shadow decisions → SCIM lifecycle → OIDC application → SPIFFE workload bridge → one local Linux PEP → narrow production pilot. Kerberos/Windows remains a separate programme.

## 7. New research direction: zero-knowledge dependency-closure queries

A significant adjacent result appeared on 3 November 2026: **VerfGraph: Efficient Zero-Knowledge Verifiable Graph Query Evaluation System** ([arXiv:2611.04240](https://arxiv.org/abs/2611.04240)). It builds on the expansion-centric model of ZKGraph and proves graph-query correctness and completeness over a committed graph without revealing private graph records or intermediate values beyond the query result. The authors implement it using Plonky3 AIR with a transparent FRI-based polynomial commitment backend and report, on LDBC SNB workloads, 33.9×–59.7× proof-generation speedups and 4.5×–6.6× lower peak prover memory for complex IC queries versus ZKGraph; verification stays in the millisecond range and proofs remain below 1 MB. The same-backend comparison also isolates improvements to expansion/SSSP proof constructions. These are preprint author-reported results, not an independent Mycelix replication.

This is directly relevant to RSCAC. Instead of returning a private dependency list to every verifier, a realm could commit to a policy/entity graph snapshot and prove that a specified dependency-closure query produced the exact answer over that snapshot. Candidate graph operations include:

- one-hop expansion over membership, group, role, delegation, issuer, revocation and explicit-deny edges;
- bounded multi-hop expansion for inherited group/delegation authority;
- set union/deduplication across dependency families;
- filtered expansion over resource, action, audience, purpose and policy epoch;
- count/cardinality and path/ancestor queries where they support a complete policy proof.

The public statement would bind the exact request digest, query-plan/schema digest, policy graph root, authority frontier, resolver version, closure result commitment and proof. The private witness would contain sensitive relationship records and intermediate traversal state. The relying PEP would verify the proof, then independently check that the graph root is signed/admitted by the correct realm/issuer and that its per-authority frontier satisfies the request's required freshness.

### Crucial distinction: proof of query completeness is not proof of graph completeness

VerfGraph can establish that the answer is complete with respect to the committed graph and exact query semantics. It does not prove that the authority graph contains every real-world fact, that the graph root was authorized, or that an issuer did not omit a revocation before committing the graph.

Mycelix therefore needs two independent guarantees:

1. **Authoritative graph snapshot:** each realm/issuer root is authenticated under its own admitted authority, key epoch, sequence and lifecycle rules. Root rotation, fork detection, append-only history and revocation remain under Mycelix's existing authority/frontier work.
2. **Private query completeness:** a proof establishes that the dependency cut is the complete result of the pinned query over those exact committed roots, without disclosing unrelated private relationships.

For a cross-realm request, do not collapse all roots into one global sequence. Prove closure over a tuple of realm/issuer roots and retain the partial-order frontier semantics for each authority.

### Privacy and side-channel contract

Never publish the raw cut, employee group edges, private resource relationships or unsalted stable relationship hashes to the public DHT or a transparency log by default. Even a hash can be linkable when identifiers or candidate relationships have low entropy. Use local/private or encrypted transport for the full witness; publish only purpose/audience/expiry-bound commitments and proof material whose disclosure profile has been reviewed. The public proof may still leak graph-root epochs, query shape, result cardinality or timing; these are explicit side channels to measure, not assume away.

Explore a staged path:
1. prototype authenticated graph-query completeness without privacy, over a frozen synthetic directory graph;
2. prove set-based expansion and negative/deny-edge coverage over a committed snapshot;
3. adapt a zero-knowledge backend for sensitive relationship records;
4. add multiple independently signed authority roots and request-scoped causal frontier checks;
5. measure prover latency, verifier latency, proof size, memory, update costs and leakage.

Do not write a new ZK engine before running a matched feasibility spike against VerfGraph/ZKGraph and existing Mycelix graph representations. The research question is integration/adaptation: can a complete authorization-dependency query be expressed in the supported operator set without changing its semantics, especially around explicit denies, revocation and nested delegation?

### Proposed VerfGraph comparison

Use LDBC SNB Interactive/Business Intelligence workloads as external graph-query baselines where useful, but build a separate Directory Fabric corpus with synthetic and consented enterprise-like membership/delegation/revocation graphs. Report:
- exact query semantics and completeness oracle;
- graph root and update snapshot binding;
- proof-generation time, peak prover memory, proof bytes and verifier time;
- query cardinality, branching factor, depth, cycle and deny-edge handling;
- incremental update / root rollover / proof regeneration costs;
- private-record leakage and metadata leakage;
- whether the proof remains valid after graph-root or authority-frontier advance.

The potential Mycelix differentiator is not “we use zero knowledge.” It is **a privacy-preserving, independently verifiable proof that the request's complete authority dependency cut was derived from the exact sovereign graph snapshots whose authority histories the PEP accepts**. This remains a research target, not an implemented capability. The proposed VerfGraph comparator is an arXiv preprint dated 3 November 2026; its LDBC SNB results are author-reported and have not been independently reproduced by Mycelix. The supported operator/query semantics, graph-root authentication and multi-authority/deny closure requirements must be matched before any comparison or reuse claim.

VerfGraph's evaluation uses LDBC SNB **Interactive** query families (IS3/IS4/IS5/IC1/IC2/IC9/IC13, with IC6 and IC14 V2 for added count/weighted-SSSP support) at 60k/120k/240k rows in the relevant fact tables. Those are graph-query baselines, not authorization workloads. The benchmark profile therefore keeps LDBC as an external operator-comparison track and adds a separate MYC-DIR authority graph plus multi-root federation workloads; matched comparison must use the same query semantics and committed data snapshot.

## 8. Benchmark protocol additions

The machine-readable benchmark preregistration now includes 38 metrics, six dependency-cut-size profiles (1/8/32/64/128/256 dependencies), 25 fault schedules and 12 comparator groups covering Cedar, SCITT, VerfGraph, ZKGraph and LDBC SNB. The causal-cut corpus has 24 scenarios (8 Allow, 4 Deny, 12 Indeterminate) and its own schema.

New non-compensable gates:

- Zero false complete-closure claims when one required dependency is omitted.
- Zero Allow decisions with incomplete, unverified, unavailable or non-dominating mandatory dependencies.
- 100% detection of seeded omissions against a separately held-out dependency oracle.
- Zero accepted decision-receipt substitutions.
- Zero effect commits without an exact matching Allow receipt.
- Zero duplicate commits per logical operation/idempotency key.
- Zero new mutation attempts while a prior attempt remains Unknown without exact no-effect reconciliation.
- Zero SCITT inclusion receipts treated as identity, permission or effect authority.
- Zero permits surviving omission of a policy/authority dependency declared mandatory for the request.

Provisional pure-model targets (not measured):

| Metric | Target |
|---|---:|
| Local cut/frontier assessment p95 at 64 dependencies | ≤500 μs |
| Decision receipt composition p95 (excluding signatures) | ≤100 μs |
| False complete dependency closures | 0 |
| Allow with incomplete required cut | 0 |
| Omitted mandatory dependency detection | 100% |
| Effect commit without exact Allow receipt | 0 |
| Duplicate logical effect | 0 |
| Retry while unknown/unreconciled | 0 |

Report dependency-cut size relative to the reachable policy/entity graph so the project can measure whether request-scoped resolution actually reduces the currentness burden. Report worst-case and distribution, not just averages. Do not measure only the pure local function: also measure independent closure resolver latency, signature verification, cache hits/misses, proof bytes, graph snapshot storage and effect reconciliation. For zero-knowledge closure, measure proof generation, verifier time, proof bytes, prover memory, graph-root rollover and leakage; add a held-out test that mutates or omits deny/revocation edges from the query plan. A proof over an authentic but stale graph root must remain non-authorizing when the request requires a newer authority frontier.

Use at least 10 independent process-level repetitions, open-loop offered load and exact source/build/runtime/graph/corpus digests. Keep actual Holochain conductor tests separate from pure-reference tests and synthetic peer simulations. Benchmark all comparators on matched policy/request semantics and freshness requirements; do not collapse security and sovereignty into a single weighted score.

## 8. Finite-state model

A first bounded TLA+ composition model is authored at [DirectoryFabricCausalCutReceipts.tla](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricCausalCutReceipts.tla), with a safe configuration and four single-switch negative configurations: omitted dependency closure, scalar frontier, retry while Unknown and duplicate commit. The [model note](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricCausalCutReceipts-model-2026-11-21.md) states the invariants, expected mutant counterexamples and abstraction limits.

**No TLC execution result is claimed.** The model is a prospective finite assurance artifact, not proof of graph resolver completeness, cryptographic trust or Holochain runtime behaviour.

## 9. Adversarial matrix

The companion 24-case corpus includes positive controls and attacks across:

- complete and incomplete dependency closure;
- unverified resolver self-assertion;
- omitted revocation and issuer dependencies;
- independent progress masking a stale required authority;
- higher sequence without proven ancestry;
- same authority supplying multiple roles at distinct positions;
- unrelated authority staleness that must not block a genuinely complete minimal cut;
- exact default-deny and explicit-forbid cases;
- skip-on-error permit trap when a mandatory authority input is missing;
- request/audience, realm, policy, graph and cut substitution;
- SCITT receipt presented as authority;
- Allow with no effect observation;
- effect-owner commit/reject/unknown/no-effect reconciliation;
- changed logical operation or idempotency identity;
- duplicate committed effects;
- commit evidence attached to a Deny decision.

The corpus is finite and synthetic. It does not establish completeness of any real policy graph.

## 11. Implementation and qualification gates

1. Review the RSCAC dependency vocabulary, resolver/verifier independence and policy-graph snapshot semantics.
2. Independently implement the closure resolver and hidden completeness oracle; do not derive both expected and observed dependencies from the same code.
3. Define the canonical dependency-graph snapshot and proof format, pinned verifier identity, signature profile, hash agility, revocation and anti-rollback behaviour.
4. Add property-based and differential tests for omitted deny/revocation edges, stale required dependencies, unrelated stale authorities, graph changes between resolve/verify and policy error boundaries.
5. Compose full-context policy evaluation from #4752/#4743 and #4854; retain their exact witness and deny-preservation rules.
6. Connect Pulse #5771/#4903/#5924 so actual host admission and DHT currentness are not replaced by a local application projection.
7. Connect LISE #6236/#6235 for effect-owner inventory, bitemporal propagation and candidate-blind scoring.
8. Add signed decision receipts, nonce consumption, key lifecycle, trusted-clock and independently signed effect-owner observations.
9. Add optional SCITT statement registration only after the relying party's trust policy authenticates the appropriate transparency-service roots.
10. Re-run matched local, relationship, directory, workload and distributed-runtime benchmarks with exact reproducibility receipts.

The selected Holochain version matrix and runtime regression gate remain in [MYC-DIR-HC-QUAL-001 / #6435](https://github.com/Luminous-Dynamics/mycelix/issues/6435). This reference composition does not bypass that gate.

## 12. External standards and comparator references

- **Cedar authorization algorithm:** [Cedar Authorization](https://docs.cedarpolicy.com/auth/authorization.html). Keep policy evaluation errors distinct from missing, mandatory Mycelix authority dependencies. A Cedar `permit` result cannot repair an incomplete request-scoped authority cut.
- **SCITT transparency:** [RFC 9943](https://www.rfc-editor.org/rfc/rfc9943.html). Use transparency registration for signed decision statements only after a relying party pins the transparency-service profile and roots. Inclusion is not truth, permission or effect.
- **SpiceDB consistency:** [Authzed consistency](https://authzed.com/docs/spicedb/concepts/consistency). Compare using `at_least_as_fresh` ZedTokens and, separately, fully consistent reads. Mycelix must demonstrate the additional value of a complete cut across independent authority histories; freshness tokens alone are not a differentiator.
- **OpenFGA consistency:** [OpenFGA consistency](https://openfga.dev/docs/interacting/consistency). Compare `HIGHER_CONSISTENCY` and `MINIMIZE_LATENCY` on equivalent relationship graphs and request sets.
- **OAuth security:** [RFC 9700](https://www.rfc-editor.org/rfc/rfc9700.html) and [DPoP / RFC 9449](https://www.rfc-editor.org/rfc/rfc9449.html). Audience and sender binding are adapter security controls; neither token validity nor proof-of-possession establishes local organizational authority.
- **SPIFFE federation:** [SPIFFE Federation Specification](https://spiffe.io/docs/latest/spiffe-specs/spiffe_federation/). Bundle trust is trust-domain scoped; do not treat an arbitrary valid SVID or bundle as a Mycelix membership or resource permission.

## 12.1 Mutation-boundary revocation fencing

The request-scoped causal cut answers whether the PEP evaluated against the required authority evidence. The decision/effect receipts answer what was decided and what an effect owner observed. A remaining race exists between decision issuance and mutation arrival: a process can pause, the network can delay a command, and revocation may occur before the old command reaches the resource.

The added `effect_fence.rs` reference contract places the final stale-command check at the resource owner:

1. A fence is scoped to an exact realm/resource/action and one effect owner. There is no global directory-wide counter.
2. The owner persists a monotonically increasing generation. A command carries a token for that exact scope, owner and generation.
3. The command binds the exact Allow decision digest, request digest, logical operation, idempotency commitment and fence token.
4. The resource owner compares the token with its persisted generation **inside the same atomic transaction as the protected mutation**. Old generations, unissued future generations and wrong-scope tokens are rejected.
5. A revocation-fence plan names the independently inventoried effect owners and each owner's target generation. Completion requires every owner to durably install its target and return an exact acknowledgement. Missing or unknown acknowledgements leave the revocation pending; a quorum is insufficient for a strict completion claim.

This composes a well-known distributed-systems fencing pattern with request-scoped authorization and effect receipts; the fencing primitive itself is not claimed as novel. The research contribution to evaluate is whether the full composition offers stronger, auditable revocation semantics across sovereign authority roots and heterogeneous effect owners without a global identity database.

Important limits: the owner inventory can be incomplete; owner acknowledgements can be forged or stale without authenticated signatures and durable state proofs; a bypass write path defeats the guard; and non-fenceable third-party APIs cannot receive the strong guarantee. The completion timestamp is the all-owner durable-ack point, not the time a revocation was merely requested. During propagation, status is pending/indeterminate. This gives a measurable revocation window instead of promising instantaneous revocation in an asynchronous network.

### Fencing-specific benchmark contract

The preregistration now includes:
- stale-token mutation accepts and post-fence old-command accepts (both must remain zero);
- owner acknowledgement coverage and revocation-to-fence-complete latency;
- owner bypass attempts, unknown acknowledgements, rollback/restart and inventory omission;
- crash/recovery between generation persistence, acknowledgement and mutation;
- delayed in-flight mutation arriving immediately before and after the final owner acknowledgement;
- a non-fenceable external API control, which must remain explicitly best-effort and cannot be counted as strongly revoked.

The safety claim is scoped precisely: **after** every required owner has durably acknowledged the fence, no earlier-generation mutation may commit through a correctly integrated owner. It does not promise that revocation has completed before that acknowledgement point.

The finite-state companion `DirectoryFabricEffectFence.tla` models per-owner generation installation, exact acknowledgement aggregation and delayed command arrival. Its negative-control configuration deliberately accepts a stale command after all owners acknowledge, and the invariant is expected to fail under that mutant. Model/config files are source-present only; TLC has not been run.

## 12.2 PostgreSQL effect-owner transaction adapter

Mycelix-DeSci already has a PostgreSQL authority backend, serializable authority commits, signed database-write leases, governed database epochs, recovery reconciliation and an at-least-once publication outbox. The Directory Fabric work should reuse those controls. The new effect-owner fence is a separate per-resource mutation boundary, not a replacement database authority or write lease.

The reference adapter lives at:
- `tools/myc-dir-fabric-reference/postgres/001_effect_owner.sql`
- `tools/myc-dir-fabric-reference/postgres/test_effect_owner.sql`
- `tools/myc-dir-fabric-reference/postgres/POSTGRES_EFFECT_OWNER.md`

It uses a sample integer-minor-unit balance mutation to demonstrate atomicity. The trusted PEP must independently verify the signed decision, exact Allow outcome, request binding and closure proof. SQL stores the decision digest but does not verify Ed25519 or infer that the digest means Allow.

**Locking refinement:** each effect transaction takes sorted transaction-scoped advisory locks for its idempotency key and logical operation, then obtains `SELECT ... FOR SHARE` on the exact fence row. Same-row shared locks are compatible, so different protected objects can be changed concurrently. Revocation updates the generation under an exclusive row lock; the conflicting lock waits for all earlier shared-lock effect transactions to commit or abort. Under PostgreSQL `READ COMMITTED`, a waiting `SELECT FOR SHARE` locks and returns the updated row version after the revocation commits, so an old generation is rejected. The mutation, effect journal and idempotency record remain in one SQL transaction.

This gives a precise ordering:
- an effect that already holds the shared fence lock may commit before the generation advance;
- an exclusive revocation waits for those effect transactions to drain;
- after the new generation commits, a new effect with an earlier generation cannot commit;
- an exact replay may return its prior immutable receipt identity without executing the mutation again.

The schema includes owner-local generation state, idempotent revocation records, a protected-object example and an append-only effect ledger. The single-session SQL self-test covers first commit, exact replay, binding substitution, stale-generation rejection and revocation idempotency. Two-session race tests, crash recovery, synchronous-replica failover and SQL runtime validation remain separate gates.

For durability, the caller must set `synchronous_commit=on` and publish the owner acknowledgement only after COMMIT is confirmed. An asynchronous replica must not be promoted while missing an acknowledged generation. If the COMMIT response is lost, the result is unknown until it is reconciled against the authoritative primary/epoch using the same idempotency key. The existing Mycelix-DeSci database-epoch/recovery protocol remains the authority for primary changes.

A database-local transaction cannot atomically commit an arbitrary remote API effect. The existing at-least-once outbox may deliver more than once; each remote receiver must independently deduplicate and enforce its own fence generation. Otherwise classify the remote API as best-effort, not strongly revoked.

See the [PostgreSQL effect-owner runbook](../../tools/myc-dir-fabric-reference/postgres/POSTGRES_EFFECT_OWNER.md). This reference schema has not been applied to a database and is not production-integrated.

## 13. Explicit claim ceiling

This work is a pure reference contract and finite synthetic corpus. It does not establish:

- that any resolver found every real policy/authority dependency;
- that a dependency graph snapshot is complete, authentic or non-equivocating;
- that closure/effect receipts are signed by trustworthy independent parties;
- that a cryptographic ancestry/inclusion proof was verified;
- that a trusted clock exists or that time/lease data is correct;
- that a Holochain host grant was revoked or a conductor rejected a call;
- that an external service/resource effect occurred or did not occur;
- that the effect-owner route/session inventory is complete;
- that SCITT, OIDC, SCIM, LDAP, SPIFFE or Cedar integration is protocol-conformant;
- that the system is production-ready, faster than SOTA or a drop-in Active Directory replacement.

Source-present, structurally inspected, compiled, test-executed, model-checked, runtime-qualified, protocol-conformant and physically measured remain distinct evidence states.
