# Mycelix Directory Fabric — Architecture and Qualification Proposal v0.1

**Status:** research architecture / design-only  
**Programme:** [MYC-DIR-FABRIC-001](https://github.com/Luminous-Dynamics/mycelix/issues/6428)  
**Claim ceiling:** no production directory, protocol server, Windows domain replacement, or benchmark result is established by this document.

## 1. Executive thesis

Mycelix can plausibly become a decentralized alternative to the *organizational identity, directory, trust and access-control plane* commonly built around Active Directory and cloud identity providers.

The product should be called a **Directory Fabric**, not a Holochain LDAP clone. Its core abstraction is a set of autonomous organizational realms that publish and verify carefully scoped trust and authority statements. People, devices, workloads and services retain distinct identities. Each realm governs its own membership and policy. Federation is explicit and revocable.

The key design decision is to separate the **distributed authority plane** from the **local enforcement plane**:

- Holochain maintains agent-authored histories, validation and distribution for appropriate public statements.
- Local enforcement points evaluate requests against authenticated, versioned policy snapshots and bounded leases.
- Protocol adapters expose incumbent interfaces and preserve their semantics.
- No login or resource access should synchronously depend on global DHT convergence.

A DID, credential, Holochain agent key, SPIFFE identity, login session or peer-validation result is evidence. None is an organizational authorization decision by itself.

## 2. What is being replaced — and what is not

Active Directory Domain Services is more than a user directory. Its logical model includes forests, domains and OUs, while domain controllers provide authentication, group information, trust and replication; OUs and ACLs support delegated administration. A real alternative must address the operational contract, not just store user and group records.

| Capability | Directory Fabric role | Initial treatment |
|---|---|---|
| Principal and device directory | Realm-scoped identity graph | Native Mycelix model |
| Organizational membership | Signed, scoped membership assertions | Native realm policy |
| Groups and roles | Relationship graph + deterministic policy evaluation | Native model; bounded group expansion |
| Delegation and admin control | Explicit authority lineage, quorum and expiry | Compose Mycelix governance/authority work |
| Human authentication | External authenticator / OIDC bridge / passkey | Integrate; do not invent passwords |
| Workload identity | SPIFFE/SPIRE and key-bound workload subjects | Integrate and bind to machine identity |
| Application SSO | OIDC first; SAML where required | Adapter programme |
| Provisioning and deprovisioning | SCIM 2.0 | Standards-based adapter |
| Legacy directory reads | LDAP projection / synchronization | Read-only first |
| Kerberos and Windows logon | KDC, trust or AD bridge | Separate research; not provided by Holochain |
| DNS, PKI, certificates | Existing DNS/CA integration, then dedicated qualification | Not implied by identity records |
| Group Policy / endpoint control | Desired-state policy agents and platform adapters | Future independent programme |
| Audit and evidence | Signed decision and effect receipts | Native evidence boundary |

**V1 is a directory control plane and interoperability layer. It is not a drop-in replacement for Windows domain join, Kerberos ticketing, SMB authorization, DNS, certificate services, Group Policy or endpoint management.**

## 3. Existing Mycelix assets and gaps

The existing identity project already contains DID registry, credential-schema, revocation, recovery, MFA, trust-credential and related zomes. That is a better starting point than creating another identity stack.

| Existing owner | Reuse in Directory Fabric | Boundary that remains |
|---|---|---|
| mycelix-identity DID registry | Subject key bindings and DID document discovery | A DID does not prove HR identity, realm membership or administrator status |
| Credential schema / verifiable credential zomes | Typed membership and qualification claims | Credential signature validity does not establish issuer admission or currentness |
| Revocation zome | Credential lifecycle evidence | DHT visibility is not instantaneous revocation propagation |
| Recovery / MFA | Holder and administrator key lifecycle | Recovery authority must be scoped; holder recovery is not organization-root recovery |
| issuer-directory work (#2603, #3393, #3403, #3428) | Issuer key admission, freshness and currentness | Exact external issuer profile and clock/cache policy still govern the result |
| delegated authority (#3659) | Realm-scoped administrative and resource delegation | A host call capability is not full business or organizational authority |
| contextual authority and exact capability applicability (#4743) | Reuse the full-request-context witness rule for resource, action, audience and time | Never combine individually valid dimensions from unrelated grants |
| decidable policy constraint subsumption and deny preservation (#4854) | Consume the existing bounded policy algebra for group/role/delegation transformations | Directory Fabric must not invent a second generic policy engine or treat syntax equality as semantic subsumption |
| delayed visibility / revocation work (#4903, #5924, #6236) | Bounded staleness and enforcement exposure semantics | Do not claim globally instantaneous revocation |
| machine/workload identity (#6050) | Distinct device, controller, key epoch and workload identities | No inferred hardware uniqueness or attestation |
| interoperability TCK (#889) | Protocol fixture ownership | Current TCK is non-authoritative; no protocol pass is implied |
| PEP / lease work (#1149, #1191) | Local enforcement and bounded cached authority | Each concrete enforcement path still needs qualification |

The repository's own top-level README describes the Identity cluster as pre-alpha and warns that multi-agent coverage and deployment maturity differ by cluster. Accordingly, this programme begins with architecture, contracts and adversarial fixtures. Do not infer production readiness from the existence of zomes or unit tests.

## 4. Paradigm shifts

### 4.1 From a central directory to realm-owned constitutional authority

An organization is a trust realm with its own:
- stable realm identifier and schema version;
- root-of-authority and recovery policy;
- admitted issuers and authentication methods;
- administrators, delegated administrators and quorum rules;
- membership and group namespaces;
- policy epochs and rollback protection;
- federation allowlist and outgoing disclosure rules.

A realm is not merely a tenant ID in a global database. It is an independently governed authority context. There is no universal Mycelix root that can silently grant roles inside every realm.

### 4.2 From a flat user table to a typed identity graph

Keep these subjects distinct:

- Human principal: a person-controlled identity, possibly represented by one or more credentials.
- Organization realm: the authority namespace and policy owner.
- Device: a durable machine subject with a separate key epoch and instance identity.
- Workload: a short-lived process/service identity bound to a trust domain, audience and runtime evidence.
- Application: a client or service registered with exact redirect, audience and grant scope.
- Resource: a typed object or service protected by a policy.
- Group: a realm-local relationship set with explicit nesting semantics.
- Role binding: an authorization relationship, not an identity attribute.
- Issuer: an authority whose exact keys and status must be admitted.
- Federation link: a bounded trust contract between two realms.

One human can hold multiple realm-specific memberships. One device can host multiple workload identities. A machine identity does not prove what software is running. A workload identity does not prove a human operator has authorized its action.

### 4.3 From role labels to proof-carrying authority relationships

Every positive decision should be attributable to one coherent authority path:

subject → authenticated key/credential → admitted issuer → realm membership → group/role/delegation → exact resource + action + audience + purpose → current policy epoch → decision.

Do not construct a synthetic permission by taking a resource from one grant, an action from another, an audience from a third and an expiry from a fourth. The full request context must be satisfied by a coherent capability/policy witness. This must reuse the existing #4743 full-context applicability boundary and #4854 decidable constraint-subsumption / deny-preservation work. Directory Fabric should define realm-scoped relationship projections and input bindings, not another general-purpose policy algebra. Group nesting, role composition and federation mapping must be shown to preserve the parent policy's complete allow and deny denotations.

The request context should also carry explicitly sourced device posture, workload identity, network/location and risk signals where a realm policy chooses to use them. These are contextual evidence with a named issuer, freshness and confidence profile, not magical trust attributes and not automatically authoritative merely because an IdP reports them. This is the interoperability seam to compare with Entra Conditional Access; do not claim identical capability from a simple roles/claims mapper.

### 4.4 From instantaneous revocation claims to bounded exposure proofs

A revocation record, central status flag or successful DHT publish is not proof that every session, PEP, token, worker, connector and application has stopped accepting old authority.

Model each enforcement surface:
- effective revocation time under the realm's trusted time profile;
- time observed by that surface;
- cached epoch and lease expiry;
- last successful freshness check;
- last accepted effect;
- maximum stale-authority exposure;
- recovery after partition/restart.

High-risk operations require fresh authority evidence and fail closed when that evidence is missing. Lower-risk offline operations may continue only under a predeclared lease with explicit maximum staleness. This is bounded availability, not global consistency.

### 4.5 From a global profile to minimum-disclosure identity

Do not replicate full employee records or group membership graphs to every peer. Store private attributes locally or in explicitly encrypted systems. Share only the minimum statement needed by the verifier, with an exact audience, purpose, nonce, expiry and policy reference.

Public hashes and links can still leak existence, relationship and timing metadata. Privacy review must include DHT actions, links, indexes, logs, telemetry and backups — not only entry payload bytes.

### 4.6 From a network-dependent login path to local-first enforcement

A user should not need a reachable DHT quorum for every local file or application access check. Each protected resource has a local Policy Enforcement Point (PEP). It evaluates a signed snapshot with explicit freshness and lease rules. Network events update the snapshot asynchronously. A decision that cannot prove required freshness becomes Indeterminate and is denied for the relevant risk tier.

### 4.7 From implicit federation to explicit bilateral trust

Realm A does not automatically trust every identity, group or role from Realm B. A federation contract states:
- accepted issuer and key profile;
- accepted subject namespace and proof types;
- permitted audiences, resources, actions and purposes;
- mapping rules and maximum delegation depth;
- expiry and revocation behaviour;
- disclosure minimization;
- conflict and rollback handling;
- the target realm's right to reject the mapped authority.

A foreign identity is not automatically a local member. Cross-realm authentication is not cross-realm authorization.

## 5. Holochain's correct role

Holochain has useful architectural properties for this design: each agent maintains a signed source chain; public data can be published to a DHT and validated by peers; private data remains local; separate DNAs define separate application networks. Holochain's own model is agent-centric rather than a single globally authoritative server.

Recommended division:

| Concern | Holochain mapping | Avoid |
|---|---|---|
| Administrative history | Signed source-chain actions by authorized realm agents | Treating any agent key as a person or administrator |
| Public issuer/realm discovery | Minimal validated public entries and links | Publishing a full employee/group directory |
| Private attributes | Local/private storage or client-side encryption with explicit key distribution | Assuming private entry data is replicated confidentially |
| Membership proof | Signed issuer statement and policy-bound reference | Treating DHT presence as membership currentness |
| Federation contracts | Explicit signed realm-to-realm statements | Auto-trusting peers or global reputation |
| Zome-call protection | Holochain capability grants for remote function-call access | Treating CapGrant as a complete application authorization model |
| Local authorization | Local PEP/PDP using cached verified policy and lease | A network/DHT query per protected action |
| Decision evidence | Minimal signed decision receipts | Logging secrets or claiming a receipt proves an external effect |

### Critical Holochain constraints

1. **DHT visibility is asynchronous.** A peer's current view can be incomplete. A missing update is not proof that no update exists.
2. **Host capabilities are a call boundary, not the entire policy model.** Holochain checks whether a caller can reach a zome function. The application must still check the exact realm, principal, resource, action, audience, purpose, epoch and delegation.
3. **The author-grant case matters.** Components sharing the same agent key may be able to call without a remote grant; semantic authorization must also cover local/cross-cell paths.
4. **Capability revocation is not retroactive erasure.** Already disclosed data cannot be made unknown again. A removed grant must be tested at the actual host enforcement point, not just in an application projection.
5. **Version migration is security-sensitive.** Holochain 0.7.1 changes capability-grant types and requires existing 0.7.0 grants to be re-issued; direct signals also require explicit grants. The exact runtime/HDK/DNA compatibility matrix must be frozen before choosing an implementation target.
6. **Do not make the DHT a password or secret store.** Never write password hashes, private keys, Kerberos long-term secrets, raw CapGrant secrets, session tokens or unencrypted sensitive attributes to public DHT records.

Use Holochain to distribute independently authored and validated statements — not as an assumption of globally serializable directory state.

## 6. Core data and decision contracts

The first reference model should use typed identifiers and explicit versions. Suggested logical types:

- RealmId, RealmEpoch, RealmPolicyDigest
- PrincipalId, DeviceId, DeviceInstanceId, WorkloadId, ApplicationId
- IssuerId, IssuerKeyId, CredentialId, CredentialStatus
- GroupId, MembershipId, RoleId, RoleBindingId, DelegationId
- FederationLinkId, ResourceId, AudienceId, PurposeId
- LeaseId, DecisionId, EffectReceiptId

### Authority statement

An authority statement should bind:
- schema and policy version;
- realm and realm epoch;
- issuer identity and admitted signing-key epoch;
- subject type and identifier;
- relationship type and exact scope;
- resource/action/audience/purpose constraints;
- issue time, not-before and expiry under a named clock profile;
- parent delegation or membership lineage;
- revocation/currentness reference;
- canonical payload digest and signature.

Do not use an empty expiry to mean "never expires" by default. Every credential and delegation type must choose an explicit lifecycle.

### Authorization request

An authorization request is the complete tuple:

~~~text
(realm, subject, subject-key-epoch, workload, resource, action,
 audience, purpose, context, policy-epoch, evaluation-time)
~~~

The evaluator may return:
- Allow — every required predicate has a coherent witness.
- Deny — a known predicate is false or an explicit deny applies.
- Indeterminate — evidence, freshness, issuer state, policy or required observation is incomplete/unavailable.

For enforcement, Indeterminate is non-allow. Preserve the reason; do not collapse it into Deny in audit output.

### Local policy snapshot / lease

A local snapshot binds:
- realm and policy epoch/digest;
- full signed policy material or an authenticated content-addressed reference;
- source and signer identities;
- issuer/key admission profile;
- subject/group/resource index version;
- revocation frontier / last integrated authority event;
- issued-at, expires-at and trusted-clock profile;
- allowed risk tier and actions while offline;
- rollback protection and the local high-water mark.

A lease is never a universal "user still valid" token. It is scoped to a specific realm, subject, action/resource class and audience, with a maximum age. High-risk privileges need shorter freshness horizons than low-risk read-only access.

### Decision versus effect

Keep the evidence chain explicit:

~~~text
request
→ authenticated subject/workload
→ issuer and credential qualification
→ realm membership and relationship resolution
→ policy evaluation
→ signed authorization decision
→ exact effect request
→ external execution
→ independent observation/reconciliation
~~~

A signed Allow does not prove that a file was read, a payment settled or a machine changed. An unknown external result must remain unknown until reconciled.

## 7. Protocol interoperability map

| Protocol / system | Correct responsibility | Directory Fabric strategy |
|---|---|---|
| SCIM 2.0 | Provision users and groups across systems | Implement pinned RFC 7642/7643/7644 profiles; include RFC 9865 cursor pagination, RFC 9944 device/EndpointApp resources and RFC 9967 asynchronous security events where supported |
| OIDC / OAuth 2.0 | Application sign-in and delegated API tokens | Integrate standard authorization server/client profiles; use current OAuth security BCP; bind tokens to audience and sender where appropriate |
| SAML 2.0 | Enterprise application federation | Adapter for legacy app ecosystems; do not make it the internal authority database |
| LDAP v3 | Directory object search and legacy read/write interface | Read-only projection first; strict mapping of object, group and ACL semantics; no unsupported semantic claims |
| Kerberos V5 | Ticket-based network authentication | Separate KDC / trust-bridge architecture; never claim that a Holochain AgentPubKey is a Kerberos principal or that DHT records issue tickets |
| SPIFFE/SPIRE | Workload identity and trust-domain federation | Prefer interoperable SVIDs and bundles over a new workload identity format; map to a Mycelix machine/workload subject without conflating it with operator authority |
| SSH certificates | Short-lived user/host SSH authentication | Evaluate an SSH CA adapter with realm-scoped principals and revocation/freshness policy |
| AD DS / Entra / FreeIPA | Existing directory / IdP | Import, synchronize and shadow-compare first; incumbent remains enforcement authority until a separate cutover qualification |
| Group Policy / MDM | Endpoint desired state and configuration enforcement | Future policy-agent integration; directory membership does not itself execute device policy |

SCIM is provisioning, not authentication. OAuth tokens are not unlimited delegation. SAML assertions are not Mycelix realm membership without a profile and issuer-admission rule. LDAP compatibility is not equivalent to AD DS compatibility. A protocol adapter must preserve unknown/unsupported semantics rather than silently flattening them.

## 8. Compatibility and migration strategy

### Phase 0 — Observe only

Connect to a test AD DS, FreeIPA or other incumbent directory. Import a read-only snapshot. Normalize users, groups, devices and service principals without changing them. Compare Mycelix's computed authorization decision with incumbent decisions. Record disagreements; never enforce from Mycelix.

### Phase 1 — Provisioning and identity bridge

Implement SCIM import/export for a pinned profile. Support create/update/deactivate and groups. Bind every external object to its source system, external identifier, mapping version and last-seen cursor/event. Deprovisioning must invalidate sessions/tokens through each effect-capable path, not only mark a SCIM resource inactive.

### Phase 2 — OIDC and workload identity

Expose OIDC for a pilot application and integrate SPIFFE for one service-to-service path. Use proof-of-possession or mutual-TLS-bound tokens where appropriate. Map claims to realm-local subjects only through an explicit issuer profile and policy.

### Phase 3 — Local PEP

Protect one Linux service/resource with a Rust local PEP. Evaluate cached realm policy locally, bind each decision to exact context and currentness, and test process restart and network partition. Add one low-risk read path before any write path.

### Phase 4 — Limited production pilot

Run in dual-control or shadow mode. Require decision agreement, operational recovery, admin review and external security review. Start with a narrowly scoped service and a rollback path. Do not disable the incumbent directory.

### Phase 5 — Legacy Windows / Kerberos expansion

Only begin after separate protocol and operational work for KDCs, DNS, PKI, machine accounts, group policy, offline logon and Windows client behaviour. A directory data model alone does not deliver these features.

## 9. Benchmark programme

### 9.1 Correctness gates — hard gates, not performance scores

- Zero false accepts on the frozen corpus and a separately frozen held-out corpus.
- 100% known-answer agreement with an independently authored oracle.
- Zero secret/private attribute leakage into public DHT payloads, links, logs, telemetry and receipts.
- Zero privilege expansion through nested groups, role composition, federation or subdelegation.
- Zero stale high-risk access after the required freshness lease expires.
- Every revocation run reports effective exposure for each live enforcement surface.
- Every unknown or missing authority input remains non-allow.
- All outputs bind exact source SHA, DNA hash, policy/schema/corpus digest, Holochain/HDK version, clock profile, trust roots, topology, hardware and evaluator identity.

One false accept is a hard correctness failure even if all latency targets are met.

### 9.2 Initial scale profiles

| Profile | Principals | Relationship edges | Conductors | Purpose |
|---|---:|---:|---:|---|
| Smoke | 1,000 | 10,000 | 3–5 real | Basic multi-agent path |
| Development | 10,000 | 100,000 | 20 real | Churn, cache and nested groups |
| Qualification | 100,000 | 1,000,000 | 100 real | Scale and fault qualification |
| Stress | 1,000,000 | 10,000,000 | 500+ real | Capacity and failure-domain study |

These are workload design points, not statements that current Holochain deployments have been proven at these sizes. Simulated peers must be reported separately from real conductors.

### 9.3 Performance targets to challenge

Provisional engineering goals for a declared reference host and policy profile — not current measurements and not yet competitive claims:

| Metric | Initial target | Conditions |
|---|---:|---|
| Warm local PEP/PDP decision | p95 ≤10 ms; p99 ≤25 ms | Excludes human MFA and external IdP |
| Healthy-mesh revocation visibility | p95 ≤5 s; p99 ≤30 s | Fixed topology and offered load; report tail failures |
| Known-answer correctness | 100% | Independent oracle and frozen/held-out fixtures |
| False authorization | 0 | Any false accept fails the profile |
| Sensitive public-DHT disclosure | 0 | Inspect payloads, links, logs and telemetry |
| Offline high-risk action after lease expiry | 0 accepted | Include restart and clock uncertainty |
| SCIM deprovision | Measure event-to-session/PEP effect | Include refresh tokens and background workers |

For each target, publish a baseline first. If the baseline cannot meet a target, report the result and revise the architecture — do not loosen the security invariant to make the graph green.

### 9.4 Required measurement dimensions

- PEP/PDP p50, p95, p99 and max latency under warm and cold caches.
- Offered load, accepted throughput and overload behaviour; avoid only closed-loop throughput.
- Group membership and nested-group resolution cost.
- SCIM provisioning/deprovisioning throughput and cursor recovery.
- Revocation propagation and measured stale authority exposure across every PEP/session/token/worker/connector.
- DHT bytes, gossip bandwidth, peer CPU, memory and storage.
- Restart/recovery time and lost or duplicated events.
- Offline behaviour by risk tier and lease age.
- Admin labour, number of control-plane dependencies, operational cost and recovery complexity.
- User and operator usability: enrollment time, helpdesk recovery, lockout rate and safe key rotation.

Run at least 10 independent repetitions for headline results; publish raw distributions, hardware, software, runtime, topology and exact revision. Keep p50/p95/p99 from separate process runs rather than averaging tail percentiles.

### 9.5 Matched industry comparisons

| Comparator | What to compare | What not to claim |
|---|---|---|
| Microsoft AD DS | LDAP/group semantics, Kerberos/Windows logon, replication, delegation and admin experience | Do not compare a Holochain DHT lookup to a full AD domain controller and call it replacement |
| Microsoft Entra ID | OIDC/SAML sign-in, conditional access, identity governance and device integration | Cloud login features are not equivalent to decentralized data ownership |
| FreeIPA | LDAP/Kerberos/DNS/CA integration, multi-master behaviour and Linux operations | A membership graph alone is not a FreeIPA domain |
| Keycloak / Zitadel / Authentik | OIDC/SAML interoperability, token issuance, client management and operator workflow | Authentication-server latency is not the same work unit as DHT convergence |
| SPIRE | Workload attestation, SVID issuance, rotation and federation | SPIFFE workload identities do not replace human/organizational directories |
| OpenFGA / SpiceDB | Relationship-based authorization semantics, graph traversal, consistency and latency | A Zanzibar-style tuple evaluation does not provide identity proof, issuer admission or Holochain distribution |
| Holochain baseline | DHT convergence, validation, partition tolerance, peer churn and local-first operation | Simulated peers or eventual data visibility are not proof of production availability |

Only compare like-for-like contracts: same principal/group corpus, same operation semantics, same hardware class, same consistency/freshness policy, same offered load and same failure injection. Separate correctness, throughput, operational burden and decentralization/sovereignty; do not hide them in a single score.

## 10. Adversarial qualification matrix

The companion corpus at docs/benchmarks/MYC_DIR_FABRIC_V0_1.json freezes initial expected decisions for:
- identity without realm membership;
- unadmitted issuer and stale issuer key;
- cross-realm access without a federation contract;
- nested-group cycles and role-composition escalation;
- authorized positive controls;
- delayed revocation and bounded offline leases;
- administrator key rotation and quorum disagreement;
- workload audience/trust-domain mismatch;
- machine clone / unsupported hardware uniqueness;
- SCIM deprovision with residual sessions;
- exact resource/action mismatch;
- break-glass approval and expiry;
- policy rollback;
- private-data publication;
- replayed verifier challenge;
- unknown downstream effect outcome.

The corpus is synthetic and finite. Passing it later would prove conformance only to the exact corpus/profile, not universal identity safety.

## 11. Security and privacy non-negotiables

- No central operator may silently become root authority for all realms.
- No global reputation score may mint organization membership or administrator rights.
- No public DHT publication of secrets or private staff attributes.
- No "missing record means no revocation" shortcut.
- No clock timestamp is trusted without a named time profile and uncertainty bound.
- No cached Allow survives its declared expiry for an action requiring fresher proof.
- No protocol token becomes blanket authority.
- No role/group mapping may widen privileges.
- No emergency bypass without explicit approval, narrow scope, expiry and reconciliation.
- No machine uniqueness or hardware attestation claim without a real attestation verifier and evidence.
- No successful authorization receipt is mislabeled as proof of external execution.
- No deployment or compliance claim from schema/unit tests alone.

## 12. Next engineering sequence

1. Review and freeze this architecture and the independent corpus with #6428.
2. Reconcile identity API and DNA versions across the standalone and workspace mirrors.
3. Reuse #4743 and #4854 for full-context authorization and policy constraint subsumption; add only the Directory Fabric realm/relationship projection and exact evidence binding. Define deterministic Rust reference types for that projection; no Holochain dependency in the pure policy core.
4. Add property-based, mutation and metamorphic cases for graph/role/lease/revocation semantics.
5. Connect the reference model to the existing Holochain identity and authority zomes.
6. Add true multi-agent conductor tests for delayed visibility, key rotation, revocation and partition behaviour.
7. Implement SCIM read-only ingestion and OIDC shadow mapping before any write-capable adapter.
8. Benchmark against named incumbent implementations on matched profiles.
9. Only then trial local enforcement for one low-risk resource.

Do not wait for hosted CI to write and validate the design corpus, reference model and local tests. But never describe a skipped/queued CI run, a static check, or a synthetic peer simulation as a runtime qualification pass.

## 13. References

- Microsoft AD DS logical model: https://learn.microsoft.com/en-us/windows-server/identity/ad-ds/plan/understanding-the-active-directory-logical-model
- Holochain agent-centric architecture: https://www.holochain.org/how-does-it-work/
- Holochain capability-based security: https://developer.holochain.org/build/capabilities
- Holochain source-chain querying: https://developer.holochain.org/build/querying-source-chains/
- Holochain 0.7.1 upgrade notes: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- SCIM core protocol: RFC 7644, https://www.rfc-editor.org/rfc/rfc7644
- SCIM cursor pagination: RFC 9865, https://www.rfc-editor.org/rfc/rfc9865
- SCIM device schema extensions: RFC 9944, https://www.rfc-editor.org/rfc/rfc9944
- SCIM Security Event Token profile: RFC 9967, https://www.rfc-editor.org/rfc/rfc9967
- OAuth 2.0 Security Best Current Practice: RFC 9700, https://datatracker.ietf.org/doc/html/rfc9700
- OAuth DPoP: RFC 9449, https://datatracker.ietf.org/doc/html/rfc9449
- OAuth mutual-TLS client authentication: RFC 8705, https://datatracker.ietf.org/doc/html/rfc8705
- SPIFFE federation: https://spiffe.io/docs/latest/spiffe-specs/spiffe_federation/

---

**Decision requested from reviewers:** agree or amend the realm/authority model, local enforcement and lease boundary, privacy classification, v1 protocol perimeter, and benchmark profile before an implementation API is frozen.
