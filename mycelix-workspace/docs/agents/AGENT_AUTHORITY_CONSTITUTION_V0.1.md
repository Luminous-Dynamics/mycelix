# Mycelix Agent Authority Constitution v0.1

Status: **normative composition contract**

AGENT-000 defines the cross-domain authority boundary for software agents, AI agents,
human-operated automation, delegated services, and other machine principals that use
Mycelix.

It does not create an agent runtime, grant authority, establish legal personhood,
authenticate a model, select an AI provider, implement A2A or MCP, issue payment
credentials, or enable external effects. It defines invariants that later agent,
identity, commerce, governance, integration, and protocol-adapter work may not weaken.

## Governing theorem

> A consequential machine action is admissible only from explicit principal legitimacy,
> exact agent identity, qualified intent, attenuated and current authority, exact action
> binding, and an independently enforced effect boundary. Model output, intelligence,
> reputation, protocol presence, runtime attestation, or Mycelix participation cannot
> create authority by themselves.

The model may propose. The authority system decides whether the proposal fits an
already legitimate and current authorization.

## Eight planes remain distinct

### 1. Principal / legitimacy plane

Defines the human, organization, institution, or other legitimate authority source on
whose behalf an agent may act.

```text
PrincipalIdentity != AgentIdentity
ControllerRelationship != DelegatedAuthority
MycelixParticipation != Legitimacy
```

### 2. Agent identity plane

Defines a stable agent principal independently from any particular model process,
runtime instance, host, or session.

```text
StableAgentPrincipal != RuntimeInstance
AgentIdentity != ModelIdentity
AgentCard != Authority
```

### 3. Runtime / provenance plane

Describes the concrete workload, model, software, host, build, key material, and
attestation evidence used for one execution context.

```text
RuntimeAttestation != Authority
ModelProvenance != Authority
SupplyChainEvidence != Authority
```

A policy may require a runtime assurance level before authority can be exercised.
Satisfying that evidence requirement does not create the underlying authority.

### 4. Intent plane

Represents what a principal has actually authorized an agent to pursue.

```text
Mission != Intent
IntentProposal != QualifiedIntent
QualifiedIntent != AuthorityGrant
```

Natural-language interpretation is untrusted input to authorization. A model may
propose a typed intent, but consequential authorization requires deterministic
validation and the configured human/institutional approval rule.

### 5. Authority plane

Represents the exact capability, resource, audience, time interval, delegation lineage,
current generation, and consumable budget available to an agent.

```text
Authentication != Authorization
Competence != Authority
Reputation != Authority
Intelligence != Authority
```

### 6. Action plane

Binds authority to the exact consequential action and canonical action profile.

```text
AuthorizedActionClass != QualifiedExactAction
QualifiedIntent != QualifiedExactAction
```

A model assertion such as `I complied with the user's intent` is not action
qualification.

### 7. Effect plane

Owns the trusted transition from qualified action to an external service or physical
effect.

```text
QualifiedExactAction != ExternalEffect
DispatchStarted != EffectCommitted
ReturnedError != DefinitelyNotCommitted
```

### 8. Evidence / accountability plane

Preserves enough bounded provenance to reconstruct what was authorized, attempted,
observed, disputed, superseded, revoked, or left indeterminate without requiring
disclosure of hidden reasoning or unrelated private data.

```text
Receipt != Truth
Receipt != ChainOfThought
ObservedOutcome != GoodOutcome
```

## Cross-domain invariants

### AA-001 — legitimacy has an explicit source

Every consequential agent action must identify the principal, institution, or other
recognized authority source on whose behalf it is attempted.

No agent, model, runtime, network position, Mycelix record, benchmark score, or
popularity signal may bootstrap its own legitimacy.

### AA-002 — controller, agent principal, and runtime are separate identities

The system must distinguish at least:

- the controlling or mandating principal;
- the stable agent principal; and
- the concrete runtime/workload instance.

Changing a runtime or model must not silently rewrite historical agent identity.
Conversely, persistence of stable agent identity must not cause a newly substituted
runtime to inherit runtime-sensitive authority without requalification.

### AA-003 — model and runtime provenance are evidence, not authority

Model identity, version, weights digest, provider, runtime build, software supply-chain
state, workload identity, hardware attestation, and similar provenance may satisfy
policy requirements.

They cannot independently mint permission to act.

### AA-004 — mission is not intent

A natural-language mission, prompt, conversation, or inferred goal is not itself a
machine-enforceable authorization.

Consequential use requires a typed intent whose semantics are explicit enough for an
independent implementation to evaluate.

### AA-005 — an intent proposal is not a qualified intent

An LLM or agent may transform a mission into a candidate constraint set. That
translation remains an untrusted proposal until the configured approval and
deterministic validation rules produce a `QualifiedIntent` or equivalent opaque
capability.

### AA-006 — intent is not authority

A valid intent describes what is wanted or permitted by a principal. It does not prove
that the acting agent possesses the institutional, financial, operational, or resource
authority required to execute it.

Intent and authority must be joined explicitly.

### AA-007 — authentication is not authorization

Proof that an agent, controller, runtime, key, workload, or service is authentic does
not establish that it may perform a consequential action.

Authentication results may contribute evidence to authorization; they do not replace
authorization.

### AA-008 — competence, reputation, intelligence, and confidence are not authority

No benchmark, model score, reputation value, Phi/consciousness signal, stake, social
rank, prediction, confidence, or apparent intelligence may independently create
authority over another principal or resource.

These signals may inform explicitly adopted policy where appropriate, but their role
must remain distinguishable from the legitimacy source.

### AA-009 — consequential authority is exactly scoped

Consequential agent authority must be scoped, where applicable, to an exact:

- actor/agent principal;
- controlling or mandating principal;
- capability/action class;
- resource or subject;
- audience/provider;
- policy/rulebook;
- validity interval;
- delegation lineage; and
- current authority generation.

Broad authority must be attenuated before an exact effect when policy requires it.

### AA-010 — delegation attenuates; it does not amplify

A child delegation may preserve or narrow the parent's delegable scope. It may not add
roles, capabilities, resources, audiences, validity, or onward-delegation rights that
the qualified parent does not possess.

`Delegation != AuthorityMint`

### AA-011 — consumable authority is conserved across fan-out

Subset attenuation alone is insufficient for consumable resources.

For every independently conserved budget dimension, a delegation fan-out must satisfy:

```text
retained_budget(parent)
+ sum(active_child_allocations)
<= qualified_parent_budget
```

Applicable dimensions may include money, transaction count, API quota, compute,
storage, energy, inventory, messages, or another explicitly modeled consumable.

Two children must not each receive the full parent budget merely because each child is
individually within the parent's maximum.

### AA-012 — re-delegation is explicit

Possession of a delegated capability does not imply permission to delegate it again.

Every onward edge must satisfy an explicit re-delegation rule, preserve complete
lineage, remain acyclic, and remain within bounded depth/resource limits.

### AA-013 — historical validity and current authority are separate

A historically valid grant, delegation, credential, signature, or action remains
verifiable as history after later revocation.

New consequential execution requires current authority evidence. Revoked, superseded,
expired, ambiguous, or stale authority cannot be revived from a cached positive result.

### AA-014 — exact actions are canonically bound

A consequential action must be represented under a registered canonical profile and
bound to the exact action bytes/semantic identity that authority covers.

Changing destination, amount, resource, tool arguments, provider, recipient, policy,
or another authority-relevant field must produce a different action identity or fail
qualification.

### AA-015 — deterministic enforcement decides constraint fit

A model may generate, rank, explain, or recommend actions.

Whether an exact action satisfies a qualified intent, authority envelope, budget, or
policy must be decided by deterministic/verifiable enforcement code for every
security-relevant constraint that can be represented mechanically.

A model may not self-certify that its own proposed action is authorized.

### AA-016 — long-lived secrets remain outside model context

Long-lived passwords, signing keys, wallet seeds, refresh tokens, bearer credentials,
and equivalent reusable secrets should not be placed in model context as the ordinary
agent authority mechanism.

Where integration permits, a trusted broker should retain such material and issue or
exercise short-lived, audience-bound, action/resource-scoped authority after current
qualification.

Possession of an opaque capability handle is not itself proof that the referenced
authority is still current.

### AA-017 — adapters cannot mint or widen authority

A2A, MCP, OAuth, OpenID, FIDO, AP2, payment rails, cloud APIs, DID/VC systems,
attestation frameworks, transparency services, and future adapters are edge protocols.

An adapter may translate qualified Mycelix authority into a protocol-specific request
or credential, and may translate external evidence back into a qualified observation.

It must not silently widen scope, extend validity, invent delegation, erase provenance,
or convert protocol authentication into Mycelix authority.

### AA-018 — qualified action is not an external effect

An exact qualified action grants no proof that an external system accepted, committed,
or applied it.

The effect boundary must independently bind the exact qualified action, provider/route,
attempt identity, current effect policy, and relevant runtime/effect authority before
dispatch.

### AA-019 — effect uncertainty is typed and fail-closed

External calls must not collapse uncertain commit outcomes into a boolean success flag.

At minimum, consequential adapters must be able to distinguish semantics equivalent to:

```text
Committed
DefinitelyNotCommitted
IndeterminateCommit
```

An error or timeout is not evidence that the effect did not occur.

Retry after an indeterminate effect requires reconciliation or an independently safe
idempotency theorem; ambiguity must not mint a second effect.

### AA-020 — receipts preserve accountability without requiring hidden reasoning

A consequential agent action should be able to produce a bounded evidence/receipt
lineage identifying the exact action, agent principal, controller/mandator,
qualified-intent identity, authority/delegation/currentness commitments,
provider/effect attempt, and observed result as required by the owning profile.

The receipt must not require chain-of-thought, hidden scratchpads, raw prompts,
passwords, private keys, or unrelated sensitive context.

### AA-021 — disclosure is minimum-sufficient and purpose-bound

Possession of evidence or authority does not imply universal dereference or disclosure
rights.

Agent protocols should prefer minimum sufficient qualified propositions, selective
disclosure, commitments, or references over copying complete personal, institutional,
or commercial records when policy permits.

### AA-022 — protocol presence and interoperability metadata are not trust roots

Discovery metadata, Agent Cards, tool descriptions, service manifests, MCP resources,
A2A skills, advertised model names, endpoint claims, and similar interoperability
metadata are useful for discovery.

They are not authority merely because they are signed, reachable, popular, or
well-formed. Authority-bearing claims require their designated verification and policy
path.

### AA-023 — consequential action evidence is independently reconstructible

For consequential profiles, Mycelix should support a bounded evidence package from
which an independent verifier can reconstruct the claimed identity, intent, delegation,
current authority, action binding, and effect receipt without requiring the originating
LLM, user interface, or live agent process.

A successful verification establishes only the semantics explicitly covered by the
package and verifier profile.

### AA-024 — model correctness is never a security assumption

The authority architecture must remain safe when a model is mistaken, confused,
prompt-injected, compromised, maliciously instructed, or otherwise behaves contrary to
the controller's interests.

Security-critical boundaries must not depend on a model voluntarily obeying prose
instructions such as `do not exceed this limit` when that limit can be mechanically
enforced.

## Required agent composition

A consequential agent path should converge toward:

```text
Principal / Institution
        ↓
Stable Agent Principal
        +
Runtime / Provenance Evidence
        ↓
Mission
        ↓
Typed Intent Proposal
        ↓
Deterministic validation + required approval
        ↓
QualifiedIntent
        +
Qualified delegated/current authority
        +
Conserved resource budget
        ↓
Exact canonical action
        ↓
QualifiedExactAction
        ↓
Trusted effect broker
        ↓
External attempt
        ↓
Typed effect observation / reconciliation
        ↓
AgentActionReceipt / evidence package
```

No arrow above implies that the upstream artifact automatically grants the semantics of
the downstream artifact.

## Forbidden shortcuts

Later agent profiles must not introduce any of the following as authority shortcuts:

- `model_says_authorized = true`;
- `trusted_agent = true` without a qualified semantic profile;
- a global reputation/intelligence/consciousness score as universal authority;
- `latest wins` for conflicting authority;
- an Agent Card, MCP manifest, DID document, VC, attestation, or API token treated as
  self-interpreting universal authority;
- ambient long-lived secrets exposed to the model as the default execution mechanism;
- unbounded bearer authority when an audience/action/resource bound credential can be
  used;
- silent authority widening during protocol translation;
- duplicate consumable authority created by delegation fan-out;
- automatic retry after an ambiguous external effect;
- chain-of-thought logging as an accountability requirement; or
- autonomous public or private authority derived solely from model capability.

## Relationship to existing Mycelix authority work

AGENT-000 is intentionally cross-domain and documentation-only.

Later executable agent tranches should reuse existing Mycelix foundations where their
qualified semantics match the requirement, including:

- canonical authority identity;
- generation-bound freshness/currentness;
- attenuated delegation and explicit re-delegation;
- exact action scoping;
- authority conservation;
- durable attempt/effect reconciliation;
- identity/provenance observation; and
- evidence/receipt lineage.

AGENT-000 does not claim those draft foundations are all deployed or production-ready,
and it does not duplicate them into an agent-specific authority system.

## Deliberate non-features

AGENT-000 does not introduce:

- a universal AI registry;
- legal personhood for software;
- autonomous authority based on intelligence;
- a model-provider trust hierarchy;
- a universal reputation score;
- a global mutable `trusted` or `authorized` bit;
- one universal policy language;
- a new payment rail;
- a replacement for A2A, MCP, OAuth, FIDO, DID/VC, AP2, EAT/RATS, or SCITT;
- mandatory chain-of-thought disclosure; or
- any external-effect capability.

## Immediate child tranches

The intended sequence begins:

```text
AGENT-000  agent authority constitution
AGENT-001  explicit adversary/threat census
AGENT-002  controller / stable-agent / runtime identity
AGENT-003  runtime/model/supply-chain provenance and attestation
AGENT-004  mission -> typed intent -> QualifiedIntent
AGENT-005  delegation attenuation + consumable-authority conservation
AGENT-006  exact current agent authority
AGENT-007  exact action qualification / deterministic constraint subsumption
AGENT-008  credential-less authority broker
AGENT-009  durable effect attempt + AgentActionReceipt
AGENT-010  standalone/offline independent verifier
```

Protocol adapters should follow the core theorem rather than define it.

## Qualification meaning

A green AGENT-000 check proves only that the normative corpus retains the required
separations, conservation rules, privacy boundary, effect-uncertainty discipline, and
anti-shortcut statements.

It does not establish runtime enforcement, agent security, cryptographic authenticity,
model safety, protocol interoperability, legal validity, payment safety, production
readiness, or authorization for any external effect.
