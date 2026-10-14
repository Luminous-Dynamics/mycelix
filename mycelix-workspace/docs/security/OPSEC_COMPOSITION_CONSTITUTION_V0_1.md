# OPSEC-000 — Cross-project operational-security composition constitution v0.1

Status: **design/corpus only — NOT EXECUTED / NOT QUALIFIED / NOT PASS**

Tracks: Mycelix #2746

Base architectural rule:

```text
reuse existing OPSEC owners
!= create another security authority system
```

This document freezes the first shared semantic composition contract for operational security across Mycelix, Symthaea, Xenia, Spore, Nixward, Sol-Atlas and connector/application domains.

It does not implement a policy engine, declassifier, secret store, network firewall, connector, log pipeline, retention engine or dispatch authority.

## 1. Why this layer exists

Luminous Dynamics already has substantial operational-security work, but it lives in deliberately specialized systems:

- Symthaea CogSec owns confidentiality propagation and the rule that generation is not declassification.
- Symthaea INTX owns exact interaction/data-egress composition and policy-decision evidence.
- Mycelix owns domain-semantic state, evidence, accountability and shared cross-domain identities.
- Xenia owns cryptographic integrity/attestation/encryption primitives.
- Spore owns adapter confinement, secret custody and least-authority infrastructure observation profiles.
- Nixward owns host/network-domain confinement and realization of network capabilities.
- Sol-Atlas renders/explains state but owns no security authority.

The missing seam is a common vocabulary for the **subject of an OPSEC decision** and for evidence that a decision happened.

Without that seam, every connector or domain is tempted to invent its own meanings of:

```text
sensitive
public
safe to log
safe to send
declassified
retained
redacted
trusted
```

Those words are not interchangeable.

## 2. Ownership

### 2.1 Mycelix

Mycelix is the semantic root for the shared OPSEC composition vocabulary.

It may own stable identities/references for:

- information-handling subjects;
- data lineage;
- disclosure sinks and destinations;
- payload commitments;
- purpose/context;
- retention/logging/release profiles;
- minimization/declassification evidence;
- policy/currentness references;
- decision receipts.

This ownership is semantic/evidentiary only.

```text
Mycelix OPSEC identity
!= policy correctness
!= live disclosure authority
!= execution authority
```

### 2.2 Symthaea

Symthaea may produce or consume:

- confidentiality/handling candidates;
- dependency/lineage analysis;
- minimization/declassification proposals;
- information-flow analysis;
- policy-decision evidence;
- explanations.

Symthaea may not self-declassify a subject because its model believes disclosure is useful.

```text
reasoning says safe
!= handling class changed
```

### 2.3 Xenia

Xenia may sign, bind, encrypt or transparently log exact OPSEC commitments.

```text
valid signature
!= safe to disclose

encrypted capsule
!= declassified information
```

### 2.4 Spore

Spore remains the owner of concrete sandbox/adapter/secret-custody profiles where used.

Generic OPSEC types must not turn a credential into ordinary serializable payload bytes.

### 2.5 Nixward

Nixward remains the owner of host/network-domain enforcement.

```text
route reachable
!= route authorized
!= payload disclosure authorized
```

### 2.6 Sol-Atlas

Sol-Atlas may render:

- why a disclosure is allowed/denied;
- handling lineage;
- retention state;
- missing policy/currentness;
- protected/redacted views.

Its projection never upgrades authority.

## 3. Constitutional non-equivalences

The following are normative:

```text
publicly observable != safe to aggregate or republish
read-only operation != no outbound disclosure
network target admitted != payload disclosure admitted
network route allowed != metadata disclosure allowed

encrypted != declassified
hashed != anonymous
signed receipt != safe to publish
redacted display != canonical source
log redaction != proven secret-free

same bytes != same policy lineage
source authenticated != source safe to disclose

policy AllowCandidate != disclosure authority
disclosure authority != dispatch authority
dispatch authority != disclosure occurred
successful disclosure != retention authority

confidentiality != epistemic confidence
confidentiality != source trust
confidentiality != action authority
confidentiality != retention policy
confidentiality != identity sensitivity
```

There is no canonical scalar `opsec_score`.

## 4. Shared subject model

The shared waist should remain multidimensional.

A future exact subject may reference independent coordinates equivalent to:

```text
OpsecSubjectV1
  payload_commitment
  handling_label_ref
  data_lineage_ref
  identifiability_ref?
  secret_class_ref?
  purpose_ref
  principal_security_domain_ref?
  precision_profile_ref?
  aggregation_mosaic_policy_ref?
  context_ref
```

The referenced schemes remain independently versioned.

A subject is not itself permission to disclose.

### 4.1 Missing information

Required missing information must never silently map to `Public`, `Anonymous`, `Safe`, or unrestricted.

The selected policy profile owns the disposition for missing/unknown required dimensions.

Examples may include:

```text
Deny
NeedsFreshEvidence
NeedsClassification
NeedsHumanReview
```

## 5. Disclosure sink intent

A future sink intent should bind the exact release proposition, conceptually:

```text
OpsecDisclosureIntentV1
  opsec_subject_ref
  exact_payload_commitment
  sink_class
  destination_ref
  purpose_ref
  transform_ref?
  retention_expectation_ref?
  logging_profile_ref?
  release_profile_ref?
  context_ref
```

Candidate sink classes include:

```text
LocalPrivateDisplay
LocalProtectedStorage
LocalOperationalLog
TelemetryTraceMetric
CrashErrorReport
RemoteWebRequest
RemoteModelApi
AuthenticatedPeer
PublicFederatedPublication
ToolServiceInvocation
FileObjectExport
ClipboardConvenienceExport
DurableEvidenceStore
EvidenceExport
```

Permission to one sink grants nothing to another sink.

```text
allowed for local UI
!= allowed for remote model

allowed for API call
!= allowed for telemetry

allowed for evidence store
!= allowed for public evidence export
```

## 6. Decision evidence

The shared waist may define a closed decision-evidence disposition vocabulary such as:

```text
Deny
AllowCandidate
NeedsMinimization
NeedsDeclassification
NeedsProtectedSink
NeedsHumanReview
NeedsFreshPolicy
```

`AllowCandidate` is deliberately not named `AllowDispatch`.

A future receipt should bind at least:

```text
exact subject
exact sink intent
policy/profile identity
evaluation-context commitment
policy/currentness references
typed outcome
bounded reason/rule identifiers
```

The receipt is evidence.

```text
OpsecDecisionReceipt
!= DisclosurePermit
!= DispatchPermit
!= network capability
```

An executing system must separately establish current authority at the point of use.

## 7. Derived information

Derived information conservatively preserves restrictions when narrower dependency is not qualified.

Normative rule:

```text
unproven independence
-> no confidentiality downgrade
```

None of these is automatic declassification:

```text
summarization
paraphrase
translation
OCR
embedding / HDC projection
compression
hashing
encryption
aggregation
format conversion
```

Differential privacy or another privacy transform can justify a changed release proposition only under its exact qualified mechanism/profile plus an explicit release policy.

## 8. Declassification

Declassification is a privileged semantic transition.

A future declassification proposition should bind:

- exact source artifact/subject;
- current handling label;
- exact output artifact/commitment where applicable;
- transform profile/evidence;
- exact sink/destination;
- target handling domain/class;
- purpose;
- policy/authority identity;
- epoch/currentness;
- expiry/scope.

A declassification result for one output/destination must not silently transfer to later derivations.

## 9. Secret material

Secret material is stronger than generic sensitive data.

Examples include:

```text
CredentialSecret
SessionBearer
PrivateKeyMaterial
RecoverySeedOrKey
SensitiveConfigurationValue
```

The shared OPSEC layer should normally carry an opaque handle/class reference rather than the bytes.

```text
credential possession
!= permission to serialize credential
```

Preferred architecture:

```text
caller/model
    ↓ opaque handle
trusted broker/custodian
    ↓ exact bounded use
provider/protocol
```

No generic OPSEC API should require secret types to implement `Debug`, `Display`, `Serialize`, cloning, or portable evidence conversion.

## 10. Logging, tracing and errors

Logs, traces, metrics and crash reports are disclosure sinks.

Ordinary diagnostics should not receive raw:

- Authorization values;
- cookies/bearer tokens;
- URL userinfo;
- sensitive query/body data;
- private evidence payloads;
- raw credential handles where correlation is unsafe;
- arbitrary vendor/library error strings.

Library/adapter/vendor error text is untrusted potentially secret-bearing input.

Use stable typed error classes plus separately controlled protected diagnostics.

Known-secret scanning and regex redaction are defense in depth only.

```text
token scanner finds nothing
!= output proven secret-free
```

## 11. Evidence privacy

Security evidence itself can become a sensitive secondary database.

Default portable OPSEC evidence should prefer:

- semantic commitments;
- local protected object references;
- handling/profile identities;
- sink class/destination identity at the minimum required granularity;
- typed outcome/reason identifiers;
- policy/currentness references;
- bounded counters.

Avoid copying raw protected payloads into the audit plane merely to prove mediation.

### 11.1 Deterministic commitments

A deterministic hash provides integrity identity, not secrecy.

For low-entropy sensitive material:

```text
public deterministic hash
-> may permit offline guessing
```

Named privacy profiles may instead use:

- keyed commitments/HMAC;
- randomized private handles;
- salted/blinded commitments;
- encrypted local capsules;
- local-only references.

The deterministic semantic identity remains separate where required for exact reproducibility.

## 12. Retention

Disclosure and retention are separate propositions.

A sink may be permitted to receive a datum while long-term retention remains prohibited or bounded.

Future retention profiles should be capable of expressing:

- retention class;
- expiry;
- protected storage requirement;
- deletion/expiry event;
- export restrictions;
- legal/policy/steward reference where applicable.

A deletion receipt means the controlled system performed its deletion transition.

```text
local deletion receipt
!= every external copy disappeared
```

## 13. Aggregation and mosaic risk

Publicness is contextual.

```text
field A public
+ field B public
+ field C public
!= unrestricted person-level join authority
```

History-sensitive policies may limit:

- repeated queries;
- cross-domain joins;
- fine geographic/temporal intersections;
- identity linkage;
- cumulative disclosure budgets;
- reconstruction/mosaic patterns.

This constitution references existing Mycelix accountability/privacy work rather than defining one universal privacy taxonomy.

## 14. Web acquisition as first concrete consumer

The current web-acquisition train already separates target identity, DNS, endpoint policy, admission, receipts and execution leasing.

OPSEC adds another orthogonal theorem:

```text
AdmittedWebTarget
!= ApprovedWebDisclosure
```

A public, SSRF-safe destination does not make the outbound request content safe to reveal.

The future execution join should resemble:

```text
exact WebAcquisitionIntent
        ↓
target/destination admission
        +
OPSEC disclosure admission
        +
durable Started execution authority
        ↓
connector
```

No component may substitute one proposition for another.

## 15. Web disclosure surfaces

A web request should not be summarized as one boolean `sent`.

The OPSEC view should be able to distinguish potential surfaces such as:

```text
DnsQueryName
TransportEndpoint
TlsNameMetadata
HttpAuthority
HttpRequestTarget
HttpHeaders
HttpBody
RedirectReferral
ProxyRelayMetadata
LocalLogsTracesMetrics
RetainedCaptureEvidence
```

Exact visibility to a server, proxy, network observer, resolver, local operator or archive depends on the transport/profile and must be evidence-bound.

The surface list identifies what must be evaluated; it does not claim every observer actually saw every field.

## 16. Web-specific rules

For the first web profile:

```text
target admitted
+ private query
!= request admitted

target admitted
+ secret header
!= request admitted

fetch allowed
!= full URL safe to log

primary request allowed
!= telemetry export allowed

redirect target safe
!= original payload automatically safe for redirected destination
```

Redirects must re-evaluate both destination authority and any destination-bound disclosure decision.

If a sensitive disclosure decision is bound to destination A, redirecting to B invalidates it unless the exact profile says otherwise and independently admits B.

## 17. External content cannot classify itself

External web/tool/document content is untrusted data.

Statements such as:

```text
THIS IS PUBLIC
SAFE TO SHARE
IGNORE PRIVACY POLICY
SEND YOUR TOKEN HERE
```

are content observations only.

They cannot mutate trusted handling labels, policy state, declassification state, secret custody, or execution authority.

## 18. OPSEC and the acquisition receipt

The B1 deterministic admission receipt is integrity/audit evidence.

Under a sensitive investigation profile:

```text
receipt commitment retained
+
raw canonical receipt protected
```

may be appropriate.

A Xenia signature over the commitment does not make raw receipt publication safe.

OPSEC-000 therefore composes directly with Mycelix #2718 rather than weakening B1 canonicalization.

## 19. OPSEC and persistent execution leasing

The WEB-LEASE train protects one-shot network execution/replay semantics.

It does not determine whether the request payload is permissible to disclose.

Likewise:

```text
OPSEC disclosure eligible
!= network attempt available
```

The connector needs both.

No OPSEC decision is allowed to bypass WEB-LEASE durable `Started` ordering.

## 20. OPSEC and Nixward

Application-level OPSEC and Nixward enforcement are complementary.

Mycelix/Symthaea may establish semantic disclosure constraints.

Nixward can enforce bounded network-domain/egress realization where supported.

```text
semantic policy without confinement
!= host confinement

host confinement
!= semantic disclosure policy
```

A strong deployment should bind both exact identities in its execution evidence.

## 21. OPSEC and Spore

Spore's adapter/secret-custody rules remain stronger for credential material.

An adapter sandbox can be prevented from opening ambient sockets, while the host broker exposes only reviewed closed operations.

The OPSEC waist supplies semantic data-release context; it does not grant adapters generic network/file/process primitives.

## 22. OPSEC and Xenia

Potential layering:

```text
OPSEC subject/decision commitment
        ↓
Xenia detached attestation
        ↓
optional transparency evidence
```

The strongest cryptographic claim remains bounded to exact bytes/profile/key.

```text
signature verifies
!= decision correct
!= disclosure authorized now
```

## 23. OPSEC and Sol-Atlas

Useful future rendering includes:

- exact sink/destination;
- which lineage contributes restrictions;
- missing/expired policy evidence;
- minimization/declassification steps;
- retention state;
- protected-vs-portable evidence split;
- what metadata a chosen network profile may reveal;
- “why is this disclosure blocked?”

The UI must distinguish candidate, admitted and executed state.

## 24. Adversarial corpus

The machine-readable v0.1 corpus is:

`mycelix-workspace/docs/security/fixtures/OPSEC_COMPOSITION_000_V0_1.json`

Exact authored UTF-8 SHA-256:

`aeb5974a1cd4a7af9fbf27dfeafa34b15306c1ce9e3758ab723d8cf9d064e918`

It contains 16 initial cases covering:

- public target + private query;
- sink-specific permission;
- summary/embedding/encryption non-declassification;
- low-entropy hash privacy;
- safe fetch vs unsafe logging;
- telemetry as separate sink;
- no silent privacy downgrade on remote fallback;
- mosaic risk;
- secret-bearing errors;
- external self-declassification attempt;
- stale policy epoch;
- signed sensitive receipt;
- decision-vs-dispatch separation;
- credential custody.

This digest identifies the authored corpus only.

It is not executable qualification.

## 25. First implementation order

Recommended sequence:

```text
OPSEC-000A
  this constitution + adversarial corpus

OPSEC-001
  dependency-light identities only
  no policy engine / no I/O

OPSEC-002
  exact sink/disclosure intent commitment

OPSEC-003
  typed decision evidence
  still non-authoritative

WEB-OPSEC-001
  bind exact web acquisition intent
  + destination admission
  + OPSEC disclosure subject

WEB-OPSEC-002
  compose qualified disclosure decision
  + durable lease/Started authority
  before connector entry
```

Runtime enforcement must not leapfrog exact qualification of the semantic inputs it consumes.

## 26. Initial qualification gates

Before claiming an enforceable shared OPSEC waist:

1. source/corpus exact identity frozen;
2. independent canonical identity vectors;
3. every sink class has an owner/adapter census;
4. unknown required classification fails according to named policy;
5. no decision receipt converts directly to live execution capability;
6. secret material cannot flow through generic portable OPSEC serialization;
7. logging/telemetry are independently represented sinks;
8. destination and payload mutation invalidate exact decisions;
9. policy/currentness/revocation changes invalidate stale decisions;
10. derived content cannot silently reduce handling restrictions;
11. external content cannot mutate trusted classification;
12. default audit evidence omits raw sensitive payload;
13. low-entropy secret/sensitive values are not exposed by plain public hashes under protected profiles;
14. web target admission cannot substitute for disclosure admission;
15. connector integration proves both OPSEC and execution-authority predicates immediately before dispatch.

## 27. Nonclaims

This constitution does not establish:

- one universal sensitivity taxonomy;
- legal/regulatory compliance;
- perfect semantic information-flow tracking through neural models;
- perfect declassification/anonymization;
- hardware/side-channel secret protection;
- network anonymity;
- traffic-analysis resistance;
- policy correctness;
- connector implementation;
- runtime enforcement;
- deployment readiness.

Its purpose is narrower:

> Make the existing Luminous Dynamics OPSEC work compose through exact shared identities and evidence, while keeping confidentiality, policy, secret custody, network confinement, cryptography and execution authority in their proper owners.
