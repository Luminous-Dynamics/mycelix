# OPSEC-002 — Canonical disclosure-intent commitment v1

Status: **protocol/test-vector subject only — NOT IMPLEMENTED / NOT QUALIFIED / NOT PASS**

Tracks: Mycelix #2752

Parent OPSEC core source subject:

`58a33d8656898b75c9e2330aa191c12d4751491c` (draft PR #2750)

## 1. Purpose

Freeze an implementation-independent identity for an exact proposed disclosure before any
policy decision, declassification, live permit, connector authority, or observed effect exists.

The governing theorem is:

```text
DisclosureIntentCommitmentV1
= exact identity of one release proposition

!= policy evaluation
!= declassification
!= disclosure authority
!= dispatch authority
!= proof that disclosure occurred
```

This protocol intentionally identifies **what would be released, where, why, and under which
handling/policy context**. It does not decide whether the release is permitted.

## 2. Semantic subject

V1 binds the following semantic coordinates, in this exact field order:

1. semantic schema;
2. `opsec_subject_ref`;
3. `payload_commitment_profile_ref`;
4. exact payload commitment bytes;
5. `handling_label_ref`;
6. `data_lineage_ref`;
7. closed sink tag + frozen sink name;
8. `destination_ref`;
9. `purpose_ref`;
10. `context_ref`;
11. `policy_profile_ref`;
12. `policy_epoch_ref`;
13. `retention_profile_ref`;
14. `logging_profile_ref`;
15. `release_profile_ref`;
16. optional `transform_ref`;
17. canonical disclosure-surface set.

Credential, bearer, private-key, recovery, or other secret bytes do not belong in this generic
protocol. Separate broker/custody profiles may be referenced later without normalizing secret
material into this transcript.

## 3. Domains and schemas

Exact transcript domain bytes, including the trailing NUL:

```text
mycelix-opsec-disclosure-intent-v1\0
```

Semantic schema:

```text
mycelix:opsec-disclosure-intent:v1
```

Commitment profile:

```text
mycelix:opsec-disclosure-intent-commitment:sha256:v1
```

Exact commitment domain bytes, including trailing NUL:

```text
mycelix-opsec-disclosure-intent-commitment-sha256-v1\0
```

## 4. Canonical encoding

All integer widths and endianness are normative.

```text
lp_u32_utf8(s)
  = u32be(UTF8(s).len)
    || UTF8(s)

payload commitment
  = u16be(bytes.len)
    || bytes

sink
  = u16be(frozen sink tag)
    || lp_u32_utf8(frozen sink name)

optional transform
  = 0x00
    if absent

  = 0x01
    || lp_u32_utf8(transform_ref)
    if present

surface set
  = u16be(count)
    || each canonical entry

surface entry
  = u16be(frozen surface tag)
    || lp_u32_utf8(frozen surface name)
```

The complete transcript is:

```text
TRANSCRIPT_DOMAIN
|| lp_u32_utf8(semantic_schema)
|| lp_u32_utf8(opsec_subject_ref)
|| lp_u32_utf8(payload_commitment_profile_ref)
|| u16be(payload_commitment_len)
|| payload_commitment_bytes
|| lp_u32_utf8(handling_label_ref)
|| lp_u32_utf8(data_lineage_ref)
|| u16be(sink_tag)
|| lp_u32_utf8(sink_name)
|| lp_u32_utf8(destination_ref)
|| lp_u32_utf8(purpose_ref)
|| lp_u32_utf8(context_ref)
|| lp_u32_utf8(policy_profile_ref)
|| lp_u32_utf8(policy_epoch_ref)
|| lp_u32_utf8(retention_profile_ref)
|| lp_u32_utf8(logging_profile_ref)
|| lp_u32_utf8(release_profile_ref)
|| optional_transform
|| canonical_surface_set
```

The commitment is:

```text
SHA-256(
  COMMITMENT_DOMAIN
  || u64be(transcript_len)
  || transcript
)
```

Neither JSON field order, serde layout, Rust struct layout, host endianness, map iteration, nor
display formatting is canonical authority.

## 5. Sink tags

The v1 sink tag table is frozen:

```text
 1  LocalPrivateDisplay
 2  LocalProtectedStorage
 3  LocalOperationalLog
 4  TelemetryTraceMetric
 5  CrashErrorReport
 6  RemoteWebRequest
 7  RemoteModelApi
 8  AuthenticatedPeer
 9  PublicFederatedPublication
10  ToolServiceInvocation
11  FileObjectExport
12  ClipboardConvenienceExport
13  DurableEvidenceStore
14  EvidenceExport
```

Both tag and frozen name are encoded. A tag/name mismatch is invalid.

## 6. Disclosure-surface tags

The v1 surface tag table is frozen:

```text
 1  DnsQueryName
 2  TransportEndpoint
 3  TlsNameMetadata
 4  HttpAuthority
 5  HttpRequestTarget
 6  HttpHeaders
 7  HttpBody
 8  RedirectReferral
 9  ProxyRelayMetadata
10  LocalLogsTracesMetrics
11  RetainedCaptureEvidence
```

## 7. Surface-set semantics

`disclosure_surface_set` is a mathematical set, not a historical observation sequence.

Therefore canonicalization MUST:

1. reject duplicate surface values;
2. sort the set strictly by frozen numeric tag;
3. encode the sorted set;
4. produce identical transcript bytes for caller orderings that denote the same set.

This differs from evidence protocols where observation order can itself be evidence.

## 8. Mutation law

A prior policy/evidence result bound to this commitment cannot survive a security-relevant
intent mutation unless a later profile explicitly proves a valid relationship.

At minimum these mutate intent identity:

```text
payload commitment
destination
purpose/context
handling/lineage reference
sink
policy profile
policy epoch
retention/logging/release profile
transform presence/value
disclosure-surface set
```

Thus:

```text
Decision(intent=A)
+ intent becomes B
-> decision for A cannot authorize B
```

## 9. Privacy ceiling

The SHA-256 commitment is a deterministic integrity identity.

It is not secrecy.

```text
deterministic commitment
!= high entropy
!= anonymity
!= resistance to dictionary guessing
!= permission to publish commitment
```

Sensitive/low-entropy profiles may use a private handle, keyed commitment, randomized wrapper,
or encrypted capsule for presentation/storage while retaining the canonical internal integrity
identity. This composes with Mycelix #2718 and OPSEC-000 rather than weakening this encoding.

## 10. Relationship to policy decisions

A future OPSEC-003 decision receipt should bind:

```text
exact DisclosureIntentCommitmentV1
+ exact policy/profile/currentness evidence
+ evaluation-context commitment
+ typed disposition
```

The decision remains evidence.

```text
AllowCandidate
!= live disclosure permit
```

A later current-authority adapter may consume qualified decision evidence plus revocation/currentness
state to produce a narrowly scoped permit. That is a separate theorem.

## 11. Relationship to WEB-OPSEC

WEB-OPSEC-001 / #2748 should project web disclosure propositions into this generic protocol.

A single acquisition may need multiple intent identities because release occurs at different
times/sinks:

```text
DNS query-name disclosure
!= TLS/HTTP disclosure
!= local logging
!= telemetry
!= retained capture/evidence
```

Target admission remains orthogonal:

```text
AdmittedWebTarget
!= DisclosureIntentCommitment
!= ApprovedWebDisclosure
```

## 12. Frozen golden vectors

The machine-readable corpus is:

`mycelix-workspace/docs/security/fixtures/OPSEC_DISCLOSURE_INTENT_002_V0_1.json`

Exact authored UTF-8 SHA-256:

`ac003804a3e92a64edb2cb6b36b731cf852f9719e0e0b0e19e22b519e81bfd09`

Frozen vector commitments:

- `remote-web-base` — 580 bytes — `ab07a636915e78db112e17e29ede760b41b667412249b173de75fc9ec4761ba3`
- `same-surface-set-different-input-order` — 580 bytes — `ab07a636915e78db112e17e29ede760b41b667412249b173de75fc9ec4761ba3`
- `destination-mutated` — 577 bytes — `c838d5a00674c21f728161378fe69a2d5e5ac1834990ccab5595ff45b442d182`
- `payload-mutated` — 580 bytes — `ebed6bc57bd66ac1a6e4b0ccdf36645cfaa60d1e8f5c050744b166b8ed037e48`
- `policy-epoch-mutated` — 580 bytes — `64a844c621ec73518fe29968ada970700b6daf9658cd794aad2b3fd2bc706810`
- `logging-surface-added` — 608 bytes — `636a03a2d301c61ed492db55f6d5f4078a1ccce71fd9942cb076e523142fbb9d`
- `transform-present` — 620 bytes — `e1a039b1fc58bb258fdbf6c12efd391ebf627004298d3f1d661c16479bef55fc`

The first two cases deliberately provide the same semantic surface set in different caller order.
Their full canonical transcript bytes and commitments are byte-for-byte identical.

All mutation vectors differ from the base commitment.

## 13. Qualification obligations

A future qualifier must independently establish at least:

1. exact parent/head/file-set identity;
2. exact transcript and commitment domain bytes including NUL;
3. exact semantic schema/profile strings;
4. big-endian `u16/u32/u64` widths;
5. exact field order;
6. exact sink tag/name table;
7. exact surface tag/name table;
8. duplicate surface rejection;
9. caller surface order independence;
10. optional-transform framing;
11. all seven transcript lengths and byte-for-byte transcript oracles;
12. all seven SHA-256 commitments;
13. destination/payload/policy-epoch/surface/transform mutations change commitment as specified;
14. JSON/serde/Rust layout/map order cannot influence canonical bytes;
15. the protocol exposes no policy evaluation, secret bytes, disclosure permit, network/file/tool authority, or observed-effect claim;
16. deterministic commitment privacy ceiling is documented in the qualification receipt.

## 14. Nonclaims

This protocol does not establish:

- truth/currentness of any reference;
- correctness of a handling label;
- policy correctness;
- declassification correctness;
- disclosure authorization;
- network/file/tool authorization;
- secret custody;
- actual disclosure;
- recipient identity;
- retention enforcement;
- legal compliance;
- anonymity or traffic-analysis resistance.

Its strongest claim is narrower:

> Two conforming implementations can identify the same exact proposed disclosure through the same canonical bytes and deterministic commitment without confusing that identity with permission.
