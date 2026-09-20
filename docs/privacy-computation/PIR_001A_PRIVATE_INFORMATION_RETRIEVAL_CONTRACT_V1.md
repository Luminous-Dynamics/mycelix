# PIR-001A — Private Information Retrieval Capability Contract v1

Status: architectural contract only

Tracks: #2115, #2109, #2110

Parent semantic waist: PEC-001A / #2116

## Purpose

Freeze a backend-neutral PIR semantic boundary before any retrieval backend is treated as establishing query privacy.

The governing rule is:

```text
PIR query succeeds
    != query index hidden under every model
    != client anonymous
    != access pattern hidden across sessions
    != response authentic/current
    != access authorized
```

PIR-001A implements no cryptographic protocol and qualifies no backend.

## V1 objective

The v1 PIR objective is exactly:

```text
QueryIndexPrivacy
```

That means a database holder should not learn which admitted index/key the client requested, under an exact named backend/adversary profile.

PIR-001A does not widen that objective into anonymity or sequence privacy.

## Required non-equivalences

```text
query-index privacy != client anonymity
query-index privacy != network-metadata privacy
query-index privacy != sequence/access-pattern privacy
PIR != ORAM
PIR != authorization
PIR response != database freshness
PIR response != response integrity
PIR correctness != server honesty beyond the stated model
single-query privacy != repeated-query unlinkability
```

## Retrieval semantics

Every concrete profile must bind the exact retrieval model, including:

- database representation/version;
- snapshot or epoch identity;
- index/key canonicalization;
- record-size semantics;
- missing-key behavior;
- duplicate-key behavior;
- padding policy;
- maximum database size/resource profile;
- response decoding semantics.

```text
same logical database
    != same PIR database
```

unless encoding, ordering, padding, and snapshot identity are identical.

## Trust-model vocabulary

A concrete backend must state whether it is, for example:

```text
SingleServerComputational
MultiServerNonColluding
MultiServerThresholdCollusion
```

The exact security theorem belongs to the concrete backend profile; these names are descriptive categories only.

In particular:

```text
multi-server PIR
    != privacy if the required non-collusion assumption is violated
```

and:

```text
single-server PIR
    != information-theoretic query privacy
```

unless a specific construction establishes that stronger property.

## Database snapshot binding

Every authority-bearing execution must bind the database snapshot/epoch against which the response was computed.

A future receipt should identify at least:

- database/profile identity;
- snapshot/epoch reference;
- backend/profile identity;
- query-domain identity;
- response commitment/digest/reference where applicable;
- qualification evidence.

```text
valid PIR response from snapshot A
    != response from current snapshot B
```

## Response integrity and freshness

PIR hides a query under a stated model; it does not inherently prove the server returned the correct or current value.

Where an application needs integrity/currentness, the profile must compose a separately defined mechanism such as an authenticated data structure, signed snapshot, commitment proof, ZKP, or another qualified mechanism.

```text
private retrieval
    + unauthenticated response
    != trustworthy retrieval
```

PIR-001A deliberately does not select one universal integrity mechanism.

## Leakage profile

A future profile must explicitly declare the disposition of at least:

```text
DatabaseSize
RecordSize
QueryMessageSize
ResponseMessageSize
QueryTiming
ResponseTiming
ClientNetworkIdentity
ServerIdentity
QueryIndex
QuerySequenceCorrelation
RepeatedQueryEquality
AbortBehavior
CacheBehavior
```

The PEC leakage vocabulary should carry these declarations rather than a `private: bool` shortcut.

## Sequence/access-pattern boundary

PIR-001A protects only the query objective established by the concrete protocol.

If an application requires hiding correlations across a sequence of reads/writes, the planner must not silently treat ordinary PIR as sufficient.

```text
requirement: SequenceAccessPatternPrivacy
candidate: QueryIndexPrivacy only
=> Incompatible(AccessPatternPrivacyUnavailable)
```

A later ORAM/structured-encryption profile may address broader access-pattern or encrypted-search requirements.

## Client anonymity boundary

Even a cryptographically private query may expose:

- IP/network origin;
- account/session identity;
- transport metadata;
- timing;
- request size;
- authentication credentials.

Therefore:

```text
server cannot recover query index
    != server does not know who queried
```

Transport/anonymity protections are separate composition requirements.

## Authorization boundary

```text
PIR hides requested index
    != client authorized to read any record
    != client authorized to read this record
    != database owner authorized to store/process it
```

A production profile must compose normal access-control/application authority separately without defeating the intended query-privacy property.

## Query-domain abuse boundary

A client may attempt malformed, out-of-range, multi-record, or adversarial queries.

A profile must state how it enforces the promised query cardinality/functionality and whether the server obtains evidence that the query is well formed without learning the protected index.

```text
query privacy
    != server must accept arbitrary ciphertext/program
```

## Preprocessing boundary

Some PIR constructions use server-side or client-side preprocessing, common reference strings, cached hints, or offline state.

A profile must bind:

- preprocessing algorithm/profile;
- state identity/version;
- which party stores it;
- freshness/rotation rules;
- whether preprocessing leaks or binds database contents;
- invalidation behavior after database changes.

```text
old preprocessing state + new database
    != valid current PIR profile
```

unless explicitly supported.

## First prototype boundary

The first Mycelix PIR prototype should use a synthetic or fully public registry, not sensitive personal query data.

Measure at least:

- database entries and bytes;
- preprocessing time/size;
- query bytes;
- response bytes;
- client CPU/memory;
- server CPU/memory;
- wall-clock latency;
- scaling versus database size;
- repeated-query correlation observable to the server;
- malformed-query handling.

Prototype execution remains `Experimental` until a separate profile is measured and qualified.

## Candidate implementation research boundary

The Rust ecosystem contains multiple materially different PIR constructions, including DPF-based multi-server approaches and lattice/FHE-based single-server approaches. Backend choice must therefore follow an exact threat/performance model rather than a generic `PIR=true` capability.

No external implementation is adopted or endorsed by this contract.

## Relationship to Symthaea

A future Symthaea PEC planner should be able to distinguish:

```text
private single lookup -> consider qualified PIR
private keyword/document search -> consider structured-encryption/search profile
sequence access-pattern privacy -> PIR alone insufficient; consider ORAM profile
private set overlap -> PSI, not PIR
```

Planner output remains advisory and non-authoritative.

## Nonclaims

PIR-001A establishes no computational or information-theoretic PIR security, non-collusion guarantee, client anonymity, traffic-analysis resistance, ORAM security, response integrity, database freshness, access authorization, legal/privacy compliance, production admission, or deployment readiness.
