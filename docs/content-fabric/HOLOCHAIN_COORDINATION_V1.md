# Content Fabric Holochain Coordination v1

Status: CF-05 draft contract.

## Purpose

The Content Fabric hApp is a signed metadata/evidence plane. Bulk content MUST NOT enter the DHT.

The hApp answers discovery questions such as:

- which agents claim to provide Content Fabric transport;
- which digest algorithms they claim to support;
- which public/discoverable digests they currently claim to serve;
- which endpoint an advertisement is cryptographically bound to;
- what other agents report observing from that advertisement.

It does not answer authorization, lease, payment, placement, or content-truth questions.

## Fresh endpoint binding

Every `ProviderAdvertisementV1` contains an Ed25519 Iroh endpoint public key and signature. The endpoint signs a serializer-independent, domain-separated preimage committing to:

- schema version;
- Holochain provider agent public key;
- the provider source-chain head immediately before the advertisement;
- Iroh endpoint public key;
- `mycelix/content/1` ALPN;
- maximum blob size;
- TTL;
- canonical supported digest algorithms;
- canonical self-claimed failure-domain labels.

The coordinator exposes `get_provider_binding_context(())` to obtain the exact provider identity and current chain head that the endpoint must sign. `publish_provider_advertisement` performs no chain write before the advertisement itself.

Integrity validation requires `binding_prev_action` to equal the advertisement Create action's `prev_action`. The endpoint proof is therefore single-use at one Holochain chain position; replaying the same signed advertisement after the provider chain advances fails validation.

A valid fresh signature establishes control of the advertised endpoint key at the signed chain context. It does not establish that the endpoint is reachable, trustworthy, authorized to serve a digest, or independent of another provider.

## Availability privacy and effective lifetime

`ContentAvailabilityClaimV1` is opt-in discoverability. Private/restricted replicas MAY remain completely absent from digest indexes. Absence of a claim means only "not advertised".

Publishing an availability claim does not create a CF-04 `ReadAuthorizerV1` grant.

An availability claim is usable only while **both** its own TTL and its referenced provider advertisement remain active. Effective validity is the intersection of those lifetimes and ends immediately if the advertisement is withdrawn. The integrity layer bounds the claim TTL by the parent TTL value; the later projection layer evaluates actual action timestamps.

## Append-only evidence

Provider advertisements, availability claims, withdrawals, observations, and Content Fabric index links are immutable. Replacement is represented by new actions.

Consumers derive current state from action timestamps, TTLs, withdrawals, and policy. Link insertion order is not a consensus clock.

## Failure-domain claims

Failure-domain values are canonical self-claims for operator, machine, site, ASN, region, jurisdiction, and power-domain dimensions. They are useful candidate metadata but are not independent evidence. Multiple provider identities or multiple self-claims do not by themselves satisfy placement independence requirements.

## Observation semantics

`ReplicaObservationV1` is authored by the observer. Outcomes include:

- `VerifiedComplete` — the observer claims it received the full object and verified the requested digest;
- `UnavailableOrHidden` — deliberately does not distinguish absence from authorization/privacy hiding;
- `Busy`;
- `TransferFailed`;
- `ProviderReportedIntegrityFailure`;
- `DigestMismatch` — the observer claims the received complete bytes hashed differently.

No single observation is global truth. Later durability logic may combine independent observations according to explicit evidence policy.

## Index integrity

Every index link targets an action hash and uses an empty v1 tag. Validation recomputes the expected base from the target record and requires the link author to match the target's provider/observer identity. Link deletion is forbidden in v1.

Indexes exist for:

- all providers;
- provider agent → advertisements;
- digest algorithm → advertisements;
- digest → availability claims;
- advertisement → availability claims;
- digest → observations;
- advertisement → observations;
- advertisement → withdrawals.

## Authority exclusions

The hApp MUST NOT:

- store blob payloads;
- grant or revoke transport reads;
- write into a CAS;
- create placement leases;
- declare replica independence from self-claims alone;
- instruct Symthaea to execute placement;
- delete replicas;
- bill or settle providers;
- treat an availability claim as content-integrity proof.
