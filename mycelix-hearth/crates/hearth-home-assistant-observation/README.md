# Hearth Home Assistant Observation

`hearth-home-assistant-observation` freezes the read-only semantic boundary between a Home Assistant state stream and Hearth household evidence.

## Core theorem

Home Assistant may contribute **observations**. It does not contribute household authority.

An observation is emitted only when an exact entity ID is allowlisted by an explicit profile and its state can be decoded by the profile's deterministic codec. Home Assistant's `unknown` and `unavailable` states remain epistemic source states and never become ordinary text observations.

## Privacy and trust boundary

The profile contract has no dedicated endpoint, network-address, credential, cookie, service-call, or device-control fields. `profile_ref` is validated as a bounded local identifier rather than a URL/path or transport option. Any future local transport must keep credentials outside this observation profile.

V1 maps only the entity's primary `state` plus two optional allowlisted metadata fields used to detect schema drift: `unit_of_measurement` and `device_class`. It deliberately does not ingest the arbitrary Home Assistant attribute bag.

The Home Assistant source timestamp is retained only as diagnostic provenance. Hearth freshness uses the local `received_at_micros` timestamp so clock skew on the Home Assistant host cannot manufacture a fresh observation.

## Deterministic codecs

Supported V1 state codecs are:

- exact boolean tokens;
- signed/unsigned integer ranges;
- basis points;
- fixed-point decimal strings with an explicit scale and no floating-point/exponent parsing;
- bounded text with an optional exact-value allowlist.

Unknown entity IDs are ignored as `NotAllowlisted`; malformed allowlisted data fails closed.

## HA-0.2A pure protocol contract

The [read-only WebSocket protocol state machine](HA_WEBSOCKET_PROTOCOL.md) now freezes authentication, native `subscribe_entities` correlation, initial snapshot application, reconnect epochs, and fail-closed handling before transport code is introduced. It sources the exact non-empty subscription allowlist from the validated observation profile and has no socket, URL, credential, state-write, or service-call API. The native local WebSocket client remains a later HA-0.2B step. It must version-gate `subscribe_entities`, fail closed when unavailable unless a separately qualified legacy profile is selected, keep credentials local, and must not expose `call_service`, REST state writes, or other actuation APIs through this observation contract.

The pure [compact entity stream projector](src/entity_stream.rs) stages and validates the provider's added/changed/removed groups, keeps only allowlisted primary states plus unit/device-class assertions, rejects malformed updates transactionally, validates codec/range semantics before cache commit, and does not refresh observation freshness for ignored metadata or equivalent-value deltas. Its public inputs are `apply_json` (raw compact-event bytes through strict decoding) and `apply_parsed_json` (an event view extracted from a strictly parsed outer frame); the value-level mutator is private. The [strict JSON decoder](src/strict_json.rs) caps input at 8 MiB and rejects duplicate object keys at every nesting depth before `Value`-based processing; its errors never echo raw payload data.

## End-to-end conformance and benchmarks

The [protocol-to-projector integration tests](tests/protocol_projection_contract.rs) exercise the protocol gate → compact projector → HA-0.1 mapper path, including wrong-subscription-ID and control-character negative controls.

The [observation benchmark plan](OBSERVATION_BENCHMARKS.md) defines eleven proposed acceptance objectives for transition safety, allowlist containment, transactional projection, semantic freshness, privacy, latency, memory, reconnect currentness, and capability fallback. These are targets, not measured results; performance qualification remains blocked until the native transport and executable benchmark harness exist.

## HA-0.2C lifecycle invalidation

The [observation lifecycle contract](OBSERVATION_LIFECYCLE.md) consumes projected semantic state and makes currentness explicit. It invalidates prior positives on `unknown`, `unavailable`, removal, expiry, disconnect, malformed projection, and reconnect-snapshot omission. Currentness requires the exact active epoch, an unexpired deadline, and a fully applied snapshot; cache retention alone is not enough.

The lifecycle checkpoint preserves only a profile reference, epoch high-water mark, and per-entity revisions/tombstones. It intentionally restores no positive observations. The checkpoint is serializable but not a durable store; cross-process replay protection requires an atomic, crash-safe persistence layer.

The [durable lifecycle facade](DURABLE_OBSERVATION_LIFECYCLE.md) wraps lifecycle transitions in append-and-sync-before-publish semantics and cold-start replay validation. Only an interface and in-memory fault-injection store are included; no production backend is qualified. The journal identifies positives with value-free revision and ledger-sequence references rather than publishing low-entropy state hashes.

Proposed, unmeasured targets are in [OBSERVATION_LIFECYCLE_BENCHMARKS.md](OBSERVATION_LIFECYCLE_BENCHMARKS.md). These do not establish peer authenticity, household authority, physical state, or device qualification.

Physical control, if later admitted, belongs behind the independent Hearth adapter-admission, exclusive-executor, crash-journal, household-authority, and outcome-verification theorems.
