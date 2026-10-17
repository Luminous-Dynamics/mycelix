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

The pure [compact entity stream projector](src/entity_stream.rs) stages and validates the provider's added/changed/removed groups, keeps only allowlisted primary states plus unit/device-class assertions, rejects malformed updates transactionally, and does not refresh observation freshness for ignored metadata-only deltas. It is not the network transport and does not authenticate the provider.

## End-to-end conformance and benchmarks

The [protocol-to-projector integration tests](tests/protocol_projection_contract.rs) exercise the protocol gate → compact projector → HA-0.1 mapper path, including wrong-subscription-ID and control-character negative controls.

The [observation benchmark plan](OBSERVATION_BENCHMARKS.md) defines thirteen proposed acceptance objectives for protocol safety, allowlist containment, transactional projection, lifecycle invalidation, replay resistance, semantic freshness, privacy, latency, memory, reconnect currentness, and capability fallback. These are targets, not measured results; performance qualification remains blocked until the native transport and executable benchmark harness exist.

Physical control, if later admitted, belongs behind the independent Hearth adapter-admission, exclusive-executor, crash-journal, household-authority, and outcome-verification theorems.

## Explicit observation lifecycle

The [observation lifecycle module](src/observation_lifecycle.rs) converts a projection into a deterministic, sorted batch of `Upsert` and `Invalidate` operations. See [OBSERVATION_LIFECYCLE.md](OBSERVATION_LIFECYCLE.md) for the durable versioning and consumer replay rules. Source `unknown`/`unavailable`, invalid current samples (including metadata drift or out-of-range values), provider removals, and profile entities omitted from an initial/reconnect snapshot become explicit invalidations rather than leaving a previous positive observation silently current. Each batch carries a comparable `(generation, sequence)` version; generation must increase durably across reconnects/restarts, the initial snapshot uses sequence zero, and each accepted delta advances the sequence. Downstream storage must apply a batch atomically and compare versions per entity, rejecting equal/older updates so delayed upserts cannot resurrect invalidated state. The pure mapper does not itself persist counters, mutate a database, or establish physical truth.

The [observation benchmark plan](OBSERVATION_BENCHMARKS.md) now defines thirteen proposed, explicitly unmeasured acceptance objectives, including zero stale-positive retention after a declared invalidation condition.

Physical control, if later admitted, belongs behind the independent Hearth adapter-admission, exclusive-executor, crash-journal, household-authority, and outcome-verification theorems.
