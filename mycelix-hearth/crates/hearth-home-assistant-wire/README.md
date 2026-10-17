# Hearth Home Assistant Wire

`hearth-home-assistant-wire` freezes the bounded JSON translation boundary between Home Assistant's WebSocket API and Hearth's HA-0.2A read-only protocol theorem.

## Boundary

Raw Home Assistant JSON is **untrusted transport data**. This crate may decode only the message classes required by the read-only observation session:

- `auth_required`;
- `auth_ok` / `auth_invalid`;
- `get_states` results;
- `subscribe_events(state_changed)` results;
- `state_changed` events.

It encodes only HA-0.2A's two post-authentication commands: `get_states` and `subscribe_events` for `state_changed`.

There is no generic arbitrary-command encoder and no service-call, event-fire, registry-mutation, automation-trigger, REST-write, or device-control surface.

## Phase-specific decoding

The codec does not deserialize arbitrary command results into a universal `serde_json::Value` dispatcher. Callers select the expected decoder from HA-0.2A's current phase, and that decoder admits only the expected Home Assistant message shape.

Message IDs are preserved but not trusted. HA-0.2A remains responsible for command correlation and session-epoch acceptance.

## Data minimization

A Home Assistant state object is reduced immediately to:

- exact `entity_id`;
- primary string `state`;
- optional `unit_of_measurement`;
- optional `device_class`;
- local Hearth receive timestamp.

All other attributes and fields—including `old_state`, context, friendly names, coordinates, media metadata, and integration-specific attribute bags—are ignored by deserialization and are never retained in the typed output.

Source `last_updated` is intentionally not parsed in this tranche; HA-0.1 already treats that provenance as optional and never uses it for freshness.

## Exact field-presence semantics

The codec uses a presence-aware field type wherever missing, explicit null, and value carry different protocol meanings. Plain `Option<T>` is not used for these fields because it collapses absent and null.

For `state_changed.data.new_state`:

- state object -> map/minimize a new observation frame;
- explicit JSON `null` -> entity-removal invalidation;
- field absent -> malformed event; reject and preserve the current observation.

For command results:

- successful `get_states` -> `result` must be a state array and `error` must be absent;
- successful `subscribe_events` -> `result` must be explicitly `null` and `error` must be absent;
- failed command -> `error` must be an object and `result` must be absent.

Missing result fields, explicit null where a field must be absent, and null/non-null where a payload shape is required are rejected. These distinctions make malformed payloads unable to masquerade as an explicit removal or successful subscription.

## Bounds

V1 freezes separate limits for authentication/control messages, live events, bootstrap snapshots, bootstrap entity count, server error strings, and Home Assistant version strings. Oversized or malformed inputs fail before they can construct protocol events.

## Secrets

This crate never receives or stores the Home Assistant access token. Auth-message construction stays in the future native transport so this codec remains secret-free.

## Nonclaims

This crate does not open sockets, resolve DNS, authenticate a host, establish TLS, store secrets, determine household authority, establish sensor truth, or control devices.
