# Hearth Home Assistant Observation

`hearth-home-assistant-observation` freezes the read-only semantic boundary between a Home Assistant state stream and Hearth household evidence.

## Core theorem

Home Assistant may contribute **observations**. It does not contribute household authority.

An observation is emitted only when an exact entity ID is allowlisted by an explicit profile and its state can be decoded by the profile's deterministic codec. Home Assistant's `unknown` and `unavailable` states remain epistemic source states and never become ordinary text observations.

## Privacy and trust boundary

The durable profile contains no Home Assistant URL, access token, credential, cookie, network address, service-call permission, or device-control authority. Those belong to a future local edge transport.

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

## Future HA-0.2 boundary

A future read-only local WebSocket client may authenticate to Home Assistant, fetch the initial state set, and subscribe to `state_changed`. It must keep credentials local and must not expose `call_service`, REST state writes, or other actuation APIs through this observation contract.

Physical control, if later admitted, belongs behind the independent Hearth adapter-admission, exclusive-executor, crash-journal, household-authority, and outcome-verification theorems.
