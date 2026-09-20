# Hearth Home Assistant Protocol

`hearth-home-assistant-protocol` freezes the pure read-only session theorem that sits between a future native Home Assistant WebSocket transport and HA-0.1's observation mapper.

## Command ceiling

The command algebra can express exactly two post-authentication commands:

- `get_states`
- `subscribe_events` for `state_changed`

There is no arbitrary command string and no `call_service`, `fire_event`, registry mutation, trigger automation, REST write, or device-control variant.

Authentication secret handling is deliberately outside this crate. When the Home Assistant server requests authentication, the state machine emits `NeedAuthentication`; a future native transport may source and transmit the local secret without placing it in this protocol state, Debug output, durable profile, or DHT evidence.

## Session theorem

Each connection gets a monotonically increasing session epoch. Command IDs are scoped to that epoch. Messages from an older epoch are stale and rejected even if their integer command IDs match a new session.

The required progression is:

`AwaitingAuthRequired -> AwaitingAuthResult -> AwaitingStates -> AwaitingSubscription -> Live`.

A fresh `get_states` bootstrap must succeed before the live `state_changed` subscription is admitted. Reconnect starts the theorem again from the beginning.

## State evidence

Bootstrap states and live `new_state` objects are passed through HA-0.1. A `state_changed` event whose `new_state` is absent becomes an explicit `EntityRemoved` invalidation event. Disconnect never manufactures a fresh observation; prior HA-0.1 observations simply age according to their already-frozen local freshness deadlines.

## Nonclaims

This crate does not parse JSON, open sockets, authenticate a remote host, store a token, provide TLS policy, establish Home Assistant as safety-critical truth, or control a device. Those belong to later native transport and independently qualified physical-adapter layers.
