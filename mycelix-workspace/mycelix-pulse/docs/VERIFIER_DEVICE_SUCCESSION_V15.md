# Pulse Verifier Device Succession — Runtime Protocol v15

Protocol v15 separates a verifier authority's governance slot from the physical
hardware device currently holding that slot's key material.

## Planned succession

A planned replacement must preserve the same custody slot, increment the device
generation by exactly one, present a fresh hardware-bound device identity, and
carry authenticated consent from both the retiring and replacement devices. The
retiring device must be disabled within the bounded handoff window.

## Lost-device recovery

A lost, destroyed, or suspected-compromised device cannot authorize its own
replacement. Recovery requires an exact loss notice, replacement-device proof
of possession, and threshold approval from the separately governed custody
recovery authority. Loss evidence is bounded in time and names the exact slot,
device, and generation.

## Runtime truth

Pulse publishes device succession independently from key ceremony and key
rotation claims:

- succession policy and record protocol;
- active succession hash and sequence;
- active device generation;
- planned dual-device or lost-device recovery authorization;
- retiring-device shutdown;
- externally pinned succession checkpoint;
- verified succession continuity.

The supplied runtime advertises all of these as unavailable and keeps sender-
proof acceptance disabled. A reservation challenge binds exact device-
succession evidence only after a reviewed implementation installs it.
