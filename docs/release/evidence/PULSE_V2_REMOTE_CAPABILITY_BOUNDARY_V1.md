# Pulse V2 remote capability boundary v1

This tranche freezes the capability surface required for realtime wake reception without adding runtime capability state yet.

## Authority split

The wire theorem decides **what bytes may be interpreted as Pulse V2**. The capability theorem decides **which remote entry point may be callable at all**. These are separate security boundaries.

The only acceptable future remote grant is:

- current coordinator zome only;
- function-listed, never all-functions;
- exactly `recv_remote_signal`;
- `CapAccess::Unrestricted`, because remote peers do not share a pre-negotiated cap secret;
- canonical audit tag `pulse-v2-recv-remote-signal`;
- no application-domain entry/link writes in `init`.

The current no-grant initializer remains a valid baseline. This theorem intentionally makes no claim that realtime remote delivery is enabled yet.

## Parent

Qualified post-commit authority theorem: `db83032b62b65621638a59f567348bdf1eeef605` / run `34888989570`.
