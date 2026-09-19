# Forge Guest Output Frame

FORGE-004D3B2D2B reserves the top-level hermetic guest stdout stream for exactly one bounded evidence frame.

The v1 frame is:

```text
8-byte magic/version  = MXFGOUT1
u64 big-endian length
exact JSON payload bytes
32-byte SHA-256 checksum of payload
EOF
```

The payload is a raw `GuestEvidenceEnvelopeV1`. The frame is transport, not authority: the parent must independently re-qualify the decoded envelope against the exact guest plan and qualified guest tool map.

The length is capped at 16 MiB before allocation. Empty frames, bad magic, checksum mismatches, truncation, and any trailing bytes fail closed. Successful guest execution writes nothing else to stdout; errors remain on stderr.

This channel avoids a writable host filesystem mount and does not depend on retrieving `/work` after the sandbox exits. Internal `/work` transcript/envelope files may remain as in-sandbox consistency diagnostics, but they are not the host evidence transport.
