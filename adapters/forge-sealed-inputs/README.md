# Forge Sealed Inputs

FORGE-004D3B2D1 closes the mutable host-path input race for the M0 hermetic verifier.

The isolation policy already commits every non-Nix input by role, sandbox destination, digest, and byte size. This crate snapshots those exact bytes into anonymous `memfd` files, applies the immutable Linux seal set (`F_SEAL_WRITE | F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL`), verifies the installed seals, re-reads and re-hashes the sealed bytes, and then constructs the bubblewrap command using `--ro-bind-fd FD DEST` for those artifacts.

The Nix closure remains path-mounted because its store objects are independently covered by runtime NAR qualification. The sealed-input result therefore proves input transport identity and immutability; it does not prove executable correctness, kernel isolation, verifier trust, or repository `OfflineEvidence` on its own.

The final host composition must still bind the exact bubblewrap tool artifact that supports the FD-bind primitive, preserve the sealed FDs until sandbox setup is complete, qualify parent/inside isolation, establish same-process binding, perform post-run NAR qualification, and compose the resulting evidence through the M0 gate.
