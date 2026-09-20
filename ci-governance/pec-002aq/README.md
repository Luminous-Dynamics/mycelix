# PEC-002AQ exact-source qualifier

Runner-neutral source for qualifying canonical PEC-002A without adding hosted-runner demand.

The qualifier binds the exact PEC-002A commit/tree/parent, all seven protocol-profile blobs, and the two inherited PEC semantic-core blobs. It requires a clean direct-child checkout containing exactly the four registered qualifier files, disables Git replacement objects/system config, rejects grafts/alternates/replace refs and security-relevant Git environment redirects, and verifies its own non-circular `lock.json` against the exact README/qualifier/test blob identities **and exact canonical lock bytes** before executing Rust.

It reconstructs a temporary two-crate Cargo workspace entirely from canonical Git objects and requires `cargo fmt --check`, offline workspace tests, and warnings-denied Clippy. Missing cached dependencies fail closed; there is no network fallback. The structural source probe also rejects unsafe/FFI/process/network/filesystem runtime capabilities in the profile crate.

The emitted receipt binds the qualifier lock SHA-256, exact source identities, tool versions, command return codes/output hashes, generated Cargo.lock SHA-256, the 37-case committed source-test count, and the structural probe set.

The receipt remains `StructuralOnly`. A PASS does not establish cryptographic backend security, malicious-secure MPC, FHE parameter adequacy, VOPRF/PSI security, PIR/ORAM security, production admission, or application authority.

The receipt path must resolve outside the qualifier checkout. The qualifier performs no network access and no Git/GitHub mutation.
