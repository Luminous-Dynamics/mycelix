// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Legacy post-quantum wrappers (native feature only).
//!
//! These modules use the `pqcrypto-*` / PQClean C-FFI ecosystem. They are
//! retained for compatibility and research, not as a qualified final-FIPS-204
//! or FIPS-205 provider. Do not infer standards conformance from the algorithm
//! aliases or a local sign/verify round trip. The tagged envelope enforces the
//! final-profile key/signature lengths and rejects incompatible legacy output.

pub mod dilithium;
pub mod ed25519_native;
pub mod encryption;
pub mod hybrid;
pub mod ml_kem;
pub mod sphincs;
