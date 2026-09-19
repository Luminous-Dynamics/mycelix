// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

fn main() {
    if let Err(error) = mycelix_forge_hermetic_guest::run_isolation_probe_binary() {
        eprintln!("forge-isolation-probe failed: {error}");
        std::process::exit(1);
    }
}
