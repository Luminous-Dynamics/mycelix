// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_forge_guest_plan::GUEST_PLAN_PATH;

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() != 3 || args[1] != "--plan" || args[2] != GUEST_PLAN_PATH {
        eprintln!("forge-hermetic-guest requires: --plan {GUEST_PLAN_PATH}");
        std::process::exit(2);
    }

    if let Err(error) = mycelix_forge_hermetic_guest::run_guest_from_fixed_inputs() {
        eprintln!("forge-hermetic-guest failed: {error}");
        std::process::exit(1);
    }
}
