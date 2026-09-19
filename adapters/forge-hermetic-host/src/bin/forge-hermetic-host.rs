// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::path::Path;

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() != 3 || args[1] != "--launch-manifest" {
        eprintln!("forge-hermetic-host requires: --launch-manifest <path>");
        std::process::exit(2);
    }

    if let Err(error) =
        mycelix_forge_hermetic_host::execute_from_launch_manifest(Path::new(&args[2]))
    {
        eprintln!("forge-hermetic-host failed: {error}");
        std::process::exit(1);
    }
}
