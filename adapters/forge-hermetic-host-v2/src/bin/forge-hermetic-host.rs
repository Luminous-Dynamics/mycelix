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
        mycelix_forge_hermetic_host_v2::execute_from_launch_manifest_v2(Path::new(&args[2]))
    {
        eprintln!("forge-hermetic-host v2 failed: {error}");
        std::process::exit(1);
    }
}
