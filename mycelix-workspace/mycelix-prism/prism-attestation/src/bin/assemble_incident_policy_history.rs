// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Assemble an append-only incident-policy history from signed transitions.

use prism_attestation::{
    ceremony::{absolute_path, atomic_write_private, read_private_file, require_distinct_paths},
    incident_policy::{IncidentPolicyHistory, SignedIncidentPolicyTransition},
};
use std::path::PathBuf;

const MAX_POLICY_BYTES: u64 = 256 * 1024;
const MAX_TRANSITION_BYTES: u64 = 2 * 1024 * 1024;

fn main() {
    if let Err(error) = run() {
        eprintln!("Prism incident policy history assembly failed: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let arguments = std::env::args_os().skip(1).collect::<Vec<_>>();
    if arguments.len() < 2 {
        return Err("usage: prism-assemble-incident-policy-history <genesis-policy.json> [transition-1.json ...] <history.json>".into());
    }
    let genesis_path = absolute_path(&arguments[0], "genesis incident policy")?;
    let output_path = absolute_path(arguments.last().expect("checked argument count"), "incident policy history")?;
    let transition_paths = arguments[1..arguments.len() - 1]
        .iter()
        .map(|value| absolute_path(value, "incident policy transition"))
        .collect::<Result<Vec<PathBuf>, _>>()?;
    let mut distinct = vec![genesis_path.as_path(), output_path.as_path()];
    distinct.extend(transition_paths.iter().map(PathBuf::as_path));
    require_distinct_paths(&distinct)?;

    let genesis_bytes = read_private_file(&genesis_path, MAX_POLICY_BYTES, None)?;
    let transitions = transition_paths
        .iter()
        .map(|path| {
            let bytes = read_private_file(path, MAX_TRANSITION_BYTES, None)?;
            Ok(prism_strict_json::from_slice::<SignedIncidentPolicyTransition>(&bytes)?)
        })
        .collect::<Result<Vec<_>, Box<dyn std::error::Error>>>()?;
    let history = IncidentPolicyHistory::new(&genesis_bytes, transitions)?;
    let mut encoded = serde_json::to_vec_pretty(&history)?;
    encoded.push(b'\n');
    atomic_write_private(&output_path, &encoded)?;
    println!("Assembled incident policy history at {}; root={}", output_path.display(), blake3::Hash::from_bytes(history.history_root_blake3).to_hex());
    Ok(())
}
