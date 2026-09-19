// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_linux_isolation::{
    ArtifactMount, LinuxIsolationPolicyV1, NixClosureEntry, NixClosureManifest, RuntimeArtifact,
    VerifierInvocation,
};
use mycelix_forge_sealed_inputs::{
    build_sealed_bubblewrap_command, seal_runtime_inputs,
};
use std::{fs, path::PathBuf};

fn digest(bytes: &[u8]) -> Digest {
    Digest::of_bytes(DigestAlgorithm::Sha256, bytes)
}

fn fixture() -> (
    NixClosureManifest,
    VerifierInvocation,
    LinuxIsolationPolicyV1,
    PathBuf,
) {
    let closure = NixClosureManifest::new(vec![
        NixClosureEntry::new("/nix/store/aaaa-guest", digest(b"nar-a")).unwrap(),
        NixClosureEntry::new("/nix/store/bbbb-runtime", digest(b"nar-b")).unwrap(),
    ])
    .unwrap();
    let invocation = VerifierInvocation::new(
        "/nix/store/aaaa-guest/bin/forge-hermetic-guest",
        vec!["--plan".into(), "/inputs/guest-verification-plan.json".into()],
        &closure,
    )
    .unwrap();

    let source = std::env::temp_dir().join(format!(
        "mycelix-forge-sealed-input-{}-{}",
        std::process::id(),
        std::thread::current().name().unwrap_or("test")
    ));
    let bytes = b"exact committed input bytes";
    fs::write(&source, bytes).unwrap();
    let policy = LinuxIsolationPolicyV1::strict(
        &closure,
        &invocation,
        vec![
            ArtifactMount::new(
                "guest-verification-plan",
                "/inputs/guest-verification-plan.json",
                digest(bytes),
                bytes.len() as u64,
            )
            .unwrap(),
        ],
    )
    .unwrap();
    (closure, invocation, policy, source)
}

#[test]
fn host_source_mutation_cannot_change_sealed_observation() {
    let (_closure, _invocation, policy, source) = fixture();
    let runtime = vec![RuntimeArtifact {
        role: "guest-verification-plan".into(),
        source: source.clone(),
    }];
    let sealed = seal_runtime_inputs(&policy, &runtime).unwrap();
    let before = sealed
        .artifact("guest-verification-plan")
        .unwrap()
        .observation()
        .digest()
        .clone();

    fs::write(&source, b"host changed this after sealing").unwrap();

    let after = sealed
        .artifact("guest-verification-plan")
        .unwrap()
        .observation()
        .digest()
        .clone();
    assert_eq!(before, after);
    assert!(sealed
        .artifact("guest-verification-plan")
        .unwrap()
        .observation()
        .fully_sealed());
    let _ = fs::remove_file(source);
}

#[test]
fn bubblewrap_uses_fd_bind_for_policy_inputs_only() {
    let (closure, invocation, policy, source) = fixture();
    let runtime = vec![RuntimeArtifact {
        role: "guest-verification-plan".into(),
        source: source.clone(),
    }];
    let mut sealed = seal_runtime_inputs(&policy, &runtime).unwrap();
    let command = build_sealed_bubblewrap_command(
        "/nix/store/cccc-bubblewrap/bin/bwrap",
        &policy,
        &closure,
        &invocation,
        &mut sealed,
    )
    .unwrap();

    let destination = "/inputs/guest-verification-plan.json";
    assert!(command.args.windows(3).any(|window| {
        window[0] == "--ro-bind-fd"
            && window[2] == destination
            && window[1].parse::<i32>().is_ok()
    }));
    assert!(!command
        .args
        .windows(3)
        .any(|window| window[0] == "--ro-bind" && window[2] == destination));
    assert!(command.args.windows(3).any(|window| {
        window[0] == "--ro-bind"
            && window[1] == "/nix/store/aaaa-guest"
            && window[2] == "/nix/store/aaaa-guest"
    }));
    let _ = fs::remove_file(source);
}
