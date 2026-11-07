use sha2::{Digest, Sha256};
use std::{
    env, fs,
    path::PathBuf,
    process::Command,
};

fn is_lower_hex(value: &str, width: usize) -> bool {
    value.len() == width
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn verifier_commit() -> String {
    if let Ok(output) = Command::new("git").args(["rev-parse", "HEAD"]).output() {
        if output.status.success() {
            let value = String::from_utf8_lossy(&output.stdout).trim().to_owned();
            if is_lower_hex(&value, 40) {
                return value;
            }
        }
    }

    env::var("GITHUB_SHA")
        .ok()
        .filter(|value| is_lower_hex(value, 40))
        .unwrap_or_else(|| "unknown".to_owned())
}

fn lockfile_digest() -> String {
    let Some(manifest_dir) = env::var_os("CARGO_MANIFEST_DIR") else {
        return "unknown".to_owned();
    };
    let path = PathBuf::from(manifest_dir).join("Cargo.lock");
    let Ok(contents) = fs::read(path) else {
        return "unknown".to_owned();
    };

    let digest = Sha256::digest(contents);
    let mut output = String::with_capacity(64);
    for byte in digest {
        output.push(char::from(b"0123456789abcdef"[(byte >> 4) as usize]));
        output.push(char::from(b"0123456789abcdef"[(byte & 0x0f) as usize]));
    }
    output
}

fn main() {
    println!("cargo:rerun-if-changed=Cargo.lock");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=GITHUB_SHA");
    println!(
        "cargo:rustc-env=CECB_VERIFIER_SOURCE_COMMIT={}",
        verifier_commit()
    );
    println!(
        "cargo:rustc-env=CECB_VERIFIER_CARGO_LOCK_SHA256={}",
        lockfile_digest()
    );
}