use furniture_compiler::{compile_json, verify_integrity, CompiledPacket, MAX_INPUT_BYTES};
use std::io::{self, Read};

#[derive(Clone, Copy)]
enum Mode {
    Compile,
    Verify,
}

fn main() {
    if let Err(message) = run() {
        eprintln!("furniture-compile: {message}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let mode = match args.next().as_deref() {
        None => Mode::Compile,
        Some("--verify") => Mode::Verify,
        Some(other) => return Err(format!("unknown argument {other:?}; supported: --verify")),
    };
    if args.next().is_some() {
        return Err("expected at most one argument: --verify".to_owned());
    }

    let mut input = String::new();
    io::stdin()
        .take((MAX_INPUT_BYTES + 1) as u64)
        .read_to_string(&mut input)
        .map_err(|error| format!("could not read UTF-8 input: {error}"))?;
    if input.len() > MAX_INPUT_BYTES {
        return Err(format!("input exceeds {MAX_INPUT_BYTES} bytes"));
    }

    match mode {
        Mode::Compile => {
            let packet = compile_json(&input).map_err(|error| error.to_string())?;
            if !verify_integrity(&packet).map_err(|error| error.to_string())? {
                return Err("internal packet integrity verification failed".to_owned());
            }
            let output = serde_json::to_string_pretty(&packet)
                .map_err(|error| format!("could not serialize packet: {error}"))?;
            println!("{output}");
        }
        Mode::Verify => {
            let packet: CompiledPacket = serde_json::from_str(&input)
                .map_err(|error| format!("invalid packet JSON: {error}"))?;
            if !verify_integrity(&packet).map_err(|error| error.to_string())? {
                return Err("packet integrity verification failed".to_owned());
            }
            // packet_digest is generated hexadecimal, not user-controlled free text.
            println!(
                "{{\"integrity_verified\":true,\"packet_digest\":\"{}\"}}",
                packet.packet_digest
            );
        }
    }
    Ok(())
}
