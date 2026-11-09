use emsr_dependency_hypergraph::{evaluate_bytes, independently_verify};
use std::env;
use std::fs::File;
use std::io::Read;

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("evaluate") if args.len() == 4 => {
            let input = read_bounded(&args[2])?;
            let report = evaluate_bytes(&input);
            write_json(&args[3], &report)
        }
        Some("verify") if args.len() == 4 => {
            let input = read_bounded(&args[2])?;
            let report = read_bounded(&args[3])?;
            let verification = independently_verify(&input, &report);
            println!("{}", serde_json::to_string_pretty(&verification).map_err(|e| e.to_string())?);
            if verification.status == "verified" {
                Ok(())
            } else {
                Err("independent reference verifier rejected the report".to_owned())
            }
        }
        _ => Err(
            "usage: emsr-dependency-hypergraph evaluate <input.json> <report.json>\n       emsr-dependency-hypergraph verify <input.json> <report.json>".to_owned()
        )
    }
}

fn read_bounded(path: &str) -> Result<Vec<u8>, String> {
    const MAX_BYTES: usize = 16 * 1024 * 1024;
    let file = File::open(path).map_err(|e| format!("cannot open {path}: {e}"))?;
    let mut bytes = Vec::with_capacity(64 * 1024);
    file.take((MAX_BYTES + 1) as u64)
        .read_to_end(&mut bytes)
        .map_err(|e| format!("cannot read {path}: {e}"))?;
    if bytes.len() > MAX_BYTES {
        return Err(format!("input exceeds 16 MiB: {path}"));
    }
    Ok(bytes)
}

fn write_json(path: &str, value: &impl serde::Serialize) -> Result<(), String> {
    let bytes = serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?;
    fs::write(path, bytes).map_err(|e| format!("cannot write {path}: {e}"))
}


#[cfg(test)]
mod tests {
    use super::read_bounded;
    use std::fs;
    use std::path::PathBuf;

    #[test]
    fn bounded_reader_rejects_files_larger_than_sixteen_mib() {
        const LIMIT: usize = 16 * 1024 * 1024;
        let mut path = PathBuf::from(std::env::temp_dir());
        path.push(format!(
            "emsr-dependency-hypergraph-oversize-{}-{}.json",
            std::process::id(),
            std::thread::current().name().unwrap_or("test")
        ));
        fs::write(&path, vec![b' '; LIMIT + 1]).expect("write temporary test input");
        let result = read_bounded(path.to_str().expect("UTF-8 temporary path"));
        let _ = fs::remove_file(&path);
        assert!(result.is_err());
    }

    #[test]
    fn bounded_reader_accepts_small_files_without_metadata_precheck() {
        let mut path = PathBuf::from(std::env::temp_dir());
        path.push(format!(
            "emsr-dependency-hypergraph-small-{}-{}.json",
            std::process::id(),
            std::thread::current().name().unwrap_or("test")
        ));
        fs::write(&path, b"{}").expect("write temporary test input");
        let result = read_bounded(path.to_str().expect("UTF-8 temporary path"));
        let _ = fs::remove_file(&path);
        assert_eq!(result.expect("small file is below the byte limit"), b"{}");
    }
}
