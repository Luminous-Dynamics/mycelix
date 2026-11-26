use std::fs;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let fixture = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(
                "docs/research/maritime-productive-capability/fixtures/maritime-capability-tiny-v0.tsv",
            )
        });
    let contents = fs::read_to_string(fixture)?;
    let receipt = match maritime_productive_capability::run_audit(&contents) {
        Ok(receipt) => receipt,
        Err(error) => {
            eprintln!("fixture audit failed: {error}");
            std::process::exit(2);
        }
    };
    print!("{receipt}");
    Ok(())
}
