use std::fs;
use std::io;
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
    let receipt = maritime_productive_capability::audit_fixture(&contents)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    print!("{receipt}");
    Ok(())
}
