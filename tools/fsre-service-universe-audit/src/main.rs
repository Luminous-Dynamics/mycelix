use fsre_service_universe_audit::audit;
use std::env;
use std::fs;
use std::process::ExitCode;

fn main() -> ExitCode {
    let mut args = env::args_os().skip(1);
    let Some(path) = args.next() else {
        eprintln!("usage: fsre-service-universe-audit <audit.psv>");
        return ExitCode::from(2);
    };
    if args.next().is_some() {
        eprintln!("usage: fsre-service-universe-audit <audit.psv>");
        return ExitCode::from(2);
    }

    let source = match fs::read_to_string(&path) {
        Ok(source) => source,
        Err(error) => {
            eprintln!("cannot read {}: {error}", path.to_string_lossy());
            return ExitCode::from(2);
        }
    };

    match audit(&source) {
        Ok(report) => {
            print!("{}", report.render());
            if report.disposition == "RECONCILED_TO_DECLARED_REFERENCE_FRAME" {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(1)
            }
        }
        Err(error) => {
            eprintln!("invalid audit input: {error}");
            ExitCode::from(2)
        }
    }
}
