use logistics_commons_s0::scenario::FrozenScenarioRun;

fn main() {
    let run = FrozenScenarioRun::build();
    let canonical = std::env::args().any(|arg| arg == "--canonical");

    if canonical {
        print!("{}", run.canonical_artifact());
    } else {
        println!("{}", run.summary_json());
    }

    if !run.model_invariants_hold() {
        for violation in &run.independent_violations {
            eprintln!("independent-verifier=FAIL:{violation}");
        }
        std::process::exit(1);
    }
}
