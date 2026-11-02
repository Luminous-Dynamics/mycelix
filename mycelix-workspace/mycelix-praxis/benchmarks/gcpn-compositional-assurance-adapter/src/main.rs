use gcpn_compositional_assurance_adapter::{
    evaluate_pair, evaluate_single, supported_pairs, Fault,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("# Synthetic adapter against gcpn-compositional-assurance only.");
    println!("fault,status,stage,claim,expected,observed,violation_detected");

    for fault in Fault::ALL {
        if !fault.supported() {
            println!("{},not_represented,,,,,", fault.label());
            continue;
        }

        let observation = evaluate_single(fault)?;
        let stage = observation.stage.label();
        println!(
            "{},executed,{stage},{},{},{},{}",
            fault.label(),
            observation.claim_id,
            observation.expected.label(),
            observation.observed.label(),
            observation.violation_detected
        );
    }

    let pairs = supported_pairs();
    let observations: Vec<_> = pairs
        .into_iter()
        .map(evaluate_pair)
        .collect::<Result<_, _>>()?;
    let no_false_support = observations
        .iter()
        .filter(|observation| observation.no_false_support)
        .count();

    println!(
        "pairwise_cases={} pairwise_no_false_support={}",
        observations.len(),
        no_false_support
    );
    println!(
        "scope_note=authority_delivery_receipts_accessibility_action_receipts_and_case_keys_not_represented"
    );

    Ok(())
}
