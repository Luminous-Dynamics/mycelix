use space_geometry_informed_proposal::{DefensiveGeometryProposal, GeometryMode};

fn main() {
    let proposal = DefensiveGeometryProposal {
        id: "flat-halfspace-cli-v1".into(),
        dimension: 2,
        epsilon: 0.05,
        modes: vec![GeometryMode {
            id: "positive-x-boundary".into(),
            mixture_weight: 1.0,
            normal: vec![1.0, 0.0],
            tangent_basis: vec![vec![0.0, 1.0]],
            threshold: 2.0,
            exponential_rate: 1.5,
        }],
    };
    proposal.validate().expect("fixture proposal must validate");
    let sample = proposal
        .sample_from_latents(0.5, 0.5, &[0.0, 0.0], &[0.0], 0.5)
        .expect("fixture draw must be valid");
    let log_p = proposal
        .log_target_density(&sample.point)
        .expect("target density");
    let log_geo = proposal
        .log_geometry_density(&sample.point)
        .expect("geometry density");
    let log_q = proposal
        .log_proposal_density(&sample.point)
        .expect("full mixture density");
    let weight = proposal
        .likelihood_ratio(&sample.point)
        .expect("likelihood ratio");
    println!("{{");
    println!("  \"proposal_id\": \"{}\",", proposal.id);
    println!(
        "  \"generating_component_id\": \"{}\",",
        sample.generating_component_id
    );
    println!(
        "  \"sample\": [{:.12}, {:.12}],",
        sample.point[0], sample.point[1]
    );
    println!("  \"log_target_density\": {:.12},", log_p);
    println!("  \"log_geometry_density\": {:.12},", log_geo);
    println!("  \"log_full_proposal_density\": {:.12},", log_q);
    println!("  \"likelihood_ratio\": {:.12},", weight);
    println!(
        "  \"defensive_weight_cap\": {:.12},",
        1.0 / proposal.epsilon
    );
    println!(
        "  \"event\": {},",
        proposal.is_event(&sample.point).expect("event predicate")
    );
    println!("  \"claim_ceiling\": \"flat-boundary-analytic-fixture-not-orbital-event-truth\"");
    println!("}}");
}
