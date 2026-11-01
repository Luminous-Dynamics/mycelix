use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::{env, fs};
use terra_preta_sequential_bayes_design::covariance_identifiability::{
    CovarianceComponentDesign, CovarianceComponentHypothesis, CovarianceFisherInformation,
    CovarianceObservation, VarianceComponent,
};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    true_variance_components: Vec<f64>,
    hypotheses: Vec<HypothesisInput>,
    designs: Vec<DesignInput>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DesignInput {
    design_id: String,
    observations: Vec<ObservationInput>,
    observed_values: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct HypothesisInput {
    model_id: String,
    prior_weight: f64,
    variance_components: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ObservationInput {
    observation_id: String,
    modality: String,
    physical_support_id: Option<String>,
    batch_id: Option<String>,
    instrument_id: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
struct DesignSummary {
    design_id: String,
    observation_count: usize,
    structural_rank: usize,
    component_count: usize,
    gram_condition_number: Option<f64>,
    fisher_information: Vec<Vec<f64>>,
    fisher_eigenvalues: Vec<f64>,
    fisher_rank: usize,
    fisher_condition_number: Option<f64>,
    fisher_log_determinant: Option<f64>,
    fully_identifiable: bool,
    individually_identifiable: BTreeMap<String, bool>,
    recovery_status: String,
    recovered_variance_components: Option<Vec<f64>>,
    relative_fit_residual: Option<f64>,
    posterior_log_evidence: f64,
    hypothesis_posterior_weights: BTreeMap<String, f64>,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    schema_version: String,
    data_class: String,
    scenario_id: String,
    scenario_sha256: String,
    status: String,
    true_variance_component_order: Vec<String>,
    true_variance_components: Vec<f64>,
    design_count: usize,
    design_summaries: Vec<DesignSummary>,
    limitations: Vec<String>,
}

fn validate(scenario: &Scenario) -> Result<(), String> {
    if scenario.schema_version != "0.1.0" {
        return Err("unsupported scenario schema version".into());
    }
    if scenario.data_class != "synthetic-known-truth" {
        return Err("only synthetic-known-truth covariance scenarios are accepted".into());
    }
    if scenario.scenario_id.trim().is_empty() {
        return Err("scenario_id must be non-empty".into());
    }
    if scenario.true_variance_components.len() != VarianceComponent::ALL.len()
        || scenario.true_variance_components.iter().any(|v| !v.is_finite() || *v < 0.0)
    {
        return Err("four finite non-negative variance components are required".into());
    }
    if scenario.hypotheses.is_empty() {
        return Err("at least one covariance hypothesis is required".into());
    }
    let mut hypothesis_ids = std::collections::BTreeSet::new();
    for hypothesis in &scenario.hypotheses {
        if hypothesis.model_id.trim().is_empty() || !hypothesis_ids.insert(&hypothesis.model_id) {
            return Err("covariance hypothesis IDs must be non-empty and unique".into());
        }
        if hypothesis.variance_components.len() != VarianceComponent::ALL.len()
            || hypothesis.variance_components.iter().any(|value| !value.is_finite() || *value < 0.0)
            || !hypothesis.prior_weight.is_finite()
            || hypothesis.prior_weight < 0.0
        {
            return Err(format!("covariance hypothesis {:?} has invalid priors/variances", hypothesis.model_id));
        }
    }
    let prior_total: f64 = scenario.hypotheses.iter().map(|item| item.prior_weight).sum();
    if !prior_total.is_finite() || prior_total <= 0.0 {
        return Err("covariance hypothesis prior weights must have positive sum".into());
    }
    if scenario.designs.is_empty() {
        return Err("at least one covariance design is required".into());
    }
    let mut design_ids = std::collections::BTreeSet::new();
    for design in &scenario.designs {
        if design.design_id.trim().is_empty() || !design_ids.insert(&design.design_id) {
            return Err("design IDs must be non-empty and unique".into());
        }
        if design.observations.is_empty() {
            return Err(format!("design {:?} has no observations", design.design_id));
        }
        if design.observed_values.len() != design.observations.len()
            || design.observed_values.iter().any(|value| !value.is_finite())
        {
            return Err(format!("design {:?} has invalid observed values", design.design_id));
        }
        let mut ids = std::collections::BTreeSet::new();
        for observation in &design.observations {
            if observation.observation_id.trim().is_empty()
                || !ids.insert(&observation.observation_id)
                || observation.modality.trim().is_empty()
            {
                return Err(format!("design {:?} has invalid observation identities", design.design_id));
            }
        }
    }
    Ok(())
}

fn summarize_design(
    design_input: &DesignInput,
    true_variances: &[f64],
    hypotheses: &[HypothesisInput],
) -> Result<DesignSummary, String> {
    let observations: Vec<CovarianceObservation> = design_input.observations.iter()
        .map(|observation| CovarianceObservation {
            observation_id: observation.observation_id.clone(),
            modality: observation.modality.clone(),
            physical_support_id: observation.physical_support_id.clone(),
            batch_id: observation.batch_id.clone(),
            instrument_id: observation.instrument_id.clone(),
        })
        .collect();
    let design = CovarianceComponentDesign::new(observations)?;
    let audit = design.audit()?;
    let fisher: CovarianceFisherInformation =
        design.expected_fisher_information(true_variances)?;
    if fisher.structural_rank != audit.structural_rank {
        return Err(format!(
            "design {:?} Fisher rank {} disagrees with structural rank {}",
            design_input.design_id, fisher.structural_rank, audit.structural_rank
        ));
    }
    let mut individually_identifiable = BTreeMap::new();
    for (component, identifiable) in audit.component_ids.iter()
        .zip(&audit.individually_identifiable)
    {
        individually_identifiable.insert(component.as_str().to_owned(), *identifiable);
    }

    let (recovery_status, recovered_variance_components, relative_fit_residual) =
        if audit.fully_identifiable {
            let covariance = design.covariance_from_variances(true_variances)?;
            let recovery = design.recover_exact_variance_components(&covariance)?;
            (
                "exact_synthetic_recovery".to_owned(),
                Some(recovery.variance_estimates),
                Some(recovery.relative_fit_residual),
            )
        } else {
            (
                "structurally_non_identifiable".to_owned(),
                None,
                None,
            )
        };

    let covariance_hypotheses: Vec<CovarianceComponentHypothesis> = hypotheses.iter()
        .map(|hypothesis| CovarianceComponentHypothesis {
            model_id: hypothesis.model_id.clone(),
            prior_weight: hypothesis.prior_weight,
            mean: vec![0.0; design_input.observations.len()],
            variance_components: hypothesis.variance_components.clone(),
        })
        .collect();
    let observed_indices: Vec<usize> = (0..design_input.observations.len()).collect();
    let posterior = design.update_hypothesis_posterior(
        &covariance_hypotheses,
        &observed_indices,
        &design_input.observed_values,
    )?;
    let hypothesis_posterior_weights: BTreeMap<String, f64> =
        posterior.model_ids.iter().cloned()
            .zip(posterior.weights.iter().copied())
            .collect();

    Ok(DesignSummary {
        design_id: design_input.design_id.clone(),
        observation_count: audit.observation_count,
        structural_rank: audit.structural_rank,
        component_count: audit.component_count,
        gram_condition_number: audit.gram_condition_number,
        fisher_information: fisher.information_matrix,
        fisher_eigenvalues: fisher.eigenvalues,
        fisher_rank: fisher.structural_rank,
        fisher_condition_number: fisher.condition_number,
        fisher_log_determinant: fisher.log_determinant,
        fully_identifiable: audit.fully_identifiable,
        individually_identifiable,
        recovery_status,
        recovered_variance_components,
        relative_fit_residual,
        posterior_log_evidence: posterior.log_evidence,
        hypothesis_posterior_weights,
    })
}

fn build_report(scenario: &Scenario, input: &[u8]) -> Result<Report, String> {
    validate(scenario)?;
    let design_summaries = scenario.designs.iter()
        .map(|design| summarize_design(
            design,
            &scenario.true_variance_components,
            &scenario.hypotheses,
        ))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Report {
        schema_version: "0.1.0".into(),
        data_class: "synthetic-known-truth".into(),
        scenario_id: scenario.scenario_id.clone(),
        scenario_sha256: format!("{:x}", Sha256::digest(input)),
        status: "synthetic_covariance_identifiability_reference_only".into(),
        true_variance_component_order: VarianceComponent::ALL.iter()
            .map(|component| component.as_str().to_owned())
            .collect(),
        true_variance_components: scenario.true_variance_components.clone(),
        design_count: design_summaries.len(),
        design_summaries,
        limitations: vec![
            "Structural rank is not finite-sample precision or posterior calibration.".into(),
            "Exact coefficient recovery uses a noiseless synthetic covariance oracle.".into(),
            "Physical support identity must be supplied; overlap is not converted to covariance.".into(),
            "Fisher information assumes one jointly observed Gaussian vector with fixed/known mean and the declared nominal covariance.".into(),
            "If mean/trend parameters are estimated jointly, nuisance-parameter coupling is not Schur-complemented in this reference.".into(),
            "No empirical soil covariance, specimen data or field measurement is used.".into(),
        ],
    })
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <scenario.json> <report.json>",
            args.first().map(String::as_str).unwrap_or("covariance-identifiability-bakeoff")
        ));
    }
    let bytes = fs::read(&args[1]).map_err(|error| format!("scenario read failed: {error}"))?;
    let scenario: Scenario = serde_json::from_slice(&bytes)
        .map_err(|error| format!("scenario parse failed: {error}"))?;
    let report = build_report(&scenario, &bytes)?;
    let output = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("report serialization failed: {error}"))?;
    fs::write(&args[2], output).map_err(|error| format!("report write failed: {error}"))?;
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}
