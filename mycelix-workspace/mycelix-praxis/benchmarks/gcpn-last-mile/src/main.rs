use gcpn_last_mile::{fixture_evaluations, CHANNEL_NAMES};

fn main() {
    match fixture_evaluations() {
        Ok(evaluations) => {
            println!("GCPN synthetic multi-channel last-mile benchmark v0.1");
            println!("scenario,intended,unique_reached,accessible_reached,uncovered,reached_without_accessible_format,duplicate_deliveries,cell_broadcast_receipts,radio_tv_receipts,mobile_data_receipts,community_relay_receipts,delivery_coverage,accessible_coverage,accessibility_among_reached");
            for evaluation in &evaluations {
                println!(
                    "{},{},{},{},{},{},{},{},{},{},{},{:.4},{:.4},{:.4}",
                    evaluation.scenario_id,
                    evaluation.intended_receivers,
                    evaluation.unique_receivers_reached,
                    evaluation.accessible_receivers_reached,
                    evaluation.uncovered_receivers,
                    evaluation.reached_without_accessible_format,
                    evaluation.duplicate_channel_deliveries,
                    evaluation.channel_receipts[0],
                    evaluation.channel_receipts[1],
                    evaluation.channel_receipts[2],
                    evaluation.channel_receipts[3],
                    evaluation.delivery_coverage(),
                    evaluation.accessible_coverage(),
                    evaluation.accessibility_among_reached()
                );
            }

            println!();
            println!("scenario,cohort,intended,unique_reached,accessible_reached,uncovered,reached_without_accessible_format,duplicate_deliveries");
            for evaluation in &evaluations {
                for cohort in &evaluation.cohorts {
                    println!(
                        "{},{},{},{},{},{},{},{}",
                        evaluation.scenario_id,
                        cohort.cohort_id,
                        cohort.intended,
                        cohort.unique_delivered,
                        cohort.accessible_delivered,
                        cohort.uncovered(),
                        cohort.delivered_without_accessible_format(),
                        cohort.duplicate_deliveries
                    );
                }
            }

            println!();
            println!("channel_index,channel");
            for (index, name) in CHANNEL_NAMES.iter().enumerate() {
                println!("{index},{name}");
            }
            println!("interpretation: synthetic receiver arithmetic only; not observed delivery, comprehension, action, or real-world coverage");
        }
        Err(error) => {
            eprintln!("fixture error: {error}");
            std::process::exit(2);
        }
    }
}
