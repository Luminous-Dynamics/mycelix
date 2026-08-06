// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Export a point-in-time snapshot of real orbital objects (satellites +
//! debris) as a Sol Atlas `orbital.json` dataset.
//!
//! Fetches live TLE data from CelesTrak (no auth required), propagates each
//! object to the current epoch via real SGP4 (`Propagator`), converts the
//! TEME state vector to a geodetic sub-satellite point (`subsatellite_point`),
//! and writes a flat JSON array matching Sol Atlas's other datasets'
//! convention (see `sol-atlas-core/src/types.rs`).
//!
//! This is a **point-in-time snapshot**, not a live feed — same transparency
//! pattern as `sol-atlas-leptos/src/data/reactor_twin.rs`'s replayed
//! simulation data. Re-run this example to refresh it.
//!
//! Usage: `cargo run --release --example sol_atlas_export -- [output_path]`
//! (defaults to `orbital.json` in the current directory)

use chrono::Utc;
use orbital_mechanics::coordinates::subsatellite_point;
use orbital_mechanics::propagator::Propagator;
use orbital_mechanics::tle::TwoLineElement;
use serde::Serialize;

#[derive(Serialize)]
struct OrbitalObject {
    norad_id: u32,
    name: String,
    object_type: &'static str,
    lat: f64,
    lon: f64,
    alt_km: f64,
}

/// CelesTrak TLE groups to sample. Kept small (a real satellite constellation
/// has thousands of members) so the globe stays legible — see
/// `sol-atlas-bevy`/`sol-atlas-leptos`'s marker-density conventions for other
/// point layers.
const GROUPS: &[(&str, &str, &'static str)] = &[
    ("stations", "GROUP=stations&FORMAT=tle", "Payload"),
    ("gps", "GROUP=gps-ops&FORMAT=tle", "Payload"),
    ("starlink", "GROUP=starlink&FORMAT=tle", "Payload"),
    ("debris", "GROUP=iridium-33-debris&FORMAT=tle", "Debris"),
];

/// Per-group cap so one large constellation (e.g. Starlink, thousands of
/// members) doesn't dominate the snapshot.
const PER_GROUP_LIMIT: usize = 40;

fn classify(name: &str, default_type: &'static str) -> &'static str {
    let upper = name.to_uppercase();
    if upper.contains("DEB") {
        "Debris"
    } else if upper.contains("R/B") {
        "RocketBody"
    } else {
        default_type
    }
}

fn fetch_group(group_query: &str) -> anyhow::Result<String> {
    let url = format!("https://celestrak.org/NORAD/elements/gp.php?{group_query}");
    let text = reqwest::blocking::get(&url)?.text()?;
    Ok(text)
}

fn parse_tle_blocks(raw: &str) -> Vec<TwoLineElement> {
    let lines: Vec<&str> = raw.lines().collect();
    lines
        .chunks(3)
        .filter(|chunk| chunk.len() == 3)
        .filter_map(|chunk| {
            let block = chunk.join("\n");
            TwoLineElement::parse(&block).ok()
        })
        .collect()
}

fn main() -> anyhow::Result<()> {
    let output = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "orbital.json".to_string());
    let now = Utc::now();
    let mut objects = Vec::new();

    for (label, query, default_type) in GROUPS {
        eprintln!("Fetching {label}...");
        let raw = match fetch_group(query) {
            Ok(text) => text,
            Err(e) => {
                eprintln!("  skipped {label}: {e}");
                continue;
            }
        };
        let tles = parse_tle_blocks(&raw);
        eprintln!("  parsed {} TLEs", tles.len());

        for tle in tles.into_iter().take(PER_GROUP_LIMIT) {
            let Ok(propagator) = Propagator::from_tle(&tle) else {
                continue;
            };
            let Ok(state) = propagator.propagate_to(now) else {
                continue;
            };
            let geo = subsatellite_point(&state.state, now);
            let name = tle
                .name
                .clone()
                .unwrap_or_else(|| format!("NORAD {}", tle.norad_id));
            objects.push(OrbitalObject {
                norad_id: tle.norad_id,
                object_type: classify(&name, default_type),
                name,
                lat: geo.latitude_deg,
                lon: geo.longitude_deg,
                alt_km: geo.altitude_km,
            });
        }
    }

    eprintln!("Total objects: {}", objects.len());
    let json = serde_json::to_string_pretty(&objects)?;
    std::fs::write(&output, &json)?;
    eprintln!("Wrote {} ({} bytes)", output, json.len());
    Ok(())
}
