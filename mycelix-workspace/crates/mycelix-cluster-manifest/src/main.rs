use mycelix_cluster_manifest::{
    BridgeDeclaration, BridgeDirection, CivicTier, ClusterCatalog, ClusterDependency,
    ClusterManifest, DataSensitivity, SecurityPolicy,
};
use serde_json;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut catalog = ClusterCatalog::default();

    // Prototyping: Add a few core manifests to test the exporter
    catalog.clusters.push(ClusterManifest {
        id: "commons".to_string(),
        name: "Commons".to_string(),
        bio_name: "Homeostasis".to_string(),
        version: "0.1.0".to_string(),
        description: "Commons cluster".to_string(),
        min_tier: CivicTier::Participant,
        security_policy: SecurityPolicy::Standard,
        happ_role: "commons".to_string(),
        zomes: vec!["land".to_string(), "care".to_string()],
        dependencies: vec![],
        bridges: vec![],
        entry_types: vec![],
        color_primary: "#22C55E".to_string(),
        color_glow: "#86EFAC".to_string(),
        frontend_url: None,
    });

    println!("{}", serde_json::to_string_pretty(&catalog)?);
    Ok(())
}
