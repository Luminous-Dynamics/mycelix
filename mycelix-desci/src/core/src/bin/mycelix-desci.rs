// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Mycelix-DeSci CLI
//!
//! Command-line interface for interacting with Mycelix-DeSci

use clap::{Parser, Subcommand};
use mycelix_desci_core::{Config, Error, Result, logging};
use std::path::PathBuf;

mod commands;

#[derive(Parser)]
#[command(name = "mycelix-desci")]
#[command(about = "Mycelix-DeSci - Verifiable Infrastructure for Decentralized Science", long_about = None)]
#[command(version)]
struct Cli {
    /// Configuration file path
    #[arg(short, long, global = true)]
    config: Option<PathBuf>,

    /// Log level (debug, info, warn, error)
    #[arg(short = 'l', long, global = true)]
    log_level: Option<String>,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Initialize configuration
    Init {
        /// Output directory for configuration
        #[arg(short, long, default_value = ".mycelix")]
        output: PathBuf,
    },

    /// Calculate hash of a file or directory
    Hash {
        /// File or directory to hash
        path: PathBuf,

        /// Hash algorithm (blake3, sha256)
        #[arg(short = 'a', long, default_value = "blake3")]
        algorithm: String,
    },

    /// Upload a dataset and create a claim
    Upload {
        /// Dataset file to upload
        file: PathBuf,

        /// Epistemic tier (E0-E4)
        #[arg(short, long, default_value = "E0")]
        tier: String,

        /// Category (genomics, longevity, climate, etc.)
        #[arg(short, long)]
        category: String,

        /// Description of the dataset
        #[arg(short, long)]
        description: String,

        /// Provenance information
        #[arg(short, long)]
        provenance: Option<String>,

        /// License
        #[arg(short = 'L', long)]
        license: Option<String>,

        /// Keywords (comma-separated)
        #[arg(short, long)]
        keywords: Option<String>,
    },

    /// Verify a claim
    Verify {
        /// Claim ID to verify
        claim_id: String,

        /// Dataset file (optional, for hash verification)
        #[arg(short, long)]
        file: Option<PathBuf>,
    },

    /// Query claims
    Query {
        /// Category filter
        #[arg(short, long)]
        category: Option<String>,

        /// Minimum epistemic tier
        #[arg(short = 't', long)]
        min_tier: Option<String>,

        /// Keyword search
        #[arg(short, long)]
        keywords: Option<String>,

        /// Output format (json, text, table)
        #[arg(short = 'f', long, default_value = "table")]
        format: String,

        /// Maximum number of results
        #[arg(short = 'n', long, default_value = "10")]
        limit: usize,
    },

    /// Display information about a claim
    Info {
        /// Claim ID
        claim_id: String,

        /// Output format (json, text)
        #[arg(short = 'f', long, default_value = "text")]
        format: String,
    },

    /// Initialize a new append-only scientific credential registry
    CredentialRegistryInit {
        /// Durable credential registry file to create
        #[arg(long)]
        registry: PathBuf,

        /// Initial registry administrator actor identifier
        #[arg(long)]
        actor: String,

        /// Mode-0600 file containing the administrator's 32-byte Ed25519 seed
        #[arg(long)]
        signing_key_file: PathBuf,

        /// Mode-0600 file containing the independent acceptance-service seed
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,

        /// Public bootstrap trust JSON file to create
        #[arg(long)]
        bootstrap_trust_output: PathBuf,

        /// Optional initial organization membership
        #[arg(long)]
        organization: Option<String>,
    },

    /// Register an actor in an initialized scientific credential registry
    CredentialActorRegister {
        /// Durable credential registry file
        #[arg(long)]
        registry: PathBuf,

        /// JSON array of trusted credential-registry bootstrap public keys
        #[arg(long)]
        bootstrap_trust_file: PathBuf,

        /// Existing registry administrator actor identifier
        #[arg(long)]
        administrator: String,

        /// Mode-0600 file containing the administrator's 32-byte Ed25519 seed
        #[arg(long)]
        administrator_signing_key_file: PathBuf,

        /// Mode-0600 file containing the independent acceptance-service seed
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,

        /// Optional JSON array of historical trusted acceptance-service public keys
        #[arg(long)]
        acceptance_trust_file: Option<PathBuf>,

        /// New actor identifier
        #[arg(long)]
        actor: String,

        /// New actor's 32-byte Ed25519 public key as 64 hexadecimal characters
        #[arg(long)]
        public_key: String,

        /// Scientific role; repeat for multiple roles
        #[arg(long = "role", required = true)]
        roles: Vec<String>,

        /// Organization membership; repeat for multiple organizations
        #[arg(long = "organization")]
        organizations: Vec<String>,
    },

    /// Initialize threshold governance over an existing credential registry
    CredentialGovernanceInit {
        #[arg(long)]
        registry: PathBuf,
        #[arg(long)]
        bootstrap_trust_file: PathBuf,
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,
        #[arg(long)]
        acceptance_trust_file: Option<PathBuf>,
        #[arg(long)]
        governance: PathBuf,
        #[arg(long)]
        administrator: String,
        #[arg(long)]
        administrator_signing_key_file: PathBuf,
        #[arg(long, default_value_t = 2)]
        approval_threshold: u16,
        #[arg(long, default_value_t = 86_400)]
        activation_delay_seconds: u64,
        #[arg(long, default_value_t = 604_800)]
        proposal_ttl_seconds: u64,
    },

    /// Open a threshold-governed credential or authority-service proposal
    CredentialGovernancePropose {
        #[arg(long)]
        registry: PathBuf,
        #[arg(long)]
        bootstrap_trust_file: PathBuf,
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,
        #[arg(long)]
        acceptance_trust_file: Option<PathBuf>,
        #[arg(long)]
        governance: PathBuf,
        #[arg(long)]
        administrator: String,
        #[arg(long)]
        administrator_signing_key_file: PathBuf,
        /// JSON-encoded CredentialGovernanceAction
        #[arg(long)]
        action_file: PathBuf,
        #[arg(long)]
        reason: String,
    },

    /// Add one unique administrator approval to a governance proposal
    CredentialGovernanceApprove {
        #[arg(long)]
        registry: PathBuf,
        #[arg(long)]
        bootstrap_trust_file: PathBuf,
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,
        #[arg(long)]
        acceptance_trust_file: Option<PathBuf>,
        #[arg(long)]
        governance: PathBuf,
        #[arg(long)]
        administrator: String,
        #[arg(long)]
        administrator_signing_key_file: PathBuf,
        #[arg(long)]
        proposal_id: uuid::Uuid,
    },

    /// Cancel a pending governance proposal
    CredentialGovernanceCancel {
        #[arg(long)]
        registry: PathBuf,
        #[arg(long)]
        bootstrap_trust_file: PathBuf,
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,
        #[arg(long)]
        acceptance_trust_file: Option<PathBuf>,
        #[arg(long)]
        governance: PathBuf,
        #[arg(long)]
        administrator: String,
        #[arg(long)]
        administrator_signing_key_file: PathBuf,
        #[arg(long)]
        proposal_id: uuid::Uuid,
        #[arg(long)]
        reason: String,
    },

    /// Execute a mature threshold-approved governance proposal
    CredentialGovernanceExecute {
        #[arg(long)]
        registry: PathBuf,
        #[arg(long)]
        bootstrap_trust_file: PathBuf,
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,
        #[arg(long)]
        acceptance_trust_file: Option<PathBuf>,
        #[arg(long)]
        governance: PathBuf,
        #[arg(long)]
        administrator: String,
        #[arg(long)]
        administrator_signing_key_file: PathBuf,
        #[arg(long)]
        proposal_id: uuid::Uuid,
    },

    /// Publish and optionally export a transparency checkpoint
    CredentialGovernanceCheckpoint {
        #[arg(long)]
        registry: PathBuf,
        #[arg(long)]
        bootstrap_trust_file: PathBuf,
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,
        #[arg(long)]
        acceptance_trust_file: Option<PathBuf>,
        #[arg(long)]
        governance: PathBuf,
        #[arg(long)]
        administrator: String,
        #[arg(long)]
        administrator_signing_key_file: PathBuf,
        #[arg(long)]
        output: Option<PathBuf>,
    },

    /// Sign and append an external witness attestation for a published checkpoint
    CredentialGovernanceWitnessCheckpoint {
        #[arg(long)]
        registry: PathBuf,
        #[arg(long)]
        bootstrap_trust_file: PathBuf,
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,
        #[arg(long)]
        acceptance_trust_file: Option<PathBuf>,
        #[arg(long)]
        governance: PathBuf,
        #[arg(long)]
        witness_actor: String,
        #[arg(long)]
        witness_organization: String,
        #[arg(long)]
        witness_signing_key_file: PathBuf,
        #[arg(long)]
        checkpoint_file: PathBuf,
    },

    /// Sign a short-lived authority-write lease for one governed PostgreSQL primary
    AuthorityWriteLeaseSign {
        /// JSON file containing an unsigned AuthorityWriteLease
        #[arg(long)]
        lease_file: PathBuf,
        /// Mode-0600 file containing the independent lease issuer Ed25519 seed
        #[arg(long)]
        signing_key_file: PathBuf,
        /// Output path for the signed lease JSON; must not already exist
        #[arg(long)]
        output: PathBuf,
    },

    /// Atomically import validated file authority journals into an empty PostgreSQL schema
    CredentialAuthorityImportPostgres {
        #[arg(long)]
        database_url: String,
        #[arg(long)]
        registry: PathBuf,
        #[arg(long)]
        governance: PathBuf,
        #[arg(long)]
        bootstrap_trust_file: PathBuf,
        #[arg(long)]
        acceptance_signing_key_file: PathBuf,
        #[arg(long)]
        acceptance_trust_file: Option<PathBuf>,
        #[arg(long)]
        outbox_signing_key_file: PathBuf,
        /// Deployment identifier bound by the bootstrap write lease
        #[arg(long)]
        deployment_id: String,
        /// Current signed bootstrap authority-write lease JSON
        #[arg(long)]
        write_lease_file: PathBuf,
        /// JSON array of trusted authority-write lease public keys
        #[arg(long)]
        write_lease_trust_file: PathBuf,
        #[arg(long, default_value_t = false)]
        auto_migrate: bool,
    },

    /// Import mutable legacy claim JSON as explicitly unassessed canonical history
    MigrateLegacy {
        /// Legacy claim JSON file or directory containing JSON claim files
        input: PathBuf,

        /// Durable canonical event-log directory
        #[arg(long)]
        event_log: PathBuf,

        /// Migration service actor identifier
        #[arg(long)]
        actor: String,

        /// File containing the migration actor's 32-byte Ed25519 seed
        #[arg(long)]
        signing_key_file: PathBuf,

        /// Durable append-only scientific credential registry file
        #[arg(long)]
        credential_registry: PathBuf,

        /// JSON array of trusted credential-registry bootstrap public keys
        #[arg(long)]
        credential_bootstrap_trust_file: PathBuf,

        /// Independent authority-receipt audit directory
        #[arg(long)]
        authority_audit: PathBuf,

        /// File containing the receipt service's 32-byte Ed25519 seed
        #[arg(long)]
        receipt_signing_key_file: PathBuf,

        /// Optional JSON array of historical trusted receipt public keys
        #[arg(long)]
        receipt_trust_file: Option<PathBuf>,

        /// Optional organization under whose authority the migration runs
        #[arg(long)]
        organization: Option<String>,

        /// Identifier for the source legacy system
        #[arg(long, default_value = "mycelix-desci-legacy-json")]
        source_system: String,

        /// Optional path for the machine-readable migration report
        #[arg(long)]
        report: Option<PathBuf>,
    },

    /// Display configuration
    Config {
        /// Show configuration
        #[command(subcommand)]
        action: ConfigAction,
    },
}

#[derive(Subcommand)]
enum ConfigAction {
    /// Show current configuration
    Show,

    /// Validate configuration
    Validate,
}

#[tokio::main]
async fn main() {
    if let Err(e) = run().await {
        eprintln!("Error: {}", e);
        std::process::exit(1);
    }
}

async fn run() -> Result<()> {
    let cli = Cli::parse();

    // Load configuration
    let mut config = if let Some(config_path) = cli.config {
        Config::from_file(config_path)?
    } else {
        Config::load().unwrap_or_default()
    };

    // Override log level if specified
    if let Some(level) = cli.log_level {
        config.logging.level = level;
    }

    // Initialize logging
    logging::init(&config.logging);

    // Execute command
    match cli.command {
        Commands::Init { output } => commands::init::execute(output).await,
        Commands::Hash { path, algorithm } => commands::hash::execute(path, algorithm).await,
        Commands::Upload {
            file,
            tier,
            category,
            description,
            provenance,
            license,
            keywords,
        } => {
            commands::upload::execute(
                file,
                tier,
                category,
                description,
                provenance,
                license,
                keywords,
            )
            .await
        }
        Commands::Verify { claim_id, file } => commands::verify::execute(claim_id, file).await,
        Commands::Query {
            category,
            min_tier,
            keywords,
            format,
            limit,
        } => commands::query::execute(category, min_tier, keywords, format, limit).await,
        Commands::Info { claim_id, format } => commands::info::execute(claim_id, format).await,
        Commands::CredentialRegistryInit {
            registry,
            actor,
            signing_key_file,
            acceptance_signing_key_file,
            bootstrap_trust_output,
            organization,
        } => {
            commands::credential_registry::initialize(
                registry,
                actor,
                signing_key_file,
                acceptance_signing_key_file,
                bootstrap_trust_output,
                organization,
            )
            .await
        }
        Commands::CredentialActorRegister {
            registry,
            bootstrap_trust_file,
            administrator,
            administrator_signing_key_file,
            acceptance_signing_key_file,
            acceptance_trust_file,
            actor,
            public_key,
            roles,
            organizations,
        } => {
            commands::credential_registry::register_actor(
                registry,
                bootstrap_trust_file,
                administrator,
                administrator_signing_key_file,
                acceptance_signing_key_file,
                acceptance_trust_file,
                actor,
                public_key,
                roles,
                organizations,
            )
            .await
        }
        Commands::CredentialGovernanceInit {
            registry,
            bootstrap_trust_file,
            acceptance_signing_key_file,
            acceptance_trust_file,
            governance,
            administrator,
            administrator_signing_key_file,
            approval_threshold,
            activation_delay_seconds,
            proposal_ttl_seconds,
        } => {
            commands::credential_governance::initialize(
                registry,
                bootstrap_trust_file,
                acceptance_signing_key_file,
                acceptance_trust_file,
                governance,
                administrator,
                administrator_signing_key_file,
                approval_threshold,
                activation_delay_seconds,
                proposal_ttl_seconds,
            )
            .await
        }
        Commands::CredentialGovernancePropose {
            registry,
            bootstrap_trust_file,
            acceptance_signing_key_file,
            acceptance_trust_file,
            governance,
            administrator,
            administrator_signing_key_file,
            action_file,
            reason,
        } => {
            commands::credential_governance::propose(
                registry,
                bootstrap_trust_file,
                acceptance_signing_key_file,
                acceptance_trust_file,
                governance,
                administrator,
                administrator_signing_key_file,
                action_file,
                reason,
            )
            .await
        }
        Commands::CredentialGovernanceApprove {
            registry,
            bootstrap_trust_file,
            acceptance_signing_key_file,
            acceptance_trust_file,
            governance,
            administrator,
            administrator_signing_key_file,
            proposal_id,
        } => {
            commands::credential_governance::approve(
                registry,
                bootstrap_trust_file,
                acceptance_signing_key_file,
                acceptance_trust_file,
                governance,
                administrator,
                administrator_signing_key_file,
                proposal_id,
            )
            .await
        }
        Commands::CredentialGovernanceCancel {
            registry,
            bootstrap_trust_file,
            acceptance_signing_key_file,
            acceptance_trust_file,
            governance,
            administrator,
            administrator_signing_key_file,
            proposal_id,
            reason,
        } => {
            commands::credential_governance::cancel(
                registry,
                bootstrap_trust_file,
                acceptance_signing_key_file,
                acceptance_trust_file,
                governance,
                administrator,
                administrator_signing_key_file,
                proposal_id,
                reason,
            )
            .await
        }
        Commands::CredentialGovernanceExecute {
            registry,
            bootstrap_trust_file,
            acceptance_signing_key_file,
            acceptance_trust_file,
            governance,
            administrator,
            administrator_signing_key_file,
            proposal_id,
        } => {
            commands::credential_governance::execute(
                registry,
                bootstrap_trust_file,
                acceptance_signing_key_file,
                acceptance_trust_file,
                governance,
                administrator,
                administrator_signing_key_file,
                proposal_id,
            )
            .await
        }
        Commands::CredentialGovernanceCheckpoint {
            registry,
            bootstrap_trust_file,
            acceptance_signing_key_file,
            acceptance_trust_file,
            governance,
            administrator,
            administrator_signing_key_file,
            output,
        } => {
            commands::credential_governance::checkpoint(
                registry,
                bootstrap_trust_file,
                acceptance_signing_key_file,
                acceptance_trust_file,
                governance,
                administrator,
                administrator_signing_key_file,
                output,
            )
            .await
        }
        Commands::CredentialGovernanceWitnessCheckpoint {
            registry,
            bootstrap_trust_file,
            acceptance_signing_key_file,
            acceptance_trust_file,
            governance,
            witness_actor,
            witness_organization,
            witness_signing_key_file,
            checkpoint_file,
        } => {
            commands::credential_governance::witness_checkpoint(
                registry,
                bootstrap_trust_file,
                acceptance_signing_key_file,
                acceptance_trust_file,
                governance,
                witness_actor,
                witness_organization,
                witness_signing_key_file,
                checkpoint_file,
            )
            .await
        }
        Commands::AuthorityWriteLeaseSign {
            lease_file,
            signing_key_file,
            output,
        } => commands::postgres_authority::sign_write_lease(lease_file, signing_key_file, output),
        Commands::CredentialAuthorityImportPostgres {
            database_url,
            registry,
            governance,
            bootstrap_trust_file,
            acceptance_signing_key_file,
            acceptance_trust_file,
            outbox_signing_key_file,
            deployment_id,
            write_lease_file,
            write_lease_trust_file,
            auto_migrate,
        } => {
            commands::postgres_authority::import_file_authority(
                database_url,
                registry,
                governance,
                bootstrap_trust_file,
                acceptance_signing_key_file,
                acceptance_trust_file,
                outbox_signing_key_file,
                deployment_id,
                write_lease_file,
                write_lease_trust_file,
                auto_migrate,
            )
            .await
        }
        Commands::MigrateLegacy {
            input,
            event_log,
            actor,
            signing_key_file,
            credential_registry,
            credential_bootstrap_trust_file,
            authority_audit,
            receipt_signing_key_file,
            receipt_trust_file,
            organization,
            source_system,
            report,
        } => {
            commands::migrate_legacy::execute(
                input,
                event_log,
                actor,
                signing_key_file,
                credential_registry,
                credential_bootstrap_trust_file,
                authority_audit,
                receipt_signing_key_file,
                receipt_trust_file,
                organization,
                source_system,
                report,
            )
            .await
        }
        Commands::Config { action } => match action {
            ConfigAction::Show => commands::config::show(config).await,
            ConfigAction::Validate => commands::config::validate(config).await,
        },
    }
}
