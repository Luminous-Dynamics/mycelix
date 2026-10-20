//! Bounded executable witness for ExternalInstrumentProfile V1.
//! This is semantic conformance scaffolding, not a ledger or financial authority.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Profile {
    FiatUnit,
    BankDeposit,
    NativeChainAsset,
    TokenAsset,
    Stablecoin,
    WrappedOrBridged,
    MutualCredit,
    IntegralItc,
    ValueflowsRole,
    SymtropySimulationAsset,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Instrument {
    pub profile: Profile,
    pub identity: &'static str,
    pub origin: &'static str,
    pub issuer: Option<&'static str>,
    pub network: Option<&'static str>,
    pub contract_or_mint: Option<&'static str>,
    pub unit: &'static str,
    pub scale: u8,
    pub evidence: &'static str,
    pub valid: bool,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Projection {
    pub source_identity: &'static str,
    pub target_identity: &'static str,
    pub origin: &'static str,
    pub unit: &'static str,
    pub scale: u8,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Accepted,
    Rejected,
}

pub fn validate_identity(i: &Instrument) -> Decision {
    if i.identity.is_empty() || i.origin.is_empty() || i.unit.is_empty()
        || i.evidence.is_empty() || i.claim_ceiling.is_empty() || !i.valid
    {
        return Decision::Rejected;
    }

    match i.profile {
        Profile::FiatUnit => Decision::Accepted,
        Profile::BankDeposit => {
            if i.issuer.is_some() { Decision::Accepted } else { Decision::Rejected }
        }
        Profile::NativeChainAsset => {
            if i.network.is_some() { Decision::Accepted } else { Decision::Rejected }
        }
        Profile::TokenAsset | Profile::Stablecoin => {
            if i.network.is_some() && i.contract_or_mint.is_some() {
                Decision::Accepted
            } else {
                Decision::Rejected
            }
        }
        Profile::WrappedOrBridged => {
            if i.network.is_some() && i.contract_or_mint.is_some() && i.issuer.is_some() {
                Decision::Accepted
            } else {
                Decision::Rejected
            }
        }
        Profile::MutualCredit | Profile::IntegralItc => {
            if i.issuer.is_some() { Decision::Accepted } else { Decision::Rejected }
        }
        Profile::ValueflowsRole => Decision::Accepted,
        Profile::SymtropySimulationAsset => {
            if i.network.is_some() { Decision::Accepted } else { Decision::Rejected }
        }
    }
}

pub fn project(source: &Instrument, target_identity: &'static str) -> Result<Projection, Decision> {
    if validate_identity(source) == Decision::Rejected || target_identity.is_empty() {
        return Err(Decision::Rejected);
    }
    Ok(Projection {
        source_identity: source.identity,
        target_identity,
        origin: source.origin,
        unit: source.unit,
        scale: source.scale,
        claim_ceiling: source.claim_ceiling,
    })
}

pub fn preserves_origin(p: &Projection, source: &Instrument) -> bool {
    p.source_identity == source.identity && p.origin == source.origin
}

pub fn preserves_unit(p: &Projection, source: &Instrument) -> bool {
    p.unit == source.unit && p.scale == source.scale
}

pub fn preserves_claim_ceiling(p: &Projection, source: &Instrument) -> bool {
    p.claim_ceiling == source.claim_ceiling
}

#[cfg(test)]
mod tests {
    use super::*;

    fn token() -> Instrument {
        Instrument {
            profile: Profile::TokenAsset,
            identity: "eip155:1/erc20:0xTOKEN",
            origin: "chain:eip155:1",
            issuer: Some("issuer:A"),
            network: Some("eip155:1"),
            contract_or_mint: Some("0xTOKEN"),
            unit: "TOKEN",
            scale: 18,
            evidence: "tx:abc",
            valid: true,
            claim_ceiling: "on_chain_observation",
        }
    }

    #[test]
    fn exact_identity_and_unit_survive_projection() {
        let source = token();
        let p = project(&source, "mycelix:event:1").unwrap();
        assert!(preserves_origin(&p, &source));
        assert!(preserves_unit(&p, &source));
        assert!(preserves_claim_ceiling(&p, &source));
    }

    #[test]
    fn missing_chain_or_contract_is_rejected() {
        let mut source = token();
        source.network = None;
        assert_eq!(validate_identity(&source), Decision::Rejected);
    }

    #[test]
    fn ticker_cannot_replace_authoritative_identity() {
        let source = token();
        let p = project(&source, "mycelix:event:TOKEN").unwrap();
        assert_ne!(p.target_identity, source.identity);
        assert!(preserves_origin(&p, &source));
    }

    #[test]
    fn scale_mutation_is_detectable() {
        let source = token();
        let mut p = project(&source, "mycelix:event:1").unwrap();
        p.scale = 6;
        assert!(!preserves_unit(&p, &source));
    }

    #[test]
    fn origin_mutation_is_detectable() {
        let source = token();
        let mut p = project(&source, "mycelix:event:1").unwrap();
        p.origin = "local:mint";
        assert!(!preserves_origin(&p, &source));
    }

    #[test]
    fn stablecoin_requires_chain_and_contract() {
        let mut source = token();
        source.profile = Profile::Stablecoin;
        source.contract_or_mint = None;
        assert_eq!(validate_identity(&source), Decision::Rejected);
    }

    #[test]
    fn simulation_asset_requires_world_identity() {
        let mut source = token();
        source.profile = Profile::SymtropySimulationAsset;
        source.network = None;
        assert_eq!(validate_identity(&source), Decision::Rejected);
    }
}
