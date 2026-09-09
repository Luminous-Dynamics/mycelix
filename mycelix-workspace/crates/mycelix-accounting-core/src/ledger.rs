use std::collections::BTreeMap;

use crate::{AccountRef, AuthorityRef, LedgerEntryId, Money};

/// Value classes are intentionally non-substitutable.
///
/// In particular, an ordinary owned balance policy must never silently dispose
/// of value that is merely held, owed, escrowed, or awaiting resolution.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ValueClass {
    OwnedBalance,
    ThirdPartyCustody,
    AccruedObligation,
    EscrowedValue,
    SuspenseValue,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LedgerSide {
    Debit,
    Credit,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccountingEvent {
    ObligationCreated,
    ObligationAdjusted,
    ValueHeld,
    ValueReleased,
    SettlementAuthorized,
    SettlementExecuted,
    SettlementReversed,
    WithholdingApplied,
    FeeApplied,
    FxConverted,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LedgerPosting {
    pub account: AccountRef,
    pub side: LedgerSide,
    pub amount: Money,
    pub value_class: ValueClass,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LedgerEntry {
    id: LedgerEntryId,
    event: AccountingEvent,
    postings: Vec<LedgerPosting>,
    basis: AuthorityRef,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LedgerError {
    NoPostings,
    ZeroPosting { index: usize },
    Overflow { asset: String },
    Unbalanced {
        asset: String,
        debits: u128,
        credits: u128,
    },
}

impl LedgerEntry {
    pub fn new(
        id: LedgerEntryId,
        event: AccountingEvent,
        postings: Vec<LedgerPosting>,
        basis: AuthorityRef,
    ) -> Result<Self, LedgerError> {
        validate_balanced(&postings)?;
        Ok(Self {
            id,
            event,
            postings,
            basis,
        })
    }

    pub fn id(&self) -> &LedgerEntryId {
        &self.id
    }

    pub fn event(&self) -> AccountingEvent {
        self.event
    }

    pub fn postings(&self) -> &[LedgerPosting] {
        &self.postings
    }

    pub fn basis(&self) -> &AuthorityRef {
        &self.basis
    }
}

pub fn validate_balanced(postings: &[LedgerPosting]) -> Result<(), LedgerError> {
    if postings.is_empty() {
        return Err(LedgerError::NoPostings);
    }

    let mut totals: BTreeMap<String, (u128, u128)> = BTreeMap::new();
    for (index, posting) in postings.iter().enumerate() {
        if posting.amount.units() == 0 {
            return Err(LedgerError::ZeroPosting { index });
        }

        let key = posting.amount.asset().as_str().to_owned();
        let totals_for_asset = totals.entry(key.clone()).or_insert((0, 0));
        let slot = match posting.side {
            LedgerSide::Debit => &mut totals_for_asset.0,
            LedgerSide::Credit => &mut totals_for_asset.1,
        };
        *slot = slot
            .checked_add(posting.amount.units())
            .ok_or(LedgerError::Overflow { asset: key })?;
    }

    for (asset, (debits, credits)) in totals {
        if debits != credits {
            return Err(LedgerError::Unbalanced {
                asset,
                debits,
                credits,
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AssetId, RefError};

    fn asset() -> Result<AssetId, RefError> {
        AssetId::new("SAP:micro-v1")
    }

    #[test]
    fn balanced_entry_preserves_value_class_distinction() -> Result<(), RefError> {
        let sap = asset()?;
        let entry = LedgerEntry::new(
            LedgerEntryId::new("entry:royalty:1")?,
            AccountingEvent::ObligationCreated,
            vec![
                LedgerPosting {
                    account: AccountRef::new("expense:royalty")?,
                    side: LedgerSide::Debit,
                    amount: Money::new(sap.clone(), 327),
                    value_class: ValueClass::OwnedBalance,
                },
                LedgerPosting {
                    account: AccountRef::new("payable:creator:alice")?,
                    side: LedgerSide::Credit,
                    amount: Money::new(sap, 327),
                    value_class: ValueClass::AccruedObligation,
                },
            ],
            AuthorityRef::new("rights-resolution:29")?,
        );
        assert!(entry.is_ok());
        Ok(())
    }

    #[test]
    fn unbalanced_entry_fails_closed() -> Result<(), RefError> {
        let sap = asset()?;
        let result = LedgerEntry::new(
            LedgerEntryId::new("entry:bad")?,
            AccountingEvent::FeeApplied,
            vec![
                LedgerPosting {
                    account: AccountRef::new("cash")?,
                    side: LedgerSide::Debit,
                    amount: Money::new(sap.clone(), 5),
                    value_class: ValueClass::OwnedBalance,
                },
                LedgerPosting {
                    account: AccountRef::new("fee")?,
                    side: LedgerSide::Credit,
                    amount: Money::new(sap, 4),
                    value_class: ValueClass::OwnedBalance,
                },
            ],
            AuthorityRef::new("policy:fee-v1")?,
        );
        assert!(matches!(result, Err(LedgerError::Unbalanced { .. })));
        Ok(())
    }

    #[test]
    fn zero_posting_is_not_a_balancing_escape_hatch() -> Result<(), RefError> {
        let sap = asset()?;
        let result = validate_balanced(&[LedgerPosting {
            account: AccountRef::new("cash")?,
            side: LedgerSide::Debit,
            amount: Money::zero(sap),
            value_class: ValueClass::OwnedBalance,
        }]);
        assert_eq!(result, Err(LedgerError::ZeroPosting { index: 0 }));
        Ok(())
    }
}
