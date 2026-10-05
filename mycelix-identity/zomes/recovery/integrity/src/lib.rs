                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete their entries".into(),
                ));
            }

            match original.action().entry_type() {
                Some(EntryType::App(entry_def))
                    if entry_def.entry_index() == EntryDefIndex::from(0)
                        || entry_def.entry_index() == EntryDefIndex::from(1)
                        || entry_def.entry_index() == EntryDefIndex::from(2)
                        || entry_def.entry_index() == EntryDefIndex::from(3)
                        || entry_def.entry_index() == EntryDefIndex::from(4)
                        || entry_def.entry_index() == EntryDefIndex::from(5) =>
                {
                    // EntryTypes declaration order:
                    // RecoveryConfig, RecoveryRequest, RecoveryVote,
                    // RecoveryApprovalCertificate, SelfRecoveryConfig,
                    // SelfRecoveryRequest. All are security-sensitive state
                    // and must remain append-only.
                    Ok(ValidateCallbackResult::Invalid(
                        "Recovery security entries cannot be deleted".into(),
                    ))
                }
                _ => Ok(ValidateCallbackResult::Valid),
            }