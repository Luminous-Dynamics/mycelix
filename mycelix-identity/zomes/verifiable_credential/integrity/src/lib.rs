        let json = serde_json::to_string(&dc).unwrap();
        let back: DerivedCredential = serde_json::from_str(&json).unwrap();
        assert_eq!(dc, back);
    }
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn test_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(2_000_000_000_000_000),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),