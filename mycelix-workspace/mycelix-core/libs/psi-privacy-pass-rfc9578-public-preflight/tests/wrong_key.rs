use futures::executor::block_on;
use psi_privacy_pass_rfc9578_public_preflight::{
    PreflightError, verify_public_token_via_pinned_backend_v1,
};

const WRONG_PUBLIC_KEY_SPKI_DER_HEX: &str = "30820152303d06092a864886f70d01010a3030a00d300b0609608648016503040202a11a301806092a864886f70d010108300b0609608648016503040202a2030201300382010f003082010a0282010100d0b87841943c6b4923e9d05733e32733eaebad32b89337d6aca739a4aeaf568eedad36b3b1ef3b01c540b1034780c2ae86d9becfae5fbc0ca1a8fd0b9cd0e4fdc109a4963e1c64aed8c08f294460f2042440fbb7646a939866fd1dbbde91786bec0cbdc88a4baf3915b303f0ed9e86cfbf38b922990da82e68cac72714a6d5f0fed457969ce3736082bf2016cca0c93396eb69765a48008043bf583a86a932ac8fb42939da54c48d4801732a902ddf229e3cd60dd0ccb0a91569b104d1bbc75e3514edb9afa56d144396e56b86c49ee58d0710d38671153735605a29fb37b6289fe7833fca44224d5b02a0a320f7726b84fea229409ebbd1784da1d5972063cb0203010001";

fn decode_hex(value: &str) -> Vec<u8> {
    assert_eq!(value.len() % 2, 0);
    value
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let high = (pair[0] as char).to_digit(16).unwrap();
            let low = (pair[1] as char).to_digit(16).unwrap();
            ((high << 4) | low) as u8
        })
        .collect()
}

#[test]
fn valid_but_different_rsa_key_is_rejected_by_full_key_identity() {
    let fixture: serde_json::Value = serde_json::from_str(include_str!(
        "../fixtures/public_go_vector_0.json"
    ))
    .unwrap();
    let token = decode_hex(fixture["token_hex"].as_str().unwrap());
    let wrong_spki = decode_hex(WRONG_PUBLIC_KEY_SPKI_DER_HEX);

    let result = block_on(verify_public_token_via_pinned_backend_v1(
        &wrong_spki,
        &token,
        None,
    ));
    assert_eq!(result, Err(PreflightError::TokenKeyIdMismatch));
}
