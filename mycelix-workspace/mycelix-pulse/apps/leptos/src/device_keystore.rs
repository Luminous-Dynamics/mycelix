// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Same-device-only custody for Pulse V2 private material.
//!
//! The non-exportable AES wrapping key is structured-cloned into IndexedDB.
//! Hybrid private seeds are persisted only as AES-GCM ciphertext. Clearing the
//! browser profile or moving to another device intentionally makes the keys
//! unavailable in the working alpha.

use js_sys::{Promise, Uint8Array};
use mycelix_crypto::pulse_v2::{MlDsaSigner, RecipientKeyPair};
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::JsFuture;
use zeroize::Zeroize;

const DEVICE_IDENTITY_RECORD: &str = "pulse-v2-device-identity";
const RECIPIENT_SECRET_BYTES: usize = 96;

#[wasm_bindgen(inline_js = r#"
const DB_NAME = 'mycelix-pulse-device-keys-v2';
const STORE = 'device_keys';
const WRAPPING_KEY = 'non-exportable-aes-gcm-wrapping-key';

function requestResult(request) {
  return new Promise((resolve, reject) => {
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error || new Error('IndexedDB request failed'));
  });
}

function transactionDone(transaction) {
  return new Promise((resolve, reject) => {
    transaction.oncomplete = () => resolve();
    transaction.onerror = () => reject(transaction.error || new Error('IndexedDB transaction failed'));
    transaction.onabort = () => reject(transaction.error || new Error('IndexedDB transaction aborted'));
  });
}

async function openDatabase() {
  const request = indexedDB.open(DB_NAME, 1);
  request.onupgradeneeded = () => {
    const db = request.result;
    if (!db.objectStoreNames.contains(STORE)) db.createObjectStore(STORE);
  };
  return requestResult(request);
}

async function getRecord(db, name) {
  const tx = db.transaction(STORE, 'readonly');
  return requestResult(tx.objectStore(STORE).get(name));
}

async function putRecord(db, name, value) {
  const tx = db.transaction(STORE, 'readwrite');
  tx.objectStore(STORE).put(value, name);
  await transactionDone(tx);
}

async function wrappingKey(db) {
  let key = await getRecord(db, WRAPPING_KEY);
  if (!key) {
    key = await crypto.subtle.generateKey({name: 'AES-GCM', length: 256}, false, ['encrypt', 'decrypt']);
    await putRecord(db, WRAPPING_KEY, key);
  }
  if (!(key instanceof CryptoKey) || key.extractable || key.algorithm.name !== 'AES-GCM') {
    throw new Error('Invalid device wrapping key');
  }
  return key;
}

export async function pulseStoreDeviceSecret(name, secret) {
  const db = await openDatabase();
  try {
    const key = await wrappingKey(db);
    const nonce = crypto.getRandomValues(new Uint8Array(12));
    const additionalData = new TextEncoder().encode('mycelix-pulse/device-secret/v2\0' + name);
    const ciphertext = await crypto.subtle.encrypt(
      {name: 'AES-GCM', iv: nonce, additionalData}, key, secret
    );
    await putRecord(db, name, {version: 2, nonce, ciphertext});
  } finally {
    db.close();
  }
}

export async function pulseLoadDeviceSecret(name) {
  const db = await openDatabase();
  try {
    const key = await wrappingKey(db);
    const record = await getRecord(db, name);
    if (!record || record.version !== 2) throw new Error('Pulse V2 device secret is unavailable');
    const additionalData = new TextEncoder().encode('mycelix-pulse/device-secret/v2\0' + name);
    const plaintext = await crypto.subtle.decrypt(
      {name: 'AES-GCM', iv: record.nonce, additionalData}, key, record.ciphertext
    );
    return new Uint8Array(plaintext);
  } finally {
    db.close();
  }
}

export async function pulseHasDeviceSecret(name) {
  const db = await openDatabase();
  try {
    const record = await getRecord(db, name);
    return Boolean(record && record.version === 2);
  } finally {
    db.close();
  }
}
"#)]
extern "C" {
    #[wasm_bindgen(js_name = pulseStoreDeviceSecret)]
    fn store_device_secret(name: &str, secret: &Uint8Array) -> Promise;

    #[wasm_bindgen(js_name = pulseLoadDeviceSecret)]
    fn load_device_secret(name: &str) -> Promise;

    #[wasm_bindgen(js_name = pulseHasDeviceSecret)]
    fn has_device_secret(name: &str) -> Promise;
}

pub struct HybridDeviceIdentity {
    pub recipient: RecipientKeyPair,
    pub ml_dsa: MlDsaSigner,
}

#[derive(Clone, Debug)]
pub struct HybridDevicePublicBundle {
    pub x25519_public_key: [u8; 32],
    pub ml_kem_768_public_key: Vec<u8>,
    pub ml_dsa_65_public_key: Vec<u8>,
}

impl HybridDeviceIdentity {
    pub fn public_bundle(&self) -> HybridDevicePublicBundle {
        let recipient = self.recipient.public_keys();
        HybridDevicePublicBundle {
            x25519_public_key: recipient.x25519,
            ml_kem_768_public_key: recipient.ml_kem_768,
            ml_dsa_65_public_key: self.ml_dsa.public_key(),
        }
    }
}


pub async fn has_hybrid_identity() -> Result<bool, String> {
    JsFuture::from(has_device_secret(DEVICE_IDENTITY_RECORD))
        .await
        .map_err(|e| format!("Could not inspect wrapped device identity: {e:?}"))?
        .as_bool()
        .ok_or_else(|| "Device identity presence check returned a non-boolean value".to_string())
}

pub async fn create_and_store_hybrid_identity() -> Result<HybridDevicePublicBundle, String> {
    let identity = HybridDeviceIdentity {
        recipient: RecipientKeyPair::generate(),
        ml_dsa: MlDsaSigner::generate(),
    };
    let public = identity.public_bundle();
    let mut secret = identity
        .recipient
        .to_device_secret()
        .map_err(|e| e.to_string())?;
    secret.extend_from_slice(&identity.ml_dsa.to_device_secret());
    let array = Uint8Array::from(secret.as_slice());
    let stored = JsFuture::from(store_device_secret(DEVICE_IDENTITY_RECORD, &array)).await;
    secret.zeroize();
    stored.map_err(|e| format!("Could not store wrapped device identity: {e:?}"))?;
    Ok(public)
}

pub async fn load_hybrid_identity() -> Result<HybridDeviceIdentity, String> {
    let value = JsFuture::from(load_device_secret(DEVICE_IDENTITY_RECORD))
        .await
        .map_err(|e| format!("Could not load wrapped device identity: {e:?}"))?;
    let array = value
        .dyn_into::<Uint8Array>()
        .map_err(|_| "Device identity was not returned as bytes".to_string())?;
    let mut secret = array.to_vec();
    if secret.len() <= RECIPIENT_SECRET_BYTES {
        secret.zeroize();
        return Err("Wrapped Pulse V2 identity has an invalid length".into());
    }
    let recipient = RecipientKeyPair::from_device_secret(&secret[..RECIPIENT_SECRET_BYTES])
        .map_err(|e| e.to_string());
    let ml_dsa = MlDsaSigner::from_device_secret(&secret[RECIPIENT_SECRET_BYTES..])
        .map_err(|e| e.to_string());
    secret.zeroize();
    Ok(HybridDeviceIdentity {
        recipient: recipient?,
        ml_dsa: ml_dsa?,
    })
}
