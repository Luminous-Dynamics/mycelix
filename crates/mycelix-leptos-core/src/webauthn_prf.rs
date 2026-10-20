// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Read-only WebAuthn PRF client-capability inspection.
//!
//! This module does not create credentials, invoke an authenticator ceremony,
//! read or write identity storage, derive encryption keys, or claim that a
//! particular credential supports PRF. `getClientCapabilities()` reports
//! client-side support; authenticator support must be verified during a real
//! registration/assertion ceremony by checking the extension result.

/// What the current browser context reports about client-side WebAuthn PRF support.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WebAuthnPrfClientCapability {
    /// `getClientCapabilities()` returned `extension:prf = true`.
    /// This does not prove a specific authenticator will return PRF output.
    ClientAdvertisesSupport,
    /// The client returned false or omitted the capability; do not enable the PRF profile.
    NotAdvertisedOrUnknown,
    /// The WebAuthn capability API was absent or its query could not complete.
    ProbeUnavailable,
}

/// Probe the browser's WebAuthn client capabilities without prompting the user.
///
/// A result of `ClientAdvertisesSupport` is necessary but not sufficient for
/// PRF-backed key protection. After an explicit user gesture, a credential
/// must still complete a supported ceremony and return a valid PRF extension
/// result. Never downgrade a missing or unsupported PRF output to an
/// authenticator-backed status.
pub async fn inspect_webauthn_prf_client_capability() -> WebAuthnPrfClientCapability {
    #[cfg(target_arch = "wasm32")]
    {
        let (api_available, capability) = probe_browser_capability().await;
        classify_prf_capability(api_available, capability)
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        WebAuthnPrfClientCapability::ProbeUnavailable
    }
}

fn classify_prf_capability(
    api_available: bool,
    query_result: Result<Option<bool>, ()>,
) -> WebAuthnPrfClientCapability {
    if !api_available {
        return WebAuthnPrfClientCapability::ProbeUnavailable;
    }
    match query_result {
        Ok(Some(true)) => WebAuthnPrfClientCapability::ClientAdvertisesSupport,
        Ok(Some(false) | None) => WebAuthnPrfClientCapability::NotAdvertisedOrUnknown,
        Err(()) => WebAuthnPrfClientCapability::ProbeUnavailable,
    }
}

#[cfg(target_arch = "wasm32")]
async fn probe_browser_capability() -> (bool, Result<Option<bool>, ()>) {
    use js_sys::{Function, Promise, Reflect};
    use wasm_bindgen::{JsCast, JsValue};
    use wasm_bindgen_futures::JsFuture;

    let Some(window) = web_sys::window() else {
        return (false, Err(()));
    };
    let Ok(credential_constructor) =
        Reflect::get(window.as_ref(), &JsValue::from_str("PublicKeyCredential"))
    else {
        return (false, Err(()));
    };
    if credential_constructor.is_null() || credential_constructor.is_undefined() {
        return (false, Err(()));
    }
    let Ok(method) = Reflect::get(
        &credential_constructor,
        &JsValue::from_str("getClientCapabilities"),
    ) else {
        return (false, Err(()));
    };
    let Ok(function) = method.dyn_into::<Function>() else {
        return (false, Err(()));
    };

    let promise_value = match function.call0(&credential_constructor) {
        Ok(value) => value,
        Err(_) => return (true, Err(())),
    };
    let promise: Promise = promise_value.unchecked_into();
    let capabilities = match JsFuture::from(promise).await {
        Ok(value) => value,
        Err(_) => return (true, Err(())),
    };
    let value = match Reflect::get(&capabilities, &JsValue::from_str("extension:prf")) {
        Ok(value) if value.is_boolean() => value.as_bool(),
        Ok(_) => None,
        Err(_) => return (true, Err(())),
    };
    (true, Ok(value))
}

#[cfg(test)]
mod tests {
    use super::{WebAuthnPrfClientCapability, classify_prf_capability};

    #[test]
    fn only_explicit_true_advertises_client_prf_support() {
        assert_eq!(
            classify_prf_capability(true, Ok(Some(true))),
            WebAuthnPrfClientCapability::ClientAdvertisesSupport
        );
        assert_eq!(
            classify_prf_capability(true, Ok(Some(false))),
            WebAuthnPrfClientCapability::NotAdvertisedOrUnknown
        );
        assert_eq!(
            classify_prf_capability(true, Ok(None)),
            WebAuthnPrfClientCapability::NotAdvertisedOrUnknown
        );
    }

    #[test]
    fn missing_api_or_rejected_query_never_reports_support() {
        assert_eq!(
            classify_prf_capability(false, Ok(Some(true))),
            WebAuthnPrfClientCapability::ProbeUnavailable
        );
        assert_eq!(
            classify_prf_capability(true, Err(())),
            WebAuthnPrfClientCapability::ProbeUnavailable
        );
    }

    #[test]
    fn advertised_client_support_is_not_authenticator_support() {
        // The distinct variant intentionally describes only a client report;
        // the module has no API for asserting credential-level PRF success.
        let report = classify_prf_capability(true, Ok(Some(true)));
        assert_eq!(report, WebAuthnPrfClientCapability::ClientAdvertisesSupport);
    }
}

#[cfg(all(test, target_arch = "wasm32"))]
mod browser_tests {
    use super::{WebAuthnPrfClientCapability, inspect_webauthn_prf_client_capability};
    use wasm_bindgen_test::wasm_bindgen_test;

    #[wasm_bindgen_test]
    async fn capability_probe_returns_a_closed_set_without_an_authenticator_prompt() {
        let result = inspect_webauthn_prf_client_capability().await;
        assert!(matches!(
            result,
            WebAuthnPrfClientCapability::ClientAdvertisesSupport
                | WebAuthnPrfClientCapability::NotAdvertisedOrUnknown
                | WebAuthnPrfClientCapability::ProbeUnavailable
        ));
    }
}
