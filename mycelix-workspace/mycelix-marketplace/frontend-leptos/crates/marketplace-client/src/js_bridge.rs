use crate::{ClientError, ZomeTransport};
use async_trait::async_trait;
use js_sys::{Object, Reflect, Uint8Array};
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::JsFuture;

#[wasm_bindgen(module = "/holochain-bridge.js")]
extern "C" {
    #[wasm_bindgen(catch, js_name = connectMarketplace)]
    fn connect_marketplace() -> Result<js_sys::Promise, JsValue>;

    #[wasm_bindgen(catch, js_name = callMarketplaceZome)]
    fn call_marketplace_zome(
        role_name: &str,
        zome_name: &str,
        function_name: &str,
        payload: Uint8Array,
    ) -> Result<js_sys::Promise, JsValue>;

    #[wasm_bindgen(catch, js_name = disconnectMarketplace)]
    fn disconnect_marketplace() -> Result<js_sys::Promise, JsValue>;
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BridgeConnectionInfo {
    pub installed_app_id: String,
    pub agent_pub_key: Vec<u8>,
    pub host_signer_available: bool,
    pub configured_roles: Vec<String>,
    pub active_roles: Vec<String>,
}

impl BridgeConnectionInfo {
    pub fn has_active_role(&self, role: &str) -> bool {
        self.active_roles.iter().any(|candidate| candidate == role)
    }
}

#[derive(Clone, Default)]
pub struct JsBridgeTransport;

impl JsBridgeTransport {
    pub async fn connect(&self) -> Result<BridgeConnectionInfo, ClientError> {
        let promise = connect_marketplace().map_err(js_error)?;
        let value = JsFuture::from(promise).await.map_err(js_error)?;
        let object = Object::from(value);

        let installed_app_id = Reflect::get(&object, &"installedAppId".into())
            .map_err(js_error)?
            .as_string()
            .ok_or_else(|| ClientError::Unavailable("bridge omitted installedAppId".into()))?;
        let agent_value = Reflect::get(&object, &"agentPubKey".into()).map_err(js_error)?;
        let agent_pub_key = Uint8Array::new(&agent_value).to_vec();
        let host_signer_available = Reflect::get(&object, &"hostSignerAvailable".into())
            .map_err(js_error)?
            .as_bool()
            .unwrap_or(false);
        let configured_roles = string_array_property(&object, "configuredRoles")?;
        let active_roles = string_array_property(&object, "activeRoles")?;
        if !active_roles.iter().any(|role| role == crate::contract::ROLE) {
            return Err(ClientError::Unavailable(
                "bridge reported no active marketplace role".into(),
            ));
        }

        Ok(BridgeConnectionInfo {
            installed_app_id,
            agent_pub_key,
            host_signer_available,
            configured_roles,
            active_roles,
        })
    }

    pub async fn disconnect(&self) -> Result<(), ClientError> {
        let promise = disconnect_marketplace().map_err(js_error)?;
        JsFuture::from(promise).await.map_err(js_error)?;
        Ok(())
    }
}

#[async_trait(?Send)]
impl ZomeTransport for JsBridgeTransport {
    async fn call_zome(
        &self,
        role: &str,
        zome: &str,
        function: &str,
        payload: Vec<u8>,
    ) -> Result<Vec<u8>, ClientError> {
        let payload = Uint8Array::from(payload.as_slice());
        let promise = call_marketplace_zome(role, zome, function, payload).map_err(|error| {
            ClientError::Call {
                zome: zome.into(),
                function: function.into(),
                message: js_message(error),
            }
        })?;
        let result = JsFuture::from(promise).await.map_err(|error| ClientError::Call {
            zome: zome.into(),
            function: function.into(),
            message: js_message(error),
        })?;
        Ok(Uint8Array::new(&result).to_vec())
    }
}

fn js_error(error: JsValue) -> ClientError {
    ClientError::Unavailable(js_message(error))
}

fn js_message(error: JsValue) -> String {
    error
        .as_string()
        .or_else(|| Reflect::get(&error, &"message".into()).ok()?.as_string())
        .unwrap_or_else(|| format!("{error:?}"))
}


fn string_array_property(object: &Object, property: &str) -> Result<Vec<String>, ClientError> {
    let value = Reflect::get(object, &property.into()).map_err(js_error)?;
    if !js_sys::Array::is_array(&value) {
        return Err(ClientError::Unavailable(format!(
            "bridge property {property} was not an array"
        )));
    }
    let array = js_sys::Array::from(&value);
    let mut output = Vec::with_capacity(array.length() as usize);
    for item in array.iter() {
        let value = item.as_string().ok_or_else(|| {
            ClientError::Unavailable(format!("bridge property {property} contained a non-string role"))
        })?;
        output.push(value);
    }
    Ok(output)
}
