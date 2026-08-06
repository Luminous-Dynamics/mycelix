// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! WebRTC adapter for P2P video/audio calls.
//!
//! The browser boundary is implemented with typed `web_sys` APIs. SDP and
//! media values never become JavaScript source strings.

use std::cell::RefCell;

use wasm_bindgen::{JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;

thread_local! {
    static LOCAL_PEER: RefCell<Option<web_sys::RtcPeerConnection>> = RefCell::new(None);
    static REMOTE_PEER: RefCell<Option<web_sys::RtcPeerConnection>> = RefCell::new(None);
    static USER_STREAM: RefCell<Option<web_sys::MediaStream>> = RefCell::new(None);
    static SCREEN_STREAM: RefCell<Option<web_sys::MediaStream>> = RefCell::new(None);
}

fn js_error(context: &str, value: JsValue) -> String {
    format!("{context}: {value:?}")
}

fn create_peer_connection() -> Result<web_sys::RtcPeerConnection, String> {
    let config: web_sys::RtcConfiguration = js_sys::Object::new().unchecked_into();
    let server = js_sys::Object::new();
    js_sys::Reflect::set(
        &server,
        &JsValue::from_str("urls"),
        &JsValue::from_str("stun:stun.l.google.com:19302"),
    )
    .map_err(|e| js_error("Could not configure STUN server", e))?;
    let servers = js_sys::Array::new();
    servers.push(&server);
    js_sys::Reflect::set(
        config.as_ref(),
        &JsValue::from_str("iceServers"),
        servers.as_ref(),
    )
    .map_err(|e| js_error("Could not configure ICE servers", e))?;

    web_sys::RtcPeerConnection::new_with_configuration(&config)
        .map_err(|e| js_error("Could not create peer connection", e))
}

fn media_devices() -> Result<web_sys::MediaDevices, String> {
    web_sys::window()
        .ok_or_else(|| "window unavailable".to_string())?
        .navigator()
        .media_devices()
        .map_err(|e| js_error("Media devices unavailable", e))
}

/// Create an SDP offer for a new call.
pub async fn create_offer() -> Result<(String, String), String> {
    let peer = create_peer_connection()?;
    let _data_channel = peer.create_data_channel("mycelix-data");

    let offer = JsFuture::from(peer.create_offer())
        .await
        .map_err(|e| js_error("Could not create SDP offer", e))?
        .dyn_into::<web_sys::RtcSessionDescriptionInit>()
        .map_err(|e| js_error("Invalid SDP offer result", e))?;

    JsFuture::from(peer.set_local_description(&offer))
        .await
        .map_err(|e| js_error("Could not set local SDP offer", e))?;

    let sdp = offer
        .get_sdp()
        .ok_or_else(|| "Browser returned an SDP offer without SDP text".to_string())?;
    LOCAL_PEER.with(|slot| *slot.borrow_mut() = Some(peer));
    Ok((sdp, "pc-0".into()))
}

/// Accept an SDP offer and create an answer.
pub async fn create_answer(remote_sdp: &str) -> Result<String, String> {
    let peer = create_peer_connection()?;
    let offer = web_sys::RtcSessionDescriptionInit::new(web_sys::RtcSdpType::Offer);
    offer.set_sdp(remote_sdp);

    JsFuture::from(peer.set_remote_description(&offer))
        .await
        .map_err(|e| js_error("Could not set remote SDP offer", e))?;

    let answer = JsFuture::from(peer.create_answer())
        .await
        .map_err(|e| js_error("Could not create SDP answer", e))?
        .dyn_into::<web_sys::RtcSessionDescriptionInit>()
        .map_err(|e| js_error("Invalid SDP answer result", e))?;

    JsFuture::from(peer.set_local_description(&answer))
        .await
        .map_err(|e| js_error("Could not set local SDP answer", e))?;

    let sdp = answer
        .get_sdp()
        .ok_or_else(|| "Browser returned an SDP answer without SDP text".to_string())?;
    REMOTE_PEER.with(|slot| *slot.borrow_mut() = Some(peer));
    Ok(sdp)
}

/// Get user camera/microphone.
pub async fn get_user_media(video: bool, audio: bool) -> Result<(), String> {
    let constraints = web_sys::MediaStreamConstraints::new();
    constraints.set_video_bool(video);
    constraints.set_audio_bool(audio);

    let promise = media_devices()?
        .get_user_media_with_constraints(&constraints)
        .map_err(|e| js_error("Could not request user media", e))?;
    let stream = JsFuture::from(promise)
        .await
        .map_err(|e| js_error("User-media request failed", e))?
        .dyn_into::<web_sys::MediaStream>()
        .map_err(|e| js_error("Invalid user-media result", e))?;
    USER_STREAM.with(|slot| *slot.borrow_mut() = Some(stream));
    Ok(())
}

/// Get screen share stream.
pub async fn get_display_media() -> Result<(), String> {
    let constraints = web_sys::DisplayMediaStreamConstraints::new();
    constraints.set_video_bool(true);
    constraints.set_audio_bool(false);

    let promise = media_devices()?
        .get_display_media_with_constraints(&constraints)
        .map_err(|e| js_error("Could not request display media", e))?;
    let stream = JsFuture::from(promise)
        .await
        .map_err(|e| js_error("Display-media request failed", e))?
        .dyn_into::<web_sys::MediaStream>()
        .map_err(|e| js_error("Invalid display-media result", e))?;
    SCREEN_STREAM.with(|slot| *slot.borrow_mut() = Some(stream));
    Ok(())
}

fn stop_stream(stream: web_sys::MediaStream) {
    for track in stream.get_tracks().iter() {
        if let Ok(track) = track.dyn_into::<web_sys::MediaStreamTrack>() {
            track.stop();
        }
    }
}

/// Stop all media tracks and close active peer connections.
pub fn stop_media() {
    USER_STREAM.with(|slot| {
        if let Some(stream) = slot.borrow_mut().take() {
            stop_stream(stream);
        }
    });
    SCREEN_STREAM.with(|slot| {
        if let Some(stream) = slot.borrow_mut().take() {
            stop_stream(stream);
        }
    });
    LOCAL_PEER.with(|slot| {
        if let Some(peer) = slot.borrow_mut().take() {
            peer.close();
        }
    });
    REMOTE_PEER.with(|slot| {
        if let Some(peer) = slot.borrow_mut().take() {
            peer.close();
        }
    });
}
