// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! API error types and handling

use axum::{
    Json,
    http::StatusCode,
    response::{IntoResponse, Response},
};
use serde::{Deserialize, Serialize};
use utoipa::ToSchema;

pub type Result<T> = std::result::Result<T, ApiError>;

/// API error response
#[derive(Debug, Serialize, Deserialize, ToSchema)]
pub struct ApiError {
    /// Error code (e.g., "NOT_FOUND", "INVALID_REQUEST")
    pub code: String,
    /// Human-readable error message
    pub message: String,
    /// Optional additional details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

impl ApiError {
    pub fn new(code: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            code: code.into(),
            message: message.into(),
            details: None,
        }
    }

    pub fn with_details(mut self, details: serde_json::Value) -> Self {
        self.details = Some(details);
        self
    }

    pub fn not_found(resource: &str) -> Self {
        Self::new("NOT_FOUND", format!("{} not found", resource))
    }

    pub fn invalid_request(message: impl Into<String>) -> Self {
        Self::new("INVALID_REQUEST", message)
    }

    pub fn forbidden(message: impl Into<String>) -> Self {
        Self::new("FORBIDDEN", message)
    }

    pub fn conflict(message: impl Into<String>) -> Self {
        Self::new("CONFLICT", message)
    }

    pub fn gone(message: impl Into<String>) -> Self {
        Self::new("GONE", message)
    }

    pub fn not_implemented(message: impl Into<String>) -> Self {
        Self::new("NOT_IMPLEMENTED", message)
    }

    pub fn internal_error(message: impl Into<String>) -> Self {
        Self::new("INTERNAL_ERROR", message)
    }
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let status = match self.code.as_str() {
            "NOT_FOUND" => StatusCode::NOT_FOUND,
            "INVALID_REQUEST" => StatusCode::BAD_REQUEST,
            "UNAUTHORIZED" => StatusCode::UNAUTHORIZED,
            "FORBIDDEN" => StatusCode::FORBIDDEN,
            "CONFLICT" => StatusCode::CONFLICT,
            "GONE" => StatusCode::GONE,
            "NOT_IMPLEMENTED" => StatusCode::NOT_IMPLEMENTED,
            _ => StatusCode::INTERNAL_SERVER_ERROR,
        };

        (status, Json(self)).into_response()
    }
}

// Implement From for core library errors
impl From<mycelix_desci_core::Error> for ApiError {
    fn from(err: mycelix_desci_core::Error) -> Self {
        use mycelix_desci_core::Error;
        match err {
            Error::Validation(message)
            | Error::InvalidClaim(message)
            | Error::InvalidEpistemicTier(message) => ApiError::invalid_request(message),
            Error::VerificationFailed(message) | Error::Crypto(message) => {
                ApiError::forbidden(message)
            }
            Error::NotFound(message) => ApiError::new("NOT_FOUND", message),
            Error::Storage(message)
                if message.contains("optimistic concurrency")
                    || message.contains("duplicate scientific event")
                    || message.contains("duplicate scientific credential event")
                    || message.contains("credential governance optimistic concurrency")
                    || message.contains("duplicate credential governance event")
                    || message.contains("serializable append conflict")
                    || message.contains("could not serialize access")
                    || message.contains("checkpoint mirror actor already recorded")
                    || message.contains("idempotency") =>
            {
                ApiError::conflict(message)
            }
            other => ApiError::internal_error(other.to_string()),
        }
    }
}

impl From<anyhow::Error> for ApiError {
    fn from(err: anyhow::Error) -> Self {
        ApiError::internal_error(err.to_string())
    }
}

impl From<std::io::Error> for ApiError {
    fn from(err: std::io::Error) -> Self {
        ApiError::internal_error(err.to_string())
    }
}
