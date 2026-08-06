// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! JWT bearer-token authentication middleware
//!
//! Gates mutating endpoints (POST/PUT/DELETE) behind a valid JWT. Read-only
//! GET endpoints stay public. Mirrors the JWT pattern already used in
//! `mycelix-pulse/happ/backend-rs/src/middleware/auth.rs` while additionally
//! binding tokens to an explicit issuer and audience. A successful check inserts [`AuthenticatedActor`] into request
//! extensions so handlers cannot accept authoritative actor IDs from JSON.

use axum::{
    Json,
    extract::Request,
    http::{StatusCode, header::AUTHORIZATION},
    middleware::Next,
    response::{IntoResponse, Response},
};
use jsonwebtoken::{Algorithm, DecodingKey, Validation, decode};
use serde::{Deserialize, Serialize};

use crate::error::ApiError;

/// Identity established by authentication middleware.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthenticatedActor {
    subject: String,
}

impl AuthenticatedActor {
    pub fn subject(&self) -> &str {
        &self.subject
    }
}

/// Minimal JWT claims required to authorize a mutating request.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Claims {
    /// Subject (caller identity, e.g. DID or API client ID).
    pub sub: String,
    /// Token issuer. Must match `JWT_ISSUER`.
    pub iss: String,
    /// Token audience. Must match `JWT_AUDIENCE`.
    pub aud: String,
    /// Expiration timestamp (seconds since epoch).
    pub exp: i64,
}

/// Reads the JWT signing secret from the `JWT_SECRET` environment variable.
///
/// No hardcoded fallback is provided: an unset secret is a misconfiguration
/// that must fail loudly rather than silently accepting any token signed
/// with a well-known default.
fn required_environment(name: &str) -> Result<String, AuthError> {
    std::env::var(name)
        .ok()
        .map(|value| value.trim().to_string())
        .filter(|value| !value.is_empty())
        .ok_or_else(|| AuthError {
            message: format!("Server misconfiguration: {name} is not set"),
        })
}

fn jwt_secret() -> Result<String, AuthError> {
    let secret = required_environment("JWT_SECRET")?;
    if secret.len() < 32 {
        return Err(AuthError {
            message: "Server misconfiguration: JWT_SECRET must be at least 32 bytes".to_string(),
        });
    }
    Ok(secret)
}

fn jwt_validation() -> Result<Validation, AuthError> {
    let issuer = required_environment("JWT_ISSUER")?;
    let audience = required_environment("JWT_AUDIENCE")?;
    let mut validation = Validation::new(Algorithm::HS256);
    validation.set_issuer(&[issuer]);
    validation.set_audience(&[audience]);
    Ok(validation)
}

/// Authentication error, rendered as a 401 with the standard `ApiError` body.
pub struct AuthError {
    message: String,
}

impl IntoResponse for AuthError {
    fn into_response(self) -> Response {
        let body = Json(ApiError::new("UNAUTHORIZED", self.message));
        (StatusCode::UNAUTHORIZED, body).into_response()
    }
}

/// `axum::middleware::from_fn` gate: requires `Authorization: Bearer <jwt>`
/// with a valid, unexpired signature. Apply only to mutating routes via
/// `.route_layer(...)` — read-only GET routes must not be wrapped with this.
pub async fn require_auth(mut req: Request, next: Next) -> Result<Response, AuthError> {
    let auth_header = req
        .headers()
        .get(AUTHORIZATION)
        .and_then(|value| value.to_str().ok())
        .ok_or_else(|| AuthError {
            message: "Missing Authorization header".to_string(),
        })?;

    let token = auth_header
        .strip_prefix("Bearer ")
        .ok_or_else(|| AuthError {
            message: "Authorization header must use the Bearer scheme".to_string(),
        })?;

    let secret = jwt_secret()?;

    let validation = jwt_validation()?;
    let token_data = decode::<Claims>(
        token,
        &DecodingKey::from_secret(secret.as_bytes()),
        &validation,
    )
    .map_err(|e| AuthError {
        message: format!("Invalid or expired token: {}", e),
    })?;

    let subject = token_data.claims.sub.trim();
    if subject.is_empty() || subject.len() > 512 || subject.chars().any(char::is_control) {
        return Err(AuthError {
            message: "Token subject is not a valid actor identifier".to_string(),
        });
    }

    req.extensions_mut().insert(AuthenticatedActor {
        subject: subject.to_string(),
    });

    Ok(next.run(req).await)
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::{
        Router,
        body::Body,
        http::{Request as HttpRequest, StatusCode as HttpStatusCode},
        middleware::from_fn,
        routing::post,
    };
    use jsonwebtoken::{EncodingKey, Header, encode};
    use tower::ServiceExt;

    const TEST_SECRET: &str = "test-secret-for-unit-tests-only-32bytes";
    const TEST_ISSUER: &str = "https://issuer.test";
    const TEST_AUDIENCE: &str = "mycelix-desci-test";

    fn configure_test_environment() {
        // SAFETY: the auth unit tests use one consistent environment profile.
        unsafe {
            std::env::set_var("JWT_SECRET", TEST_SECRET);
            std::env::set_var("JWT_ISSUER", TEST_ISSUER);
            std::env::set_var("JWT_AUDIENCE", TEST_AUDIENCE);
        }
    }

    fn token_with_exp_and_audience(secret: &str, exp: i64, audience: &str) -> String {
        let claims = Claims {
            sub: "test-subject".to_string(),
            iss: TEST_ISSUER.to_string(),
            aud: audience.to_string(),
            exp,
        };
        encode(
            &Header::default(),
            &claims,
            &EncodingKey::from_secret(secret.as_bytes()),
        )
        .expect("token encode")
    }

    fn token_with_exp(secret: &str, exp: i64) -> String {
        token_with_exp_and_audience(secret, exp, TEST_AUDIENCE)
    }

    async fn protected_handler(
        axum::Extension(actor): axum::Extension<AuthenticatedActor>,
    ) -> String {
        actor.subject().to_string()
    }

    fn app() -> Router {
        Router::new()
            .route("/protected", post(protected_handler))
            .route_layer(from_fn(require_auth))
    }

    #[tokio::test]
    async fn rejects_missing_authorization_header() {
        configure_test_environment();
        let response = app()
            .oneshot(
                HttpRequest::builder()
                    .method("POST")
                    .uri("/protected")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), HttpStatusCode::UNAUTHORIZED);
    }

    #[tokio::test]
    async fn rejects_invalid_token() {
        configure_test_environment();
        let response = app()
            .oneshot(
                HttpRequest::builder()
                    .method("POST")
                    .uri("/protected")
                    .header(AUTHORIZATION, "Bearer not-a-real-token")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), HttpStatusCode::UNAUTHORIZED);
    }

    #[tokio::test]
    async fn rejects_expired_token() {
        configure_test_environment();
        let expired = token_with_exp(TEST_SECRET, chrono::Utc::now().timestamp() - 3600);
        let response = app()
            .oneshot(
                HttpRequest::builder()
                    .method("POST")
                    .uri("/protected")
                    .header(AUTHORIZATION, format!("Bearer {}", expired))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), HttpStatusCode::UNAUTHORIZED);
    }

    #[tokio::test]
    async fn accepts_valid_token() {
        configure_test_environment();
        let valid = token_with_exp(TEST_SECRET, chrono::Utc::now().timestamp() + 3600);
        let response = app()
            .oneshot(
                HttpRequest::builder()
                    .method("POST")
                    .uri("/protected")
                    .header(AUTHORIZATION, format!("Bearer {}", valid))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), HttpStatusCode::OK);
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap();
        assert_eq!(body.as_ref(), b"test-subject");
    }

    #[tokio::test]
    async fn rejects_token_for_another_audience() {
        configure_test_environment();
        let token = token_with_exp_and_audience(
            TEST_SECRET,
            chrono::Utc::now().timestamp() + 3600,
            "another-service",
        );
        let response = app()
            .oneshot(
                HttpRequest::builder()
                    .method("POST")
                    .uri("/protected")
                    .header(AUTHORIZATION, format!("Bearer {}", token))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), HttpStatusCode::UNAUTHORIZED);
    }

    #[tokio::test]
    async fn rejects_empty_subject() {
        configure_test_environment();
        let claims = Claims {
            sub: "   ".to_string(),
            iss: TEST_ISSUER.to_string(),
            aud: TEST_AUDIENCE.to_string(),
            exp: chrono::Utc::now().timestamp() + 3600,
        };
        let token = encode(
            &Header::default(),
            &claims,
            &EncodingKey::from_secret(TEST_SECRET.as_bytes()),
        )
        .unwrap();
        let response = app()
            .oneshot(
                HttpRequest::builder()
                    .method("POST")
                    .uri("/protected")
                    .header(AUTHORIZATION, format!("Bearer {}", token))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), HttpStatusCode::UNAUTHORIZED);
    }
}
