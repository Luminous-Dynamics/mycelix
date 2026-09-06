# SSF Reserved Issuance Rebind v0.1

This crate converges direct durable issuance reservation and historical restart reconciliation onto the exact rebound persisted-registration lineage before current authority revalidation.

The reservation manifest must embed the exact `PersistedRegistrationBindingV1` held by the rebound registration. Endpoint similarity or matching reservation IDs are insufficient.

The resulting token proves only that the registration was durably consumed for one issuance lineage. It still contains no current authority revalidation, state-install authority, or effect authority.
