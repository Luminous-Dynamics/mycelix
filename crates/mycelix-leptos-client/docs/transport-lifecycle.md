# Browser transport lifecycle

`BrowserWsTransport` stores its current [`ConnectionStatus`] and can publish
future transitions through `set_status_handler`.

Normal connection (including a conductor that becomes available during the
bounded startup retry window):

```text
Disconnected -> Connecting -> Connected
```

Unexpected close with reconnect enabled:

```text
Connected -> Error (when reported) -> Reconnecting -> Connecting -> Connected
```

The retry policy covers WebSocket creation, pre-open, authentication/app-info,
and post-connection failures. Retries use bounded exponential backoff from
`ReconnectConfig`. Duplicate
error/close callbacks cannot schedule parallel retries, and a failed reconnect
explicitly schedules the next attempt until the configured limit is exhausted.
After exhaustion the transport publishes `Disconnected`.

Calling `disconnect()` is different from an unexpected close: it clears the
stored reconnect configuration before closing the socket, publishes
`Disconnected`, increments a retry epoch that invalidates delayed callbacks,
and never reconnects automatically.

The status callback is invoked only for transitions after registration and is
called after the transport releases its internal `RefCell` borrow. Consumers
that need an initial snapshot should call `HolochainTransport::status()`.

`Connected` means authentication and app-info discovery succeeded. Authorized
zome-call signing remains a separate readiness condition exposed by
`zome_call_signer_available()`.
