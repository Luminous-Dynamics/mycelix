import { decode, encode } from "@msgpack/msgpack";

/** Decode Rust rmp-serde bytes into the value expected by @holochain/client. */
export function decodeZomePayload(payloadBytes: Uint8Array): unknown {
  return decode(payloadBytes);
}

/** Encode a coordinator response for Rust rmp-serde decoding. */
export function encodeZomeResponse(response: unknown): Uint8Array {
  return encode(response);
}
