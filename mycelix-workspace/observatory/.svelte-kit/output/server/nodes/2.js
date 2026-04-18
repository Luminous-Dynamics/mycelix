// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

export const index = 2;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/_page.svelte.js')).default;
export const imports = ["_app/immutable/nodes/2.CQPNXEGf.js","_app/immutable/chunks/CujjRMGB.js","_app/immutable/chunks/DIAPMP-w.js","_app/immutable/chunks/B4sKVuNd.js","_app/immutable/chunks/aRspOZbs.js","_app/immutable/chunks/CatO2vgv.js","_app/immutable/chunks/CBcrUV2C.js","_app/immutable/chunks/DLw1pTtx.js","_app/immutable/chunks/aGrU48bv.js"];
export const stylesheets = ["_app/immutable/assets/2.B18oszhI.css"];
export const fonts = [];
