// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

export const index = 5;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/attribution/_page.svelte.js')).default;
export const imports = ["_app/immutable/nodes/5.Olc7vXV8.js","_app/immutable/chunks/CujjRMGB.js","_app/immutable/chunks/DIAPMP-w.js","_app/immutable/chunks/B4sKVuNd.js","_app/immutable/chunks/CBcrUV2C.js","_app/immutable/chunks/CatO2vgv.js"];
export const stylesheets = [];
export const fonts = [];
