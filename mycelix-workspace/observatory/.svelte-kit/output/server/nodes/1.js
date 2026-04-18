// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

export const index = 1;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/_error.svelte.js')).default;
export const imports = ["_app/immutable/nodes/1.F12M7nDx.js","_app/immutable/chunks/CujjRMGB.js","_app/immutable/chunks/DIAPMP-w.js","_app/immutable/chunks/BefYpqPH.js","_app/immutable/chunks/D2_CMSXj.js","_app/immutable/chunks/CatO2vgv.js"];
export const stylesheets = ["_app/immutable/assets/1.CJ1n-VYq.css"];
export const fonts = [];
