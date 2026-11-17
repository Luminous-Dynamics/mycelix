// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_voevent_header::parse_header;
use std::env;
use std::fs;
use std::process::ExitCode;

fn main() -> ExitCode {
    let mut args = env::args_os().skip(1);
    let Some(path) = args.next() else {
        eprintln!("usage: voevent-header <packet.xml>");
        return ExitCode::from(2);
    };
    if args.next().is_some() {
        eprintln!("usage: voevent-header <packet.xml>");
        return ExitCode::from(2);
    }

    let bytes = match fs::read(&path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("cannot read packet: {error}");
            return ExitCode::from(2);
        }
    };

    match parse_header(&bytes) {
        Ok(header) => match serde_json::to_string_pretty(&header) {
            Ok(json) => {
                println!("{json}");
                ExitCode::SUCCESS
            }
            Err(error) => {
                eprintln!("cannot serialize parsed header: {error}");
                ExitCode::from(2)
            }
        },
        Err(error) => {
            eprintln!("packet rejected: {error}");
            ExitCode::from(1)
        }
    }
}
