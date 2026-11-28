//! Isolated harness for the production finite-game analysis modules.
//!
//! The source modules are included directly from the SDK tree. This harness
//! intentionally avoids resolving the complete Mycelix SDK/Holochain graph.

pub mod agentic {
    // Legacy implementation is included as a type dependency, but its old
    // heuristic APIs are not the subject of this strict Clippy target.
    #[allow(clippy::all)]
    pub mod game_theory {
        include!("../../../mycelix-workspace/sdk/src/agentic/game_theory.rs");
    }

    pub mod game_theory_validation {
        include!("../../../mycelix-workspace/sdk/src/agentic/game_theory_validation.rs");
    }

    pub mod game_theory_verified {
        include!("../../../mycelix-workspace/sdk/src/agentic/game_theory_verified.rs");
    }
}
