//! Deterministic, language-neutral adversarial trace generation.
//!
//! Trace bytes are an input contract, not a correctness oracle. The Rust and Go
//! generators must produce identical bytes for the same seed and trace index.

use sha2::{Digest, Sha256};

const TRACE_DOMAIN: &[u8] = b"MXSAP-SAGA-TRACE-V1\0";
const SEED_MULTIPLIER: u64 = 0xD6E8_FEB8_6659_FD93;
const SPLITMIX_GAMMA: u64 = 0x9E37_79B9_7F4A_7C15;
const SPLITMIX_MUL1: u64 = 0xBF58_476D_1CE4_E5B9;
const SPLITMIX_MUL2: u64 = 0x94D0_49BB_1331_11EB;

pub const MAX_TRACE_COMMANDS: usize = 64;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum TraceOpcode {
    CreateIntent = 1,
    Authorize = 2,
    PrepareLocal = 3,
    CommitLocal = 4,
    PrepareTreasury = 5,
    DispatchTreasury = 6,
    MarkIndeterminate = 7,
    ReconcileTreasury = 8,
    RetrySameEffect = 9,
    CommitTreasury = 10,
    CheckpointPayments = 11,
    CheckpointTreasury = 12,
    MutateIntent = 13,
    DuplicateDelivery = 14,
    MutateEvent = 15,
    StaleCheckpoint = 16,
    ForkCheckpoint = 17,
    Compensate = 18,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum TraceOwner {
    Sender = 1,
    Payments = 2,
    Treasury = 3,
    Adversary = 4,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum TraceFault {
    None = 0,
    ResponseLostBeforeEffect = 1,
    ResponseLostAfterEffect = 2,
    LookupUncertain = 3,
    ReceiptMismatch = 4,
    StaleFrontier = 5,
    WrongOwner = 6,
    PayloadMutation = 7,
    DuplicateDelivery = 8,
    ForkedCheckpoint = 9,
    MissingDependency = 10,
    AuthoritativeAbsence = 11,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TraceCommandV1 {
    pub opcode: TraceOpcode,
    pub owner: TraceOwner,
    pub fault: TraceFault,
    pub subject: u64,
    pub value: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceV1 {
    pub seed: u64,
    pub trace_index: u64,
    pub scenario: u8,
    pub operation_nonce: u64,
    pub commands: Vec<TraceCommandV1>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceError {
    InvalidScenario,
    EmptyCommands,
    TooManyCommands,
    InvalidMagic,
    TruncatedInput,
    InvalidOpcode,
    InvalidOwner,
    InvalidFault,
    NonzeroReservedByte,
    TrailingBytes,
}

impl std::fmt::Display for TraceError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::InvalidScenario => "trace scenario must be in 1..=8",
            Self::EmptyCommands => "trace must contain at least one command",
            Self::TooManyCommands => "trace exceeds the 64-command bound",
            Self::InvalidMagic => "trace domain separator is invalid",
            Self::TruncatedInput => "trace input is truncated",
            Self::InvalidOpcode => "trace contains an unknown opcode",
            Self::InvalidOwner => "trace contains an unknown owner",
            Self::InvalidFault => "trace contains an unknown fault code",
            Self::NonzeroReservedByte => "trace reserved byte must be zero",
            Self::TrailingBytes => "trace contains trailing bytes",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for TraceError {}

impl TraceV1 {
    /// Encodes the versioned trace using fixed-width, big-endian fields.
    ///
    /// Header: domain bytes, seed u64, trace index u64, scenario u8,
    /// operation nonce u64, command count u32.
    ///
    /// Command: opcode u8, owner u8, fault u8, reserved-zero u8,
    /// subject u64, value u64.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, TraceError> {
        if !(1..=8).contains(&self.scenario) {
            return Err(TraceError::InvalidScenario);
        }
        if self.commands.is_empty() {
            return Err(TraceError::EmptyCommands);
        }
        if self.commands.len() > MAX_TRACE_COMMANDS {
            return Err(TraceError::TooManyCommands);
        }

        let command_count =
            u32::try_from(self.commands.len()).map_err(|_| TraceError::TooManyCommands)?;
        let mut output = Vec::with_capacity(49 + self.commands.len() * 20);
        output.extend_from_slice(TRACE_DOMAIN);
        append_u64(&mut output, self.seed);
        append_u64(&mut output, self.trace_index);
        output.push(self.scenario);
        append_u64(&mut output, self.operation_nonce);
        output.extend_from_slice(&command_count.to_be_bytes());

        for command in &self.commands {
            output.push(command.opcode as u8);
            output.push(command.owner as u8);
            output.push(command.fault as u8);
            output.push(0);
            append_u64(&mut output, command.subject);
            append_u64(&mut output, command.value);
        }
        Ok(output)
    }

    pub fn canonical_digest(&self) -> Result<String, TraceError> {
        let digest = Sha256::digest(self.canonical_bytes()?);
        let mut output = String::with_capacity(71);
        output.push_str("sha256:");
        for byte in digest {
            use std::fmt::Write as _;
            write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
        }
        Ok(output)
    }

    /// Decode one complete canonical trace. Unknown enum values, nonzero
    /// reserved bytes, truncated records and trailing data are rejected.
    pub fn decode(bytes: &[u8]) -> Result<Self, TraceError> {
        let domain_len = TRACE_DOMAIN.len();
        let header_len = domain_len + 8 + 8 + 1 + 8 + 4;
        if bytes.len() < header_len {
            return Err(TraceError::TruncatedInput);
        }
        if &bytes[..domain_len] != TRACE_DOMAIN {
            return Err(TraceError::InvalidMagic);
        }

        let mut offset = domain_len;
        let seed = read_u64(bytes, &mut offset)?;
        let trace_index = read_u64(bytes, &mut offset)?;
        let scenario = read_byte(bytes, &mut offset)?;
        let operation_nonce = read_u64(bytes, &mut offset)?;
        let command_count = read_u32(bytes, &mut offset)?;
        if command_count == 0 {
            return Err(TraceError::EmptyCommands);
        }
        if command_count as usize > MAX_TRACE_COMMANDS {
            return Err(TraceError::TooManyCommands);
        }

        let expected_len = header_len + command_count as usize * 20;
        if bytes.len() < expected_len {
            return Err(TraceError::TruncatedInput);
        }
        if bytes.len() > expected_len {
            return Err(TraceError::TrailingBytes);
        }

        let mut commands = Vec::with_capacity(command_count as usize);
        for _ in 0..command_count {
            let opcode = decode_opcode(read_byte(bytes, &mut offset)?)?;
            let owner = decode_owner(read_byte(bytes, &mut offset)?)?;
            let fault = decode_fault(read_byte(bytes, &mut offset)?)?;
            if read_byte(bytes, &mut offset)? != 0 {
                return Err(TraceError::NonzeroReservedByte);
            }
            let subject = read_u64(bytes, &mut offset)?;
            let value = read_u64(bytes, &mut offset)?;
            commands.push(TraceCommandV1 {
                opcode,
                owner,
                fault,
                subject,
                value,
            });
        }

        let trace = Self {
            seed,
            trace_index,
            scenario,
            operation_nonce,
            commands,
        };
        trace.canonical_bytes()?;
        Ok(trace)
    }
}

fn append_u64(output: &mut Vec<u8>, value: u64) {
    output.extend_from_slice(&value.to_be_bytes());
}

fn read_byte(bytes: &[u8], offset: &mut usize) -> Result<u8, TraceError> {
    let value = *bytes.get(*offset).ok_or(TraceError::TruncatedInput)?;
    *offset += 1;
    Ok(value)
}

fn read_u32(bytes: &[u8], offset: &mut usize) -> Result<u32, TraceError> {
    let end = (*offset).checked_add(4).ok_or(TraceError::TruncatedInput)?;
    let value = bytes.get(*offset..end).ok_or(TraceError::TruncatedInput)?;
    *offset = end;
    Ok(u32::from_be_bytes(value.try_into().map_err(|_| TraceError::TruncatedInput)?))
}

fn read_u64(bytes: &[u8], offset: &mut usize) -> Result<u64, TraceError> {
    let end = (*offset).checked_add(8).ok_or(TraceError::TruncatedInput)?;
    let value = bytes.get(*offset..end).ok_or(TraceError::TruncatedInput)?;
    *offset = end;
    Ok(u64::from_be_bytes(value.try_into().map_err(|_| TraceError::TruncatedInput)?))
}

fn decode_opcode(value: u8) -> Result<TraceOpcode, TraceError> {
    match value {
        1 => Ok(TraceOpcode::CreateIntent),
        2 => Ok(TraceOpcode::Authorize),
        3 => Ok(TraceOpcode::PrepareLocal),
        4 => Ok(TraceOpcode::CommitLocal),
        5 => Ok(TraceOpcode::PrepareTreasury),
        6 => Ok(TraceOpcode::DispatchTreasury),
        7 => Ok(TraceOpcode::MarkIndeterminate),
        8 => Ok(TraceOpcode::ReconcileTreasury),
        9 => Ok(TraceOpcode::RetrySameEffect),
        10 => Ok(TraceOpcode::CommitTreasury),
        11 => Ok(TraceOpcode::CheckpointPayments),
        12 => Ok(TraceOpcode::CheckpointTreasury),
        13 => Ok(TraceOpcode::MutateIntent),
        14 => Ok(TraceOpcode::DuplicateDelivery),
        15 => Ok(TraceOpcode::MutateEvent),
        16 => Ok(TraceOpcode::StaleCheckpoint),
        17 => Ok(TraceOpcode::ForkCheckpoint),
        18 => Ok(TraceOpcode::Compensate),
        _ => Err(TraceError::InvalidOpcode),
    }
}

fn decode_owner(value: u8) -> Result<TraceOwner, TraceError> {
    match value {
        1 => Ok(TraceOwner::Sender),
        2 => Ok(TraceOwner::Payments),
        3 => Ok(TraceOwner::Treasury),
        4 => Ok(TraceOwner::Adversary),
        _ => Err(TraceError::InvalidOwner),
    }
}

fn decode_fault(value: u8) -> Result<TraceFault, TraceError> {
    match value {
        0 => Ok(TraceFault::None),
        1 => Ok(TraceFault::ResponseLostBeforeEffect),
        2 => Ok(TraceFault::ResponseLostAfterEffect),
        3 => Ok(TraceFault::LookupUncertain),
        4 => Ok(TraceFault::ReceiptMismatch),
        5 => Ok(TraceFault::StaleFrontier),
        6 => Ok(TraceFault::WrongOwner),
        7 => Ok(TraceFault::PayloadMutation),
        8 => Ok(TraceFault::DuplicateDelivery),
        9 => Ok(TraceFault::ForkedCheckpoint),
        10 => Ok(TraceFault::MissingDependency),
        11 => Ok(TraceFault::AuthoritativeAbsence),
        _ => Err(TraceError::InvalidFault),
    }
}

fn command(
    opcode: TraceOpcode,
    owner: TraceOwner,
    fault: TraceFault,
    subject: u64,
    value: u64,
) -> TraceCommandV1 {
    TraceCommandV1 {
        opcode,
        owner,
        fault,
        subject,
        value,
    }
}

/// SplitMix64 is specified here with wrapping u64 arithmetic. The seed/index
/// derivation and draw order are part of the frozen trace profile.
struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64, trace_index: u64) -> Self {
        Self {
            state: seed ^ trace_index.wrapping_mul(SEED_MULTIPLIER),
        }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(SPLITMIX_GAMMA);
        let mut value = self.state;
        value = (value ^ (value >> 30)).wrapping_mul(SPLITMIX_MUL1);
        value = (value ^ (value >> 27)).wrapping_mul(SPLITMIX_MUL2);
        value ^ (value >> 31)
    }
}

pub struct TraceGeneratorV1;

impl TraceGeneratorV1 {
    /// Generate one bounded scenario. The generator uses six PRNG draws in
    /// this fixed order: scenario, operation nonce, effect nonce,
    /// checkpoint nonce, amount, fee.
    pub fn generate(seed: u64, trace_index: u64) -> TraceV1 {
        let mut random = SplitMix64::new(seed, trace_index);
        let scenario = (random.next_u64() % 8 + 1) as u8;
        let operation_nonce = random.next_u64();
        let effect_nonce = random.next_u64();
        let checkpoint_nonce = random.next_u64();
        let amount = random.next_u64() % 1_000_000 + 1;
        let fee = random.next_u64() % 1_001;
        let total = amount + fee;

        use TraceFault as F;
        use TraceOpcode as O;
        use TraceOwner as A;

        let mut commands = vec![
            command(O::CreateIntent, A::Sender, F::None, operation_nonce, amount),
            command(O::Authorize, A::Sender, F::None, operation_nonce, fee),
            command(O::PrepareLocal, A::Payments, F::None, operation_nonce, total),
            command(O::CommitLocal, A::Payments, F::None, operation_nonce, total),
        ];

        match scenario {
            1 => commands.extend([
                command(O::PrepareTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::DispatchTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::CommitTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(
                    O::CheckpointPayments,
                    A::Payments,
                    F::None,
                    operation_nonce,
                    checkpoint_nonce,
                ),
                command(
                    O::CheckpointTreasury,
                    A::Treasury,
                    F::None,
                    effect_nonce,
                    checkpoint_nonce,
                ),
            ]),
            2 => commands.extend([
                command(O::PrepareTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::DispatchTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(
                    O::MarkIndeterminate,
                    A::Treasury,
                    F::ResponseLostBeforeEffect,
                    effect_nonce,
                    fee,
                ),
                command(
                    O::ReconcileTreasury,
                    A::Treasury,
                    F::AuthoritativeAbsence,
                    effect_nonce,
                    fee,
                ),
                command(O::RetrySameEffect, A::Treasury, F::None, effect_nonce, fee),
                command(O::DispatchTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::CommitTreasury, A::Treasury, F::None, effect_nonce, fee),
            ]),
            3 => commands.extend([
                command(O::PrepareTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::DispatchTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(
                    O::MarkIndeterminate,
                    A::Treasury,
                    F::ResponseLostAfterEffect,
                    effect_nonce,
                    fee,
                ),
                command(O::ReconcileTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::CommitTreasury, A::Treasury, F::None, effect_nonce, fee),
            ]),
            4 => commands.extend([
                command(O::PrepareTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::DispatchTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(
                    O::MarkIndeterminate,
                    A::Treasury,
                    F::ResponseLostAfterEffect,
                    effect_nonce,
                    fee,
                ),
                command(
                    O::ReconcileTreasury,
                    A::Treasury,
                    F::LookupUncertain,
                    effect_nonce,
                    fee,
                ),
                command(
                    O::CheckpointPayments,
                    A::Payments,
                    F::MissingDependency,
                    operation_nonce,
                    checkpoint_nonce,
                ),
            ]),
            5 => commands.push(command(
                O::MutateIntent,
                A::Adversary,
                F::PayloadMutation,
                operation_nonce,
                amount + 1,
            )),
            6 => commands.extend([
                command(O::PrepareTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::DispatchTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(
                    O::DuplicateDelivery,
                    A::Adversary,
                    F::DuplicateDelivery,
                    effect_nonce,
                    fee,
                ),
                command(O::CommitTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(
                    O::MutateEvent,
                    A::Adversary,
                    F::PayloadMutation,
                    operation_nonce,
                    checkpoint_nonce,
                ),
            ]),
            7 => commands.extend([
                command(
                    O::CheckpointPayments,
                    A::Payments,
                    F::None,
                    operation_nonce,
                    checkpoint_nonce,
                ),
                command(
                    O::CheckpointTreasury,
                    A::Treasury,
                    F::StaleFrontier,
                    effect_nonce,
                    checkpoint_nonce,
                ),
                command(
                    O::ForkCheckpoint,
                    A::Adversary,
                    F::ForkedCheckpoint,
                    effect_nonce,
                    checkpoint_nonce,
                ),
            ]),
            8 => commands.extend([
                command(O::PrepareTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::DispatchTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(O::CommitTreasury, A::Treasury, F::None, effect_nonce, fee),
                command(
                    O::Compensate,
                    A::Sender,
                    F::None,
                    checkpoint_nonce,
                    total,
                ),
            ]),
            _ => unreachable!("scenario is reduced to 1..=8"),
        }

        TraceV1 {
            seed,
            trace_index,
            scenario,
            operation_nonce,
            commands,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KAT: &str = include_str!("../fixtures/saga-trace-v1.kat");

    fn kat_value(name: &str) -> &str {
        KAT.lines()
            .find_map(|line| line.strip_prefix(&format!("{name}=")))
            .unwrap_or_else(|| panic!("missing KAT field {name}"))
    }

    fn hex(bytes: &[u8]) -> String {
        let mut output = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            use std::fmt::Write as _;
            write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
        }
        output
    }

    #[test]
    fn trace_v1_matches_shared_known_answer_fixture() {
        let seed = u64::from_str_radix(kat_value("seed"), 16).unwrap();
        let trace_index = kat_value("trace_index").parse::<u64>().unwrap();
        let trace = TraceGeneratorV1::generate(seed, trace_index);
        assert_eq!(trace.scenario.to_string(), kat_value("scenario"));
        assert_eq!(
            format!("{:016x}", trace.operation_nonce),
            kat_value("operation_nonce")
        );
        assert_eq!(trace.commands.len().to_string(), kat_value("command_count"));
        assert_eq!(hex(&trace.canonical_bytes().unwrap()), kat_value("canonical_bytes_hex"));
        assert_eq!(trace.canonical_digest().unwrap(), format!("sha256:{}", kat_value("sha256")));
    }

    #[test]
    fn generator_is_deterministic_and_covers_all_scenarios() {
        let mut scenarios = [false; 8];
        for seed in 0..4096 {
            let first = TraceGeneratorV1::generate(seed, seed.rotate_left(13));
            let second = TraceGeneratorV1::generate(seed, seed.rotate_left(13));
            assert_eq!(first, second);
            assert!(first.canonical_bytes().is_ok());
            scenarios[usize::from(first.scenario - 1)] = true;
        }
        assert!(scenarios.into_iter().all(|observed| observed));
    }

    #[test]
    fn trace_v1_decoder_round_trips_and_rejects_noncanonical_bytes() {
        let trace = TraceGeneratorV1::generate(0x0123_4567_89ab_cdef, 7);
        let encoded = trace.canonical_bytes().unwrap();
        assert_eq!(TraceV1::decode(&encoded), Ok(trace));

        let mut bad_reserved = encoded.clone();
        bad_reserved[TRACE_DOMAIN.len() + 8 + 8 + 1 + 8 + 4 + 3] = 1;
        assert_eq!(
            TraceV1::decode(&bad_reserved),
            Err(TraceError::NonzeroReservedByte)
        );

        let mut bad_opcode = encoded.clone();
        bad_opcode[TRACE_DOMAIN.len() + 8 + 8 + 1 + 8 + 4] = 255;
        assert_eq!(TraceV1::decode(&bad_opcode), Err(TraceError::InvalidOpcode));

        let mut bad_owner = encoded.clone();
        bad_owner[TRACE_DOMAIN.len() + 8 + 8 + 1 + 8 + 4 + 1] = 255;
        assert_eq!(TraceV1::decode(&bad_owner), Err(TraceError::InvalidOwner));

        let mut bad_fault = encoded.clone();
        bad_fault[TRACE_DOMAIN.len() + 8 + 8 + 1 + 8 + 4 + 2] = 255;
        assert_eq!(TraceV1::decode(&bad_fault), Err(TraceError::InvalidFault));

        let mut bad_magic = encoded.clone();
        bad_magic[0] ^= 1;
        assert_eq!(TraceV1::decode(&bad_magic), Err(TraceError::InvalidMagic));

        let mut trailing = encoded.clone();
        trailing.push(0);
        assert_eq!(TraceV1::decode(&trailing), Err(TraceError::TrailingBytes));

        assert_eq!(TraceV1::decode(&encoded[..encoded.len() - 1]), Err(TraceError::TruncatedInput));
    }

    #[test]
    fn trace_v1_enforces_scenario_and_command_bounds() {
        let mut trace = TraceGeneratorV1::generate(1, 0);
        trace.scenario = 0;
        assert_eq!(trace.canonical_bytes(), Err(TraceError::InvalidScenario));

        let mut trace = TraceGeneratorV1::generate(1, 0);
        let template = trace.commands[0];
        trace.commands.resize(MAX_TRACE_COMMANDS + 1, template);
        assert_eq!(trace.canonical_bytes(), Err(TraceError::TooManyCommands));

        let mut trace = TraceGeneratorV1::generate(1, 0);
        trace.commands.clear();
        assert_eq!(trace.canonical_bytes(), Err(TraceError::EmptyCommands));
    }
}
