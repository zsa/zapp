//! ZSA's mcumgr error namespace, as the host reads it.
//!
//! The mirror of `shared/include/zsa_mgmt_err.h` in the firmware, which is the
//! contract: the firmware assigns a number and a meaning, this file turns it
//! into a sentence. Codes are append-only there, so they are append-only here,
//! and the number is what matches — never the position or the name.
//!
//! The firmware will always grow codes faster than a host learns them, so a
//! code this build has never heard of has to degrade rather than guess:
//! [`lookup`] answers `None` and the caller falls back to reporting the group
//! and the number. An old zapp saying "the board refused for a reason this
//! version does not know (group 66, rc 14)" is honest; one that picked the
//! nearest sentence it did know would be a lie about a keyboard.

/// ZSA's group in mcumgr's per-user range. See the header for why 66.
pub const GROUP: u16 = 66;

/// Which half a refusal is about, for the caller that can put a number on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

/// What a refusal is about, for a caller that has more to say than the fixed
/// sentence — the battery cases, which are worth quoting real levels at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cause {
    /// A pack is too low to start an update. Carries the half that is low.
    BatteryLow(Side),
    /// An image is written and marked, and only the restart that runs the swap
    /// is being held off until the pack recovers.
    SwapHeld(Side),
    /// Something else is already updating this keyboard.
    Busy,
    /// The halves cannot reach each other well enough to carry the update.
    Link,
    /// A request this host had no business sending.
    Protocol,
}

/// One entry of the registry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Refusal {
    pub code: u16,
    pub cause: Cause,
    /// A few words, to sit inside a one-line error.
    pub short: &'static str,
    /// What happened and what to do, in whole sentences.
    pub advice: &'static str,
}

/// The codes, in the order the firmware header lists them.
///
/// Append only, and only ever after checking the number against the header.
const REGISTRY: &[Refusal] = &[
    Refusal {
        code: 1,
        cause: Cause::BatteryLow(Side::Left),
        short: "the left half's battery is too low to update",
        advice: "The left half's battery is too low for an update. An update is minutes of \
                 radio and a reboot in which the firmware is rewritten, and a half that runs \
                 out part-way has to be recovered over a cable. Charge the keyboard, or plug \
                 it in and try again — on the charger this check does not apply.",
    },
    Refusal {
        code: 2,
        cause: Cause::BatteryLow(Side::Right),
        short: "the right half's battery is too low to update",
        advice: "The right half's battery is too low for an update. An update is minutes of \
                 radio and a reboot in which the firmware is rewritten, and a half that runs \
                 out part-way has to be recovered over a cable. Charge the keyboard, or plug \
                 it in and try again — on the charger this check does not apply.",
    },
    Refusal {
        code: 3,
        cause: Cause::SwapHeld(Side::Left),
        short: "the left half is holding its restart until it has charge",
        advice: "The left half has its new firmware and is holding the restart that would \
                 install it, because its battery is too low to see that through safely. \
                 Nothing is lost and nothing is half-written. Charge the keyboard and run \
                 this again — the restart is the only step left.",
    },
    Refusal {
        code: 4,
        cause: Cause::SwapHeld(Side::Right),
        short: "the right half is holding its restart until it has charge",
        advice: "The right half has its new firmware and is holding the restart that would \
                 install it, because its battery is too low to see that through safely. \
                 Nothing is lost and nothing is half-written. Charge the keyboard and run \
                 this again — the restart is the only step left.",
    },
    Refusal {
        code: 5,
        cause: Cause::Busy,
        short: "another update is already running on this keyboard",
        advice: "Another update is already writing firmware through the left half. Only one \
                 runs at a time, so this one was refused rather than allowed to interleave \
                 with it. Wait for the one that is running to finish, or close whatever \
                 started it, and try again.",
    },
    Refusal {
        code: 6,
        cause: Cause::Busy,
        short: "another program is already updating the right half",
        advice: "Something else is already talking to the right half — another copy of this \
                 tool, or the desktop app. The route to the right half carries one \
                 conversation at a time. Close the other one and try again.",
    },
    Refusal {
        code: 7,
        cause: Cause::Busy,
        short: "the left half is busy with its own update",
        advice: "The left half is busy writing firmware of its own, and the right half is \
                 reached through it. Wait for that to finish and try again.",
    },
    Refusal {
        code: 8,
        cause: Cause::Link,
        short: "the halves' radio link could not keep up",
        advice: "The right half is reached over the halves' own radio link, and that link \
                 stopped carrying frames fast enough to keep the update moving. Put the two \
                 halves nearer each other, away from anything that might be sitting on the \
                 same radio channel, and try again.",
    },
    Refusal {
        code: 9,
        cause: Cause::Link,
        short: "the right half could not be reached",
        advice: "The request never got to the right half. It is reached over the halves' own \
                 radio link, so check the right half is switched on, charged and near the \
                 left, then try again.",
    },
    Refusal {
        code: 10,
        cause: Cause::Protocol,
        short: "the request was too large for the route to the right half",
        advice: "This host sent the right half more in one request than the route to it can \
                 carry, so it was dropped rather than passed on in pieces. That is a bug in \
                 this tool rather than anything wrong with the keyboard; nothing was written.",
    },
];

/// The registry entry for a code, if this build knows it.
///
/// `None` for a group that is not ZSA's, and for a ZSA code newer than this
/// build — the caller reports the raw group and number instead.
pub fn lookup(group: u16, rc: i64) -> Option<Refusal> {
    if group != GROUP {
        return None;
    }
    let code = u16::try_from(rc).ok()?;
    REGISTRY.iter().copied().find(|entry| entry.code == code)
}

/// The few words that go inside a one-line error, for a code this build knows.
pub fn short(group: u16, rc: i64) -> Option<&'static str> {
    lookup(group, rc).map(|refusal| refusal.short)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_group_is_the_one_the_firmware_header_fixes() {
        // 64 and up is mcumgr's per-user range; 3, 63, 64 and 65 are taken.
        assert_eq!(GROUP, 66);
        const { assert!(GROUP >= 64) };
    }

    #[test]
    fn every_code_the_firmware_defines_has_a_sentence() {
        // The numbers are the contract, so they are checked one by one rather
        // than by counting: a gap here means a refusal a user cannot read.
        for code in 1..=10 {
            let refusal = lookup(GROUP, code).unwrap_or_else(|| panic!("no entry for {code}"));
            assert_eq!(refusal.code, u16::try_from(code).unwrap());
            assert!(!refusal.short.is_empty());
            assert!(refusal.advice.len() > 40, "code {code} has no real advice");
        }
    }

    #[test]
    fn the_numbers_are_never_reused_and_never_reordered() {
        let mut seen: Vec<u16> = REGISTRY.iter().map(|entry| entry.code).collect();
        let ordered = seen.clone();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), REGISTRY.len(), "a code is defined twice");
        assert_eq!(seen, ordered, "the registry is out of order");
    }

    #[test]
    fn the_battery_codes_say_which_half_is_flat() {
        assert_eq!(lookup(GROUP, 1).unwrap().cause, Cause::BatteryLow(Side::Left));
        assert_eq!(
            lookup(GROUP, 2).unwrap().cause,
            Cause::BatteryLow(Side::Right)
        );
        assert_eq!(lookup(GROUP, 3).unwrap().cause, Cause::SwapHeld(Side::Left));
        assert_eq!(lookup(GROUP, 4).unwrap().cause, Cause::SwapHeld(Side::Right));
    }

    #[test]
    fn the_three_things_a_user_does_differently_are_three_causes() {
        // Flat battery, something else running, and the halves out of touch
        // are the three, and each needs a different action.
        assert_eq!(lookup(GROUP, 1).unwrap().cause, Cause::BatteryLow(Side::Left));
        assert_eq!(lookup(GROUP, 5).unwrap().cause, Cause::Busy);
        assert_eq!(lookup(GROUP, 9).unwrap().cause, Cause::Link);
    }

    #[test]
    fn a_code_from_a_newer_firmware_is_not_guessed_at() {
        // The case that matters: this build must never dress an unknown
        // refusal up as one it knows.
        assert_eq!(lookup(GROUP, 11), None);
        assert_eq!(lookup(GROUP, 9999), None);
        assert_eq!(short(GROUP, 11), None);
    }

    #[test]
    fn another_groups_codes_are_never_read_as_zsas() {
        // Group 1 rc 1 is an image-management error and means nothing here.
        assert_eq!(lookup(1, 1), None);
        assert_eq!(lookup(0, 1), None);
        assert_eq!(lookup(63, 1), None);
        assert_eq!(lookup(65, 1), None);
    }

    #[test]
    fn a_negative_or_oversized_rc_is_simply_unknown() {
        assert_eq!(lookup(GROUP, -1), None);
        assert_eq!(lookup(GROUP, i64::from(u32::MAX)), None);
    }
}
