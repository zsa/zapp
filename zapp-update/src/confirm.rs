//! Settling a freshly-restarted pair, the same way over both transports.
//!
//! Whether the halves were reached over Bluetooth or over the cable, the flow
//! is one thing: read what each half is running right now, decide as a pair
//! whether that is the bundle, and only then confirm both — or put both back.
//! The read never writes, so nothing is kept before the pair has been judged
//! together, which is the whole point. Each transport supplies the three
//! operations through [`PairConfirm`]; the decision itself is [`settle`], and
//! there is exactly one of it.

use anyhow::Result;
use zapp_smp::Half;

use crate::text::{named, short};

/// The three things a transport does to settle a pair, plus the notes it makes
/// around them.
///
/// `running` and `commit` are the two halves of what used to be one fused
/// query: read first, commit second, and never the second without the first
/// having matched on both halves.
pub trait PairConfirm {
    /// READ: what `half` is running right now, waiting through a restart if it
    /// is still on its way back. Writes nothing.
    fn running(&mut self, half: Half) -> Landed;

    /// COMMIT: write the confirm trailer on `half`. Only ever called once both
    /// `running` reads have matched the bundle.
    fn commit(&mut self, half: Half) -> Result<()>;

    /// Put `half` back on its previous image, by the reboot on which the
    /// unconfirmed test image reverts.
    fn revert(&mut self, half: Half) -> Result<()>;

    /// Whether the pair came out on one build. `None` is agreement; `Some` is
    /// the complaint, which [`decide`] treats as a revert like any other.
    /// Computed from the same readings the decision is made on; `&mut` so a
    /// transport can note the verdict on its own progress as it goes.
    fn mismatch(&mut self, landed: &[(Half, Landed)]) -> Option<String>;

    /// Put both halves back and produce the failure this run ends with, with
    /// whatever recovery message the transport wants. Uses [`Self::revert`] for
    /// the reboots.
    fn revert_pair(&mut self, landed: Vec<(Half, Landed)>, reasons: Vec<String>) -> anyhow::Error;

    /// Once both reads are in, before the keep-or-revert decision.
    fn read_summary(&mut self, _landed: &[(Half, Landed)]) {}

    /// The pair is being kept; announced once, before the first commit.
    fn keeping(&mut self) {}
}

/// Reads both halves, decides as a pair, and keeps or reverts.
///
/// `read` is the halves to ask what they are running, right-first; `keep` is
/// the halves to confirm if the pair is kept, right-first. They can differ: a
/// half already running the bundle on an unconfirmed boot is kept without being
/// re-read, and a half that only had to reboot is read but may already be
/// confirmed. A half whose read comes back already confirmed is never sent a
/// commit.
///
/// The one decision the whole thing turns on, shared byte-for-byte between the
/// two transports: nothing is committed until every read is in and the pair has
/// been judged together.
pub fn settle<P: PairConfirm>(pair: &mut P, read: &[Half], keep: &[Half]) -> Result<()> {
    let landed: Vec<(Half, Landed)> = read.iter().map(|&half| (half, pair.running(half))).collect();
    pair.read_summary(&landed);

    let mismatch = pair.mismatch(&landed);
    match decide(&landed, mismatch) {
        Decision::Keep => {
            if !keep.is_empty() {
                pair.keeping();
            }
            for &half in keep {
                let already = landed.iter().any(|(which, verdict)| {
                    *which == half && matches!(verdict, Landed::Ok { confirmed: true, .. })
                });
                if !already {
                    pair.commit(half)?;
                }
            }
            Ok(())
        }
        Decision::Revert { reasons } => Err(pair.revert_pair(landed, reasons)),
    }
}

/// What a restarted half came back as.
///
/// Both transports reach the same three answers, each through its own reads,
/// and hand them to the same [`decide`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Landed {
    /// Reachable, and running the image that was written.
    Ok { confirmed: bool, active: Vec<u8> },
    /// Reachable, running something else — MCUboot put the old image back, or
    /// it never had anything running at all.
    Wrong { active: Vec<u8>, version: String },
    /// Not reachable inside the window the transport gives a restart.
    Gone { why: String },
}

impl Landed {
    /// The one sentence this verdict is worth.
    pub fn why(&self, half: Half) -> String {
        match self {
            Landed::Ok { active, .. } => {
                format!("The {half} half came back running {}.", short(active))
            }
            Landed::Wrong { active, .. } if active.is_empty() => {
                format!("The {half} half came back but reports nothing running.")
            }
            Landed::Wrong { active, version } => format!(
                "The {half} half came back running {} ({}), not the image that was written.",
                short(active),
                named(version)
            ),
            Landed::Gone { why } => why.clone(),
        }
    }
}

/// Keep both halves, or put both back.
#[derive(Debug, PartialEq, Eq)]
pub enum Decision {
    Keep,
    Revert { reasons: Vec<String> },
}

/// What to do with a pair that has come back.
///
/// Pure on purpose: this is the one decision the whole run turns on, and the
/// only part of it that can be checked without a keyboard. One half short of
/// perfect is a revert — a keyboard is a pair or it is nothing. `mismatch` is
/// the transport's pair check, a revert case like any other, since two halves
/// on two builds cannot talk to each other.
pub fn decide(landed: &[(Half, Landed)], mismatch: Option<String>) -> Decision {
    let mut reasons: Vec<String> = landed
        .iter()
        .filter(|(_, verdict)| !matches!(verdict, Landed::Ok { .. }))
        .map(|(half, verdict)| verdict.why(*half))
        .collect();
    reasons.extend(mismatch);

    match reasons.is_empty() {
        true => Decision::Keep,
        false => Decision::Revert { reasons },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A pair back from its restarts, both on what was written.
    fn both_landed() -> Vec<(Half, Landed)> {
        vec![
            (
                Half::Left,
                Landed::Ok {
                    confirmed: false,
                    active: vec![0xa1; 64],
                },
            ),
            (
                Half::Right,
                Landed::Ok {
                    confirmed: false,
                    active: vec![0xb2; 64],
                },
            ),
        ]
    }

    #[test]
    fn a_pair_that_both_came_back_on_what_was_written_is_kept() {
        assert_eq!(decide(&both_landed(), None), Decision::Keep);
        // Including the resumed run, where nothing restarted at all.
        assert_eq!(decide(&[], None), Decision::Keep);
    }

    #[test]
    fn a_half_that_never_came_back_takes_the_other_half_down_with_it() {
        let mut landed = both_landed();
        landed[1].1 = Landed::Gone {
            why: "The right half did not come back within 180 s of its restart.".to_string(),
        };
        let Decision::Revert { reasons } = decide(&landed, None) else {
            panic!("a missing half is not something to keep an image over");
        };
        assert_eq!(reasons.len(), 1, "{reasons:?}");
        assert!(reasons[0].contains("right half did not come back"), "{reasons:?}");
    }

    #[test]
    fn a_half_that_came_back_on_the_wrong_image_reverts_the_pair() {
        let mut landed = both_landed();
        landed[0].1 = Landed::Wrong {
            active: vec![0x99; 64],
            version: "0.1.9".to_string(),
        };
        let Decision::Revert { reasons } = decide(&landed, None) else {
            panic!("a half on the old image is a failed update");
        };
        assert!(reasons[0].contains("The left half came back running"), "{reasons:?}");
        assert!(reasons[0].contains("0.1.9"), "{reasons:?}");
        assert!(
            reasons[0].contains("not the image that was written"),
            "{reasons:?}"
        );

        // A half reporting nothing at all is named as that rather than given an
        // empty hash to print.
        landed[0].1 = Landed::Wrong {
            active: Vec::new(),
            version: String::new(),
        };
        let Decision::Revert { reasons } = decide(&landed, None) else {
            panic!("a half with nothing running is a failed update");
        };
        assert!(reasons[0].contains("reports nothing running"), "{reasons:?}");
    }

    #[test]
    fn two_halves_that_both_came_back_on_different_builds_still_revert() {
        // Each half was checked against the bundle on its own and both passed,
        // which is not the same as the pair matching.
        let Decision::Revert { reasons } =
            decide(&both_landed(), Some("they are on two builds".to_string()))
        else {
            panic!("two builds on one keyboard is not a keyboard");
        };
        assert_eq!(reasons, vec!["they are on two builds".to_string()]);
    }
}
