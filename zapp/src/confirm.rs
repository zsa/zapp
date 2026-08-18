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
use zapp_ble::smp::Half;

use crate::zsab::{decide, Decision, Landed};

/// The three things a transport does to settle a pair, plus the notes it prints
/// around them.
///
/// `running` and `commit` are the two halves of what used to be one fused
/// query: read first, commit second, and never the second without the first
/// having matched on both halves.
pub(crate) trait PairConfirm {
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
pub(crate) fn settle<P: PairConfirm>(pair: &mut P, read: &[Half], keep: &[Half]) -> Result<()> {
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
