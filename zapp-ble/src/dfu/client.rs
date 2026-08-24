//! Driving one STM32 update, over any three GATT channels.

use std::thread;
use std::time::{Duration, Instant};

use zapp_core::flash::FlashProgress;

use super::{
    crc32, start_payload, DfuError, Fault, Report, Status, CMD_ABORT, CMD_ENTER, CMD_STATUS,
    MAX_IMAGE,
};
use crate::backend::GattChannel;

/// How long the mass erase may take.
///
/// The whole application region goes, page by page, and the board does it
/// inside the control write's own handler — so this covers the write and the
/// answer together.
pub const ERASE_TIMEOUT: Duration = Duration::from_secs(60);

/// How long to wait for the STM32 to say the image is good.
///
/// The last chunk is what triggers it: the board sends the manifest, the STM32
/// checks the image against the CRC, and only then does anything come back.
pub const VERIFY_TIMEOUT: Duration = Duration::from_secs(60);

/// How long to give the STM32 to reach its bootloader.
///
/// The board's own attempt gives up after 5 s. This is longer on purpose: it is
/// also the window in which somebody can press the reset button by hand, which
/// is the way back when the running firmware is too broken to be asked.
pub const READY_TIMEOUT: Duration = Duration::from_secs(30);

/// How long one control opcode may take to be answered.
pub const REPLY_TIMEOUT: Duration = Duration::from_secs(10);

/// How long to sit on the notification channel before reading instead.
const LISTEN_SLICE: Duration = Duration::from_millis(500);

/// Between STATUS queries while waiting for the STM32 to come up.
const ASK_EVERY: Duration = Duration::from_secs(1);

/// How often a long wait says it is still a wait.
const NOTE_EVERY: Duration = Duration::from_secs(5);

/// Long enough for a notification already in the pipe, short enough not to be
/// felt before every command.
const DRAIN: Duration = Duration::from_millis(5);

/// Chunks to send before reading how many of them landed.
///
/// Every read is a round trip that the radio spends not carrying image, and
/// every one skipped is that much longer before a failure is noticed. Eight
/// chunks is about 2 KiB, so a 240 KiB image pays for it 120 times.
const CHECK_EVERY: usize = 8;

/// What the readiness step is doing, for a caller that wants to say so.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wait {
    /// The STM32 was already sitting in its bootloader: nothing was rebooted.
    Already,
    /// It has been told to reboot into it.
    Rebooting,
    /// Still not there, this long in.
    Waiting(Duration),
    /// It got there, after being asked or after somebody pressed reset.
    Arrived(Duration),
}

pub struct DfuClient {
    control: Box<dyn GattChannel>,
    data: Box<dyn GattChannel>,
    status: Box<dyn GattChannel>,
    /// Bytes of image per write.
    chunk: usize,
    /// How long one control opcode may take to be answered.
    reply: Duration,
}

impl DfuClient {
    pub fn new(
        control: Box<dyn GattChannel>,
        data: Box<dyn GattChannel>,
        status: Box<dyn GattChannel>,
    ) -> Self {
        let chunk = even(data.mtu());
        Self {
            control,
            data,
            status,
            chunk,
            reply: REPLY_TIMEOUT,
        }
    }

    /// Overrides what the platform said one write can carry.
    pub fn with_chunk(mut self, chunk: usize) -> Self {
        self.chunk = even(chunk.min(self.data.mtu()));
        self
    }

    /// How long to wait for a control opcode to be answered.
    ///
    /// Not the erase or the verify, which take as long as flash takes and have
    /// their own budgets.
    pub fn with_timeout(mut self, reply: Duration) -> Self {
        self.reply = reply;
        self
    }

    /// Bytes of image one write carries.
    pub fn chunk(&self) -> usize {
        self.chunk
    }

    /// Asks the board what the STM32 is doing, rather than what it last said.
    ///
    /// The write is the point: STATUS makes the board go and query the STM32
    /// over UART and answer from that. Reading the characteristic alone gives
    /// whatever was last notified, which after a reboot nobody told it about is
    /// a memory rather than a state.
    pub fn query(&mut self) -> Result<Report, DfuError> {
        self.command(CMD_STATUS, "asking what the STM32 is doing")
    }

    /// What the board last said, without asking it anything.
    pub fn read_status(&mut self) -> Result<Report, DfuError> {
        let value = self.status.read()?;
        Report::decode(&value)
    }

    /// Drops whatever transfer is open and releases the lock.
    pub fn abort(&mut self) -> Result<Report, DfuError> {
        self.command(CMD_ABORT, "dropping the transfer")
    }

    /// Reboots the STM32 into the ignition bootloader and waits for it.
    ///
    /// The board answers when it is there, or when it has given up after its
    /// own five seconds — this does not decide which of those is fatal, because
    /// a caller that is willing to wait longer can still win.
    pub fn enter(&mut self) -> Result<Report, DfuError> {
        self.command(CMD_ENTER, "rebooting the STM32 into its bootloader")
    }

    /// Makes sure the STM32 is in its bootloader, rebooting it only if it is
    /// not.
    ///
    /// Three ways this ends well: it was already there, it went there when
    /// asked, or somebody pressed the reset button while this was waiting. The
    /// last one is why the deadline is not the board's: after ENTER gives up
    /// there is still a person who can put it there by hand, and a client that
    /// stopped listening at that point would refuse an update that was one
    /// button press from working.
    pub fn make_ready(
        &mut self,
        timeout: Duration,
        on_wait: &dyn Fn(Wait),
    ) -> Result<(), DfuError> {
        let started = Instant::now();

        let first = self.query()?;
        locked(&first)?;
        if first.status.in_bootloader() {
            on_wait(Wait::Already);
            return Ok(());
        }

        on_wait(Wait::Rebooting);
        // A refusal is not returned here, only a lock. The board gives up after
        // five seconds and says so, and that is exactly the moment a person
        // reaches for the reset button — so the answer is noted and the waiting
        // carries on rather than ending on it.
        let entered = self.enter()?;
        locked(&entered)?;
        if entered.status.in_bootloader() {
            on_wait(Wait::Arrived(started.elapsed()));
            return Ok(());
        }
        log::debug!(
            "the board could not put the STM32 in its bootloader ({}); \
             waiting in case it is reset by hand",
            entered.fault
        );

        let deadline = started + timeout;
        let mut noted = Instant::now();
        while Instant::now() < deadline {
            thread::sleep(ASK_EVERY);
            let report = self.query()?;
            locked(&report)?;
            if report.status.in_bootloader() {
                on_wait(Wait::Arrived(started.elapsed()));
                return Ok(());
            }
            if noted.elapsed() >= NOTE_EVERY {
                noted = Instant::now();
                on_wait(Wait::Waiting(started.elapsed()));
            }
        }

        Err(DfuError::NotReady {
            waited: started.elapsed(),
        })
    }

    /// Writes one image into the STM32's application region.
    ///
    /// The STM32 must already be in its bootloader — `make_ready` is what puts
    /// it there. Everything is checked before the START, because START is what
    /// erases: a file refused here leaves a working keyboard, and one refused a
    /// step later leaves a blank one.
    pub fn flash(
        &mut self,
        image: &[u8],
        on_progress: &dyn Fn(FlashProgress),
    ) -> Result<(), DfuError> {
        if image.is_empty() {
            return Err(DfuError::EmptyImage);
        }
        if image.len() > MAX_IMAGE {
            return Err(DfuError::ImageTooLarge { size: image.len() });
        }
        if self.chunk < 2 {
            return Err(DfuError::ChunkTooSmall(self.chunk));
        }
        let total = image.len();

        on_progress(FlashProgress::Erasing {
            bytes_erased: 0,
            total_bytes: total,
        });
        self.start(total as u32, crc32(image))?;
        on_progress(FlashProgress::Erasing {
            bytes_erased: total,
            total_bytes: total,
        });

        on_progress(FlashProgress::Writing {
            bytes_written: 0,
            total_bytes: total,
        });
        let mut sent = 0usize;
        for (nth, chunk) in image.chunks(self.chunk).enumerate() {
            self.data.write(chunk)?;
            sent += chunk.len();

            // Not after the last one: that chunk is what makes the board send
            // the manifest, and the answer to it is waited for below.
            if sent < total && nth % CHECK_EVERY == CHECK_EVERY - 1 {
                let report = self.read_status()?.check()?;
                let landed = report.received.map_or(sent, |received| {
                    (received as usize).min(total)
                });
                on_progress(FlashProgress::Writing {
                    bytes_written: landed,
                    total_bytes: total,
                });
            }
        }
        on_progress(FlashProgress::Writing {
            bytes_written: total,
            total_bytes: total,
        });

        // The last chunk sends the manifest, the STM32 verifies the image
        // against the CRC, and DONE is the board saying it accepted it.
        on_progress(FlashProgress::Resetting);
        self.settle(
            |report| report.status == Status::Done,
            VERIFY_TIMEOUT,
            "waiting for the STM32 to check the image",
        )?;
        on_progress(FlashProgress::Complete);
        Ok(())
    }

    /// Claims the board, checks the STM32 is there, and erases the region.
    fn start(&mut self, total: u32, crc: u32) -> Result<(), DfuError> {
        self.drain();
        // The board erases inside its own write handler, so this call is held
        // open for as long as that takes and may well time out underneath us.
        // The answer still comes as a notification, so a write that did not
        // come back cleanly is not yet a failure — the wait below decides.
        if let Err(err) = self.control.write(&start_payload(total, crc)) {
            log::warn!("the START write did not come back cleanly ({err}); waiting anyway");
        }
        // Notifications only. A read here would answer with whatever state the
        // board was last in — READY, from the ENTER a moment ago — and the
        // erase would still be running under it.
        let ready = self.notified(ERASE_TIMEOUT, "erasing the STM32's flash")?;
        ready.check()?;
        match ready.status {
            Status::Ready => Ok(()),
            status => Err(DfuError::Malformed(format!(
                "the keyboard answered a start with {status} rather than being ready"
            ))),
        }
    }

    /// One control opcode, and whatever the board answers it with.
    ///
    /// The answer is returned as it came, refusal and all: a failed ENTER is
    /// the board reporting what happened to the STM32, and only the caller
    /// knows whether that ends the job.
    fn command(&mut self, opcode: u8, doing: &'static str) -> Result<Report, DfuError> {
        self.drain();
        self.control.write(&[opcode])?;
        self.notified(self.reply, doing)
    }

    /// Waits for the next notification the board sends of its own accord.
    ///
    /// Nothing is read: this is for the steps where the characteristic still
    /// holds the state the board was in before the command, and reading it
    /// would answer the previous question.
    fn notified(&mut self, timeout: Duration, doing: &'static str) -> Result<Report, DfuError> {
        let deadline = Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(DfuError::Timeout {
                    doing,
                    waited: timeout,
                });
            }
            if let Some(value) = self.status.notification(left)? {
                return Report::decode(&value);
            }
        }
    }

    /// Waits for a state, listening first and reading when nothing arrives.
    ///
    /// The read is the safety net for a notification that was missed — a
    /// subscription BlueZ dropped, a platform that cannot subscribe at all —
    /// and is only safe where the state being waited for is one the board
    /// cannot already have been in.
    fn settle(
        &mut self,
        accept: impl Fn(&Report) -> bool,
        timeout: Duration,
        doing: &'static str,
    ) -> Result<Report, DfuError> {
        let deadline = Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(DfuError::Timeout {
                    doing,
                    waited: timeout,
                });
            }
            let report = match self.status.notification(left.min(LISTEN_SLICE))? {
                Some(value) => Report::decode(&value)?,
                None => self.read_status()?,
            };
            let report = report.check()?;
            if accept(&report) {
                return Ok(report);
            }
        }
    }

    /// Throws away answers to whatever came before.
    ///
    /// A notification still in the pipe belongs to the last command, and taking
    /// it for this one's would read a stale READY as the erase having finished.
    fn drain(&mut self) {
        while let Ok(Some(stale)) = self.status.notification(DRAIN) {
            log::debug!("dropping a notification left over from an earlier command: {stale:?}");
        }
    }
}

/// Fails on the one refusal that waiting cannot fix.
///
/// A lock is held by another transport — the nRF's own update path, or the
/// relay to the right half — and it does not lapse because somebody stared at
/// it. Everything else about a keyboard that is not ready yet might.
fn locked(report: &Report) -> Result<(), DfuError> {
    match report.fault == Fault::Locked {
        true => Err(DfuError::Refused {
            status: report.status,
            fault: report.fault,
        }),
        false => Ok(()),
    }
}

/// Rounded down to an even length.
///
/// The board splits a chunk into even-length UART frames and carries an odd
/// trailing byte over to the next one, which works and is one more path than an
/// even chunk exercises.
fn even(bytes: usize) -> usize {
    bytes & !1
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    use crate::dfu::CMD_START;
    use crate::BleError;

    /// The bits of a Voyager 2's left half this protocol touches.
    ///
    /// Written as the firmware behaves rather than as the client hopes:
    /// notifications only on a state change, the byte count only on a read, and
    /// a status characteristic that keeps saying whatever it last said.
    #[derive(Default)]
    struct Board {
        /// Whether the STM32 answers as being in its bootloader.
        in_bootloader: bool,
        /// Queries left before it gets there by itself, for the reset-button
        /// case.
        arrives_after: Option<u32>,
        /// ENTER works.
        enter_works: bool,
        /// Something else holds the update lock.
        locked: bool,
        /// A transfer is open: data writes are taken.
        active: bool,
        total: u32,
        crc: u32,
        received: u32,
        image: Vec<u8>,
        last: (u8, u8),
        notifications: VecDeque<Vec<u8>>,
        /// The image the STM32 would accept. Anything else fails to verify.
        expect_crc: Option<u32>,
    }

    impl Board {
        fn new() -> Self {
            Self {
                enter_works: true,
                last: (0x00, 0x00),
                ..Self::default()
            }
        }

        fn notify(&mut self, status: u8, fault: u8) {
            self.last = (status, fault);
            let one_byte = status != 0x03 && fault == 0x00;
            self.notifications.push_back(match one_byte {
                true => vec![status],
                false => vec![status, fault],
            });
        }

        fn control(&mut self, value: &[u8]) {
            match value[0] {
                CMD_START => {
                    let total = u32::from_le_bytes(value[1..5].try_into().unwrap());
                    let crc = match value.len() >= 9 {
                        true => u32::from_le_bytes(value[5..9].try_into().unwrap()),
                        false => 0,
                    };
                    if total == 0 || total as usize > MAX_IMAGE {
                        return self.notify(0x03, 0x04);
                    }
                    if self.locked {
                        return self.notify(0x03, 0x03);
                    }
                    if !self.in_bootloader {
                        return self.notify(0x04, 0x00);
                    }
                    self.total = total;
                    self.crc = crc;
                    self.received = 0;
                    self.image.clear();
                    self.active = true;
                    self.notify(0x01, 0x00);
                }
                CMD_ABORT => {
                    self.active = false;
                    self.notify(0x00, 0x00);
                }
                CMD_STATUS => {
                    if let Some(left) = self.arrives_after.as_mut() {
                        match *left {
                            0 => self.in_bootloader = true,
                            _ => *left -= 1,
                        }
                    }
                    let (status, fault) = match (self.in_bootloader, self.active) {
                        (false, _) => (0x04, 0x00),
                        (true, true) => (0x01, 0x00),
                        (true, false) => (0x00, 0x00),
                    };
                    self.notify(status, fault);
                }
                CMD_ENTER => {
                    if self.locked {
                        return self.notify(0x03, 0x03);
                    }
                    match self.enter_works {
                        true => {
                            self.in_bootloader = true;
                            self.notify(0x01, 0x00);
                        }
                        // NOT_RDY with ERR_ENTER, which is what the firmware
                        // sends when the STM32 never answers.
                        false => self.notify(0x04, 0x06),
                    }
                }
                other => panic!("an opcode the board does not have: {other:#04x}"),
            }
        }

        fn data(&mut self, value: &[u8]) {
            if !self.active {
                return; // write-without-response: the refusal goes nowhere
            }
            self.received += value.len() as u32;
            self.image.extend_from_slice(value);
            if self.received >= self.total {
                self.active = false;
                let good = self
                    .expect_crc
                    .unwrap_or_else(|| crc32(&self.image));
                match self.crc == 0 || self.crc == good {
                    true => self.notify(0x02, 0x00),
                    false => self.notify(0x03, 0x05),
                }
            }
        }

        fn read(&self) -> Vec<u8> {
            let mut value = vec![self.last.0, self.last.1];
            value.extend_from_slice(&self.received.to_le_bytes());
            value
        }
    }

    /// One characteristic of the shared board.
    struct Wire {
        board: Arc<Mutex<Board>>,
        which: Which,
        mtu: usize,
    }

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Which {
        Control,
        Data,
        Status,
    }

    impl GattChannel for Wire {
        fn mtu(&self) -> usize {
            self.mtu
        }

        fn read(&mut self) -> Result<Vec<u8>, BleError> {
            Ok(self.board.lock().unwrap().read())
        }

        fn write(&mut self, value: &[u8]) -> Result<(), BleError> {
            assert!(value.len() <= self.mtu, "a write longer than the MTU");
            let mut board = self.board.lock().unwrap();
            match self.which {
                Which::Control => board.control(value),
                Which::Data => board.data(value),
                Which::Status => panic!("the status characteristic is not written"),
            }
            Ok(())
        }

        fn notification(&mut self, _timeout: Duration) -> Result<Option<Vec<u8>>, BleError> {
            assert!(self.which == Which::Status, "only the status char notifies");
            Ok(self.board.lock().unwrap().notifications.pop_front())
        }
    }

    fn client_for(board: &Arc<Mutex<Board>>, mtu: usize) -> DfuClient {
        let wire = |which| {
            Box::new(Wire {
                board: Arc::clone(board),
                which,
                mtu,
            }) as Box<dyn GattChannel>
        };
        DfuClient::new(wire(Which::Control), wire(Which::Data), wire(Which::Status))
    }

    fn ready_board() -> Arc<Mutex<Board>> {
        Arc::new(Mutex::new(Board {
            in_bootloader: true,
            ..Board::new()
        }))
    }

    #[test]
    fn a_chunk_is_the_write_budget_rounded_to_an_even_length() {
        let board = ready_board();
        // 252-byte ATT MTU, less the 3-byte header, less the odd byte.
        assert_eq!(client_for(&board, 249).chunk(), 248);
        assert_eq!(client_for(&board, 244).chunk(), 244);
        // And an override never outgrows what the link can carry.
        assert_eq!(client_for(&board, 100).with_chunk(4096).chunk(), 100);
        assert_eq!(client_for(&board, 249).with_chunk(65).chunk(), 64);
    }

    #[test]
    fn the_bytes_the_stm32_ends_up_with_are_the_bytes_that_went_in() {
        let board = ready_board();
        let image: Vec<u8> = (0..5000u32).map(|byte| (byte % 251) as u8).collect();
        let seen = Mutex::new(Vec::new());

        client_for(&board, 249)
            .flash(&image, &|progress| {
                if let FlashProgress::Writing { bytes_written, .. } = progress {
                    seen.lock().unwrap().push(bytes_written);
                }
            })
            .unwrap();

        let board = board.lock().unwrap();
        assert_eq!(board.image, image);
        assert_eq!(board.crc, crc32(&image), "the START announced the checksum");
        assert_eq!(board.total, 5000);
        assert!(!board.active, "the transfer was closed");

        // Progress runs from nothing to the whole image, and never backwards.
        let seen = seen.into_inner().unwrap();
        assert_eq!(seen.first(), Some(&0));
        assert_eq!(seen.last(), Some(&image.len()));
        assert!(seen.windows(2).all(|pair| pair[0] <= pair[1]), "{seen:?}");
    }

    #[test]
    fn every_write_is_even_and_within_the_budget() {
        let board = ready_board();
        // An odd image over an odd budget: the last chunk is the only odd one.
        let image: Vec<u8> = (0..1001u32).map(|byte| byte as u8).collect();
        client_for(&board, 101).flash(&image, &|_| {}).unwrap();
        assert_eq!(board.lock().unwrap().image, image);
    }

    #[test]
    fn an_image_the_stm32_refuses_says_it_arrived_corrupt() {
        let board = ready_board();
        board.lock().unwrap().expect_crc = Some(0x1234_5678);
        let err = client_for(&board, 249)
            .flash(&[0x42; 600], &|_| {})
            .unwrap_err();
        assert!(
            matches!(
                err,
                DfuError::Refused {
                    fault: Fault::Verify,
                    ..
                }
            ),
            "{err:?}"
        );
        assert!(err.to_string().contains("corrupt"));
    }

    #[test]
    fn a_board_that_is_locked_is_refused_before_anything_is_erased() {
        let board = ready_board();
        board.lock().unwrap().locked = true;
        let err = client_for(&board, 249)
            .flash(&[0x42; 600], &|_| {})
            .unwrap_err();
        assert!(
            matches!(
                err,
                DfuError::Refused {
                    fault: Fault::Locked,
                    ..
                }
            ),
            "{err:?}"
        );
        assert!(board.lock().unwrap().image.is_empty());
    }

    #[test]
    fn an_image_past_the_app_region_never_reaches_the_keyboard() {
        // Refused here rather than there: the board would answer BadLength, but
        // only after a round trip, and the message would name no file.
        let board = ready_board();
        let err = client_for(&board, 249)
            .flash(&vec![0u8; MAX_IMAGE + 1], &|_| {})
            .unwrap_err();
        assert!(matches!(err, DfuError::ImageTooLarge { size } if size == MAX_IMAGE + 1));
        assert_eq!(board.lock().unwrap().total, 0, "nothing was started");

        let err = client_for(&board, 249).flash(&[], &|_| {}).unwrap_err();
        assert!(matches!(err, DfuError::EmptyImage), "{err:?}");
    }

    #[test]
    fn an_image_that_exactly_fills_the_region_is_taken() {
        let board = ready_board();
        client_for(&board, 249)
            .flash(&vec![0xa5; MAX_IMAGE], &|_| {})
            .unwrap();
        assert_eq!(board.lock().unwrap().received as usize, MAX_IMAGE);
    }

    #[test]
    fn a_keyboard_already_in_its_bootloader_is_not_rebooted() {
        let board = ready_board();
        let seen = Mutex::new(Vec::new());
        client_for(&board, 249)
            .make_ready(READY_TIMEOUT, &|wait| seen.lock().unwrap().push(wait))
            .unwrap();
        assert_eq!(seen.into_inner().unwrap(), vec![Wait::Already]);
    }

    #[test]
    fn a_keyboard_that_is_not_there_yet_is_told_to_go() {
        let board = Arc::new(Mutex::new(Board::new()));
        let seen = Mutex::new(Vec::new());
        client_for(&board, 249)
            .make_ready(READY_TIMEOUT, &|wait| seen.lock().unwrap().push(wait))
            .unwrap();

        let seen = seen.into_inner().unwrap();
        assert_eq!(seen.first(), Some(&Wait::Rebooting));
        assert!(matches!(seen.last(), Some(Wait::Arrived(_))), "{seen:?}");
        assert!(board.lock().unwrap().in_bootloader);
    }

    #[test]
    fn a_reset_pressed_by_hand_is_noticed_after_enter_gave_up() {
        // The case the board cannot report: ENTER failed, and somebody put the
        // keyboard into its bootloader themselves while this was waiting.
        let board = Arc::new(Mutex::new(Board {
            enter_works: false,
            arrives_after: Some(2),
            ..Board::new()
        }));
        let seen = Mutex::new(Vec::new());
        client_for(&board, 249)
            .make_ready(Duration::from_secs(10), &|wait| {
                seen.lock().unwrap().push(wait)
            })
            .unwrap();
        assert!(matches!(
            seen.into_inner().unwrap().last(),
            Some(Wait::Arrived(_))
        ));
    }

    #[test]
    fn a_keyboard_that_never_gets_there_gives_up_rather_than_waiting_on() {
        let board = Arc::new(Mutex::new(Board {
            enter_works: false,
            ..Board::new()
        }));
        let err = client_for(&board, 249)
            .make_ready(Duration::from_secs(2), &|_| {})
            .unwrap_err();
        assert!(matches!(err, DfuError::NotReady { .. }), "{err:?}");
    }

    #[test]
    fn a_locked_keyboard_is_not_waited_out() {
        let board = Arc::new(Mutex::new(Board {
            locked: true,
            ..Board::new()
        }));
        let started = Instant::now();
        let err = client_for(&board, 249)
            .make_ready(Duration::from_secs(30), &|_| {})
            .unwrap_err();
        assert!(
            matches!(
                err,
                DfuError::Refused {
                    fault: Fault::Locked,
                    ..
                }
            ),
            "{err:?}"
        );
        assert!(started.elapsed() < Duration::from_secs(5), "it waited");
    }

    #[test]
    fn a_status_query_asks_the_board_rather_than_reading_what_it_last_said() {
        let board = Arc::new(Mutex::new(Board::new()));
        let mut client = client_for(&board, 249);
        assert_eq!(client.query().unwrap().status, Status::NotReady);

        board.lock().unwrap().in_bootloader = true;
        assert_eq!(client.query().unwrap().status, Status::Idle);
        assert_eq!(client.abort().unwrap().status, Status::Idle);
    }

    #[test]
    fn a_read_carries_the_count_a_notification_does_not() {
        let board = ready_board();
        board.lock().unwrap().received = 4242;
        let mut client = client_for(&board, 249);
        assert_eq!(client.read_status().unwrap().received, Some(4242));
        assert_eq!(client.query().unwrap().received, None);
    }

    /// A board that answers nothing at all, for the cases where none does.
    struct Deaf;
    impl GattChannel for Deaf {
        fn mtu(&self) -> usize {
            249
        }
        fn read(&mut self) -> Result<Vec<u8>, BleError> {
            Ok(vec![0x00, 0x00, 0, 0, 0, 0])
        }
        fn write(&mut self, _value: &[u8]) -> Result<(), BleError> {
            Ok(())
        }
        fn notification(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, BleError> {
            thread::sleep(timeout.min(Duration::from_millis(20)));
            Ok(None)
        }
    }

    #[test]
    fn a_keyboard_that_says_nothing_times_out_rather_than_hanging() {
        let mut client = DfuClient::new(Box::new(Deaf), Box::new(Deaf), Box::new(Deaf))
            .with_timeout(Duration::from_millis(50));
        let err = client.query().unwrap_err();
        assert!(matches!(err, DfuError::Timeout { .. }), "{err:?}");
    }
}
