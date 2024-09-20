#![no_std]

macro_rules! bit {
    ($x:expr) => {
        1 << $x
    };
}

pub mod io;
pub mod irq;
pub mod bits32;
pub mod controlregs;
pub mod tlb;
pub mod task;
pub mod dtables;
#[allow(dead_code)]
pub mod segmentation;

#[cfg(target_arch = "x86")]
pub(crate) use core::arch::x86 as arch;

use core::arch::asm;

pub mod current {
    #[cfg(target_arch = "x86")]
    pub use crate::bits32::*;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
/// x86 Protection levels
///
/// # Note
/// This should not contain values larger than 2 bits, otherwise
/// segment descriptor code needs to be adjusted accordingly.
pub enum Ring {
    Ring0 = 0b00,
    Ring1 = 0b01,
    Ring2 = 0b10,
    Ring3 = 0b11,
}


/// Stops instruction execution and places the processor in a HALT state.
///
/// An enabled interrupt (including NMI and SMI), a debug exception, the BINIT#
/// signal, the INIT# signal, or the RESET# signal will resume execution. If an
/// interrupt (including NMI) is used to resume execution after a HLT instruction,
/// the saved instruction pointer (CS:EIP) points to the instruction following
/// the HLT instruction.
///
/// # Safety
/// Will cause a general protection fault if used outside of ring 0.
#[inline(always)]
pub unsafe fn halt() {
    asm!("hlt", options(att_syntax, nomem, nostack)); // check if preserves_flags
}
