//! Processor state stored in the EFLAGS register.

use crate::Ring;
use core::arch::asm;
use tock_registers::{
    debug::RegisterDebugInfo,
    fields::{Field, FieldValue},
    register_bitfields, LocalRegisterCopy,
};

register_bitfields!(u32,
    pub EFlagsBitField [
        /// ID Flag (ID)
        FLAGS_ID OFFSET(21) NUMBITS(1) [],
        /// Virtual Interrupt Pending (VIP)
        FLAGS_VIP OFFSET(20) NUMBITS(1) [],
        /// Virtual Interrupt Flag (VIF)
        FLAGS_VIF OFFSET(19) NUMBITS(1) [],
        /// Alignment Check (AC)
        FLAGS_AC OFFSET(18) NUMBITS(1) [],
        /// Virtual-8086 Mode (VM)
        FLAGS_VM OFFSET(17) NUMBITS(1) [],
        /// Resume Flag (RF)
        FLAGS_RF OFFSET(16) NUMBITS(1) [],
        /// Nested Task (NT)
        FLAGS_NT OFFSET(14) NUMBITS(1) [],
        /// I/O Privilege Level (IOPL)
        FLAGS_IOPL OFFSET(12) NUMBITS(2) [
            FLAGS_IOPL0 = 0b00,
            FLAGS_IOPL1 = 0b01,
            FLAGS_IOPL2 = 0b10,
            FLAGS_IOPL3 = 0b11
        ],
        /// Overflow Flag (OF)
        FLAGS_OF OFFSET(11) NUMBITS(1) [],
        /// Direction Flag (DF)
        FLAGS_DF OFFSET(10) NUMBITS(1) [],
        /// Interrupt Enable Flag (IF)
        FLAGS_IF OFFSET(9) NUMBITS(1) [],
        /// Trap Flag (TF)
        FLAGS_TF OFFSET(8) NUMBITS(1) [],
        /// Sign Flag (SF)
        FLAGS_SF OFFSET(7) NUMBITS(1) [],
        /// Zero Flag (ZF)
        FLAGS_ZF OFFSET(6) NUMBITS(1) [],
        /// Auxiliary Carry Flag (AF)
        FLAGS_AF OFFSET(4) NUMBITS(1) [],
        /// Parity Flag (PF)
        FLAGS_PF OFFSET(2) NUMBITS(1) [],
        /// Bit 1 is always 1.
        FLAGS_A1 OFFSET(1) NUMBITS(1) [],
        /// Carry Flag (CF)
        FLAGS_CF OFFSET(0) NUMBITS(1) []
    ]
);

pub struct EFlags {
    register: LocalRegisterCopy<u32, EFlagsBitField::Register>,
}

impl EFlags {
    /// Creates a new Flags entry. Ensures bit 1 is set.
    pub const fn new() -> EFlags {
        EFlags {
            register: LocalRegisterCopy::new(0),
        }
    }

    /// Creates a new Flags with the given I/O privilege level.
    pub const fn from_priv(iopl: Ring) -> EFlags {
        let field = match iopl {
            Ring::Ring0 => EFlagsBitField::FLAGS_IOPL::FLAGS_IOPL0,
            Ring::Ring1 => EFlagsBitField::FLAGS_IOPL::FLAGS_IOPL1,
            Ring::Ring2 => EFlagsBitField::FLAGS_IOPL::FLAGS_IOPL2,
            Ring::Ring3 => EFlagsBitField::FLAGS_IOPL::FLAGS_IOPL3,
        };
        EFlags {
            register: LocalRegisterCopy::new(field.value),
        }
    }

    pub fn bits(&self) -> u32 {
        self.register.get()
    }

    pub fn contains(&self, field: Field<u32, EFlagsBitField::Register>) -> bool {
        self.register.is_set(field)
    }

    pub fn set(&mut self, value: FieldValue<u32, EFlagsBitField::Register>) {
        self.register.modify(value);
    }
}

#[inline(always)]
pub fn from_bits_truncate(val: u32) -> EFlags {
    let mut mask = 0;
    for field in EFlagsBitField::Register::fields() {
        mask |= field.mask;
    }
    EFlags {
        register: LocalRegisterCopy::new(val & mask),
    }
}

#[inline(always)]
pub unsafe fn read() -> EFlags {
    let r: u32;
    asm!("pushfl; popl {0}", out(reg) r, options(att_syntax));
    from_bits_truncate(r)
}

#[inline(always)]
pub unsafe fn set(val: EFlags) {
    asm!("pushl {0}; popfl", in(reg) val.bits(), options(att_syntax));
}

/// Clears the AC flag bit in EFLAGS register.
///
/// This disables any alignment checking of user-mode data accesses.
/// If the SMAP bit is set in the CR4 register, this disallows
/// explicit supervisor-mode data accesses to user-mode pages.
///
/// # Safety
///
/// This instruction is only valid in Ring 0 and requires
/// that the CPU supports the instruction (check CPUID).
#[inline(always)]
pub unsafe fn clac() {
    asm!("clac");
}

/// Sets the AC flag bit in EFLAGS register.
///
/// This may enable alignment checking of user-mode data accesses.
/// This allows explicit supervisor-mode data accesses to user-mode
/// pages even if the SMAP bit is set in the CR4 register.
///
/// # Safety
///
/// This instruction is only valid in Ring 0 and requires
/// that the CPU supports the instruction (check CPUID).
#[inline(always)]
pub unsafe fn stac() {
    asm!("stac");
}
