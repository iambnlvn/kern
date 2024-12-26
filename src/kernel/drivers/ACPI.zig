const std = @import("std");
const kernel = @import("../kernel.zig");
const arch = kernel.arch;
const CPU = arch.CPU;
const EsMemorySumBytes = kernel.EsMemorySumBytes;
const Region = kernel.memory.Region;
pub var driver: Driver = undefined;

pub const Driver = extern struct {
    procCount: u64,
    IOAPICCount: u64,
    interruptOverrideCount: u64,
    LAPICNMICount: u64,
    procs: [254]CPU,
    IOAPICS: [16]IOAPIC,
    interruptOverrides: [256]InterruptOverride,
    LAPICNMIs: [32]LAPICNMI,
    rdsp: *RootSystemDescriptorPointer,
    madt: *MultipleAPICDescriptionTable,
    LAPICAddr: [*]volatile 32,
    LAPICTicksPerMS: u64,

    PS2ControllerUnvailable: bool,
    VGAControllerUnvailable: bool,
    centuryRegIDX: u8,
    HPETBaseAddr: ?[*]volatile u64,
    HPETPeriod: u64,
    computer: u64,

    const RSDPSignature = 0x2052545020445352;
    const RSDTSignature = 0x54445352;
    const XSDTSignature = 0x54445358;
    const MADTSignature = 0x43495041;
    const FADTSignature = 0x50434146;
    const HPETSignature = 0x54455048;

    pub const IOAPIC = extern struct {
        id: u8,
        address: [*]volatile u32,
        GSIBase: u32,

        pub fn read(self: *@This(), reg: u32) u32 {
            self.address[0] = reg;
            return self.address[4];
        }

        pub fn write(self: *@This(), reg: u32, val: u32) void {
            self.address[0] = reg;
            self.address[4] = val;
        }
    };

    pub const InterruptOverride = extern struct {
        IRQSource: u8,
        GSINum: u32,
        isActiveLow: bool,
        islevelTriggered: bool,
    };

    pub const LAPICNMI = extern struct {
        proc: u8,
        lintIndex: u8,
        isActiveLow: bool,
        isLevelTriggered: bool,
    };

    pub const RootSystemDescriptorPointer = extern struct {
        signature: u32,
        length: u32,
        id: u64,
        tableID: u64,
        OEMRevision: u32,
        creatorID: u32,
        creatorRevision: u32,

        pub const headerLen = 36;

        pub fn check(self: *align(1) @This()) void {
            if (EsMemorySumBytes(@as([*]u8, @ptrCast(self)), self.length) != 0) {
                std.debug.panic("ACPI table checksum failed!", .{});
            }
        }
    };

    pub const MultipleAPICDescriptionTable = extern struct {
        LAPICAddr: u32,
        flags: u32,
    };
};
