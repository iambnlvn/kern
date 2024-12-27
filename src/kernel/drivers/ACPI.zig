const std = @import("std");
const kernel = @import("../kernel.zig");
const arch = kernel.arch;
const CPU = arch.CPU;
const EsMemorySumBytes = kernel.EsMemorySumBytes;
const Region = kernel.memory.Region;
pub var driver: Driver = undefined;

pub const Driver = extern struct {
    // Metadata
    procCount: u64,
    IOAPICCount: u64,
    interruptOverrideCount: u64,
    LAPICNMICount: u64,

    // Data structures
    procs: [254]CPU,
    IOAPICS: [16]IOAPIC,
    interruptOverrides: [256]InterruptOverride,
    LAPICNMIs: [32]LAPICNMI,

    // ACPI Tables
    rdsp: *RootSystemDescriptorPointer,
    madt: *align(1) DescriptorTable,

    // Memory and timing
    LAPICAddr: [*]volatile u32,
    LAPICTicksPerMS: u64,

    // Peripheral configuration
    PS2ControllerUnvailable: bool,
    VGAControllerUnvailable: bool,
    centuryRegIDX: u8,
    HPETBaseAddr: ?[*]volatile u64,
    HPETPeriod: u64,

    // Computer-specific identifier
    computer: u64,

    // Constants for signatures
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
        interruptSrc: u8,
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

    pub const DescriptorTable = extern struct {
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

    pub fn parseTables(self: *@This()) void {
        var isXSDT = false;
        const sdt = blk: {
            if (@as(?*RootSystemDescriptorPointer, @ptrFromInt(kernel.addrSpace.mapPhysical(arch.FindRootSystemDescriptorPointer(), 16384, Region.Flags.empty())))) |rsdp| {
                self.rsdp = rsdp;

                isXSDT = self.rsdp.revision == 2 and self.rdsp.RSDTAddr != 0;
                const sdtPA =
                    if (isXSDT) self.rdsp.XSDTAddr else self.rdsp.RSDTAddr;

                break :blk @as(*align(1) DescriptorTable, @ptrFromInt(kernel.addrSpace.mapPhysical(sdtPA, 16384, Region.Flags.empty())));
            } else {
                std.debug.panic("unable to get RSDP", .{});
            }
        };

        const isValid = ((sdt.signature == XSDTSignature and isXSDT) or (sdt.signature == RSDTSignature and !isXSDT)) and sdt.length < 16384 and EsMemorySumBytes(@as([*]u8, @ptrCast(sdt)), sdt.length) == 0;

        if (!isValid) std.debug.panic("system descriptor pointer is invalid", .{});

        const tableCount = (sdt.length - @sizeOf(DescriptorTable)) >> (@as(u2, 2) + @intFromBool(isXSDT));

        if (tableCount == 0) std.debug.panic("no tables found", .{});

        const tableListAddr = @intFromPtr(sdt) + DescriptorTable.headerLen;

        var madtFound = false;
        var i: u64 = 0;

        while (i < tableCount) : (i += 1) {
            const address =
                if (isXSDT) @as([*]align(1) u64, @ptrFromInt(tableListAddr))[i] else @as([*]align(1) u32, @ptrFromInt(tableListAddr))[i];

            const header = @as(*align(1) DescriptorTable, @ptrFromInt(kernel.addrSpace.mapPhysical(address, @sizeOf(DescriptorTable), Region.Flags.empty())));

            switch (header.signature) {
                MADTSignature => {
                    const madtHeader = @as(*align(1) DescriptorTable, @ptrFromInt(kernel.addrSpace.mapPhysical(address, header.length, Region.Flags.empty())));
                    madtHeader.check();

                    if (@as(?*align(1) MultipleAPICDescriptionTable, @ptrFromInt(@as(*align(1) u8, @intFromPtr(madtHeader) + DescriptorTable.headerLen)))) |madt| {
                        madtFound = true;

                        const startLen = madtHeader.length - DescriptorTable.headerLen - @sizeOf(MultipleAPICDescriptionTable);
                        var length = startLen;
                        var madtBytes = @as([*]u8, @ptrFromInt(@intFromPtr(madt) + @sizeOf(MultipleAPICDescriptionTable)));
                        self.LAPICAddr = @as([*]volatile u32, @ptrFromInt(mapPhysicalMem(madt.LAPICAddr, 0x10000)));

                        var entryLen: u8 = undefined;

                        while (length != 0 and length <= startLen) {
                            const entryType = madtBytes[0];
                            entryLen = madtBytes[1];

                            switch (entryType) {
                                0 => {
                                    if (madtBytes[4] & 1 == 0) {
                                        madtBytes = @as([*]u8, @ptrFromInt(@intFromPtr(madtBytes) + entryLen));
                                        length -= entryLen;
                                        continue;
                                    }
                                    const cpu = &self.procs[self.procCount];
                                    cpu.processorID = madtBytes[2];
                                    cpu.APICID = madtBytes[3];
                                    self.procCount += 1;
                                },
                                1 => {
                                    const madtU32 = @as([*]align(1) u32, @ptrCast(madtBytes));
                                    var ioApic = &self.IOAPICS[self.IOAPICCount];
                                    ioApic.id = madtBytes[2];
                                    ioApic.address = @as([*]volatile u32, @ptrFromInt(mapPhysicalMem(madtU32[1], 0x10000)));
                                    _ = ioApic.read(0);
                                    ioApic.GSIBase = madtU32[2];
                                    self.IOAPICCount += 1;
                                },
                                2 => {
                                    const madtU32 = @as([*]align(1) u32, @ptrCast(madtBytes));
                                    const interruptOverride = &self.interruptOverrides[self.interruptOverrideCount];
                                    interruptOverride.interruptSrc = madtBytes[3];
                                    interruptOverride.GSINum = madtU32[1];
                                    interruptOverride.isActiveLow = madtBytes[8] & 2 != 0;
                                    interruptOverride.islevelTriggered = madtBytes[8] & 8 != 0;
                                    self.interruptOverrideCount += 1;
                                },
                                4 => {
                                    const nmi = &self.LAPICNMIs[self.LAPICNMICount];
                                    nmi.proc = madtBytes[2];
                                    nmi.lintIndex = madtBytes[5];
                                    nmi.isActiveLow = madtBytes[3] & 2 != 0;
                                    nmi.isLevelTriggered = madtBytes[3] & 8 != 0;
                                    self.LAPICNMICount += 1;
                                },
                                else => {},
                            }

                            madtBytes = @as([*]u8, @ptrFromInt(@intFromPtr(madtBytes) + entryLen));
                            length -= entryLen;
                        }

                        if (self.procCount > 256 or self.IOAPICCount > 16 or self.interruptOverrideCount > 256 or self.LAPICNMICount > 32) {
                            std.debug.panic("wrong numbers", .{});
                        }
                    } else {
                        std.debug.panic("ACPI initialization - couldn't find the MADT table", .{});
                    }
                },
                FADTSignature => {
                    const fadt = @as(*align(1) DescriptorTable, @ptrFromInt(kernel.addrSpace.mapPhysical(address, header.length, Region.Flags.empty())));
                    fadt.check();

                    if (header.length > 109) {
                        const fadtBytes = @as([*]u8, @ptrCast(fadt));
                        self.centuryRegIDX = fadtBytes[108];
                        const bootArchFlags = fadtBytes[109];
                        self.PS2ControllerUnvailable = (~bootArchFlags & (1 << 1)) != 0;
                        self.VGAControllerUnvailable = (bootArchFlags & (1 << 2)) != 0;
                    }

                    _ = kernel.addrSpace.free(@intFromPtr(fadt), 0, false);
                },
                HPETSignature => {
                    const hpet = @as(*align(1) DescriptorTable, @ptrFromInt(kernel.addrSpace.mapPhysical(address, header.length, Region.Flags.empty())));
                    hpet.check();

                    const headerBytes = @as([*]u8, @ptrCast(header));
                    if (header.length > 52 and headerBytes[52] == 0) {
                        if (@as(?[*]volatile u64, @ptrFromInt(kernel.addrSpace.mapPhysical(@as(*align(1) u64, @ptrCast(&headerBytes[44].*)), 1024, Region.Flags.empty())))) |HPETbA| {
                            self.HPETBaseAddr = HPETbA;
                            self.HPETBaseAddr.?[2] |= 1;

                            self.HPETPeriod = self.HPETBaseAddr.?[0] >> 32;
                        } else {
                            std.debug.panic("failed to map HPET base address", .{});
                        }
                    }

                    _ = kernel.addrSpace.free(@intFromPtr(hpet), 0, false);
                },
                else => {},
            }

            _ = kernel.addrSpace.free(@intFromPtr(header), 0, false);
        }

        if (!madtFound) std.debug.panic("MADT not found", .{});
    }

    fn mapPhysicalMem(pa: u64, length: u64) u64 {
        return kernel.addrSpace.mapPhysical(pa, length, Region.Flags.fromFlag(.notCachable));
    }
};

pub const RootSystemDescriptorPointer = extern struct {
    signature: u64,
    checksum: u8,
    exChecksum: u8,
    OEMID: [6]u8,
    revision: u8,
    RSDTAddr: u32,
    length: u32,
    XSDTAddr: u64,
    reserved: [3]u8,

    pub const signature = 0x2052545020445352;
};
