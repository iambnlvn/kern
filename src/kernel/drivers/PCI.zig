const std = @import("std");
const kernel = @import("../kernel.zig");
const utils = @import("../kernelUtils.zig");
const roundUp = utils.roundUp;
const arch = kernel.arch;
const pageSize = arch.pageSize;

pub const Device = extern struct {
    deviceID: u32,
    sybsystemID: u32,
    domain: u32,
    classCode: u8,
    subClassCode: u8,
    progIF: u8,
    bus: u8,
    slot: u8,
    func: u8,
    interruptPin: u8,
    interruptLine: u8,
    baseVA: [6]u64,
    basePA: [6]u64,
    baseSize: [6]u64,
    baseAddrSpace: [6]u32,

    const maxCount = 32;
    pub fn readConfig(comptime T: type, self: *@This(), offset: u8) T {
        const bitWidth = @sizeOf(T) * 8;
        return switch (bitWidth) {
            8 => @as(T, @truncate(arch.readPciConfig(self.bus, self.slot, self.func, offset, 8))),
            16 => @as(T, @truncate(arch.readPciConfig(self.bus, self.slot, self.func, offset, 16))),
            32 => @as(T, @truncate(arch.readPciConfig(self.bus, self.slot, self.func, offset, 32))),
            else => @panic("Unsupported bit width"),
        };
    }
    pub fn writeConfig(comptime T: type, self: *@This(), offset: u8, value: T) void {
        const bitWidth = @sizeOf(T) * 8;
        switch (bitWidth) {
            8 => arch.writePciConfig(self.bus, self.slot, self.func, offset, value, 8),
            16 => arch.writePciConfig(self.bus, self.slot, self.func, offset, value, 16),
            32 => arch.writePciConfig(self.bus, self.slot, self.func, offset, value, 32),
            else => @panic("Unsupported bit width"),
        }
    }
};
