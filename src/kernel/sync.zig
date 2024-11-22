const std = @import("std");
const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;
const arch = kernel.arch;

pub const SpinLock = extern struct {
    state: Volatile(u8),
    ownerCpuId: Volatile(u8),
    interruptsEnabled: Volatile(bool),
    const Self = @This();
    //Todo!: implement
    pub fn aquire() void {
        if (kernel.scheduler.panic.readVolatile()) return;
    }

    pub fn release(self: *Self) void {}
    pub fn assertLocked(self: *Self) void {}
};
