const std = @import("std");
pub const SpinLock = @import("sync.zig").SpinLock;
pub const scheduling = @import("scheduling.zig");
const Scheduler = scheduling.Scheduler;
pub const ds = @import("ds.zig");
pub const WAIT_NO_TIMEOUT = std.math.maxInt(u64);
pub const MAX_WAIT_COUNT = 8;
pub export var scheduler: Scheduler = undefined;

pub fn Volatile(comptime T: type) type {
    return extern struct {
        value: T,

        pub inline fn readVolatile(self: *const volatile @This()) T {
            return self.value;
        }

        pub inline fn writeVolatile(self: *volatile @This(), value: T) void {
            self.value = value;
        }

        pub inline fn accessVolatile(self: *volatile @This()) *volatile T {
            return &self.value;
        }
        pub inline fn increment(self: *volatile @This()) void {
            self.writeVolatile(self.readVolatile() + 1);
        }

        pub inline fn decrement(self: *volatile @This()) void {
            self.writeVolatile(self.readVolatile() - 1);
        }

        pub inline fn compareAndSwapAtom(self: *@This(), expectedValue: T, newVal: T) ?T {
            return @cmpxchgStrong(@TypeOf(self.value), &self.value, expectedValue, newVal, .SeqCst, .SeqCst);
        }
    };
}

pub const arch = blk: {
    const currentArch = @import("builtin").target.cpu.arch;
    switch (currentArch) {
        .x86_64 => break :blk @import("arch/x86_64.zig"), //Todo: implement arch
        else => @compileError(std.fmt.comptimePrint("Unsupported arch {s}\n", .{currentArch.genericName()})),
    }
};

pub fn zeroes(comptime T: type) T {
    var zeroVal: T = undefined;
    @memset(std.mem.asBytes(&zeroVal), 0);
    return zeroVal;
}
