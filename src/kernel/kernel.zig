const std = @import("std");
pub const SpinLock = @import("sync.zig").SpinLock;
pub const scheduler = @import("scheduling.zig").Scheduler;

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
    };
}

pub const arch = blk: {
    const currentArch = @import("builtin").target.cpu.arch;
    switch (currentArch) {
        .x86_64 => break :blk @import("arch/x8664.zig"), //Todo: implement arch
        else => @compileError(std.fmt.comptimePrint("Unsupported arch {s}\n", .{currentArch.genericName()})),
    }
};
