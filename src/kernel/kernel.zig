const std = @import("std");
pub const SpinLock = sync.SpinLock;
pub const scheduling = @import("scheduling.zig");
const Scheduler = scheduling.Scheduler;
pub const ds = @import("ds.zig");
pub const WAIT_NO_TIMEOUT = std.math.maxInt(u64);
pub const MAX_WAIT_COUNT = 8;
pub const sync = @import("sync.zig");
pub const memory = @import("memory.zig");

const Heap = memory.Heap;
pub export var scheduler: Scheduler = undefined;
pub export var coreAddressSpace: memory.AddressSpace = undefined;
pub export var mmCoreRegions: [*]memory.Region = undefined;
pub export var mmCoreRegionCount: u64 = 0;
pub export var addrSpace: memory.AddressSpace = undefined;
pub export var heapCore: Heap = undefined;
pub export var physicalMemoryManager: memory.Physical.Allocator = undefined;

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

        pub inline fn atomicFetchAdd(self: *@This(), value: T) T {
            return @atomicRmw(T, &self.value, .Add, value, .SeqCst);
        }
        pub inline fn atomicFetchSub(self: *@This(), value: T) T {
            return @atomicRmw(T, &self.value, .Sub, value, .SeqCst);
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

//Todo!: implement a way to write to the screen
// This function is intended to be called when the kernel encounters an unrecoverable error.
// pub fn kPanic(message: []const u8) noreturn {
//     arch.disableInterrupts();
//     arch.halt();
// }

pub const CrashReason = extern struct {
    errorCode: FatalError,
    duringSysCall: i32,
};

pub const FatalError = enum(u32) {
    abort,
    incorrectFileAccess,
    incorrectNodeType,
    insufficientPermissions,
    invalidBuffer,
    invalidHandle,
    invalidMemoryRegion,
    outOfRange,
    processorException,
    recursiveBatch,
    unknownSyscall,
};

pub export fn EsMemoryZero(dst: u64, byteCount: u64) callconv(.C) void {
    if (byteCount == 0) return;
    const slice = @as([*]u8, @ptrFromInt(dst))[0..byteCount];
    @memset(slice, 0);
}
