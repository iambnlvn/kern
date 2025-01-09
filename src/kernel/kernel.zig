const std = @import("std");
pub const SpinLock = sync.SpinLock;
pub const scheduling = @import("scheduling.zig");
const Scheduler = scheduling.Scheduler;
pub const ds = @import("ds.zig");
pub const WAIT_NO_TIMEOUT = std.math.maxInt(u64);
pub const MAX_WAIT_COUNT = 8;
pub const sync = @import("sync.zig");
pub const memory = @import("memory.zig");
pub const object = @import("obj.zig");
pub const drivers = @import("drivers.zig");
const RandomNumberGenerator = @import("kernelUtils.zig").RandomNumberGenerator;
const Heap = memory.Heap;
pub export var scheduler: Scheduler = undefined;
pub export var coreAddressSpace: memory.AddressSpace = undefined;
pub export var mmCoreRegions: [*]memory.Region = undefined;
pub export var mmCoreRegionCount: u64 = 0;
pub export var addrSpace: memory.AddressSpace = undefined;
pub export var heapCore: Heap = undefined;
pub export var heapFixed: Heap = undefined;
pub export var physicalMemoryManager: memory.Physical.Allocator = undefined;
pub export var globalData: *GlobalData = undefined;
pub export var ipiLock: sync.SpinLock = undefined;
pub export var process: scheduling.Process = undefined;
pub export var mmCoreRegionArrayCommit: u64 = 0;
pub var rng: RandomNumberGenerator = undefined;

pub const ES_INVALID_HANDLE: u64 = 0x0;
pub const ES_CURRENT_THREAD: u64 = 0x10;
pub const ES_CURRENT_PROCESS: u64 = 0x11;
pub const ES_SUCCESS: u64 = -1;
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
        .x86_64 => break :blk @import("arch/x86_64.zig"),
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

pub export fn EsMemoryCopyReverse(destination: u64, source: u64, byteCount: u64) callconv(.C) void {
    if (byteCount == 0) return;

    var dst = &(@as([*]u8, @ptrFromInt(destination))[0..byteCount][byteCount - 1]);
    var src = &(@as([*]u8, @ptrFromInt(source))[0..byteCount][byteCount - 1]);

    var bytes: u64 = byteCount;
    while (bytes >= 1) {
        dst.* = src.*;
        src = @as(*u8, @ptrFromInt(@intFromPtr(src) - 1));
        dst = @as(*u8, @ptrFromInt(@intFromPtr(destination) - 1));
        bytes -= 1;
    }
}

pub export fn EsMemoryCopy(dst: u64, src: u64, byteCount: u64) callconv(.C) void {
    if (byteCount == 0) return;

    const destSlice = @as([*]u8, @ptrFromInt(dst))[0..byteCount];
    const srcSlice = @as([*]const u8, @ptrFromInt(src))[0..byteCount];
    @memcpy(destSlice, srcSlice);
}

pub export fn EsMemoryMove(startAddr: u64, endAddr: u64, amount: i64, isZeroEmptySpace: bool) callconv(.C) void {
    if (endAddr < startAddr) return;

    if (amount > 0) {
        const amountU = @as(u64, @intCast(amount));
        EsMemoryCopyReverse(startAddr + amountU, startAddr, endAddr - startAddr);

        if (isZeroEmptySpace) EsMemoryZero(startAddr, amountU);
    } else if (amount < 0) {
        const amountU = @as(u64, @intCast(@abs(amount) catch unreachable));
        EsMemoryCopy(startAddr - amountU, startAddr, endAddr - startAddr);
        if (isZeroEmptySpace) EsMemoryZero(endAddr - amountU, amountU);
    }
}

pub fn EsHeapReallocate(ptr: usize, newSize: usize, zero: bool, heap: *Heap) usize {
    if (ptr == 0) {
        return heap.alloc(newSize);
    }

    const oldSize = heap.getAllocationSize(ptr);
    if (newSize <= oldSize) {
        return ptr;
    }

    const newPtr = heap.alloc(newSize);
    if (newPtr == 0) {
        return 0;
    }

    @memcpy(@as([*]u8, @ptrCast(newPtr)), @as([*]const u8, @ptrCast(ptr))[0..oldSize]);

    if (zero) {
        @memset(@as([*]u8, @ptrCast(newPtr))[oldSize..newSize], 0);
    }

    heap.free(ptr);

    return newPtr;
}

export fn EsHeapFree(addr: u64, expectedSize: u64, heap: *Heap) callconv(.C) void {
    heap.free(addr, expectedSize);
}

pub export fn EsMemorySumBytes(source: [*]u8, byteCount: u64) callconv(.C) u8 {
    if (byteCount == 0) return 0;

    const slice = source[0..byteCount];
    var total: u64 = 0;
    for (slice) |byte| {
        total += byte;
    }

    return @as(u8, @truncate(total));
}

pub const GlobalData = extern struct {
    clickChainTimeoutMS: Volatile(i32),
    doubleClickTimeoutMS: Volatile(i32),
    uiScale: Volatile(f32),
    uiScaleFactor: Volatile(f32),
    uiScaleFactorInverse: Volatile(f32),
    swapLeftAndRightButtons: Volatile(bool),
    showCursorShadow: Volatile(bool),
    useSmartQuotes: Volatile(bool),
    enableHoverState: Volatile(bool),
    animationTimeMultiplier: Volatile(f32),
    schedulerTimeMS: Volatile(u64),
    schedulerTimeOffset: Volatile(u64),
    keyboardLayout: Volatile(u16),
    keyboardLayoutVersion: Volatile(u16),
    keyboardLayoutVariant: Volatile(u16),
    keyboardLayoutVariantVersion: Volatile(u16),
    keyboardLayoutCountry: Volatile(u16),
};

pub const Pool = extern struct {
    elementSize: u64,
    cacheEntries: [cacheCount]u64,
    cacheEntryCount: u64,
    mutex: sync.Mutex,

    const cacheCount = 16;

    pub fn add(self: *@This(), elementSize: u64) u64 {
        _ = self.mutex.acquire();
        defer self.mutex.release();

        if (self.elementSize != 0 and elementSize != self.elementSize) std.debug.panic("pool size mismatch", .{});
        self.elementSize = elementSize;

        if (self.cacheEntryCount != 0) {
            self.cacheEntryCount -= 1;
            const address = self.cacheEntries[self.cacheEntryCount];

            @memset(@as([*]u8, @ptrFromInt(address))[0..self.elementSize], 0);
            return address;
        } else {
            return heapFixed.alloc(self.elementSize, true);
        }
    }

    pub fn remove(self: *@This(), ptr: u64) void {
        _ = self.mutex.acquire();
        defer self.mutex.release();

        if (ptr == 0) return;

        if (self.cacheEntryCount == cacheCount) {
            heapFixed.free(ptr, self.elementSize);
        } else {
            self.cacheEntries[self.cacheEntryCount] = ptr;
            self.cacheEntryCount += 1;
        }
    }
};

var formatBuff: [0x4000]u8 align(0x1000) = undefined;
var formatLock: sync.SpinLock = undefined;

pub fn log(comptime format: []const u8, args: anytype) void {
    logEx(&formatBuff, &formatLock, format, args);
}

pub fn logEx(buffer: []u8, buffLock: *sync.SpinLock, comptime fmt: []const u8, args: anytype) void {
    buffLock.acquire();
    const logMsg = std.fmt.bufPrint(buffer[0..], fmt, args) catch |err|
        {
        var fmtBuff: [512]u8 = undefined;
        const errorStr = std.fmt.bufPrint(fmtBuff[0..], "Unable to print: {}", .{err}) catch unreachable;
        panic(errorStr);
    };
    drivers.serial.write(logMsg);
    buffLock.release();
}

pub fn panic(message: []const u8) noreturn {
    arch.disableInterrupts();
    _ = arch.IPI.send(arch.IPI.kernelPanic, true, -1);
    scheduler.panic.writeVolatile(true);
    drivers.serial.write("KERNEL PANIC:\n");
    drivers.serial.write(message);
    arch.halt();
}

var panicLock: sync.SpinLock = undefined;
var panicBuff: [0x4000]u8 align(0x1000) = undefined;

pub fn panicf(comptime fmt: []const u8, args: anytype) noreturn {
    arch.disableInterrupts();
    _ = arch.IPI.send(arch.IPI.kernelPanic, true, -1);
    scheduler.panic.writeVolatile(true);
    logEx(&panicBuff, &panicLock, "KERNEL PANIC:\n" ++ fmt, args);
    arch.halt();
}

pub const MessageQueue = extern struct {
    messages: ds.Array(MessageObject, .core),
    mouseMovedMessage: u64,
    windowResizedMessage: u64,
    eyedropResultMessage: u64,
    keyRepeatMessage: u64,
    pinged: bool,
    mutex: sync.Mutex,
    notEmpty: sync.Event,
};

pub const MessageObject = extern struct {
    array: [5]u64,
};
