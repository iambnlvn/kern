const std = @import("std");
const kernel = @import("kernel.zig");

const ES_CURRENT_THREAD = kernel.ES_CURRENT_THREAD;
const ES_CURRENT_PROCESS = kernel.ES_CURRENT_PROCESS;
const ES_INVALID_HANDLE = kernel.ES_INVALID_HANDLE;

const arch = kernel.arch;

const SharedRegion = kernel.memory.SharedRegion;

const Process = kernel.scheduling.Process;
const Thread = kernel.scheduling.Thread;

const Mutex = kernel.sync.Mutex;

pub const Type = enum(u32) {
    couldNotResolve = 0,
    none = 0x80000000,
    proc = 0x1,
    thread = 0x2,
    window = 0x4,
    sharedMem = 0x8,
    node = 0x10,
    event = 0x20,
    constBuff = 0x40,
    posixFD = 0x100,
    pipe = 0x200,
    embeddedWindow = 0x400,
    connection = 0x4000,
    device = 0x8000,
};

const ResolveHandleStatus = enum(u8) {
    failed = 0,
    noClose = 1,
    normal = 2,
};

pub const Handle = extern struct {
    object: ?*anyopaque,
    flags: u32,
    type: Type,

    const TableL2 = extern struct {
        t: [256]Handle,

        const entryCount = 256;
    };

    const TableL1 = extern struct {
        t: [256]?*TableL2,
        u: [256]u16,

        const entryCount = 256;
    };

    pub const Table = extern struct {
        l1r: TableL1,
        lock: Mutex,
        process: ?*Process,
        destroyed: bool,
        handleCount: u32,

        pub fn resolveHandle(_: *@This(), outHandle: *Handle, inHandle: u64, maskType: u32) ResolveHandleStatus {
            if (inHandle == ES_CURRENT_THREAD and maskType & @intFromEnum(Type.thread) != 0) {
                outHandle.type = .thread;
                outHandle.object = arch.getCurrentThread();
                outHandle.flags = 0;

                return .noClose;
            } else if (inHandle == ES_CURRENT_PROCESS and maskType & @intFromEnum(Type.proc) != 0) {
                outHandle.type = .process;
                outHandle.object = arch.getCurrentThread().?.process;
                outHandle.flags = 0;

                return .noClose;
            } else if (inHandle == ES_INVALID_HANDLE and maskType & @intFromEnum(Type.none) != 0) {
                outHandle.type = .none;
                outHandle.object = null;
                outHandle.flags = 0;

                return .noClose;
            }
        }

        pub fn destroy(self: *@This()) void {
            _ = self.lock.acquire();
            defer self.lock.release();

            if (self.destroyed) return;

            self.destroyed = true;

            const l1 = &self.l1r;

            var i: u64 = 0;
            while (i < TableL1.entryCount) : (i += 1) {
                if (l1.u[i] == 0) continue;

                var k: u64 = 0;
                while (k < TableL1.entryCount) : (k += 1) {
                    const handle = &l1.t[i].?.t[k];
                    if (handle.object != null) closeHandle(handle.object, handle.flags);
                }

                kernel.heapFixed.free(@intFromPtr(l1.t[i]), 0);
            }
        }
    };
};

pub fn openHandle(object: anytype, flags: u32) bool {
    _ = flags;
    var noHandles = false;
    var failed = false;

    const objType = @TypeOf(object);
    switch (objType) {
        *Process => {
            const proc = @as(*Process, @ptrCast(object));
            noHandles = proc.handleCount.atomicFetchAdd(1) == 0;
        },
        *Thread => {
            const th = @as(*Thread, @ptrCast(object));
            noHandles = th.handleCount.atomicFetchAdd(1) == 0;
        },
        *SharedRegion => {
            const region = @as(*SharedRegion, @ptrCast(object));
            _ = region.mutex.acquire();
            noHandles = (region.handleCount.readVolatile() == 0);
            if (!noHandles) region.handleCount.increment();
            region.mutex.release();
        },
        else => {
            failed = true;
        },
    }

    if (noHandles) std.debug.panic("object had no handles", .{});

    return !failed;
}

pub fn closeHandle(object: anytype, flags: u32) void {
    _ = flags;
    const objType = @TypeOf(object);
    switch (objType) {
        *Process => {
            const _process = @as(*Process, @ptrCast(object));
            const previous = _process.handleCount.atomicFetchSub(1);
            if (previous == 0) std.debug.panic("All handles to process have been closed", .{}) else if (previous == 1) std.debug.panic("All handles to process have been closed", .{});
        },
        *Thread => {
            const thread = @as(*Thread, @ptrCast(object));
            const previous = thread.handleCount.atomicFetchSub(1);

            if (previous == 0) std.debug.panic("All handles to thread have been closed", .{}) else if (previous == 1) thread.remove();
        },
        else => std.debug.panic("Unknown object type", .{}),
    }
}
