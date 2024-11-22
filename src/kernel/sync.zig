const std = @import("std");
const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;
const Thread = kernel.scheduling.Thread;
const arch = kernel.arch;
const LinkedList = @import("ds.zig").LinkedList;

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

pub const Event = extern struct {
    autoReset: Volatile(bool),
    state: Volatile(u64),
    blockedThreads: LinkedList(Thread),
    handleCount: Volatile(u64),
    const Self = @This();

    pub fn set(self: *Self, defaultFlag: ?bool) bool {
        const flag = defaultFlag orelse false;
        if (self.state.readVolatile() != 0 and !flag) {
            std.debug.panic("Event already set");
        }

        kernel.scheduler.dispachSpinLock.aquire();
        var unblockedThreads = Volatile(bool){ .value = false };

        if (self.state.readVolatile() == 0) {
            self.state.writeVolatile(1);

            if (kernel.scheduler.started.readVolatile()) {
                if (self.blockedThreads.count != 0) unblockedThreads.writeVolatile(true);
                // Todo!: notify blocked threads
            }
        }

        kernel.scheduler.dispachSpinLock.release();
        return unblockedThreads.readVolatile();
    }

    pub fn reset(self: *Self) void {
        if (self.blockedThreads.first != null and self.state.readVolatile() != 0) {
            std.debug.panic("Event has blocked threads");
        }
        self.state.writeVolatile(0);
    }

    pub fn poll(self: *Self) bool {
        if (self.autoReset.readVolatile()) {
            if (self.state.compareAndSwapAtom(@intFromBool(true), @intFromBool(false))) |val| {
                return val != 0;
            } else {
                return true;
            }
        } else {
            return self.state.readVolatile() != 0;
        }
    }

    pub fn wait(self: *Self) bool {
        return self.waitTimeout(kernel.WAIT_NO_TIMEOUT);
    }

    pub fn waitTimeout(self: *Self, timeOutInMS: u64) bool {
        var ev: [2]Event = undefined;
        ev[0] = self;

        if (timeOutInMS == kernel.WAIT_NO_TIMEOUT) {
            return self.waitMultiple(&ev, 1) == 0;
        } else {
            //Todo!: implement with a timer
        }
    }

    // pub fn waitMultiple(_: Self, evPtr: [*]*Event, evLen: u64) u8 {
    //     const events = evPtr[0..evLen];
    //     if (events.len == 0) std.debug.panic("No events");
    //     if (events.len > kernel.MAX_WAIT_COUNT) {
    //         std.debug.panic("Too many events"); //TODO: this should be a kernel panic
    //     }
    //     //should also check if timer interrupt is enabled

    //     //Note: the following code is not implemented yet, it is just a placeholder
    //     const thread = arch.getCurentThread().?;
    //     thread.blocking.event.count = events.len;

    //     var evItems = std.mem.zeroes([512]LinkedList(Thread).Node);
    //     thread.blocking.event.items = &evItems;

    //     defer thread.blocking.event.items = null;

    //     for (thread.blocking.event.items.?[0..thread.blocking.event.count], 0..) |*evItem, i| {
    //         evItem.value = thread;
    //         thread.blocking.event.array[i] = events[i];
    //     }
    //     while (!thread.terminating.readVolatile() or thread.terminatableState.readVolatile() != .userBlockRequest) {
    //         for (events, 0..) |event, i| {
    //             if (event.autoReset.readVolatile()) {
    //                 if (event.state.readVolatile() != 0) {
    //                     thread.state.writeVolatile(.active);
    //                     const result = event.state.compareAndSwapAtom(0, 1);
    //                     if (result) |resultUnwrapped| {
    //                         if (resultUnwrapped != 0) return i;
    //                     } else {
    //                         return i;
    //                     }

    //                     thread.state.writeVolatile(.waitingEvent);
    //                 }
    //             } else {
    //                 if (event.state.readVolatile() != 0) {
    //                     thread.state.writeVolatile(.active);
    //                     return i;
    //                 }
    //             }
    //         }

    //         arch.fakeTimeInterrupt();
    //     }
    //     return std.math.maxInt(u64);
    // }
};
