const std = @import("std");
const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;
const scheduling = kernel.scheduling;
const Thread = scheduling.Thread;
const Timer = scheduling.Timer;
const arch = kernel.arch;
const LinkedList = @import("ds.zig").LinkedList;
const zeroes = kernel.zeroes;

pub const SpinLock = extern struct {
    state: Volatile(u8),
    ownerCpuId: Volatile(u8),
    interruptsEnabled: Volatile(bool),
    const Self = @This();

    pub fn acquire(self: *@This()) void {
        if (kernel.scheduler.panic.readVolatile()) return;
        const interruptsEnabled = arch.areInterruptsEnabled();
        arch.disableInterrupts();

        const maybeLs = arch.getLocalStorage();

        if (maybeLs) |ls| {
            ls.spinlockCount += 1;
        }

        _ = self.state.compareAndSwapAtom(0, 1);
        @fence(.SeqCst);
        self.interruptsEnabled.writeVolatile(interruptsEnabled);

        if (maybeLs) |ls| {
            self.ownerCpuId.writeVolatile(@as(u8, ls.processorID));
        }
    }

    pub fn release(self: *Self) void {
        self.releaseEx(false);
    }

    pub fn releaseEx(self: *@This(), comptime force: bool) void {
        if (kernel.scheduler.panic.readVolatile()) return;

        const maybeLs = arch.getLocalStorage();
        if (maybeLs) |ls| {
            ls.spinLockCount -= 1;
        }

        if (force) {
            self.assertLocked();
        }

        const interruptsEnabledprev = self.interruptsEnabled.readVolatile();
        @fence(.SeqCst);

        self.state.writeVolatile(0);
        if (interruptsEnabledprev) arch.enableInterrupts();
    }

    pub fn assertLocked(self: *Self) void {
        if (kernel.scheduler.panic.readVolatile()) return;

        if (self.state.readVolatile() == 0 or arch.areInterruptsEnabled()) {
            kernel.panic("Spinlock not correctly acquired\n");
        }
    }
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
            kernel.panic("Event already set");
        }

        kernel.scheduler.dispatchSpinLock.acquire();
        var unblockedThreads = Volatile(bool){ .value = false };

        if (self.state.readVolatile() == 0) {
            self.state.writeVolatile(1);

            if (kernel.scheduler.started.readVolatile()) {
                if (self.blockedThreads.count != 0) unblockedThreads.writeVolatile(true);
                kernel.scheduler.notifyObject(&self.blockedThreads, &self.autoReset.readVolatile(), null);
            }
        }

        kernel.scheduler.dispatchSpinLock.release();
        return unblockedThreads.readVolatile();
    }

    pub fn reset(self: *Self) void {
        if (self.blockedThreads.first != null and self.state.readVolatile() != 0) {
            kernel.panic("Event has blocked threads");
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
            var timer = zeroes(Timer);
            timer.set(timeOutInMS);
            ev[1] = &timer.event;
            const index = waitMultiple(&ev, 2);
            timer.remove();
            return index == 0;
        }
    }

    pub fn waitMultiple(_: Self, evPtr: [*]*Event, evLen: u64) u8 {
        const events = evPtr[0..evLen];
        if (events.len == 0) kernel.panic("No events") else if (events.len > kernel.MAX_WAIT_COUNT) kernel.panic("Too many events") else if (!arch.areInterruptsEnabled()) kernel.panic("Interrupts disabled");

        const thread = arch.getCurrentThread().?;
        thread.blocking.event.count = events.len;

        var evItems = zeroes([512]LinkedList(Thread).Node);
        thread.blocking.event.items = &evItems;

        defer thread.blocking.event.items = null;

        for (thread.blocking.event.items.?[0..thread.blocking.event.count], 0..) |*evItem, i| {
            evItem.value = thread;
            thread.blocking.event.array[i] = events[i];
        }
        while (!thread.terminatableState.readVolatile() or thread.terminatableState.readVolatile() != .userBlockRequest) {
            for (events, 0..) |event, i| {
                if (event.autoReset.readVolatile()) {
                    if (event.state.readVolatile() != 0) {
                        thread.state.writeVolatile(.active);
                        const result = event.state.compareAndSwapAtom(0, 1);
                        if (result) |resultUnwrapped| {
                            if (resultUnwrapped != 0) return i;
                        } else {
                            return i;
                        }

                        thread.state.writeVolatile(.waitingEvent);
                    }
                } else {
                    if (event.state.readVolatile() != 0) {
                        thread.state.writeVolatile(.active);
                        return i;
                    }
                }
            }

            arch.fakeTimerInterrupt();
        }
        return std.math.maxInt(u64);
    }
};

pub const WriterLock = extern struct {
    blockedThreads: LinkedList(Thread),
    state: Volatile(i64),
    const Self = @This();
    pub const shared = false;
    pub const exclusive = true;

    pub fn take(self: *Self, write: bool) bool {
        return self.takeEx(write, false);
    }

    pub fn takeEx(self: *Self, write: bool, poll: bool) bool {
        var done = false;
        const maybeCurrThread = arch.getCurrentThread();

        if (maybeCurrThread) |thread| {
            thread.blocking.writer.lock = self;
            thread.blocking.writer.type = write;
            @fence(.SeqCst);
        }

        while (true) {
            kernel.scheduler.dispatchSpinLock.acquire();

            if (write) {
                if (self.state.readVolatile() == 0) {
                    self.state.writeVolatile(-1);
                    done = true;
                }
            } else {
                if (self.state.readVolatile() >= 0) {
                    self.state.increment();
                    done = true;
                }
            }

            kernel.scheduler.dispatchSpinLock.release();

            if (poll or done) break else {
                if (maybeCurrThread) |thread| {
                    thread.state.writeVolatile(.waitingWriterLock);
                    arch.fakeTimeInterrupt();
                    thread.state.writeVolatile(.active);
                } else {
                    kernel.panic("No current thread"); // the scheduler should always have a current thread otherwise it is a bug(schuduler not ready)
                }
            }
        }
        return done;
    }

    pub fn returnLock(self: *Self, write: bool) void {
        kernel.scheduler.dispatchSpinLock.acquire();
        const state = self.state.readVolatile();

        switch (state) {
            -1 => {
                if (!write) kernel.panic("attempt to return shared access to an exclusively owned lock\n");
                self.state.writeVolatile(0);
            },
            0 => {
                kernel.panic("attempt to return access to an unowned lock\n");
            },
            else => {
                if (write) kernel.panic("attempting to return exclusive access to a shared lock\n");
                self.state.decrement();
            },
        }

        if (self.state.readVolatile() == 0) {
            kernel.scheduler.notifyObject(&self.blockedThreads, true, null);
        }

        kernel.scheduler.dispatchSpinLock.release();
    }

    pub fn assertLocked(self: *Self) void {
        if (self.state.readVolatile() == 0) kernel.panic("unlocked\n");
    }

    pub fn assertExclusive(self: *Self) void {
        const lockState = self.state.readVolatile();
        if (lockState == 0) kernel.panic("unlocked\n") else if (lockState > 0) kernel.panic("shared mode\n");
    }

    pub fn assertShared(self: *Self) void {
        const state = self.state.readVolatile();
        if (state == 0) kernel.panic("unlocked\n") else if (state < 0) kernel.panic("exclusive mode\n");
    }

    pub fn convExclusiveToShared(self: *Self) void {
        kernel.scheduler.dispatchSpinLock.acquire();
        self.assertExclusive();
        self.state.writeVolatile(1);
        kernel.scheduler.notifyObject(&self.blockedThreads, true, null);
        kernel.scheduler.dispatchSpinLock.release();
    }
};

pub const Mutex = extern struct {
    ownerThread: ?*align(1) volatile Thread,
    blockedThreads: LinkedList(Thread),

    const Self = @This();
    pub fn acquire(self: *Self) bool {
        if (kernel.scheduler.panic.readVolatile()) return false;

        var currentThread = blk: {
            const threadAddr = addrBlk: {
                if (arch.getCurrentThread()) |th| {
                    if (th.terminatableState.readVolatile() == .terminatable) {
                        kernel.panic("terminatable thread trying to acquire a mutex");
                    }

                    if (self.ownerThread != null and self.ownerThread != th) {
                        kernel.panic("thread trying to acquire a mutex that is already owned");
                    }
                    break :addrBlk @intFromPtr(th);
                } else {
                    break :addrBlk 1;
                }
            };
            break :blk @as(*align(1) volatile Thread, @ptrFromInt(threadAddr));
        };

        if (!arch.areInterruptsEnabled()) {
            kernel.panic("mutex acquire with interrupts disabled");
        }
        while (true) {
            kernel.scheduler.dispatchSpinLock.acquire();
            const oldThread = self.ownerThread;
            if (oldThread == null) {
                self.ownerThread = currentThread;
                kernel.scheduler.dispatchSpinLock.release();
            }
            if (oldThread == null) break;
            @fence(.SeqCst);

            if (arch.getLocalStorage()) |ls| {
                if (ls.isSchedulerReady) {
                    if (currentThread.state.readVolatile() != .active) {
                        kernel.panic("thread trying to acquire a mutex while not active");
                    }

                    currentThread.blocking.mutex = self;
                    @fence(.SeqCst);

                    currentThread.state.writeVolatile(.waitingMutex);

                    kernel.scheduler.dispatchSpinLock.acquire();

                    const spin = if (self.ownerThread) |owner| owner.executing.readVolatile() else false;

                    kernel.scheduler.dispatchSpinLock.release();

                    if (!spin and currentThread.blocking.mutex.?.ownerThread != null) {
                        arch.fakeTimeInterrupt();
                    }

                    while ((!currentThread.terminating.readVolatile() or currentThread.terminatableState.readVolatile() != .userBlockRequest) and self.ownerThread != null) {
                        currentThread.state.writeVolatile(.waitingMutex);
                    }

                    currentThread.state.writeVolatile(.active);
                    // case when the thread is terminating so the mutex is not aquired
                    if (currentThread.terminating.readVolatile() and currentThread.terminatableState.readVolatile() == .userBlockRequest) {
                        return false;
                    }
                }
            }
        }
        @fence(.SeqCst);

        if (self.ownerThread != currentThread) {
            kernel.panic("mutex acquire failed due to invalid owner thread");
        }
        return true;
    }

    pub fn release(self: *Self) void {
        if (kernel.scheduler.panic.readVolatile()) return;

        self.assertLocked();

        const maybeCurrentThread = arch.getCurrentThread();
        kernel.scheduler.dispatchSpinLock.acquire();
        if (maybeCurrentThread) |currentThread| {
            if (@cmpxchgStrong(?*align(1) volatile Thread, &self.ownerThread, currentThread, null, .SeqCst, .SeqCst) != null) {
                kernel.panic("mutex release failed due to invalid owner thread");
            }
        } else {
            self.ownerThread = null;
        }

        const preempt = self.blockedThreads.count != 0;

        if (kernel.scheduler.started.readVolatile()) {
            kernel.scheduler.notifyObject(&self.blockedThreads, preempt, null);
        }

        kernel.scheduler.dispatchSpinLock.release();
        if (preempt) arch.fakeTimeInterrupt();
    }

    pub fn assertLocked(self: *Self) void {
        const currentThread = blk: {
            if (arch.getCurentThread()) |th| {
                break :blk @as(*align(1) Thread, @ptrCast(th));
            } else {
                break :blk @as(*align(1) Thread, @ptrFromInt(1));
            }
        };

        if (self.ownerThread != currentThread) {
            const ownerThreadId = if (self.ownerThread) |owner| owner.id else 0;
            const currentThreadId = currentThread.id;
            kernel.panic("mutex not locked by current thread. Owner thread ID: {}, Current thread ID: {}", .{ ownerThreadId, currentThreadId });
        }
    }
};
