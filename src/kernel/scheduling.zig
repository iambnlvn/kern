const std = @import("std");
const ds = @import("ds.zig");
const LinkedList = ds.LinkedList;
const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;
const SpinLock = kernel.SpinLock;
const Sync = @import("sync.zig");
const Event = Sync.Event;
const Mutex = Sync.Mutex;

pub const Thread = extern struct {
    inSafeCopy: bool,
    item: LinkedList(Thread).Node,
    procItem: LinkedList(Process).Node,
    process: *Process,
    id: u64,
    execProcId: u32,
    cpuTimeSlices: Volatile(u64),
    handleCount: Volatile(u64),

    userStackBase: u64,
    userStackReserve: u64,
    userStackCommit: Volatile(u64),
    kernelStackBase: u64,
    kernelStack: u64,

    tlsAddr: u64,
    timerAdjustAddr: u64,
    timerAdjustTicks: u64,
    lastInterruptTicks: u64,

    isKernelThread: bool,
    isPageGenThread: bool,
    type: Thread.Type,
    priority: i8,
    blockedThreadPriorities: [Thread.priorityCount]i32,
    policy: Thread.Policy,
    affinity: u32,
    state: Volatile(Thread.state),
    terminatableState: Volatile(Thread.TerminatableState),
    terminating: Volatile(bool),
    executing: Volatile(bool),
    paused: Volatile(bool),
    yieldIpiReceived: Volatile(bool),
    blocking: extern union {
        mutex: ?*volatile Mutex,
        writer: extern struct {
            // lock: ?*volatile WriterLock,
            type: bool,
        },
        event: extern struct {
            items: ?[*]volatile LinkedList(Thread).Node,
            array: [kernel.MAX_WAIT_COUNT]?*volatile Event,
            count: u64,
        },
    },
    killedEvent: Event,

    pub const Priority = enum(i8) {
        normal = 0,
        low = 1,
    };
    const priorityCount = std.enums.values(Priority).len;

    pub const Type = enum(i8) {
        normal = 0,
        idle = 1,
        asyncTask,
    };

    pub const flags = ds.Bitflag(enum(u32) {
        userLand = 0,
        lowPriority = 1,
        paused,
        asyncTask,
        idle,
    });

    pub const state = enum(i8) {
        active = 0,
        waitingMutex = 1,
        waitingEvent,
        waitingWriterLock,
        terminated,
    };
    pub const TerminatableState = enum(i8) {
        invalidTerminatableState = 0,
        terminatable = 1,
        inSysCall,
        userBlockRequest,
    };

    pub const Policy = enum {
        FIFO,
        RoundRobin,
        Other,
        Batch,
        Idle,
        Deadline,
    };
};

const Process = extern struct {
    // addrSpace: *AddrSpace, //Todo!: implement this
    threads: LinkedList(Thread),
    // execNode: ?*Node, //Todo!: implement this
    execName: ?[32]u8,
    data: ProcCreateData,
    permissions: Process.Permission,
    creationFlags: Process.CreationFlags,
    type: Process.Type,
    id: u64,
    handleCount: Volatile(u64),
    allItems: LinkedList(Process).Node,
    crashed: bool,
    allThreadsPaused: bool,
    allThreadsTerminated: bool,
    blockShutdown: bool,
    preventNewThreads: bool,
    exitStatusCode: i32,
    killedEvent: Event,
    execState: Process.ExecState,
    execStartRequest: bool,
    execLoadAttempComplete: Event,
    execMainThread: ?*Thread,

    cpuTimeSlices: u64,
    idleTimeSlices: u64,

    pub const Type = enum(u32) {
        normal,
        kernel,
        desktop,
    };

    pub const Permission = ds.Bitflag(enum(u32) {
        network = 0,
        processCreation = 1,
        processOpen = 2,
        screenModify = 3,
        shutDown = 4,
        takeSysSnapshot = 5,
        getVolumeInfo = 6,
        winMgr = 7,
        posixSubSystem = 8,
    });

    pub const CreationFlags = ds.Bitflag(enum(u32) {
        paused = 0,
    });
    const ProcCreateData = extern struct {
        sysData: u64,
        sybSysData: u64,
        userDara: u64,
        subSysID: u8,
    };

    const ExecState = enum(u8) {
        notLoaded = 0,
        failed = 1,
        loaded,
        running,
        paused,
        terminated,
    };
};

pub const Scheduler = extern struct {
    dispachSpinLock: SpinLock,
    activeTimersSpinLock: SpinLock,
    activeThreads: [Thread.priorityCount]LinkedList(Thread),
    pausedThreads: LinkedList(Thread),
    terminatedThreads: LinkedList(Thread),
    asyncTaskSpinLock: SpinLock,
    allThreads: LinkedList(Thread),
    allProcesses: LinkedList(Process),
    nextThreadID: u64,
    nextProcessID: u64,
    nextProcID: u64,
    activeProcessCount: Volatile(u64),
    blockShutdownProcCount: Volatile(u64),
    started: Volatile(bool),
    panic: Volatile(bool),
    shutdown: Volatile(bool),
    timeMs: Volatile(u64),

    pub fn notifyObject(
        self: *@This(),
        blockedThreads: *LinkedList(Thread),
        unblockAll: bool,
        prevMutexOwner: ?*Thread,
    ) void {
        self.dispachSpinLock.assertLocked();
        var unblockedThread = blockedThreads.first;
        if (unblockedThread == null) return;

        while (true) {
            if (@intFromPtr(unblockedThread)) {
                std.debug.panic("unblockedThread is null");
            }

            const nextUnblockedThread = unblockedThread.?.next;
            const nextUnblockedThreadValue = nextUnblockedThread.?.value.?;

            self.unblockThread(nextUnblockedThreadValue, prevMutexOwner);
            unblockedThread = nextUnblockedThread;
            if (!(unblockedThread != null and unblockAll)) break;
        }
    }

    pub fn unblockThread(self: *@This(), unblockedThread: *Thread, prevMutexOwner: ?*Thread) void {
        _ = prevMutexOwner;

        self.dispachSpinLock.assertLocked();
        switch (unblockedThread.state.readVolatile()) {
            .waitingMutex => {
                if (unblockedThread.blocking.mutex == null) {
                    std.debug.panic("unblockedThread.blocking.mutex is null");
                }
                unblockedThread.blocking.mutex.?.unlock();
            },
            .waitingEvent => {
                if (unblockedThread.blocking.event.items == null) {
                    std.debug.panic("unblockedThread.blocking.event.items is null");
                }
                if (unblockedThread.blocking.event.count == 0) {
                    std.debug.panic("unblockedThread.blocking.event.count is 0");
                }
                unblockedThread.blocking.event.count -= 1;
                if (unblockedThread.blocking.event.count == 0) {
                    unblockedThread.state.writeVolatile(Thread.state.active);
                }
            },
            .waitingWriterLock => {
                if (unblockedThread.blocking.writer.type) {
                    std.debug.panic("unblockedThread.blocking.writer.type is true");
                }
                unblockedThread.state.writeVolatile(Thread.state.active);
            },
            else => std.debug.panic("Unexpected or invalid thread state", .{}),
        }
        unblockedThread.state.writeVolatile(.active);
        if (!unblockedThread.executing.readVolatile()) {
            self.addThreadToActiveThreads(unblockedThread, true);
        }
    }

    pub fn addThreadToActiveThreads(self: *@This(), thread: Thread, isFirst: bool) void {
        if (thread.type == .asyncTask) return;

        self.ensureThreadIsActive(thread);
        self.ensureThreadIsNotExecuting(thread);
        self.ensureThreadIsNormal(thread);
        self.ensureThreadPriorityIsValid(thread);
        self.ensureThreadIsNotInList(thread);

        if (thread.paused.readVolatile() and thread.terminatableState.readVolatile() == .terminatable) {
            self.pausedThreads.prepend(&thread.item);
        } else {
            const priority = @as(u64, @intCast(self.getThreadEffectivePriority(thread)));
            if (isFirst) {
                self.activeThreads[priority].prepend(&thread.item);
            } else {
                self.activeThreads[priority].append(&thread.item);
            }
        }
    }

    fn ensureThreadIsActive(_: *@This(), thread: Thread) void {
        if (thread.state.readVolatile() != .active) {
            std.debug.panic("Thread is not active", .{});
        }
    }

    fn ensureThreadIsNotExecuting(_: *@This(), thread: Thread) void {
        if (thread.executing.readVolatile()) {
            std.debug.panic("Thread is executing", .{});
        }
    }

    fn ensureThreadIsNormal(_: *@This(), thread: Thread) void {
        if (thread.type != .normal) {
            std.debug.panic("Thread is not normal", .{});
        }
    }

    fn ensureThreadPriorityIsValid(_: *@This(), thread: Thread) void {
        if (thread.priority < 0) {
            std.debug.panic("Thread priority is less than 0", .{});
        } else if (thread.priority >= Thread.priorityCount) {
            std.debug.panic("Thread priority is greater than or equal to priorityCount", .{});
        }
    }

    fn ensureThreadIsNotInList(_: *@This(), thread: Thread) void {
        if (thread.item.list != null) {
            std.debug.panic("Thread is already in a list", .{});
        }
    }

    pub fn getThreadEffectivePriority(self: *@This(), thread: *Thread) i8 {
        self.dispachSpinLock.assertLocked();
        for (thread.blockedThreadPriorities[0..@as(u64, @intCast(thread.priority))], 0..) |priority, idx| {
            if (priority != 0) return @as(i8, @intCast(idx));
        }
        return thread.priority;
    }
};
