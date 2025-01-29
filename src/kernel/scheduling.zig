const std = @import("std");
const ds = @import("ds.zig");
const LinkedList = ds.LinkedList;
const Bitflag = ds.Bitflag;
const List = ds.List;
const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;
const SpinLock = kernel.SpinLock;
const Sync = @import("sync.zig");
const Event = Sync.Event;
const Mutex = Sync.Mutex;
const WriterLock = Sync.WriterLock;
const memory = kernel.memory;
const arch = @import("./arch/x86_64.zig");
const pageSize = arch.pageSize;
pub const Thread = extern struct {
    inSafeCopy: bool,
    item: LinkedList(Thread).Node,
    allItems: LinkedList(Thread).Node,
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
    // policy: Policy,
    // affinity: u32,
    state: Volatile(Thread.State),
    terminatableState: Volatile(Thread.TerminatableState),
    terminating: Volatile(bool),
    executing: Volatile(bool),
    paused: Volatile(bool),
    yieldIpiReceived: Volatile(bool),

    tempAddrSpace: ?*volatile memory.AddressSpace,
    interruptCtx: ?*arch.InterruptContext,
    lastKnowExecAddr: u64,
    killAsyncTask: AsyncTask,

    blocking: extern union {
        mutex: ?*volatile Mutex,
        writer: extern struct {
            lock: ?*volatile WriterLock,
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

    pub const Flags = ds.Bitflag(enum(u32) {
        userLand = 0,
        lowPriority = 1,
        paused,
        asyncTask,
        idle,
    });

    pub const State = enum(i8) {
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
    pub fn remove(self: *@This()) void {
        kernel.scheduler.threadPool.remove(@intFromPtr(self));
    }

    pub export fn setTemporaryAddressSpace(space: ?*memory.AddressSpace) callconv(.C) void {
        kernel.scheduler.dispatchSpinLock.acquire();
        const thread = arch.getCurrentThread().?;
        const oldSpace = if (thread.tempAddrSpace) |tas| tas else &kernel.addrSpace;
        thread.tempAddrSpace = space;
        const newSpace = if (space) |sp| sp else &kernel.addrSpace;
        newSpace.openRef();
        newSpace.openRef();
        arch.setAddressSpace(&newSpace.arch);
        kernel.scheduler.dispatchSpinLock.release();

        (@as(*memory.AddressSpace, @ptrCast(oldSpace))).closeRef();
        (@as(*memory.AddressSpace, @ptrCast(oldSpace))).closeRef();
    }

    fn kill(task: *AsyncTask) void {
        const thread = @as(Thread, @fieldParentPtr("killAsyncTask", task));
        Thread.setTemporaryAddressSpace(thread.process.addrSpace);

        _ = kernel.scheduler.allThreadsMutex.acquire();
        kernel.scheduler.allThreadsMutex.release();

        _ = thread.process.threadsMutex.acquire();
        thread.process.?.threads.remove(&thread.procItem);
        const lastThread = thread.process.threads.count == 0;
        thread.process.threadsMutex.release();

        if (lastThread) ProcessKill(thread.process.?);

        _ = kernel.addrSpace.free(thread.kernelStackBase, 0, false);
        if (thread.userStackBase != 0) _ = thread.process.addrSpace.free(thread.userStackBase, 0, false);
        _ = thread.killedEvent.set(false);
        kernel.object.closeHandle(thread.process, 0);
        kernel.object.closeHandle(thread, 0);
    }
    pub fn spawn(startAddr: u64, arg1: u64, flags: Thread.Flags, maybeProc: ?*Process, arg2: u64) callconv(.C) ?*Thread {
        if (startAddr == 0 and !flags.contains(.idle)) kernel.panic("Start address is 0");
        const isUserLand = flags.contains(.isUserLand);
        const parentThread = arch.getCurrentThread();
        const process = if (maybeProc) |proc| proc else &kernel.process;
        if (isUserLand and process == &kernel.process) kernel.panic("cannot add isUserLand thread to kernel process");

        _ = process.threadsMutex.acquire();
        defer process.threadsMutex.release();

        if (process.preventNewThreads) return null;

        const thread = @as(?*Thread, @ptrFromInt(kernel.scheduler.threadPool.add(@sizeOf(Thread)))) orelse return null;
        const kernStackSize = 0x5000;
        const userStackRes: u64 = if (isUserLand) 0x400000 else kernStackSize;
        const userStackCommit: u64 = if (isUserLand) 0x10000 else 0;
        var userStack: u64 = 0;
        var kernelStack: u64 = 0;

        var failed = false;
        if (!flags.contains(.idle)) {
            kernelStack = kernel.addrSpace.alloc(kernStackSize, memory.Region.Flags.fromFlag(.fixed), 0, true);
            failed = (kernelStack == 0);
            if (!failed) {
                if (isUserLand) {
                    userStack = process.addrSpace.alloc(userStackRes, memory.Region.Flags.empty(), 0, false);

                    const region = process.addrSpace.findAndPin(userStack, userStackRes).?;
                    _ = process.addrSpace.reserveMutex.acquire();
                    failed = !process.addrSpace.commitCount(region, (userStackRes - userStackCommit) / pageSize, userStackCommit / pageSize);
                    process.addrSpace.reserveMutex.release();
                    process.addrSpace.unpinRegion(region);
                    failed = failed or userStack == 0;
                } else {
                    userStack = kernelStack;
                }
            }
        }

        if (!failed) {
            thread.paused.writeVolatile((parentThread != null and process == parentThread.?.process and parentThread.?.paused.readVolatile()) or flags.contains(.paused));
            thread.handleCount.writeVolatile(2);
            thread.isKernelThread = !isUserLand;
            thread.priority = if (flags.contains(.lowPriority)) @intFromEnum(Thread.Priority.low) else @intFromEnum(Thread.Priority.normal);
            thread.kernelStackBase = kernelStack;
            thread.userStackBase = if (isUserLand) userStack else 0;
            thread.userStackRes = userStackRes;
            thread.userStackCommit.writeVolatile(userStackCommit);
            thread.terminatableState.writeVolatile(if (isUserLand) .terminatable else .inSyscall);
            thread.type =
                if (flags.contains(.asyncTask)) Thread.Type.asyncTask else if (flags.contains(.idle)) Thread.Type.idle else Thread.Type.normal;
            thread.id = @atomicRmw(u64, &kernel.scheduler.nextThreadID, .Add, 1, .SeqCst);
            thread.process = process;
            thread.item.value = thread;
            thread.allItems.value = thread;
            thread.procItem.value = thread;

            if (thread.type != .idle) {
                thread.interruptCtx = arch.initThread(kernelStack, kernStackSize, thread, startAddr, arg1, arg2, isUserLand, userStack, userStackRes);
            } else {
                thread.state.writeVolatile(.active);
                thread.executing.writeVolatile(true);
            }

            process.threads.append(&thread.procItem);

            _ = kernel.scheduler.allProcessesMutex.acquire();
            kernel.scheduler.allThreads.prepend(&thread.allItems);
            kernel.scheduler.allThreadsMutex.release();

            _ = kernel.object.openHandle(process, 0);

            if (thread.type == .normal) {
                kernel.scheduler.dispatchSpinLock.acquire();
                kernel.scheduler.addThreadToActiveThreads(thread, true);
                kernel.scheduler.dispatchSpinLock.release();
            }
            return thread;
        } else {
            if (userStack != 0) _ = process.addrSpace.free(userStack, 0, false);
            if (kernelStack != 0) _ = kernel.addrSpace.free(kernelStack, 0, false);
            kernel.scheduler.threadPool.remove(@intFromPtr(thread));
            return null;
        }
    }

    // pub const Policy = enum(u16) {
    //     FIFO,
    //     RoundRobin,
    //     Other,
    //     Batch,
    //     Idle,
    //     Deadline,
    // };
};

pub const Node = extern struct {
    driverNode: u64,
    handleCount: Volatile(u64),
    dirEntry: u64,
    filesystem: u64,
    id: u64,
    writerLock: WriterLock,
    nodeError: i64,
    flags: Volatile(u32),
    cacheItem: List,
};
pub const Process = extern struct {
    addrSpace: *memory.AddressSpace,
    messageQueue: kernel.MessageQueue,
    threads: LinkedList(Thread),
    threadsMutex: Mutex,
    execNode: ?*Node,
    execName: [32]u8,
    data: ProcCreateData,
    permissions: Process.Permission,
    creationFlags: Process.CreationFlags,
    type: Process.Type,
    id: u64,
    handleCount: Volatile(u64),
    handleTable: kernel.object.Handle.Table,
    allItems: LinkedList(Process).Node,
    crashed: bool,
    crashMutex: Mutex,
    crashReason: kernel.CrashReason,
    allThreadsPaused: bool,
    allThreadsTerminated: bool,
    blockShutdown: bool,
    preventNewThreads: bool,
    exitStatusCode: i32,
    killedEvent: Event,
    execState: ExecState,
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

    pub const Permission = Bitflag(enum(u64) {
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

    pub const CreationFlags = Bitflag(enum(u32) {
        paused = 0,
    });

    pub fn crash(self: *Process, crashReason: *kernel.CrashReason) callconv(.C) void {
        if (self == &kernel.process) kernel.panic("kernel process has crashed");
        if (self.type != .normal) kernel.panic("A critical process has crashed");

        const pauseFlag = false;
        _ = self.crashMutex.acquire();
        if (!self.crashed) {
            self.crashed = true;
            pause = true;

            self.crashReason = crashReason.*;
            if (!kernel.scheduler.shutdown.readVolatile()) {
                // notify the desktop
            }
        }

        self.crashMutex.release();

        if (pauseFlag) {
            self.pause(false);
        }
    }

    pub fn pause(self: *Process, resumeAfter: bool) callconv(.C) void {
        _ = self.threadsMutex.acquire();
        var maybreThreadNode = self.threads.first;

        while (maybreThreadNode) |threadNode| {
            const thread = threadNode.value.?;
            maybreThreadNode = threadNode.next;

            ThreadPause(thread, resumeAfter);
        }

        self.threadsMutex.release();
    }
};
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

pub const Scheduler = extern struct {
    dispatchSpinLock: SpinLock,
    activeTimersSpinLock: SpinLock,
    activeThreads: [Thread.priorityCount]LinkedList(Thread),
    pausedThreads: LinkedList(Thread),
    terminatedThreads: LinkedList(Thread),
    asyncTaskSpinLock: SpinLock,
    allThreads: LinkedList(Thread),
    allProcesses: LinkedList(Process),
    allThreadsMutex: Mutex,
    allProcessesMutex: Mutex,
    nextThreadID: u64,
    nextProcessID: u64,
    nextProcID: u64,
    allProcessesTerminatedEv: Event,
    activeProcessCount: Volatile(u64),
    blockShutdownProcCount: Volatile(u64),
    started: Volatile(bool),
    panic: Volatile(bool),
    shutdown: Volatile(bool),
    timeMs: Volatile(u64),
    activeTimers: LinkedList(Timer),
    threadPool: kernel.Pool,
    processPool: kernel.Pool,
    addrSpacePool: kernel.Pool,

    pub fn notifyObject(
        self: *@This(),
        blockedThreads: *LinkedList(Thread),
        unblockAll: bool,
        prevMutexOwner: ?*Thread,
    ) void {
        self.dispatchSpinLock.assertLocked();
        var unblockedThread = blockedThreads.first;
        if (unblockedThread == null) return;

        while (true) {
            if (@intFromPtr(unblockedThread)) {
                kernel.panic("unblockedThread is null");
            }

            const nextUnblockedThread = unblockedThread.?.next;
            const nextUnblockedThreadValue = nextUnblockedThread.?.value.?;

            self.unblockThread(nextUnblockedThreadValue, prevMutexOwner);
            unblockedThread = nextUnblockedThread;
            if (!(unblockedThread != null and unblockAll)) break;
        }
    }

    pub fn pickThread(self: *@This(), local: *arch.LocalStorage) ?*Thread {
        self.dispatchSpinLock.assertLocked();

        if ((local.asyncTaskList.nextOrFirst != null or local.isInAsyncTask) and local.asyncTaskThread.?.state.readVolatile() == .active) {
            return local.asyncTaskThread;
        }

        for (self.activeThreads) |*threadNode| {
            if (threadNode.first) |first| {
                first.removeFromList();
                return first.value;
            }
        }

        return local.idleThread;
    }

    pub fn maybeUpdateList(self: *@This(), thread: *Thread) void {
        if (thread.type == .asyncTask) return;
        if (thread.type != .normal) kernel.panic("trying to update the active list of a non-normal thread");

        self.dispatchSpinLock.assertLocked();

        if (thread.state.readVolatile() != .active or thread.executing.readVolatile()) return;
        if (thread.item.list == null) kernel.panic("Thread is active and not executing, but it is not found in any active thread list. Thread ID: {}, State: {}", .{ thread.id, thread.state });

        const effectivePriority = @as(u64, @intCast(self.getThreadEffectivePriority(thread)));
        if (&self.activeThreads[effectivePriority] == thread.item.list) return;
        thread.item.removeFromList();
    }
    pub fn yield(self: *@This()) void {
        if (!self.started.readVolatile()) return;

        if (arch.getLocalStorage()) |local| {
            if (!local.isSchedulerReady) return;

            // Handle active timers
            if (local.processorID == 0) {
                self.handleActiveTimers();
            }

            // Ensure no spinlocks are acquired
            self.ensureNoSpinlocks(local);

            // Disable interrupts before acquiring dispatcher lock
            arch.disableInterrupts();

            self.dispatchSpinLock.acquire();

            // Ensure no interrupts are enabled while holding the dispatcher lock
            self.ensureInterruptsDisabled();

            // Ensure the current thread is executing
            self.ensureThreadIsExecuting(local);

            // Handle thread state and transitions
            self.updateThreadState(local);

            // Thread management logic for different blocking states
            self.handleBlockingStates(local);

            // Manage thread scheduling
            self.scheduleNewThread(local);

            // Switch context to the new thread
            self.switchContext(local);
        }
    }

    fn handleActiveTimers(self: *@This()) void {
        self.timeMs = arch.getTimeMS();
        kernel.globalData.schedulerTimeMS.writeVolatile(self.timeMs);

        self.activeTimersSpinLock.acquire();
        var maybeTimerNode = self.activeTimers.first;

        while (maybeTimerNode) |timerNode| {
            const timer = timerNode.value.?;
            const next = timerNode.next;

            if (timer.triggerInMs <= self.timeMs) {
                self.activeTimers.remove(timerNode);
                _ = timer.event.set(false);
                if (@as(?AsyncTask.Callback, @ptrFromInt(timer.callback))) |cb| {
                    timer.asyncTask.register(cb);
                }
            } else {
                break;
            }

            maybeTimerNode = next;
        }
        self.activeTimersSpinLock.release();
    }

    fn ensureNoSpinlocks(local: *arch.LocalStorage) void {
        if (local.spinlockCount != 0) {
            kernel.panic("Cannot yield: {d} spinlocks are still acquired", .{local.spinlockCount});
        }
    }

    fn ensureInterruptsDisabled(self: *@This()) void {
        if (self.dispatchSpinLock.interruptsEnabled.readVolatile()) {
            kernel.panic("Cannot proceed: interrupts were enabled when the scheduler lock was acquired. This may lead to inconsistent state or race conditions.");
        }
    }

    fn ensureThreadIsExecuting(local: *arch.LocalStorage) void {
        if (!local.currentThread.?.executing.readVolatile()) {
            kernel.panic("Cannot yield: current thread is not executing");
        }
    }

    fn updateThreadState(self: *@This(), local: *arch.LocalStorage) void {
        const killThread = local.currentThread.?.terminatableState.readVolatile() == .terminatable and local.currentThread.?.terminating.readVolatile();
        const keepThreadAlive = local.currentThread.?.terminatableState.readVolatile() == .userBlockRequest and local.currentThread.?.terminating.readVolatile();

        if (killThread) {
            local.currentThread.?.state.writeVolatile(.terminated);
            local.currentThread.?.killAsyncTask.register(Thread.kill);
        } else if (local.currentThread.?.state.readVolatile() == .waitingMutex) {
            self.handleWaitingMutex(local, keepThreadAlive);
        } else if (local.currentThread.?.state.readVolatile() == .waitingEvent) {
            self.handleWaitingEvent(local, keepThreadAlive);
        } else if (local.currentThread.?.state.readVolatile() == .waitingWriterLock) {
            self.handleWaitingWriterLock(local);
        }
    }

    fn handleWaitingMutex(self: *@This(), local: *arch.LocalStorage, keepThreadAlive: bool) void {
        const mtx = @as(*Mutex, @ptrCast(local.currentThread.?.blocking.mutex.?));

        if (!keepThreadAlive and mtx.ownerThread != null) {
            mtx.ownerThread.?.blockedThreadPriorities[@as(u64, @intCast(local.currentThread.?.priority))] += 1;
            self.maybeUpdateList(@as(*Thread, @ptrCast(@alignCast(@as(@alignOf(Thread), mtx.ownerThread.?)))));
            mtx.blockedThreads.append(&local.currentThread.?.item);
        } else {
            local.currentThread.?.state.writeVolatile(.active);
        }
    }

    fn handleWaitingEvent(local: *arch.LocalStorage, keepThreadAlive: bool) void {
        if (keepThreadAlive) {
            local.currentThread.?.state.writeVolatile(.active);
        } else {
            var unblocked = false;

            for (local.currentThread.?.blocking.event.array[0..local.currentThread.?.blocking.event.count]) |event| {
                if (event.?.state.readVolatile() != 0) {
                    local.currentThread.?.state.writeVolatile(.active);
                    unblocked = true;
                    break;
                }
            }

            if (!unblocked) {
                for (local.currentThread.?.blocking.event.array[0..local.currentThread.?.blocking.event.count], 0..) |ev, i| {
                    const event = @as(*Event, @ptrCast(ev.?));
                    const item = @as(*LinkedList(Thread).Node, @ptrCast(&local.currentThread.?.blocking.event.items.?[i]));
                    event.blockedThreads.append(item);
                }
            }
        }
    }

    fn handleWaitingWriterLock(local: *arch.LocalStorage) void {
        const lock = @as(*WriterLock, @ptrCast(local.currentThread.?.blocking.writer.lock.?));
        if ((local.currentThread.?.blocking.writer.type == WriterLock.shared and lock.state.readVolatile() >= 0) or
            (local.currentThread.?.blocking.writer.type == WriterLock.exclusive and lock.state.readVolatile() == 0))
        {
            local.currentThread.?.state.writeVolatile(.active);
        } else {
            lock.blockedThreads.append(&local.currentThread.?.item);
        }
    }

    fn scheduleNewThread(self: *@This(), local: *arch.LocalStorage) void {
        if (!local.currentThread.?.state.readVolatile() == .active) return;

        if (local.currentThread.?.type == .normal) {
            self.addThreadToActiveThreads(local.currentThread.?, false);
        } else if (local.currentThread.?.type == .idle or local.currentThread.?.type == .asyncTask) {
            local.currentThread.?.state.writeVolatile(.active);
        } else {
            kernel.panic("unrecognized thread type");
        }
    }

    fn switchContext(self: *@This(), local: *arch.LocalStorage) void {
        const newThread = self.pickThread(local) orelse kernel.panic("Could not find a thread to execute");
        local.currentThread = newThread;

        if (newThread.executing.readVolatile()) kernel.panic("thread in active queue already executing");

        newThread.executing.writeVolatile(true);
        newThread.execProcId = local.processorID;
        newThread.cpuTimeSlices.increment();
        if (newThread.type == .idle) newThread.process.idleTimeSlices += 1 else newThread.process.cpuTimeSlices += 1;

        arch.nextTimer(1);

        const newCtx = newThread.interruptCtx;
        const addrSpace = if (newThread.tempAddrSpace) |tas| @as(*memory.AddressSpace, @ptrCast(tas)) else newThread.process.addrSpace;
        addrSpace.openRef();

        // Define oldAddrSpace as the address space of the current thread,
        // or as its temp address space if available.
        const oldAddrSpace = if (local.currentThread.?.tempAddrSpace) |tas|
            @as(*memory.AddressSpace, @ptrCast(tas))
        else
            local.currentThread.?.process.addrSpace;

        arch.switchContext(newCtx, &addrSpace.arch, newThread.kernelStack, newThread, oldAddrSpace);
        kernel.panic("context switch unexpectedly returned");
    }

    //TODO?: should I keep this implementation
    // pub fn yield(self: *@This(), ctx: *arch.InterruptContext) void {
    //     if (!self.started.readVolatile()) return;
    //     if (arch.getLocalStorage()) |local| {
    //         if (!local.isSchedulerReady) return;

    //         if (local.processorID == 0) {
    //             self.timeMs = arch.getTimeMS();
    //             kernel.globalData.schedulerTimeMS.writeVolatile(self.timeMs);

    //             self.activeTimersSpinLock.acquire();

    //             var maybeTimerNode = self.activeTimers.first;

    //             while (maybeTimerNode) |timerNode| {
    //                 const timer = timerNode.value.?;
    //                 const next = timerNode.next;

    //                 if (timer.triggerInMs <= self.timeMs) {
    //                     self.activeTimers.remove(timerNode);
    //                     _ = timer.event.set(false);
    //                     if (@as(?AsyncTask.Callback, @ptrFromInt(timer.callback))) |cb| {
    //                         timer.asyncTask.register(cb);
    //                     }
    //                 } else {
    //                     break;
    //                 }
    //                 maybeTimerNode = next;
    //             }
    //         }
    //         self.activeTimersSpinLock.release();

    //         if (local.spinlockCount != 0) kernel.panic("Cannot yield: {d} spinlocks are still acquired", .{local.spinlockCount});

    //         arch.disableInterrupts();

    //         self.dispatchSpinLock.acquire();

    //         if (self.dispatchSpinLock.interruptsEnabled.readVolatile()) {
    //             kernel.panic("Cannot proceed: interrupts were enabled when the scheduler lock was acquired. This may lead to inconsistent state or race conditions.");
    //         }

    //         if (!local.currentThread.?.executing.readVolatile()) {
    //             kernel.panic("Cannot yield: current thread is not executing");
    //         }

    //         const oldAddrSpace = if (local.currentThread.?.tempAddrSpace) |tas| @as(*memory.AddressSpace, @ptrCast(tas)) else local.currentThread.?.process.addrSpace;
    //         local.currentThread.?.interruptCtx = ctx;
    //         local.currentThread.?.executing.writeVolatile(false);

    //         const killThread = local.currentThread.?.terminatableState.readVolatile() == .terminatable and local.currentThread.?.terminating.readVolatile();
    //         const keepThreadAlive = local.currentThread.?.terminatableState.readVolatile() == .userBlockRequest and local.currentThread.?.terminating.readVolatile();

    //         if (killThread) {
    //             local.currentThread.?.state.writeVolatile(.terminated);

    //             local.currentThread.?.killAsyncTask.register(Thread.kill);
    //         } else if (local.currentThread.?.state.readVolatile() == .waitingMutex) {
    //             const mtx = @as(*Mutex, @ptrCast(local.currentThread.?.blocking.mutex.?));

    //             if (!keepThreadAlive and mtx.ownerThread != null) {
    //                 mtx.ownerThread.?.blockedThreadPriorities[@as(u64, @intCast(local.currentThread.?.priority))] += 1;
    //                 self.maybeUpdateList(@as(*Thread, @ptrCast(@alignCast(@as(@alignOf(Thread), mtx.ownerThread.?)))));
    //                 mtx.blockedThreads.append(&local.currentThread.?.item);
    //             } else {
    //                 local.currentThread.?.state.writeVolatile(.active);
    //             }
    //         } else if (local.currentThread.?.state.readVolatile() == .waitingEvent) {
    //             if (keepThreadAlive) {
    //                 local.currentThread.?.state.writeVolatile(.active);
    //             } else {
    //                 var unblocked = false;

    //                 for (local.currentThread.?.blocking.event.array[0..local.currentThread.?.blocking.event.count]) |event| {
    //                     if (event.?.state.readVolatile() != 0) {
    //                         local.currentThread.?.state.writeVolatile(.active);
    //                         unblocked = true;
    //                         break;
    //                     }
    //                 }

    //                 if (!unblocked) {
    //                     for (local.currentThread.?.blocking.event.array[0..local.currentThread.?.blocking.event.count], 0..) |ev, i| {
    //                         const event = @as(*Event, @ptrCast(ev.?));
    //                         const item = @as(*LinkedList(Thread).Node, @ptrCast(&local.currentThread.?.blocking.event.items.?[i]));
    //                         event.blockedThreads.append(item);
    //                     }
    //                 }
    //             }
    //         } else if (local.currentThread.?.state.readVolatile() == .waitingWriterLock) {
    //             const lock = @as(*WriterLock, @ptrCast(local.currentThread.?.blocking.writer.lock.?));
    //             if ((local.currentThread.?.blocking.writer.type == WriterLock.shared and lock.state.readVolatile() >= 0) or
    //                 (local.currentThread.?.blocking.writer.type == WriterLock.exclusive and lock.state.readVolatile() == 0))
    //             {
    //                 local.currentThread.?.state.writeVolatile(.active);
    //             } else {
    //                 lock.blockedThreads.append(&local.currentThread.?.item);
    //             }
    //         }

    //         if (!killThread and local.currentThread.?.state.readVolatile() == .active) {
    //             if (local.currentThread.?.type == .normal) {
    //                 self.addThreadToActiveThreads(local.currentThread.?, false);
    //             } else if (local.currentThread.?.type == .idle or local.currentThread.?.type == .asyncTask) {
    //                 local.currentThread.?.state.writeVolatile(.active);
    //             } else {
    //                 kernel.panic("unrecognized thread type");
    //             }
    //         }

    //         const newThread = self.pickThread(local) orelse kernel.panic("Could not find a thread to execute");
    //         local.currentThread = newThread;
    //         if (newThread.executing.readVolatile()) kernel.panic("thread in active queue already executing");

    //         newThread.executing.writeVolatile(true);
    //         newThread.execProcId = local.processorID;
    //         newThread.cpuTimeSlices.increment();
    //         if (newThread.type == .idle) newThread.process.idleTimeSlices += 1 else newThread.process.cpuTimeSlices += 1;

    //         arch.nextTimer(1);
    //         const newCtx = newThread.interruptCtx;
    //         const addrSpace = if (newThread.tempAddrSpace) |tas| @as(*memory.AddressSpace, @ptrCast(tas)) else newThread.process.addrSpace;
    //         addrSpace.openRef();
    //         arch.switchContext(newCtx, &addrSpace.arch, newThread.kernelStack, newThread, oldAddrSpace);
    //         kernel.panic("do context switch unexpectedly returned");
    //     }
    // }
    pub fn unblockThread(self: *@This(), unblockedThread: *Thread, prevMutexOwner: ?*Thread) void {
        _ = prevMutexOwner;

        self.dispatchSpinLock.assertLocked();
        switch (unblockedThread.state.readVolatile()) {
            .waitingMutex => {
                if (unblockedThread.blocking.mutex == null) {
                    kernel.panic("unblockedThread.blocking.mutex is null");
                }
                unblockedThread.blocking.mutex.?.unlock();
            },
            .waitingEvent => {
                if (unblockedThread.blocking.event.items == null) {
                    kernel.panic("unblockedThread.blocking.event.items is null");
                }
                if (unblockedThread.blocking.event.count == 0) {
                    kernel.panic("unblockedThread.blocking.event.count is 0");
                }
                unblockedThread.blocking.event.count -= 1;
                if (unblockedThread.blocking.event.count == 0) {
                    unblockedThread.state.writeVolatile(Thread.state.active);
                }
            },
            .waitingWriterLock => {
                if (unblockedThread.blocking.writer.type) {
                    kernel.panic("unblockedThread.blocking.writer.type is true");
                }
                unblockedThread.state.writeVolatile(Thread.state.active);
            },
            else => kernel.panic("Unexpected or invalid thread state"),
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
            kernel.panic("Thread is not active");
        }
    }

    fn ensureThreadIsNotExecuting(_: *@This(), thread: Thread) void {
        if (thread.executing.readVolatile()) {
            kernel.panic("Thread is executing");
        }
    }

    fn ensureThreadIsNormal(_: *@This(), thread: Thread) void {
        if (thread.type != .normal) {
            kernel.panic("Thread is not normal");
        }
    }

    fn ensureThreadPriorityIsValid(_: *@This(), thread: Thread) void {
        if (thread.priority < 0) {
            kernel.panic("Thread priority is less than 0");
        } else if (thread.priority >= Thread.priorityCount) {
            kernel.panic("Thread priority is greater than or equal to priorityCount");
        }
    }

    fn ensureThreadIsNotInList(_: *@This(), thread: Thread) void {
        if (thread.item.list != null) {
            kernel.panic("Thread is already in a list");
        }
    }

    pub fn getThreadEffectivePriority(self: *@This(), thread: *Thread) i8 {
        self.dispatchSpinLock.assertLocked();
        for (thread.blockedThreadPriorities[0..@as(u64, @intCast(thread.priority))], 0..) |priority, idx| {
            if (priority != 0) return @as(i8, @intCast(idx));
        }
        return thread.priority;
    }
};

export fn ThreadPause(thread: *Thread, resumeAfter: bool) callconv(.C) void {
    kernel.scheduler.dispatchSpinLock.acquire();

    if (thread.paused.readVolatile() == !resumeAfter) return;

    thread.paused.writeVolatile(!resumeAfter);

    if (!resumeAfter and thread.terminatableState.readVolatile() == .terminatable) {
        if (thread.state.readVolatile() == .active) {
            if (thread.executing.readVolatile()) {
                if (thread == arch.getCurrentThread()) {
                    kernel.scheduler.dispatchSpinLock.release();

                    arch.fakeTimerInterrupt();

                    if (thread.paused.readVolatile()) kernel.panic("current thread incorrectly resumed");
                } else {
                    arch.IPI.sendYield(thread);
                }
            } else {
                thread.item.removeFromList();
                kernel.scheduler.addThreadToActiveThreads(thread, false);
            }
        }
    } else if (resumeAfter and thread.item.list == &kernel.scheduler.pausedThreads) {
        kernel.scheduler.pausedThreads.remove(&thread.item);
        kernel.scheduler.addThreadToActiveThreads(thread, false);
    }

    kernel.scheduler.dispatchSpinLock.release();
}

export fn threadExit(thread: *Thread) callconv(.C) void {
    kernel.scheduler.dispatchSpinLock.acquire();

    var yield = false;
    const wasTerminating = thread.terminating.readVolatile();
    if (!wasTerminating) {
        thread.terminating.writeVolatile(true);
        thread.paused.writeVolatile(false);
    }

    if (thread == arch.getCurrentThread()) {
        thread.terminatableState.writeVolatile(.terminatable);
        yield = true;
    } else if (!wasTerminating and !thread.executing.readVolatile()) {
        switch (thread.terminatableState.readVolatile()) {
            .terminatable => {
                if (thread.state.readVolatile() != .active) kernel.panic("terminatable thread non-active");

                thread.item.removeFromList();
                thread.killAsyncTask.register(Thread.kill);
                yield = true;
            },
            .userBblockRequest => {
                const threadState = thread.state.readVolatile();
                if (threadState == .waitingMutex or threadState == .waitingEvent) kernel.scheduler.unblockThread(thread, null);
            },
            else => {},
        }
    }

    kernel.scheduler.dispatchSpinLock.release();
    if (yield) arch.fakeTimerInterrupt();
}

pub const AsyncTask = extern struct {
    item: ds.List,
    cb: u64,
    const Callback = fn (*@This()) void;

    pub fn register(self: *@This(), cb: Callback) void {
        kernel.scheduler.asyncTaskSpinLock.acquire();
        if (self.cb == 0) {
            self.cb = @intFromPtr(cb);
            arch.getLocalStorage().?.asyncTaskList.insert(&self.item, false);
        }
        kernel.scheduler.asyncTaskSpinLock.release();
    }
};

pub const Timer = extern struct {
    event: Event,
    asyncTask: AsyncTask,
    node: LinkedList(Timer).Node,
    triggerInMs: u64,
    callback: ?AsyncTask.Callback,
    arg: u64,

    pub fn setEx(self: *@This(), triggerInMS: u64, maybeCb: ?AsyncTask.Callback, maybeArg: u64) void {
        kernel.scheduler.activeTimersSpinLock.acquire();

        removeFromActiveTimersIfNeeded(self);

        self.event.reset();
        self.triggerInMs = triggerInMS + kernel.scheduler.timeMs.readVolatile();
        self.callback = @intFromPtr(maybeCb);
        self.arg = maybeArg;
        self.node.value = self;

        insertTimerInCorrectPosition(self);

        kernel.scheduler.activeTimersSpinLock.release();
    }

    fn removeFromActiveTimersIfNeeded(self: *@This()) void {
        if (self.node.list != null) {
            kernel.scheduler.activeTimers.remove(&self.node);
        }
    }

    fn insertTimerInCorrectPosition(self: *@This()) void {
        var maybeTimer = kernel.scheduler.activeTimers.first;
        while (maybeTimer != null) {
            const timer = maybeTimer.?.value.?;
            const next = maybeTimer.?.next;
            if (timer.triggerInMs > self.triggerInMs) break;

            maybeTimer = next;
        }

        if (maybeTimer) |timer| {
            kernel.scheduler.activeTimers.prepend(&self.node, timer);
        } else {
            kernel.scheduler.activeTimers.append(&self.node);
        }
    }

    pub fn set(self: *@This(), triggerInMS: u64) void {
        self.setEx(triggerInMS, null, 0);
    }

    pub fn remove(self: *@This()) void {
        kernel.scheduler.activeTimersSpinLock.acquire();
        if (self.callback != null) kernel.panic("timer with callback cannot be removed");
        if (self.node.list != null) kernel.scheduler.activeTimers.remove(&self.node);
        kernel.scheduler.activeTimersSpinLock.release();
    }
};

export fn ProcessKill(process: *Process) callconv(.C) void {
    if (process.handleCount.readVolatile() == 0) kernel.panic("process is on the all process list but there are no handles in it");

    _ = kernel.scheduler.activeProcessCount.atomicFetchAdd(1);

    _ = kernel.scheduler.allProcessesMutex.acquire();
    kernel.scheduler.allProcesses.remove(&process.allItems);

    if (kernel.physicalMemoryManager.nextProcToBalance == process) {
        kernel.physicalMemoryManager.nextProcToBalance = if (process.allItems.next) |next| next.value else null;
        kernel.physicalMemoryManager.nextProcToBalance = null;
        kernel.physicalMemoryManager.balanceResumePosition = 0;
    }

    kernel.scheduler.allProcessesMutex.release();

    kernel.scheduler.dispatchSpinLock.acquire();
    process.allThreadsTerminated = true;
    kernel.scheduler.dispatchSpinLock.release();
    _ = process.killedEvent.set(true);
    process.handleTable.destroy();
    process.addrSpace.destroy();
}

export fn procExit(proc: *Process, status: i32) callconv(.C) void {
    _ = proc.threadsMutex.acquire();
    proc.exitStatusCode = status;
    proc.preventNewThreads = true;

    const currentThread = arch.getCurrentThread().?;
    const isCurrentProc = currentThread == proc;

    var currentThreadFound = false;
    var threadNode = proc.threads.first;

    while (threadNode) |node| {
        const thread = node.value.?;
        threadNode = node.next;

        if (thread != currentThread) {
            threadExit(thread);
        } else if (isCurrentProc) {
            currentThreadFound = true;
        } else kernel.panic("current thread is in the wrong process");
    }

    proc.threadsMutex.release();

    if (!currentThreadFound and isCurrentProc) kernel.panic("current thread not found in process") else if (isCurrentProc) {
        threadExit(currentThread);
    }
}
