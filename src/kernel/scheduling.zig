const std = @import("std");
const ds = @import("./ds.zig");
const LinkedList = ds.LinkedList;
const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;
const SpinLock = kernel.SpinLock;

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
    executing: Volatile(bool),
    paused: Volatile(bool),
    yieldIpiReceived: Volatile(bool),

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

    execState: Process.ExecState,
    execStartRequest: bool,

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
};
