const std = @import("std");
const kernel = @import("./../kernel.zig");
const Thread = kernel.scheduling.Thread;
const List = kernel.ds.List;
const Mutex = kernel.sync.Mutex;
const zeroes = kernel.zeroes;
const memory = kernel.memory;
pub const kernelAddrSpaceStart = 0xFFFF900000000000;
pub const kernelAddrSpaceSize = 0xFFFFF00000000000 - 0xFFFF900000000000;
pub const pageSize = 0x1000;
pub const lowMemMapStart = 0xFFFFFE0000000000;
pub const lowMemLimit = 0x100000000;
pub const coreMemStartRegion = 0xFFFF8001F0000000;
pub const coreMemRegionCount = (0xFFFF800200000000 - 0xFFFF8001F0000000) / @sizeOf(memory.Region);
pub const coreAddrSpaceStart = 0xFFFF800100000000;
pub const coreAddrSpaceSize = 0xFFFF8001F0000000 - 0xFFFF800100000000;
pub const modulesStart = 0xFFFFFFFF90000000;
pub const modulesSize = 0xFFFFFFFFC0000000 - 0xFFFFFFFF90000000;

pub fn initThread(
    kernelStack: u64,
    kernelStackSize: u64,
    thread: *Thread,
    startAddr: u6,
    arg1: u64,
    arg2: u64,
    isUserLand: bool,
    userStack: u64,
    userStackSize: u64,
) callconv(.C) *InterruptContext {
    const base = kernelStack + kernelStackSize - 8;
    const ctx = @as(*InterruptContext, @ptrFromInt(base - @sizeOf(InterruptContext)));
    thread.kernelStackBase = base;

    const ptr = @as(*u64, @ptrFromInt(base));

    const fnPtr = getKThreadTerminateAddr();
    ptr.* = fnPtr;
    ctx.fxsave[32] = 0x80;
    ctx.fxsave[33] = 0x1F;

    if (isUserLand) {
        ctx.cs = 0x5b;
        ctx.ds = 0x63;
        ctx.ss = 0x63;
    } else {
        ctx.cs = 0x48;
        ctx.ds = 0x50;
        ctx.ss = 0x50;
    }

    ctx._check = 0x1234567890ABCDEF;
    ctx.rflags = 1 << 9;
    ctx.rip = startAddr;
    if (ctx.rip == 0) std.debug.panic("RIP is null", .{});
    ctx.rsp = userStack + userStackSize - 8;
    ctx.rdi = arg1;
    ctx.rsi = arg2;

    return ctx;
}

pub const InterruptContext = extern struct {
    ds: u64,
    fxsave: [512 + 16]u8,
    _check: u64,
    cr8: u64,
    cr2: u64,
    r15: u64,
    r14: u64,
    r13: u64,
    r12: u64,
    r11: u64,
    r10: u64,
    r9: u64,
    r8: u64,
    rdi: u64,
    rsi: u64,
    rbp: u64,
    rdx: u64,
    rcx: u64,
    rbx: u64,
    rax: u64,
    intNum: u64,
    errorCode: u64,
    rip: u64,
    cs: u64,
    rflags: u64,
    rsp: u64,
    ss: u64,

    fn checkSanity(self: *@This()) void {
        if (self.cs > 0x100 or self.ds > 0x100 or self.ss > 0x100 or (self.rip >= 0x1000000000000 and self.rip < 0xFFFF000000000000) or (self.rip < 0xFFFF800000000000 and self.cs == 0x48)) {
            std.debug.panic("Failed to check context sanity", .{});
        }
    }
};

pub extern fn getKThreadTerminateAddr() callconv(.C) u64;
pub extern fn switchContext(
    context: *InterruptContext,
    archAddrSpace: *AddressSpace,
    threadKernelStack: u64,
    newThread: *Thread,
    oldThread: *AddressSpace,
) callconv(.C) void;

pub extern fn getLocalStorage() callconv(.C) ?*LocalStorage;
pub extern fn getCurrentThread() callconv(.C) ?*Thread;
pub extern fn areInterruptsEnabled() callconv(.C) bool;
pub extern fn ProcessorReadTimeStamp() callconv(.C) u64;
pub extern fn halt() callconv(.C) noreturn;
pub extern fn enableInterrupts() callconv(.C) void;
pub extern fn debugOutByte(byte: u8) callconv(.C) void;
pub extern fn disableInterrupts() callconv(.C) void;
pub extern fn fakeTimerInterrupt() callconv(.C) void;
extern fn MMArchSafeCopy(dest: u64, source: u64, byteCount: u64) callconv(.C) bool;

const LocalStorage = extern struct {
    currentThread: ?*Thread,
    idleThread: ?*Thread,
    asyncTaskThread: ?*Thread,
    panicCtx: ?*InterruptContext,
    IRQSwitchCtx: bool,
    isSchedulerReady: bool,
    isInIRQ: bool,
    isInAsyncTask: bool,
    processorID: 32,
    spinlockCount: u64,
    cpu: ?*CPU,
    asyncTaskList: List,

    //TODO: imlement get and set functions
};
pub const CPU = extern struct {
    processorID: u8,
    kernelProcessorID: u8,
    APICID: u8,
    isbootProcessor: bool,
    kernelStack: *align(1) u64,
    local: ?*LocalStorage,
};

pub const AddressSpace = extern struct {
    cr3: u64,
    commit: Commit,
    commitedPagePerTable: u64,
    activePageTableCount: u64,
    mutex: Mutex,
};
const Commit = extern struct {
    L1: [*]u8,
    commitL1: [L1_COMMIT_SIZE]u8,
    L2: [L2_COMMIT_SIZE]u8,
    L3: [L3_COMMIT_SIZE]u8,

    const L1_COMMIT_SIZE = 1 << 8;
    const L2_COMMIT_SIZE = 1 << 14;
    const L3_COMMIT_SIZE = 1 << 5;
};
export fn InterruptHandler(ctx: *InterruptContext) callconv(.C) void {
    if (kernel.scheduler.panic.readVolatile() and ctx.intNum != 2) return;
    if (areInterruptsEnabled()) std.debug.panic("interrupts were enabled at the start of the interrupt handler", .{});

    const interrupt = ctx.intNum;
    var maybeLS = getLocalStorage();
    if (maybeLS) |ls| {
        if (ls.currentThread) |currentThread| currentThread.lastInterruptTicks = ProcessorReadTimeStamp();
        if (ls.spinlockCount != 0 and ctx.cr8 != 0xe) std.debug.panic("spinlock count is not zero", .{});
    }

    if (interrupt < 0x20) {
        if (interrupt == 2) {
            maybeLS.?.panicCtx = ctx;
            halt();
        }

        const supervisor = ctx.cs & 3 == 0;

        if (!supervisor) {
            if (ctx.cs != 0x5b and ctx.cs != 0x6b) std.debug.panic("Invalid CS", .{});
            const currentThread = getCurrentThread().?;
            if (currentThread.isKernelThread) std.debug.panic("Kernel thread is executing user code", .{});
            const prevTerminatableState = currentThread.terminatableState.readVolatile();
            currentThread.terminatableState.writeVolatile(.inSysCall);

            if (maybeLS) |ls| if (ls.spinlockCount != 0) std.debug.panic("user exception occurred with spinlock acquired", .{});

            enableInterrupts();
            maybeLS = null;
            if ((interrupt == 14 and !handlePageFault(ctx.cr2, if (ctx.errorCode & 2 != 0) memory.HandlePageFaultFlags.fromFlag(.write) else memory.handlePageFaultFlags.empty())) or interrupt != 14) {
                const rbp = ctx.rbp;
                const traceDepth: u32 = 0;
                while (rbp != 0 and traceDepth < 32) {
                    if (!MMArchIsBufferInUserRange(rbp, 16)) break;
                    var val: u64 = 0;
                    if (!MMArchSafeCopy(@intFromPtr(&val), rbp + 8, @sizeOf(u64))) break;
                    if (val == 0) break;
                    if (!MMArchSafeCopy(@intFromPtr(&rbp), rbp, @sizeOf(u64))) break;
                }

                var crashReason = zeroes(kernel.CrashReason);
                crashReason.errorCode = kernel.FatalError.processorException;
                crashReason.duringSysCall = -1;
                currentThread.process.?.crash(&crashReason);
            }

            if (currentThread.terminatableState.readVolatile() != .inSysCall) std.debug.panic("Thread changed terminatable state during interrupt", .{});

            currentThread.terminatableState.writeVolatile(prevTerminatableState);
            if (currentThread.terminating.readVolatile() or currentThread.paused.readVolatile()) fakeTimerInterrupt();
            disableInterrupts();
        } else {
            if (ctx.cs != 0x48) std.debug.panic("Invalid CS", .{});
            if (interrupt == 14) {
                if (ctx.errorCode & (1 << 3) != 0) std.debug.panic("unresolvable page fault", .{});

                if (maybeLS) |local| if (local.spinlockCount != 0 and (ctx.cr2 >= 0xFFFF900000000000 and ctx.cr2 < 0xFFFFF00000000000)) std.debug.panic("page fault occurred with spinlocks active", .{});

                if (ctx.fxsave & 0x200 != 0 and ctx.cr8 != 0xe) {
                    enableInterrupts();
                    maybeLS = null;
                }

                const res = handlePageFault(ctx.cr2, (if (ctx.errorCode & 2 != 0) memory.HandlePageFaultFlags.fromFlag(.write) else memory.HandlePageFaultFlags.empty()).orFlag(.forSupervisor));
                var safeCopy = false;
                if (!res) {
                    if (getCurrentThread().?.inSafeCopy and ctx.cr2 < 0x8000000000000000) {
                        safeCopy = true;
                        ctx.rip = ctx.r8;
                    }
                }

                if (res or safeCopy) disableInterrupts() else std.debug.panic("Unhandleable page fault", .{});
            } else {
                std.debug.panic("Unable to resolve exception", .{});
            }
        }
    } else if ((interrupt == 0xFF) or (interrupt >= 0x20 and interrupt < 0x30) or (interrupt >= 0xF0 and interrupt < 0xFE)) {
        std.debug.panic("Not implemented", .{});
    }
}

fn interruptHandlerMaker(comptime num: u64, comptime hasErrorCode: bool) type {
    return extern struct {
        fn routine() callconv(.Naked) noreturn {
            @setRuntimeSafety(false);
            if (comptime !hasErrorCode) {
                //push error code if it is not included
                asm volatile (
                    \\.intel_syntax noprefix
                    \\push 0
                );
            }
            //push interrupt number
            asm volatile (".intel_syntax noprefix\npush " ++ std.fmt.comptimePrint("{}", .{num}));
            //push general purpose regs onto the stack to save their state
            asm volatile (
                \\.intel_syntax noprefix
                \\cld
                \\push rax
                \\push rbx
                \\push rcx
                \\push rdx
                \\push rsi
                \\push rdi
                \\push rbp
                \\push r8
                \\push r9
                \\push r10
                \\push r11
                \\push r12
                \\push r13
                \\push r14
                \\push r15
                \\mov rax, cr8
                \\push rax
                \\mov rax, 0x123456789ABCDEF
                \\push rax
                \\mov rbx, rsp
                \\and rsp, ~0xf
                \\fxsave [rsp - 512]
                \\mov rsp, rbx
                \\sub rsp, 512 + 16
                \\xor rax, rax
                \\mov ax, ds
                \\push rax
                \\mov ax, 0x10
                \\mov ds, ax
                \\mov es, ax
                \\mov rax, cr2
                \\push rax
                \\mov rdi, rsp
                \\mov rbx, rsp
                \\and rsp, ~0xf
                \\call InterruptHandler
                \\mov rsp, rbx
                \\xor rax, rax
                \\jmp ReturnFromInterruptHandler
            );
            unreachable;
        }
    };
}
//This is currently not fully implemented, acts as a placeHolder
pub export fn handlePageFault(faultyAddr: u64, flags: memory.HandlePageFaultFlags) bool {
    const va = faultyAddr & ~@as(u64, pageSize - 1);
    const forSupervisor = flags.contains(.forSupervisor);

    if (!areInterruptsEnabled()) {
        std.debug.panic("Page fault with interrupts disabled\n", .{});
    }

    const faultInVeryLowMem = va < pageSize;
    if (!faultInVeryLowMem) {
        if (va >= lowMemMapStart and va < lowMemMapStart + lowMemLimit and forSupervisor) {
            const physicalAddr = va - lowMemMapStart;
            const mapPageFlags = memory.MapPageFlags.fromFlag(.commitTablesNow);
            _ = mapPage(&kernel.addrSpace, physicalAddr, va, mapPageFlags); //TODO!: implement mapPage
            return true;
        } else if (va >= coreMemStartRegion and va < coreMemStartRegion + coreMemRegionCount * @sizeOf(memory.Region) and forSupervisor) {
            const physicalAllocationFlags = memory.Physical.Flags.fromFlag(.zeroed);
            const physicalAddr = memory.physicalAllocWithFlags(physicalAllocationFlags);
            const mapPageFlags = memory.MapPageFlags.fromFlag(.commitTablesNow);
            _ = mapPage(&kernel.addrSpace, physicalAddr, va, mapPageFlags);
            return true;
        } else if (va >= coreAddrSpaceStart and va < coreAddrSpaceStart + coreAddrSpaceSize and forSupervisor) {
            return kernel.coreAddrSpace.handlePageFault(va, flags);
        } else if (va >= kernelAddrSpaceStart and va < kernelAddrSpaceSize and forSupervisor) {
            return kernel.addrSpace.handlePgaeFault(va, flags);
        } else if (va >= modulesStart and va < modulesStart + modulesSize and forSupervisor) {
            return kernel.addrSpace.handlePageFault(va, flags);
        } else {
            if (getCurrentThread()) |currentThread| {
                const space = if (currentThread.TempAddrSpace) |tempAddrSpace| @as(*memory.AddressSpace, @ptrCast(tempAddrSpace)) else currentThread.process.?.addrSpace;
                return space.handlePageFault(va, flags);
            } else {
                std.debug.panic("unreachable path\n,", .{});
            }
        }
    }

    return false;
}

export fn MMArchIsBufferInUserRange(baseAddr: u64, byteCount: u64) callconv(.C) bool {
    if (baseAddr & 0xFFFF800000000000 != 0) return false;
    if (byteCount & 0xFFFF800000000000 != 0) return false;
    if ((baseAddr + byteCount) & 0xFFFF800000000000 != 0) return false;
    return true;
}
