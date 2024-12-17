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
pub const entryPerPageTableBitCount = 9;
const pageBitCount = 0xc;

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
pub extern fn invalidatePage(page: u64) callconv(.C) void;
pub extern fn ProcessorReadCR3() callconv(.C) u64;
pub extern fn EarlyAllocPage() callconv(.C) u64;
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
            const askedPhysicalAddr = va - lowMemMapStart;
            const mapPageFlags = memory.MapPageFlags.fromFlag(.commitTablesNow);
            _ = mapPage(&kernel.addrSpace, askedPhysicalAddr, va, mapPageFlags);
            return true;
        } else if (va >= coreMemStartRegion and va < coreMemStartRegion + coreMemRegionCount * @sizeOf(memory.Region) and forSupervisor) {
            const physicalAllocationFlags = memory.Physical.Flags.fromFlag(.zeroed);
            const askedPhysicalAddr = memory.physicalAllocWithFlags(physicalAllocationFlags);
            const mapPageFlags = memory.MapPageFlags.fromFlag(.commitTablesNow);
            _ = mapPage(&kernel.addrSpace, askedPhysicalAddr, va, mapPageFlags);
            return true;
        } else if (va >= coreAddrSpaceStart and va < coreAddrSpaceStart + coreAddrSpaceSize and forSupervisor) {
            return kernel.coreAddressSpace.handlePageFault(va, flags);
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

pub export fn mapPage(space: *memory.AddressSpace, askedPhysicalAddr: u64, va: u64, flags: memory.MapPageFlags) callconv(.C) bool {
    if ((va | askedPhysicalAddr) & (pageSize - 1) != 0) {
        std.debug.panic("Pages are not aligned", .{});
    }

    if (kernel.physicalMemoryManager.pageFrameDBCount != 0 and (askedPhysicalAddr >> pageBitCount < kernel.physicalMemoryManager.pageFrameDBCount)) {
        const frameState = kernel.physicalMemoryManager.pageFrames[askedPhysicalAddr >> pageBitCount].state.readVolatile();

        if (frameState != .active and frameState != .unusable) {
            std.debug.panic("Page frame is not active nor unusable", .{});
        }
    }

    if (askedPhysicalAddr == 0) std.debug.panic("Physical address is null", .{});
    if (va == 0) std.debug.panic("Virtual address is null", .{});

    if (askedPhysicalAddr < 0xFFFF800000000000 and ProcessorReadCR3() != space.arch.cr3) {
        std.debug.panic("Mapping physical address in wrong address space", .{});
    }

    const aquireFrameLock = !flags.contains(.noNewTables) and !flags.contains(.frameLockAquired);
    if (aquireFrameLock) _ = kernel.physicalMemoryManager.pageFrameMutex.acquire();
    defer if (aquireFrameLock) kernel.physicalMemoryManager.pageFrameMutex.release();

    const aquireSpaceLock = !flags.contains(.noNewTables);

    if (aquireSpaceLock) {
        space.arch.mutex.acquire();
    }

    defer if (aquireSpaceLock) space.arch.mutex.release();

    const physicalAddress = askedPhysicalAddr & 0xFFFFFFFFFFFFF000;
    const virtualAddress = va & 0x0000FFFFFFFFF000;

    const indices = PageTables.computeIndices(virtualAddress);

    if (space != &kernel.coreAddressSpace and space != &kernel.addrSpace) {
        const L4Index = indices[@intFromEnum(PageTables.Level.level4)];
        if (space.arch.commit.L3[L4Index >> 3] & (@as(u8, 1) << @as(u3, @truncate(L4Index & 0b111))) == 0) std.debug.panic("attempt to map using uncommited L3 page table\n");

        const L3Index = indices[@intFromEnum(PageTables.Level.level3)];
        if (space.arch.commit.L3[L4Index >> 3] & (@as(u8, 1) << @as(u3, @truncate(L3Index & 0b111))) == 0) std.debug.panic("attempt to map using uncommited L2 page table\n");

        const L2Index = indices[@intFromEnum(PageTables.Level.level2)];
        if (space.arch.commit.L3[L4Index >> 3] & (@as(u8, 1) << @as(u3, @truncate(L2Index & 0b111))) == 0) std.debug.panic("attempt to map using uncommited L1 page table\n");
    }

    handleMissingPageTable(space, .level4, indices, flags);
    handleMissingPageTable(space, .level3, indices, flags);
    handleMissingPageTable(space, .level2, indices, flags);

    const oldValue = PageTables.access(.level1, indices).*;
    var value = physicalAddress | 0b11;

    if (flags.contains(.writeCombining)) value |= 16;
    if (flags.contains(.notCacheable)) value |= 24;
    if (flags.contains(.user)) value |= 7 else value |= 1 << 8;
    if (flags.contains(.readOnly)) value &= ~@as(u64, 2);
    if (flags.contains(.copied)) value |= 1 << 9;

    value |= (1 << 5);
    value |= (1 << 6);

    if (oldValue & 1 != 0 and !flags.contains(.overwrite)) {
        if (flags.contains(.ignoreIfMapped)) {
            return false;
        }

        if (oldValue & ~@as(u64, pageSize - 1) != physicalAddress) {
            std.debug.panic("attempt to map page tha has already been mapped", .{});
        }

        if (oldValue == value) {
            std.debug.panic("attempt to rewrite page translation", .{});
        } else {
            const writable = oldValue & 2 == 0 and value & 2 != 0;
            if (!writable) {
                std.debug.panic("attempt to change flags mapping address", .{});
            }
        }
    }

    PageTables.access(.level1, indices).* = value;

    invalidatePage(va);

    return true;
}

pub export fn unMapPages(
    space: *memory.AddressSpace,
    virtualAddrStart: u64,
    pageCount: u64,
    flags: memory.UnmapPagesFlags,
    unmapMax: u64,
    resumePos: ?*u64,
) callconv(.C) void {
    _ = kernel.physicalMemoryManager.pageFrameMutex.acquire();
    defer kernel.physicalMemoryManager.pageFrameMutex.release();

    _ = space.arch.mutex.acquire();
    space.arch.mutex.release();

    const tableBase = virtualAddrStart & 0x0000FFFFFFFFF000;
    const start: u64 = if (resumePos) |rp| rp.* else 0;

    var page = start;
    while (page < pageCount) : (page += 1) {
        const va = (page << pageBitCount) + tableBase;
        const indices = PageTables.computeIndices(va);

        comptime var level: PageTables.Level = .level4;
        if (PageTables.access(level, indices).* & 1 == 0) {
            page -= (va >> pageBitCount) % (1 << (entryPerPageTableBitCount * @intFromEnum(level)));
            page += 1 << (entryPerPageTableBitCount * @intFromEnum(level));
            continue;
        }

        level = .level3;
        if (PageTables.access(level, indices).* & 1 == 0) {
            page -= (va >> pageBitCount) % (1 << (entryPerPageTableBitCount * @intFromEnum(level)));
            page += 1 << (entryPerPageTableBitCount * @intFromEnum(level));
            continue;
        }

        level = .level2;
        if (PageTables.access(level, indices).* & 1 == 0) {
            page -= (va >> pageBitCount) % (1 << (entryPerPageTableBitCount * @intFromEnum(level)));
            page += 1 << (entryPerPageTableBitCount * @intFromEnum(level));
            continue;
        }

        const translation = PageTables.access(.level1, indices).*;

        if (translation & 1 == 0) continue;

        const copy = (translation & (1 << 9)) != 0;

        if (copy and flags.contains(.balanceFile) and !flags.contains(.freeCopied)) continue;

        if ((~translation & (1 << 5) != 0) or (~translation & (1 << 6) != 0)) {
            std.debug.panic("page found without accessed or dirty bit set", .{});
        }

        PageTables.access(.level1, indices).* = 0;
        const pa = translation & 0x0000FFFFFFFFF000;

        if (flags.contains(.free) or (flags.contains(.free_copied) and copy)) {
            memory.physical_free(pa, true, 1);
        } else if (flags.contains(.balanceFile)) {
            _ = unmapMax;
            //TODO: implement balance file
        }
    }
    //TODO!: invalide pages

    // MMArchInvalidatePages(virtualAddrStart, pageCount);
}

inline fn handleMissingPageTable(
    space: *memory.AddressSpace,
    comptime level: PageTables.Level,
    indices: PageTables.Indices,
    flags: memory.MapPageFlags,
) void {
    if (PageTables.access(level, indices).* & 1 == 0) {
        if (flags.contains(.noNewTables)) std.debug.panic("no new tables flag set but a table was missing", .{});

        const physicalAllocationFlags = memory.Physical.Flags.fromFlag(.lockAquired);
        const physicalAllocation = memory.physical_allocate_with_flags(physicalAllocationFlags) | 0b111;
        PageTables.access(level, indices).* = physicalAllocation;
        const prevLevel = comptime @as(PageTables.Level, @enumFromInt(@intFromEnum(level) - 1));

        const page = @intFromPtr(PageTables.access_at_index(prevLevel, indices[@intFromEnum(prevLevel)]));
        invalidatePage(page);
        const pageSlice = @as([*]u8, @ptrFromInt(page & ~@as(u64, pageSize - 1)))[0..pageSize];
        @memset(pageSlice, 0);
        space.arch.activePageTableCount += 1;
    }
}
const PageTables = extern struct {
    const Level = enum(u8) {
        level1 = 0,
        level2 = 1,
        level3 = 2,
        level4 = 3,

        const count = std.enums.values(PageTables.Level).len;
    };

    fn access(comptime level: Level, indices: Indices) *volatile u64 {
        return accessAt(level, indices[@intFromEnum(level)]);
    }

    fn accessAt(comptime level: Level, index: u64) *volatile u64 {
        return switch (level) {
            .level1 => @as(*volatile u64, @ptrFromInt(0xFFFFFF0000000000 + index * @sizeOf(u64))),
            .level2 => @as(*volatile u64, @ptrFromInt(0xFFFFFF7F80000000 + index * @sizeOf(u64))),
            .level3 => @as(*volatile u64, @ptrFromInt(0xFFFFFF7FBFC00000 + index * @sizeOf(u64))),
            .level4 => @as(*volatile u64, @ptrFromInt(0xFFFFFF7FBFDFE000 + index * @sizeOf(u64))),
        };
    }
    const Indices = [PageTables.Level.count]u64;

    inline fn computeIndices(va: u64) Indices {
        var indices: Indices = undefined;
        inline for (comptime std.enums.values(PageTables.Level)) |value| {
            indices[@intFromEnum(value)] = va >> (pageBitCount + entryPerPageTableBitCount * @intFromEnum(value));
        }
        return indices;
    }
};

pub fn freeAddressSpace(space: *memory.AddressSpace) callconv(.C) void {
    var i: u64 = 0;
    while (i < 256) : (i += 1) {
        if (PageTables.accessAt(.level4, i).* == 0) continue;

        var j = i * entryPerPageTableBitCount;
        while (j < (i + 1) * entryPerPageTableBitCount) : (j += 1) {
            if (PageTables.accessAt(.level3, j).* == 0) continue;

            var k = j * entryPerPageTableBitCount;
            while (k < (j + 1) * entryPerPageTableBitCount) : (k += 1) {
                if (PageTables.accessAt(.level2, k).* == 0) continue;

                memory.physicalFree(PageTables.accessAt(.level2, k).* & ~@as(u64, pageSize - 1), false, 1);
                space.arch.activePageTableCount -= 1;
            }

            memory.physicalFree(PageTables.accessAt(.level3, j).* & ~@as(u64, pageSize - 1), false, 1);
            space.arch.activePageTableCount -= 1;
        }

        memory.physicalFree(PageTables.accessAt(.level4, i).* & ~@as(u64, pageSize - 1), false, 1);
        space.arch.activePageTableCount -= 1;
    }

    if (space.arch.activePageTableCount != 0) {
        std.debug.panic("space has still active page tables", .{});
    }

    _ = kernel.coreAddressSpace.reserveMutex.acquire();
    const l1CommitRegion = kernel.coreAddressSpace.findRegion(@intFromPtr(space.arch.commit.L1)).?;
    unMapPages(&kernel.coreAddressSpace, l1CommitRegion.descriptor.baseAddr, l1CommitRegion.descriptor.pageCount, memory.UnmapPagesFlags.fromFlag(.free), 0, null);
    kernel.coreAddressSpace.unreserve(l1CommitRegion, false, false);
    kernel.coreAddressSpace.reserveMutex.release();
    memory.decommit(space.arch.commitedPagePerTable * pageSize, true);
}
