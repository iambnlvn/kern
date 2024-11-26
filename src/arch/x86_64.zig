const std = @import("std");
const Thread = @import("./../kernel/scheduling.zig").Thread; //Todo: update the import path to come from kernel directly
const List = @import("./../kernel/kernel.zig").ds.List;
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
    // mutex: Mutex, //TODO!: implement
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
