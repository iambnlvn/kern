const std = @import("std");
const kernel = @import("../kernel.zig");
const Thread = kernel.scheduling.Thread;
const List = kernel.ds.List;
const Mutex = kernel.sync.Mutex;
const zeroes = kernel.zeroes;
const memory = kernel.memory;
const Volatile = kernel.Volatile;
const ACPI = @import("../drivers/ACPI.zig");
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
const timerInterrupt = 0x40;
const ipiLock = kernel.ipiLock;

export var tlbShootdownVirtualAddress: Volatile(u64) = undefined;
export var tlbShootdownPageCount: Volatile(u64) = undefined;
export var timeStampCounterSynchronizationValue = Volatile(u64){ .value = 0 };
const invalidateAllPagesThreshold = 1024;
pub var modulePtr: u64 = modulesStart;

var getTimeFromPitMsStarted = false;
var getTimeFromPitMsLast: u64 = 0;
var getTimeFromPitMSCumulative: u64 = 0;

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
pub extern fn ProcessorInvalidateAllPages() callconv(.C) void;
pub const KIRQHandler = fn (interruptIdx: u64, ctx: u64) callconv(.C) bool;
pub extern fn out8(port: u16, value: u8) callconv(.C) void;
pub extern fn in8(port: u16) callconv(.C) u8;
pub extern fn out16(port: u16, value: u16) callconv(.C) void;
pub extern fn in16(port: u16) callconv(.C) u16;
pub extern fn out32(port: u16, value: u32) callconv(.C) void;
pub extern fn in32(port: u16) callconv(.C) u32;

pub extern fn setAddressSpace(AddressSpace: *memory.AddressSpace) callconv(.C) void;

pub const LocalStorage = extern struct {
    currentThread: ?*Thread,
    idleThread: ?*Thread,
    asyncTaskThread: ?*Thread,
    panicCtx: ?*InterruptContext,
    IRQSwitchCtx: bool,
    isSchedulerReady: bool,
    isInIRQ: bool,
    isInAsyncTask: bool,
    processorID: u32,
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
    const maybeLS = getLocalStorage();
    if (maybeLS) |ls| {
        if (ls.currentThread) |currentThread| currentThread.lastInterruptTicks = ProcessorReadTimeStamp();
        if (ls.spinlockCount != 0 and ctx.cr8 != 0xe) std.debug.panic("spinlock count is not zero", .{});
    }

    if (interrupt < 0x20) {
        handleException(ctx, maybeLS);
    } else if ((interrupt == 0xFF) or (interrupt >= 0x20 and interrupt < 0x30) or (interrupt >= 0xF0 and interrupt < 0xFE)) {
        std.debug.panic("Not implemented", .{});
    }
}

fn handleException(ctx: *InterruptContext, maybeLS: ?*LocalStorage) void {
    if (ctx.intNum == 2) {
        maybeLS.?.panicCtx = ctx;
        halt();
    }

    const supervisor = ctx.cs & 3 == 0;

    if (!supervisor) {
        handleUserException(ctx, maybeLS);
    } else {
        handleKernelException(ctx, maybeLS);
    }
}

fn handleUserException(ctx: *InterruptContext, maybeLS: ?*LocalStorage) void {
    if (ctx.cs != 0x5b and ctx.cs != 0x6b) std.debug.panic("Invalid CS", .{});
    const currentThread = getCurrentThread().?;
    if (currentThread.isKernelThread) std.debug.panic("Kernel thread is executing user code", .{});
    const prevTerminatableState = currentThread.terminatableState.readVolatile();
    currentThread.terminatableState.writeVolatile(.inSysCall);

    if (maybeLS) |ls| if (ls.spinlockCount != 0) std.debug.panic("user exception occurred with spinlock acquired", .{});

    enableInterrupts();
    maybeLS = null;
    if ((ctx.intNum == 14 and !handlePageFault(ctx.cr2, if (ctx.errorCode & 2 != 0) memory.HandlePageFaultFlags.fromFlag(.write) else memory.handlePageFaultFlags.empty())) or ctx.intNum != 14) {
        handleCrash(ctx, currentThread);
    }

    if (currentThread.terminatableState.readVolatile() != .inSysCall) std.debug.panic("Thread changed terminatable state during interrupt", .{});

    currentThread.terminatableState.writeVolatile(prevTerminatableState);
    if (currentThread.terminating.readVolatile() or currentThread.paused.readVolatile()) fakeTimerInterrupt();
    disableInterrupts();
}

fn handleKernelException(ctx: *InterruptContext, maybeLS: ?*LocalStorage) void {
    if (ctx.cs != 0x48) std.debug.panic("Invalid CS", .{});
    if (ctx.intNum == 14) {
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

fn handleCrash(ctx: *InterruptContext, currentThread: *Thread) void {
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
comptime {
    asm (
        \\  .intel_syntax noprefix
        \\  .global CPUSetup1
        \\CPUSetup1:
        // Check for no-execute (NXE) bit support in CPU
        \\  mov eax, 0x80000001          // Query extended features (CPUID)
        \\  cpuid
        \\  and edx, 1 << 20            // Check bit 20 (NXE support)
        \\  shr edx, 20                 // Extract NXE bit (0 or 1)
        \\  mov rax, OFFSET pagingNXESupport // Address of the pagingNXESupport variable
        \\  and [rax], edx              // Store NXE support status (1 if supported, 0 otherwise)
        \\  cmp edx, 0                  // Check if NXE is supported
        \\  je .noPagingNXEsupport      // Jump if NXE is not supported
        \\  mov ecx, 0xC0000080         // Target the Extended Feature Enable Register (EFER)
        \\  rdmsr                      // Read the EFER register
        \\  or eax, 1 << 11             // Set bit 11 (NXE enable)
        \\  wrmsr                      // Write back to the EFER register
        \\.noPagingNXEsupport:

        // Initialize the x87 FPU
        \\  fninit                      // Reset the x87 FPU
        \\  mov rax, OFFSET .cw         // Load address of control word
        \\  fldcw [rax]                 // Load the x87 control word
        \\  jmp .cwa                    // Skip control word data
        \\.cw: .short 0x037a            // Control word for x87 (enable specific precision and exceptions)
        \\.cwa:

        // Ensure the kernel cannot execute userland pages
        \\  xor eax, eax                // Clear EAX to query basic CPUID information
        \\  cpuid                       // Get CPUID basic information
        \\  cmp eax, 7                  // Check if extended features are supported
        \\  jb .noSMEPSupport           // Jump if the CPUID instruction is not supported (based on prior flags)
        \\  mov eax, 7                  // Request extended features (leaf 7)
        \\  xor ecx, ecx                // Sub-leaf index 0
        \\  cpuid                       // Execute CPUID instruction
        \\  and ebx, 1 << 7             // Check for SMEP (bit 7 in EBX)
        \\  shr ebx, 7                  // Extract SMEP bit (0 or 1)
        \\  mov rax, OFFSET pagingSMEPSupport // Address of pagingSMEPSupport variable
        \\  and [rax], ebx              // Store SMEP support status (1 if supported, 0 otherwise)
        \\  cmp ebx, 0                  // Check if SMEP is supported
        \\  je .noSMEPSupport           // Jump if SMEP is not supported
        \\  mov word ptr [rax], 2       // Set SMEP state to 2 (indicating enabled)
        \\  mov rax, cr4                // Load CR4 register
        \\  or rax, 1 << 20             // Set bit 20 (SMEP enable)
        \\  mov cr4, rax                // Write back to CR4
        \\.noSMEPSupport:
        // Enable PCID support, if available
        \\  mov eax, 1                  // Request basic feature set (leaf 1)
        \\  xor ecx, ecx                // Sub-leaf index 0
        \\  cpuid                       // Execute CPUID instruction
        \\  and ecx, 1 << 17            // Check for PCID (bit 17 in ECX)
        \\  shr ecx, 17                 // Extract PCID bit (0 or 1)
        \\  mov rax, OFFSET pagingPCIDSupport // Address of pagingPCIDSupport variable
        \\  and [rax], ecx              // Store PCID support status (1 if supported, 0 otherwise)
        \\  cmp ecx, 0                  // Check if PCID is supported
        \\  je .noPCIDSupport           // Jump if PCID is not supported
        \\  mov rax, cr4                // Load CR4 register
        \\  or rax, 1 << 17             // Set bit 17 (PCID enable)
        \\  mov cr4, rax                // Write back to CR4
        \\.noPCIDSupport:

        // Enable global pages
        \\  mov rax, cr4                // Load CR4 register
        \\  or rax, 1 << 7              // Set bit 7 (Global Pages enable)
        \\  mov cr4, rax                // Write back to CR4 register

        // Enable TCE support, if available
        \\  mov eax, 0x80000001         // Query extended features (CPUID leaf 0x80000001)
        \\  xor ecx, ecx                // Clear ECX (sub-leaf index 0)
        \\  cpuid                       // Execute CPUID instruction
        \\  and ecx, 1 << 17            // Check for TCE support (bit 17 in ECX)
        \\  shr ecx, 17                 // Extract TCE support bit (0 or 1)
        \\  mov rax, OFFSET pagingTCESupport // Address of pagingTCESupport variable
        \\  and [rax], ecx              // Store TCE support status (1 if supported, 0 otherwise)
        \\  cmp ecx, 0                  // Check if TCE is supported
        \\  je .noTceSupport            // Jump if TCE is not supported
        \\  mov ecx, 0xC0000080         // Target Extended Feature Enable Register (EFER)
        \\  rdmsr                      // Read the EFER register
        \\  or eax, 1 << 15             // Set bit 15 (TCE enable)
        \\  wrmsr                      // Write back to the EFER register
        \\.noTceSupport:
        \\  mov rax, cr0                // Load CR0 register
        \\  or rax, 1 << 16             // Set bit 16 (related to paging and protection enable)
        \\  mov cr0, rax                // Write back to CR0 register
        // Enable MMX, SSE, and SSE2
        \\  mov rax, cr0                // Load CR0 register
        \\  mov rbx, cr4                // Load CR4 register
        \\  and rax, ~4                 // Clear EM (bit 2) in CR0 (enable FPU instructions)
        \\  or rax, 2                   // Set MP (bit 1) in CR0 (monitor co-processor)
        \\  or rbx, 512 + 1024          // Set OSFXSR (bit 9) and OSXMMEXCPT (bit 10) in CR4 (enable SSE/SSE2)
        \\  mov cr0, rax                // Write updated CR0 register
        \\  mov cr4, rbx                // Write updated CR4 register

        // Detect SSE3 and SSSE3, if available
        \\  mov eax, 1                  // Query processor features (CPUID leaf 1)
        \\  cpuid                       // Execute CPUID instruction
        \\  test ecx, 1 << 0            // Check for SSE3 support (bit 0 in ECX)
        \\  jnz .hasSSE3                // Jump if SSE3 is supported
        \\  mov rax, OFFSET simdSSE3Support // Address of simdSSE3Support variable
        \\  and byte ptr [rax], 0       // Mark SSE3 as unsupported (clear to 0)
        \\.hasSSE3:
        \\  test ecx, 1 << 9            // Check for SSSE3 support (bit 9 in ECX)
        \\  jnz .hasSSSE3               // Jump if SSSE3 is supported
        \\  mov rax, OFFSET simdSSSE3Support // Address of simdSSSE3Support variable
        \\  and byte ptr [rax], 0       // Mark SSSE3 as unsupported (clear to 0)
        \\.hasSSSE3:
        // Enable SYSCALL and SYSRET by modifying the MSRs (Model-Specific Registers)
        \\  mov ecx, 0xC0000080           // MSR for SYSCALL/SYSRET control (Star MSR)
        \\  rdmsr                        // Read MSR[0xC0000080] into EDX:EAX
        \\  or eax, 1                    // Set the SYSCALL enable bit (bit 0)
        \\  wrmsr                        // Write back to MSR[0xC0000080]
        \\  add ecx, 1                   // Move to the next MSR (LSTAR)
        \\  rdmsr                        // Read LSTAR MSR (SYSRET return address)
        \\  mov edx, 0x005B0048          // Set return address for SYSRET (some code segment)
        \\  wrmsr                        // Write back to LSTAR MSR
        \\  add ecx, 1                   // Move to the next MSR (CSTAR)
        \\  mov rdx, OFFSET SyscallEntry // Set the address for SYSCALL handler
        \\  mov rax, rdx                 // Load the address of the handler into RAX
        \\  shr rdx, 32                  // Move upper 32 bits to the lower part of RDX
        \\  wrmsr                        // Write to CSTAR MSR (Call SYSRET entry)
        \\  add ecx, 2                   // Move to the next MSR (SFMASK)
        \\  rdmsr                        // Read SFMask MSR (interrupt flags)
        // Clear direction and interrupt flag when we enter ring 0 (set bits 9 and 10)
        \\  mov eax, (1 << 10) | (1 << 9)  // Set bits 9 and 10 to clear direction flag and interrupt flag
        \\  wrmsr                        // Write back to SFMask MSR

        // Assign PAT2 to WC (Write Combining)
        \\  mov ecx, 0x277               // PAT MSR (0x277) for page attribute table
        \\  xor rax, rax                 // Clear RAX (to prepare for the PAT modification)
        \\  xor rdx, rdx                 // Clear RDX
        \\  rdmsr                        // Read PAT MSR
        \\  and eax, 0xFFF8FFFF          // Clear bits related to PAT2
        \\  or eax, 0x00010000           // Set PAT2 to WC (Write Combining)
        \\  wrmsr                        // Write back to PAT MSR

        // Set up CPU local storage
        \\  .setupCPULocalStorage:
        \\  mov ecx, 0xC0000101           // MSR for CPU local storage (FS/GS base addresses)
        \\  mov rax, OFFSET _cpuLocalStorage // Load address of local storage
        \\  mov rdx, OFFSET _cpuLocalStorage // Load address of local storage
        \\  shr rdx, 32                  // Move upper 32 bits of local storage address to the upper part of RDX
        \\  mov rdi, OFFSET _cpuLocalStorageIdx // Load the local storage index address
        \\  add rax, [rdi]               // Add index to the base address for local storage
        // Space for 4 8-byte values at gs:0 - gs:31
        \\  add qword ptr [rdi], 32           // Update memory at GS:0 (for local storage or thread-specific data)
        \\  wrmsr                            // Write back to MSR (Model-Specific Register)
        // Load the IDT (Interrupt Descriptor Table Register)
        \\  .loadIdtr:
        \\  mov rax, OFFSET idtDescriptor    // Load address of the IDT descriptor structure
        \\  mov word ptr [rax], 0x1000       // Set IDT limit (size - 1), 0x1000 indicates size 4096 bytes
        \\  mov qword ptr [rax + 2], OFFSET idtData // Load address of the IDT table
        \\  lidt [rax]                       // Load IDT descriptor into IDTR register
        \\  sti                              // Enable interrupts

        // Enable APIC (Advanced Programmable Interrupt Controller)
        \\  .enableAPIC:
        // In AMD CPUs, the APIC is always enabled by default, but we need to configure it
        \\  mov ecx, 0x1B                    // Access MSR (Model-Specific Register) for APIC control
        \\  rdmsr                            // Read the current MSR value into EDX:EAX
        \\  or eax, 0x800                    // Set bit 11 to enable APIC
        \\  wrmsr                            // Write modified MSR value back to control APIC state
        \\  and eax, ~0xFFF                  // Mask the lower 12 bits to configure the APIC base
        \\  mov edi, eax                     // Store the result in EDI register (APIC base address)
        \\  mov rax, 0xFFFFFE00000000F0      // Calculate APIC base address (interrupt vectors)
        \\  add rax, rdi                     // Add the APIC base address to RAX
        \\  mov ebx, [rax]                   // Load APIC value from the computed address
        \\  or ebx, 0x1FF                    // Set lower 9 bits to configure APIC behavior
        \\  mov [rax], ebx                   // Write the modified APIC value back to memory
        \\  mov rax, 0xFFFFFE00000000E0      // Next APIC register (e.g., Task Priority Register)
        \\  add rax, rdi                     // Add the APIC base address to RAX
        \\  mov dword ptr [rax], 0xFFFFFFFF  // Set the Task Priority Register to the maximum value
        \\  xor rax, rax                     // Zero out RAX register
        \\  mov cr8, rax                     // Write to CR8 register to disable interrupt processing
        \\  ret                              // Return from this function

        // Global function to retrieve the current thread's GS base address
        \\  .global getCurrentThread
        \\  getCurrentThread:
        \\  mov rax, qword ptr gs:16          // Load the current thread's address from GS:16
        \\  ret                              // Return the value of RAX
        // Global function to get the current RSP (stack pointer)
        \\  .global ProcessorGetRSP
        \\  ProcessorGetRSP:
        \\  mov rax, rsp                     // Load the value of the stack pointer into RAX
        \\  ret                              // Return the value of RAX

        // Global function to get the current RBP (base pointer)
        \\  .global ProcessorGetRBP
        \\  ProcessorGetRBP:
        \\  mov rax, rbp                     // Load the value of the base pointer into RAX
        \\  ret                              // Return the value of RAX

        // Halt the CPU (enter HALT state)
        \\  .global halt
        \\  halt:
        \\  cli                              // Clear interrupt flag (disable interrupts)
        \\  hlt                              // Halt the CPU (infinite loop)
        \\  jmp halt                         // Jump back to halt to create a loop

        // Global function to check if interrupts are enabled
        \\  .global areInterruptsEnabled
        \\  areInterruptsEnabled:
        \\  pushf                            // Push flags onto the stack
        \\  pop rax                          // Pop the flags into RAX
        \\  and rax, 0x200                   // Check the interrupt flag (IF bit)
        \\  shr rax, 9                       // Shift right to check bit 9 (Interrupt Flag)
        \\  mov rdx, cr8                     // Check the current value of CR8 (interrupt state)
        \\  cmp rdx, 0                       // If CR8 is zero, interrupts are disabled
        \\  je .done                         // Jump to done if interrupts are disabled
        \\  mov rax, 0                       // Otherwise, set RAX to 0 indicating interrupts are enabled
        \\  .done:
        \\  ret                              // Return from the function

        // Enable interrupts
        \\  .global enableInterrupts
        \\  enableInterrupts:
        \\  mov rax, 0                       // Set RAX to 0
        \\  mov cr8, rax                     // Write to CR8 to enable interrupts
        \\  sti                              // Set the interrupt flag (enable interrupts)
        \\  ret                              // Return from the function
        // Disable interrupts by modifying CR8 register
        \\  .global disableInterrupts
        \\  disableInterrupts:
        \\  mov rax, 14                      // Load the value 14 into RAX (value for disabling interrupts in CR8)
        \\  mov cr8, rax                     // Write to CR8 register to disable interrupts at the CPU level
        \\  sti                              // Set the interrupt flag (IF) to allow interrupts, to ensure interrupts are disabled at the system level
        \\  ret                              // Return from the function (interrupts are now disabled)

        // Retrieve the local storage value for the current thread (gs:0)
        \\  .global getLocalStorage
        \\  getLocalStorage:
        \\  mov rax, qword ptr gs:0            // Load the value at GS:0 (current thread's local storage)
        \\  ret                               // Return the value of the local storage (Thread-local data)

        // Simulate a timer interrupt by invoking interrupt vector 0x40
        \\  .global fakeTimerInterrupt
        \\  fakeTimerInterrupt:
        \\  int 0x40                          // Trigger interrupt 0x40 (fake timer interrupt, typically used for a timer service or software interrupts)
        \\  ret                               // Return from the interrupt simulation

        // Declare an external function `KThreadTerminate` (thread termination function)
        \\  .extern KThreadTerminate

        // Local function that jumps to the external KThreadTerminate function
        \\  .global _KThreadTerminate
        \\  _KThreadTerminate:
        \\  sub rsp, 8                        // Decrease the stack pointer by 8 to prepare space for the function call
        \\  jmp KThreadTerminate             // Jump to the external KThreadTerminate function to terminate the thread

        // Retrieve the address of the `KThreadTerminate` function for dynamic function calls
        \\  .global GetKThreadTerminateAddress
        \\  GetKThreadTerminateAddress:
        \\  mov rax, OFFSET _KThreadTerminate // Load the address of the _KThreadTerminate function into RAX
        \\  ret                               // Return the address in RAX

        // Read the CR3 register (Page Table Base Register) to retrieve the base address of the page tables
        \\  .global ProcessorReadCR3
        \\  ProcessorReadCR3:
        \\  mov rax, cr3                      // Read the current value of CR3 (holds the base address of page tables)
        \\  ret                               // Return the value of CR3 (Page Table Base Address)

        // Invalidate a single page (flush from the TLB)
        \\  .global invalidatePage
        \\  invalidatePage:
        \\  invlpg [rdi]                   // Invalidate the page at the address pointed to by RDI
        \\  ret                            // Return from the function after the page is invalidated

        // Set the address space by modifying CR3 (Page Table Base Register)
        \\  .global setAddressSpace
        \\  setAddressSpace:
        \\  mov rdi, [rdi]                 // Load the new page directory address into RDI
        \\  mov rax, cr3                  // Read the current value of CR3 register (Page Table Base Register)
        \\  cmp rax, rdi                  // Compare the current address with the new address in RDI
        \\  je .continuation              // If the current address matches, no need to update; jump to continuation
        \\  mov cr3, rdi                  // Update CR3 to point to the new page table base address
        \\  .continuation:
        \\  ret                            // Return from the function after setting the address space

        // Invalidate all pages in the TLB (Translation Lookaside Buffer)
        \\  .global ProcessorInvalidateAllPages
        \\  ProcessorInvalidateAllPages:
        \\  mov rax, cr4                  // Read the current value of CR4 register (control register)
        \\  and rax, ~(1 << 7)            // Clear the 7th bit of CR4 to disable global paging
        \\  mov cr4, rax                  // Write the updated value back to CR4
        \\  or rax, 1 << 7                // Set the 7th bit of CR4 to enable global paging
        \\  mov cr4, rax                  // Write the updated value back to CR4, effectively invalidating all TLB entries
        \\  ret                            // Return after invalidating all pages

        // Output a byte to the I/O port (0x80, for example)
        \\  .global out8
        \\  out8:
        \\  mov rdx, rdi                  // Move the I/O port address into RDX
        \\  mov rax, rsi                  // Move the byte data into RAX
        \\  out dx, al                    // Output the byte (AL) to the port specified by DX (I/O operation)
        \\  ret                            // Return after the I/O operation is completed
        // Read an 8-bit value from an I/O port
        \\  .global in8
        \\  in8:
        \\  mov rdx, rdi                  // Load the I/O port address into RDX (I/O port address is in RDI)
        \\  xor rax, rax                  // Clear RAX (set it to zero)
        \\  in al, dx                     // Read an 8-bit value from the I/O port (dx) into the AL register
        \\  ret                            // Return after the I/O read operation is complete
        // Write a 16-bit value to an I/O port
        \\  .global out16
        \\  out16:
        \\  mov rdx, rdi                  // Load the I/O port address into RDX
        \\  mov rax, rsi                  // Load the 16-bit value to write into RAX
        \\  out dx, ax                    // Write the lower 16 bits of RAX (AX register) to the I/O port specified in DX
        \\  ret                            // Return after the I/O write operation is complete

        // Read a 16-bit value from an I/O port
        \\  .global in16
        \\  in16:
        \\  mov rdx, rdi                  // Load the I/O port address into RDX
        \\  xor rax, rax                  // Clear RAX (set it to zero)
        \\  in ax, dx                     // Read a 16-bit value from the I/O port (dx) into AX
        \\  ret                            // Return after the I/O read operation is complete

        // Write a 32-bit value to an I/O port
        \\  .global out32
        \\  out32:
        \\  mov rdx, rdi                  // Load the I/O port address into RDX
        \\  mov rax, rsi                  // Load the 32-bit value to write into RAX
        \\  out dx, eax                   // Write the lower 32 bits of RAX (EAX register) to the I/O port specified in DX
        \\  ret                            // Return after the I/O write operation is complete

        // Read a 32-bit value from an I/O port
        \\  .global in32
        \\  in32:
        \\  mov rdx, rdi                  // Load the I/O port address into RDX
        \\  xor rax, rax                  // Clear RAX (set it to zero)
        \\  in eax, dx                    // Read a 32-bit value from the I/O port (dx) into EAX
        \\  ret                            // Return after the I/O read operation is complete
    );
}

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

    MMArchInvalidatePages(virtualAddrStart, pageCount);
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

pub export fn getTimeMS() callconv(.C) u64 {
    timeStampCounterSynchronizationValue.writeVolatile(((timeStampCounterSynchronizationValue.readVolatile() & 0x8000000000000000) ^ 0x8000000000000000) | ProcessorReadTimeStamp());
    if (ACPI.driver.HPETBaseAddr != null and ACPI.driver.HPETPeriod != 0) {
        //femtoseconds to milliseconds
        const fs2ms = 1000000000000;
        const reading: u128 = ACPI.driver.HPETBaseAddr.?[30];
        return @as(u64, @intCast(reading * ACPI.driver.HPETPeriod / fs2ms));
    }

    return ArchGetTimeFromPITMs();
}

const IO_PIT_COMMAND = 0x0043;
const IO_PIT_DATA = 0x0040;
fn initializePIT() void {
    out8(IO_PIT_COMMAND, 0x30);
    out8(IO_PIT_DATA, 0xff);
    out8(IO_PIT_DATA, 0xff);
    getTimeFromPitMsStarted = true;
    getTimeFromPitMsLast = 0xffff;
}

fn readPIT() u16 {
    out8(IO_PIT_COMMAND, 0);
    var x: u16 = in8(IO_PIT_DATA);
    x |= @as(u16, in8(IO_PIT_DATA)) << 8;
    return x;
}

export fn ArchGetTimeFromPITMs() callconv(.C) u64 {
    if (!getTimeFromPitMsStarted) {
        initializePIT();
        return 0;
    } else {
        const x = readPIT();
        getTimeFromPitMSCumulative += getTimeFromPitMsLast - x;
        if (x > getTimeFromPitMsLast) {
            getTimeFromPitMSCumulative += 0x10000;
        }
        getTimeFromPitMsLast = x;
        return getTimeFromPitMSCumulative * 1000 / 1193182;
    }
}
pub export fn nextTimer(ms: u64) callconv(.C) void {
    while (!kernel.scheduler.started.readVolatile()) {}
    getLocalStorage().?.isSchedulerReady = true;
    LAPIC.nextTimer(ms);
}

export fn MMArchInvalidatePages(startVA: u64, pageCount: u64) callconv(.C) void {
    ipiLock.acquire();
    tlbShootdownVirtualAddress.accessVolatile().* = startVA;
    tlbShootdownPageCount.accessVolatile().* = pageCount;

    ArchCallFunctionOnAllProcessors(TLBShootdownCallback, true);
    ipiLock.release();
}

fn TLBShootdownCallback() void {
    const pageCount = tlbShootdownPageCount.readVolatile();
    if (pageCount > invalidateAllPagesThreshold) {
        ProcessorInvalidateAllPages();
    } else {
        var i: u64 = 0;
        var page = tlbShootdownVirtualAddress.readVolatile();

        while (i < pageCount) : ({
            i += 1;
            page += pageSize;
        }) {
            invalidatePage(page);
        }
    }
}

const CallFunctionOnAllProcessorsCallback = fn () void;
export var callFunctionOnAllProcessorsCallback: CallFunctionOnAllProcessorsCallback = undefined;
export var callFunctionOnAllProcessorsRemaining: Volatile(u64) = undefined;

pub const IPI = struct {
    pub const yield = 0x41;
    pub const callFnOnAllProcs = 0xf0;
    pub const tlbShootdown = 0xf1;
    pub const kernelPanic = 0;

    pub fn send(interrupt: u64, nmi: bool, procID: i32) callconv(.C) u64 {
        if (interrupt != IPI.kernelPanic) ipiLock.assertLocked();

        var ignored: u64 = 0;

        for (ACPI.driver.procs[0..ACPI.driver.procCount]) |*processor| {
            if (procID != -1) {
                if (procID != processor.kernelProcessorID) {
                    ignored += 1;
                    continue;
                }
            } else {
                if (processor == getLocalStorage().?.cpu or processor.local == null or !processor.local.?.isSchedulerReady) {
                    ignored += 1;
                    continue;
                }
            }

            const destination = @as(u32, @intCast(processor.APICID)) << 24;
            const command = @as(u32, @intCast(interrupt | (1 << 14) | if (nmi) @as(u32, 0x400) else @as(u32, 0)));
            LAPIC.write(0x310 >> 2, destination);
            LAPIC.write(0x300 >> 2, command);

            while (LAPIC.read(0x300 >> 2) & (1 << 12) != 0) {}
        }

        return ignored;
    }
    pub fn sendYield(thread: *Thread) callconv(.C) void {
        thread.yieldIpiReceived.writeVolatile(false);
        ipiLock.acquire();
        _ = IPI.send(IPI.yield, false, -1);
        ipiLock.release();
        while (!thread.yieldIpiReceived.readVolatile()) {}
    }
};

const LAPIC = struct {
    fn write(reg: u32, value: u32) void {
        ACPI.driver.LAPICAddr[reg] = value;
    }

    fn read(reg: u32) u32 {
        return ACPI.driver.LAPICAddr[reg];
    }

    fn nextTimer(ms: u64) void {
        LAPIC.write(0x320 >> 2, timerInterrupt | (1 << 17));
        LAPIC.write(0x380 >> 2, @as(u32, @intCast(ACPI.driver.LAPICTicksPerMS * ms)));
    }

    fn endOfInterrupt() void {
        LAPIC.write(0xb0 >> 2, 0);
    }
};

fn ArchCallFunctionOnAllProcessors(cb: CallFunctionOnAllProcessorsCallback, includeThisProc: bool) void {
    ipiLock.assertLocked();
    if (cb == 0) std.debug.panic("Callback is null", .{});

    const cpuCount = ACPI.driver.procCount;
    if (cpuCount > 1) {
        @as(*volatile CallFunctionOnAllProcessorsCallback, @ptrCast(&callFunctionOnAllProcessorsCallback)).* = cb;
        callFunctionOnAllProcessorsRemaining.writeVolatile(cpuCount);
        const ignored = IPI.send(IPI.callFnOnAllProcs, false, -1);
        _ = callFunctionOnAllProcessorsRemaining.atomicFetchSub(ignored);
        while (callFunctionOnAllProcessorsRemaining.readVolatile() != 0) {}
    }

    if (includeThisProc) cb();
}

fn calculateNeeded(base: u64, end: u64, shifter: u64, commitArray: []const u8) u64 {
    var neededCount: u64 = 0;
    var i = base;

    const bitMask: u8 = 1;
    while (i < end) {
        const index = i >> shifter;
        if (commitArray[index >> 3] & (bitMask << @as(u3, @truncate(index & 0b111))) == 0) {
            neededCount += 1;
        }
        i += 1 << shifter;
    }

    return neededCount;
}

fn markCommitted(base: u64, end: u64, shifter: u64, commitArray: []u8) void {
    var i = base;
    const bitMask: u8 = 1;
    while (i < end) {
        const index = i >> shifter;
        commitArray[index >> 3] |= bitMask << @as(u3, @truncate(index & 0b111));
        i += 1 << shifter;
    }
}

pub export fn commitPageTables(space: *memory.AddressSpace, region: *memory.Region) callconv(.C) bool {
    _ = space.reserveMutex.assertLocked();

    const base = (region.descriptor.baseAddr - @as(u64, if (space == &kernel.coreAddressSpace) coreAddrSpaceStart else 0)) & 0x7FFFFFFFF000;
    const end = base + (region.descriptor.pageCount << pageBitCount);
    var needed: u64 = 0;

    // Calculate needed pages for L3 and L2 tables
    needed += calculateNeeded(base, end, pageBitCount + entryPerPageTableBitCount * 3, space.arch.commit.L3);
    needed += calculateNeeded(base, end, pageBitCount + entryPerPageTableBitCount * 2, space.arch.commit.L2);

    // Calculate needed pages for L1 tables
    var prevIdxL2i: u64 = std.math.maxInt(u64);
    var i = base;
    const l1Shifter: u64 = pageBitCount + entryPerPageTableBitCount;
    while (i < end) {
        const index = i >> l1Shifter;
        const idxL2i = index >> 15;

        if (space.arch.commit.commitL1[idxL2i >> 3] & (1 << @as(u3, @truncate(idxL2i & 0b111))) == 0) {
            needed += if (prevIdxL2i != idxL2i) 2 else 1;
        } else {
            if (space.arch.commit.L1[index >> 3] & (1 << @as(u3, @truncate(index & 0b111))) == 0) {
                needed += 1;
            }
        }

        prevIdxL2i = idxL2i;
        i += 1 << l1Shifter;
    }

    // Commit memory if needed
    if (needed != 0) {
        if (!memory.commit(needed * pageSize, true)) {
            return false;
        }
        space.arch.commitedPagePerTable += needed;
    }

    // Mark pages as committed for L3, L2, and L1 tables
    markCommitted(base, end, pageBitCount + entryPerPageTableBitCount * 3, space.arch.commit.L3);
    markCommitted(base, end, pageBitCount + entryPerPageTableBitCount * 2, space.arch.commit.L2);

    i = base;
    while (i < end) {
        const index = i >> l1Shifter;
        const idxL2i = index >> 15;
        space.arch.commit.commitL1[idxL2i >> 3] |= 1 << @as(u3, @truncate(idxL2i & 0b111));
        space.arch.commit.L1[index >> 3] |= 1 << @as(u3, @truncate(index & 0b111));
        i += 1 << l1Shifter;
    }

    return true;
}

pub export fn translateAddr(va: u64, hasWriteAccess: bool) callconv(.C) u64 {
    const addr = va & 0x0000FFFFFFFFF000;
    const indices = PageTables.computeIndices(addr);
    if (PageTables.access(.level4, indices).* & 1 == 0) return 0;
    if (PageTables.access(.level3, indices).* & 1 == 0) return 0;
    if (PageTables.access(.level2, indices).* & 1 == 0) return 0;

    const pa = PageTables.access(.level1, indices).*;

    if (hasWriteAccess and pa & 2 == 0) return 0;
    if (pa & 1 == 0) return 0;
    return pa & 0x0000FFFFFFFFF000;
}

const IO_PCI_CONFIG_ADDRESS = 0x0CF8;
const IO_PCI_DATA = 0x0CFC;
var pciConfigSpinlock: kernel.sync.SpinLock = undefined;

pub fn readPciConfig(bus: u8, device: u7, func: u8, offset: u8, size: 32) u32 {
    pciConfigSpinlock.acquire();
    defer pciConfigSpinlock.release();

    if (offset & 3 != 0) kernel.panic("PCI config read offset is not aligned");
    out32(IO_PCI_CONFIG_ADDRESS, 0x80000000 | ((@as(u32, @intCast(bus))) << 16) | ((@as(u32, @intCast(device))) << 11) | ((@as(u32, @intCast(func))) << 8) | @as(u32, @intCast(offset)));

    switch (size) {
        8 => return in8(IO_PCI_DATA),
        16 => return in16(IO_PCI_DATA),
        32 => return in32(IO_PCI_DATA),
        else => kernel.panic("Invalid PCI config read size"),
    }
}

pub fn writePciConfig(bus: u8, device: u8, func: u8, offset: u8, value: u32, size: u32) void {
    pciConfigSpinlock.acquire();
    defer pciConfigSpinlock.release();

    if (offset & 3 != 0) kernel.panic("PCI config read offset is not aligned");
    out32(IO_PCI_CONFIG_ADDRESS, 0x80000000 | ((@as(u32, @intCast(bus))) << 16) | ((@as(u32, @intCast(device))) << 11) | ((@as(u32, @intCast(func))) << 8) | @as(u32, @intCast(offset)));

    switch (size) {
        8 => return out8(IO_PCI_DATA, @as(u8, @intCast(value))),
        16 => return in16(IO_PCI_DATA, @as(u16, @intCast(value))),
        32 => return in32(IO_PCI_DATA, value),
        else => kernel.panic("Invalid PCI config read size"),
    }
}

const interruptVectorMSIStart = 0x70;
pub export var irqHandlersLock: kernel.sync.SpinLock = undefined;
pub export var msiHandlers: [interruptVectorMSIStart]MSIHandler = undefined;
pub const MSIHandler = extern struct {
    callback: ?KIRQHandler,
    ctx: u64,
};
pub const MSI = extern struct {
    address: u64,
    data: u64,
    tag: u64,

    pub fn register(handler: KIRQHandler, ctx: u64, ownerName: []const u8) @This() {
        _ = ownerName;
        irqHandlersLock.acquire();
        defer irqHandlersLock.release();

        for (msiHandlers, 0..) |*msiHandler, i| {
            if (msiHandler.callback != null) continue;

            msiHandler.* = MSIHandler{ .callback = handler, .ctx = ctx };

            return .{
                .address = 0xfee00000,
                .data = interruptVectorMSIStart + i,
                .tag = i,
            };
        }

        return zeroes(MSI);
    }

    pub fn unregister(tag: u64) void {
        irqHandlersLock.acquire();
        defer irqHandlersLock.release();
        msiHandlers[tag].callback = null;
    }
};
