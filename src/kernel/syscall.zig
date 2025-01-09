const std = @import("std");
const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;

const arch = kernel.arch;

const AddressSpace = kernel.memory.AddressSpace;
const obj = kernel.object;
const scheduling = kernel.scheduling;
const Thread = scheduling.Thread;
const Process = scheduling.Process;

pub const Func = fn (
    arg0: u64,
    arg1: u64,
    arg2: u64,
    arg3: u64,
    currentThread: *Thread,
    currentProc: *Process,
    currentAddrSpace: *AddressSpace,
    userStackPtr: ?*u64,
    fatalErr: *u8,
) callconv(.C) u64;

pub const Type = enum(u32) {
    exit = 0,
    batch = 1,

    pub const count = std.enums.values(Type).len;
};

pub fn procExit(
    arg0: u64,
    arg1: u64,
    currentProc: *Process,
    fatalErr: *u8,
) callconv(.C) u64 {
    var isSelf = false;
    var procOut: kernel.object.Handle = undefined;
    const status = currentProc.handleTable.resolveHandle(
        &procOut,
        arg0,
        @intFromEnum(kernel.object.Type.process),
    );

    if (status == .failed) {
        fatalErr.* = @intFromEnum(kernel.FatalError.invalidHandle);
        return @intFromBool(true);
    }

    const proc = @as(*Process, @ptrCast(@as(@alignOf(Process), procOut.object)));
    defer if (status == .normal) obj.closeHandle(proc, 0);
    if (proc == currentProc) isSelf = true else scheduling.procExit(proc, @as(i32, @intCast(arg1)));

    if (isSelf) scheduling.procExit(currentProc, @as(i32, @intCast(arg1)));

    fatalErr.* = @as(u8, @bitCast(@as(i8, @intCast(kernel.ES_SUCCESS))));
    return @intFromBool(false);
}
