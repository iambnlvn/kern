const kernel = @import("../kernel.zig");

const arch = kernel.arch;
const Spinlock = kernel.sync.SpinLock;

var lock: Spinlock = undefined;
pub fn write(message: []const u8) void {
    lock.acquire();
    for (message) |c| {
        arch.debugOutByte(c);
    }
    lock.release();
}
