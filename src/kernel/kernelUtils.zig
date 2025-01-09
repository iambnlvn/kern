const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;
const panic = kernel.panic;
pub fn roundUp(comptime T: type, val: T, divisor: T) T {
    return (val + divisor - 1) / divisor * divisor;
}

pub fn roundDown(comptime T: type, val: T, divisor: T) T {
    return val / divisor * divisor;
}

pub const RandomNumberGenerator = extern struct {
    state: [4]u64,
    lock: UserSpinLock,

    const UserSpinLock = extern struct {
        state: Volatile(u8),

        fn acquire(self: *@This()) void {
            @fence(.SeqCst);
            while (self.state.compareAndSwapAtom(0, 1) == null) {}
            @fence(.SeqCst);
        }

        fn release(self: *@This()) void {
            @fence(.SeqCst);
            if (self.state.readVolatile() == 0) panic("spinlock not acquired");
            self.state.writeVolatile(0);
            @fence(.SeqCst);
        }
    };
};
