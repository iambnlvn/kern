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

    pub fn addEntropy(self: *@This(), n: u64) void {
        self.lock.acquire();
        @setRuntimeSafety(false);
        var x = n;

        for (self.s) |*s| {
            x += 0x9E3779B97F4A7C15;
            x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9;
            x = (x ^ (x >> 27)) * 0x94D049BB133111EB;
            s.* ^= x ^ (x >> 31);
        }
        self.lock.release();
    }
};

pub fn alignu64(address: u64, alignment: u64) u64 {
    const mask = alignment - 1;
    return (address + mask) & ~mask;
}
