pub fn Volatile(comptime T: type) type {
    return extern struct {
        value: T,

        pub inline fn readVolatile(self: *const volatile @This()) T {
            return self.value;
        }

        pub inline fn writeVolatile(self: *volatile @This(), value: T) void {
            self.value = value;
        }

        pub inline fn accessVolatile(self: *volatile @This()) *volatile T {
            return &self.value;
        }
        pub inline fn increment(self: *volatile @This()) void {
            self.writeVolatile(self.readVolatile() + 1);
        }

        pub inline fn decrement(self: *volatile @This()) void {
            self.writeVolatile(self.readVolatile() - 1);
        }
    };
}
