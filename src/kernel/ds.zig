const std = @import("std");
const panic = std.debug.panic; //TODO!: this should be replaced with a kernel panic once implemented

pub fn zeroes(comptime T: type) T {
    var zValue: T = undefined;
    std.mem.set(u8, std.mem.asBytes(&zValue), 0);
    return zValue;
}

pub fn LinkedList(comptime T: type) type {
    return extern struct {
        first: ?*Node,
        last: ?*Node,
        count: u64,

        pub const Node = extern struct {
            previous: ?*@This(),
            next: ?*@This(),
            list: ?*LinkedList(T),
            value: ?*T,

            pub fn remove_from_list(self: *@This()) void {
                if (self.list) |list| {
                    list.remove(self);
                } else {
                    panic("list null when trying to remove node", .{});
                }
            }
        };

        pub fn prepend(self: *@This(), node: *Node) void {
            if (node.list != null) panic("inserting an node that is already in a list", .{});

            if (self.first) |first| {
                node.next = first;
                node.previous = null;
                first.previous = node;
                self.first = node;
            } else {
                self.first = node;
                self.last = node;
                node.previous = null;
                node.next = null;
            }

            self.count += 1;
            node.list = self;
            self.validate();
        }

        pub fn append(self: *@This(), node: *Node) void {
            if (node.list != null) panic("inserting an node that is already in a list", .{});

            if (self.last) |last| {
                node.previous = last;
                node.next = null;
                last.next = node;
                self.last = node;
            } else {
                self.first = node;
                self.last = node;
                node.previous = null;
                node.next = null;
            }

            self.count += 1;
            node.list = self;
            self.validate();
        }

        pub fn insertbefore(self: *@This(), node: *Node, before: *Node) void {
            if (node.list != null) panic("inserting an node that is already in a list", .{});

            if (before != self.first) {
                node.previous = before.previous;
                node.previous.?.next = node;
            } else {
                self.first = node;
                node.previous = null;
            }

            node.next = before;
            before.previous = node;

            self.count += 1;
            node.list = self;
            self.validate();
        }

        pub fn remove(self: *@This(), node: *Node) void {
            if (node.list) |list| {
                if (list != self) panic("node is in another list", .{});
            } else {
                panic("node is not in any list", .{});
            }

            if (node.previous) |previous| previous.next = node.next else self.first = node.next;

            if (node.next) |next| next.previous = node.previous else self.last = node.previous;

            node.previous = null;
            node.next = null;
            node.list = null;

            self.count -= 1;
            self.validate();
        }

        fn validate(self: *@This()) void {
            if (self.count == 0) {
                if (self.first != null or self.last != null) panic("invalid list", .{});
            } else if (self.count == 1) {
                if (self.first != self.last or
                    self.first.?.previous != null or
                    self.first.?.next != null or
                    self.first.?.list != self or
                    self.first.?.value == null)
                {
                    panic("invalid list", .{});
                }
            } else {
                if (self.first == self.last or
                    self.first.?.previous != null or
                    self.last.?.next != null)
                {
                    panic("invalid list", .{});
                }

                {
                    var node = self.first;
                    var index = self.count;

                    while (true) {
                        index -= 1;
                        if (index == 0) break;

                        if (node.?.next == node or node.?.list != self or node.?.value == null) {
                            panic("invalid list", .{});
                        }

                        node = node.?.next;
                    }

                    if (node != self.last) panic("invalid list", .{});
                }

                {
                    var node = self.last;
                    var index = self.count;

                    while (true) {
                        index -= 1;
                        if (index == 0) break;

                        if (node.?.previous == node) {
                            panic("invalid list", .{});
                        }

                        node = node.?.previous;
                    }

                    if (node != self.first) panic("invalid list", .{});
                }
            }
        }
    };
}

pub fn AVLTree(comptime T: type) type {
    return extern struct {
        root: ?*Node,
        modcheck: bool,
        longKeys: bool,

        const Tree = @This();
        const KeyDefaultType = u64;
        const Key = extern union {
            sKey: u64,
            long: extern struct {
                key: u64,
                keyByteCount: u64,
            },
        };

        pub const SearchMode = enum {
            Exact,
            SmallestAboveOrEqual,
            LargestBelowOrEqual,
        };

        pub const DuplicateKeyPolicy = enum {
            panic,
            allow,
            fail,
        };

        pub const Node = extern struct {
            value: ?*T,
            children: [2]?*Node,
            parent: ?*Node,
            tree: ?*Tree,
            key: Key,
            height: i32,

            fn rotateLeft(self: *@This()) *Node {
                const x = self;
                const y = x.children[1].?;
                const yleftchild = y.children[0];
                y.children[0] = x;
                x.children[1] = yleftchild;
                x.parent = y;
                if (yleftchild) |t| t.parent = x;

                {
                    const leftheight = if (x.children[0]) |left| left.height else 0;
                    const rightheight = if (x.children[1]) |right| right.height else 0;
                    const balance = leftheight - rightheight;
                    x.height = 1 + if (balance > 0) leftheight else rightheight;
                }

                {
                    const leftheight = if (y.children[0]) |left| left.height else 0;
                    const rightheight = if (y.children[1]) |right| right.height else 0;
                    const balance = leftheight - rightheight;
                    y.height = 1 + if (balance > 0) leftheight else rightheight;
                }

                return y;
            }

            fn rotateRight(self: *@This()) *Node {
                const y = self;
                const x = y.children[0].?;
                const yleftchild = x.children[1];
                x.children[1] = y;
                y.children[0] = yleftchild;
                y.parent = x;
                if (yleftchild) |t| t.parent = y;

                {
                    const leftheight = if (y.children[0]) |left| left.height else 0;
                    const rightheight = if (y.children[1]) |right| right.height else 0;
                    const balance = leftheight - rightheight;
                    y.height = 1 + if (balance > 0) leftheight else rightheight;
                }

                {
                    const leftheight = if (x.children[0]) |left| left.height else 0;
                    const rightheight = if (x.children[1]) |right| right.height else 0;
                    const balance = leftheight - rightheight;
                    x.height = 1 + if (balance > 0) leftheight else rightheight;
                }

                return x;
            }
            //Todo: improve arg names lol

            fn swap(self: *@This(), other: *@This()) void {
                self.parent.?.children[@intFromBool(self.parent.?.children[1] == self)] = other;
                other.parent.?.children[@intFromBool(other.parent.?.children[1] == other)] = self;

                const tempself = self.*;
                const tempother = other.*;
                self.parent = tempother.parent;
                other.parent = tempself.parent;
                self.height = tempother.height;
                other.height = tempself.height;
                self.children[0] = tempother.children[0];
                self.children[1] = tempother.children[1];
                other.children[0] = tempself.children[0];
                other.children[1] = tempself.children[1];

                if (self.children[0]) |aleft| aleft.parent = self;
                if (self.children[1]) |aright| aright.parent = self;
                if (other.children[0]) |bleft| bleft.parent = other;
                if (other.children[1]) |bright| bright.parent = other;
            }

            fn getBalanceFactor(self: *@This()) i32 {
                const leftheight = if (self.children[0]) |left| left.height else 0;
                const rightheight = if (self.children[1]) |right| right.height else 0;
                return leftheight - rightheight;
            }

            fn validate(self: *@This(), tree: *Tree, parent: ?*@This()) i32 {
                if (self.parent != parent) panic("tree panic", .{});
                if (self.tree != tree) panic("tree panic", .{});

                const leftheight = blk: {
                    if (self.children[0]) |left| {
                        if (left.compare(self) > 0) panic("invalid tree", .{});
                        break :blk left.validate(tree, self);
                    } else {
                        break :blk @as(i32, 0);
                    }
                };

                const rightheight = blk: {
                    if (self.children[1]) |right| {
                        if (right.compare(self) < 0) panic("invalid tree", .{});
                        break :blk right.validate(tree, self);
                    } else {
                        break :blk @as(i32, 0);
                    }
                };

                const height = 1 + if (leftheight > rightheight) leftheight else rightheight;
                if (height != self.height) panic("invalid tree", .{});

                return height;
            }

            fn compareKeys(key1: u64, key2: u64) i32 {
                if (key1 < key2) return -1;
                if (key1 > key2) return 1;
                return 0;
            }

            fn compare(self: *@This(), other: *@This()) i32 {
                return compareKeys(self.key.sKey, other.key.sKey);
            }
        };
    };
}
