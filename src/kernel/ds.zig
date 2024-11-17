const std = @import("std");
const panic = std.debug.panic; //TODO!: this should be replaced with a kernel panic once implemented

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
