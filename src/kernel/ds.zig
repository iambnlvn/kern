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

        pub fn insert(self: *@This(), node: *Node, nodevalue: ?*T, key: KeyDefaultType, dupkeypol: DuplicateKeyPolicy) bool {
            self.validate();

            if (node.tree != null) {
                panic("node already in a tree\n", .{});
            }

            node.tree = self;
            node.key = zeroes(Key);
            node.key.sKey = key;
            node.children[0] = null;
            node.children[1] = null;
            node.value = nodevalue;
            node.height = 1;

            var link = &self.root;
            var parent: ?*Node = null;

            while (true) {
                if (link.*) |n| {
                    if (n.compare(node) == 0) {
                        if (dupkeypol == .panic) panic("avl duplicate panic", .{}) else if (dupkeypol == .fail) return false;
                    }

                    const childIdx = @intFromBool(n.compare(n) > 0);
                    link = &n.children[childIdx];
                    parent = n;
                } else {
                    link.* = node;
                    node.parent = parent;
                    break;
                }
            }

            var fakeRoot = zeroes(Node);
            self.root.?.parent = &fakeRoot;
            fakeRoot.tree = self;
            fakeRoot.children[0] = self.root;

            var itemIterator = node.parent.?;
            //Todo: refactor this
            while (itemIterator != &fakeRoot) {
                const leftheight = if (itemIterator.children[0]) |left| left.height else 0;
                const rightheight = if (itemIterator.children[1]) |right| right.height else 0;
                const balance = leftheight - rightheight;
                itemIterator.height = 1 + if (balance > 0) leftheight else rightheight;
                var newRoot: ?*Node = null;
                var oldParent = itemIterator.parent.?;

                if (balance > 1 and Node.compareKeys(key, itemIterator.children[0].?.key.skey) <= 0) {
                    const rightRot = itemIterator.rotateRight();
                    newRoot = rightRot;
                    const oldParentChilldIdx = @intFromBool(oldParent.children[1] == itemIterator);
                    oldParent.children[oldParentChilldIdx] = rightRot;
                } else if (balance > 1 and Node.compareKeys(key, itemIterator.children[0].?.key.skey) > 0 and itemIterator.children[0].?.children[1] != null) {
                    itemIterator.children[0] = itemIterator.children[0].?.rotateLeft();
                    itemIterator.children[0].?.parent = itemIterator;
                    const rightRot = itemIterator.rotateRight();
                    newRoot = rightRot;
                    const oldParentChilldIdx = @intFromBool(oldParent.children[1] == itemIterator);
                    oldParent.children[oldParentChilldIdx] = rightRot;
                } else if (balance < -1 and Node.compareKeys(key, itemIterator.children[1].?.key.skey) > 0) {
                    const leftRotation = itemIterator.rotateLeft();
                    newRoot = leftRotation;
                    const oldParentChilldIdx = @intFromBool(oldParent.children[1] == itemIterator);
                    oldParent.children[oldParentChilldIdx] = leftRotation;
                } else if (balance < -1 and Node.compareKeys(key, itemIterator.children[1].?.key.skey) <= 0 and itemIterator.children[1].?.children[0] != null) {
                    itemIterator.children[1] = itemIterator.children[1].?.rotateRight();
                    itemIterator.children[1].?.parent = itemIterator;
                    const leftRotation = itemIterator.rotateLeft();
                    newRoot = leftRotation;
                    const oldParentChilldIdx = @intFromBool(oldParent.children[1] == itemIterator);
                    oldParent.children[oldParentChilldIdx] = leftRotation;
                }

                if (newRoot) |newRootUnwrapped| newRootUnwrapped.parent = oldParent;
                itemIterator = oldParent;
            }

            self.root = fakeRoot.children[0];
            self.root.?.parent = null;

            self.validate();
            return true;
        }
        // Todo: refactor this
        pub fn remove(self: *@This(), node: *Node) void {
            if (self.modcheck) panic("concurrent modification", .{});
            self.modcheck = true;
            defer self.modcheck = false;

            self.validate();
            if (node.tree != self) panic("node not in tree", .{});

            var fakeRoot = zeroes(Node);
            self.root.?.parent = &fakeRoot;
            fakeRoot.tree = self;
            fakeRoot.children[0] = self.root;

            if (node.children[0] != null and node.children[1] != null) {
                const smallest = 0;
                const a = self.findRecuresively(node.children[1], smallest, .SmallestAboveOrEqual).?;
                const b = node;
                a.swap(b);
            }

            const link = &node.parent.?.children[@intFromBool(node.parent.?.children[1] == node)];
            link.* = if (node.children[0]) |left| left else node.children[1];

            node.tree = null;
            var itemIter = blk: {
                if (link.*) |linkUnwrapped| {
                    linkUnwrapped.parent = node.parent;
                    break :blk link.*.?;
                } else break :blk node.parent.?;
            };

            while (itemIter != &fakeRoot) {
                const leftHeight = if (itemIter.children[0]) |left| left.height else 0;
                const rightHeight = if (itemIter.children[1]) |right| right.height else 0;
                const balance = leftHeight - rightHeight;
                itemIter.height = 1 + if (balance > 0) leftHeight else rightHeight;

                var newRoot: ?*Node = null;
                var oldParent = itemIter.parent.?;

                if (balance > 1) {
                    const leftBalance = if (itemIter.children[0]) |left| left.getBalanceFactor() else 0;
                    if (leftBalance >= 0) {
                        const rightRotation = itemIter.rotateRight();
                        newRoot = rightRotation;
                        const oldParentChildIdx = @intFromBool(oldParent.children[1] == itemIter);
                        oldParent.children[oldParentChildIdx] = rightRotation;
                    } else {
                        itemIter.children[0] = itemIter.children[0].?.rotateLeft();
                        itemIter.children[0].?.parent = itemIter;
                        const rightRotation = itemIter.rotateRight();
                        newRoot = rightRotation;
                        const oldParentChildIdx = @intFromBool(oldParent.children[1] == itemIter);
                        oldParent.children[oldParentChildIdx] = rightRotation;
                    }
                } else if (balance < -1) {
                    const rightBalance = if (itemIter.children[1]) |left| left.getBalanceFactor() else 0;
                    if (rightBalance <= 0) {
                        const leftRotation = itemIter.rotateLeft();
                        newRoot = leftRotation;
                        const oldParentChildIdx = @intFromBool(oldParent.children[1] == itemIter);
                        oldParent.children[oldParentChildIdx] = leftRotation;
                    } else {
                        itemIter.children[1] = itemIter.children[1].?.rotate_right();
                        itemIter.children[1].?.parent = itemIter;
                        const leftRotation = itemIter.rotateLeft();
                        newRoot = leftRotation;
                        const oldParentChildIdx = @intFromBool(oldParent.children[1] == itemIter);
                        oldParent.children[oldParentChildIdx] = leftRotation;
                    }
                }

                if (newRoot) |newRootUnwrapped| newRootUnwrapped.parent = oldParent;
                itemIter = oldParent;
            }

            self.root = fakeRoot.children[0];
            if (self.root) |root| {
                if (root.parent != &fakeRoot) panic("incorrect root parent", .{});
                root.parent = null;
            }

            self.validate();
        }

        pub fn findRecuresively(self: *@This(), maybeRoot: ?*Node, key: KeyDefaultType, searchMode: SearchMode) ?*Node {
            if (maybeRoot) |root| {
                if (Node.compareKeys(root.key.sKey, key) == 0) return root;

                switch (searchMode) {
                    .exact => return self.findRecuresively(root.children[0], key, searchMode),

                    .SmallestAboveOrEqual => {
                        if (Node.compareKeys(root.key.sKey, key) > 0) {
                            if (self.findRecuresively(root.children[0], key, searchMode)) |node| return node else return root;
                        } else {
                            return self.findRecuresively(root.children[1], key, searchMode);
                        }
                    },

                    .LargestBelowOrEqual => {
                        if (Node.compareKeys(root.key.sKey, key) < 0) {
                            if (self.findRecuresively(root.children[1], key, searchMode)) |node| return node else return root;
                        } else return self.findRecuresively(root.children[0], key, searchMode);
                    },
                }
            } else {
                return null;
            }
        }
        fn validate(self: *@This()) void {
            if (self.root) |root| {
                _ = root.validate(self, null);
            } else {
                return;
            }
        }

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
