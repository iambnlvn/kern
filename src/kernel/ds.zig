const std = @import("std");
const panic = kernel.panic;
const zeroes = @import("kernel.zig").zeroes;
const kernel = @import("kernel.zig");
const addressSpace = kernel.addrSpace;
const Region = kernel.memory.Region;
const Heap = kernel.memory.Heap;
const heapCore = kernel.heapCore;
const heapFixed = kernel.heapFixed;
const EsMemoryZero = kernel.EsMemoryZero;
const EsMemoryMove = kernel.EsMemoryMove;
const EsMemoryCopy = kernel.EsMemoryCopy;
const EsHeapReallocate = kernel.EsHeapReallocate;

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

            pub fn removeFromList(self: *@This()) void {
                if (self.list) |list| {
                    list.remove(self);
                } else panic("list null when trying to remove node");
            }
        };

        pub fn prepend(self: *@This(), node: *Node) void {
            if (node.list != null) panic("inserting an node that is already in a list");

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
            if (node.list != null) panic("inserting an node that is already in a list");

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
            if (node.list != null) panic("inserting an node that is already in a list");

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
                if (list != self) panic("node is in another list");
            } else panic("node is not in any list");

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
                if (self.first != null or self.last != null) panic("invalid list");
            } else if (self.count == 1) {
                if (self.first != self.last or
                    self.first.?.previous != null or
                    self.first.?.next != null or
                    self.first.?.list != self or
                    self.first.?.value == null)
                {
                    panic("invalid list");
                }
            } else {
                if (self.first == self.last or
                    self.first.?.previous != null or
                    self.last.?.next != null)
                {
                    panic("invalid list");
                }

                {
                    var node = self.first;
                    var index = self.count;

                    while (true) {
                        index -= 1;
                        if (index == 0) break;

                        if (node.?.next == node or node.?.list != self or node.?.value == null) {
                            panic("invalid list");
                        }

                        node = node.?.next;
                    }

                    if (node != self.last) panic("invalid list");
                }

                {
                    var node = self.last;
                    var index = self.count;

                    while (true) {
                        index -= 1;
                        if (index == 0) break;

                        if (node.?.previous == node) {
                            panic("invalid list");
                        }

                        node = node.?.previous;
                    }

                    if (node != self.first) panic("invalid list");
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
            if (node.tree != null) {
                panic("Node already inserted in the tree. Cannot insert again.");
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
                    const cmp = n.compare(node);
                    if (cmp == 0) {
                        if (dupkeypol == .panic) panic("Duplicate key found. Insert operation failed.") else if (dupkeypol == .fail) return false;
                    }

                    const childIdx = @intFromBool(cmp > 0);
                    link = &n.children[childIdx];
                    parent = n;
                } else {
                    link.* = node;
                    node.parent = parent;
                    break;
                }
            }

            var itemIterator = node.parent;
            while (itemIterator) |item| {
                const leftHeight = if (item.children[0]) |left| left.height else 0;
                const rightHeight = if (item.children[1]) |right| right.height else 0;
                const balance = leftHeight - rightHeight;
                item.height = 1 + if (balance > 0) leftHeight else rightHeight;

                if (balance > 1) {
                    if (Node.compareKeys(key, item.children[0].?.key.sKey) <= 0) {
                        item.rotateRight();
                    } else {
                        item.children[0] = item.children[0].?.rotateLeft();
                        item.rotateRight();
                    }
                } else if (balance < -1) {
                    if (Node.compareKeys(key, item.children[1].?.key.sKey) > 0) {
                        item.rotateLeft();
                    } else {
                        item.children[1] = item.children[1].?.rotateRight();
                        item.rotateLeft();
                    }
                }

                itemIterator = item.parent;
            }

            self.root = if (self.root) |root| root else node;
            self.root.?.parent = null;

            return true;
        }

        pub fn print(self: *AVLTree) void {
            if (self.root != null) {
                self.printNode(self.root.?);
            } else {
                std.debug.print("Tree is empty.\n", .{});
            }
        }

        // Helper recursive function to print the nodes in in-order traversal
        fn printNode(self: *AVLTree, node: *Node) void {
            if (node.children[0] != null) {
                self.printNode(node.children[0].?); // Traverse left subtree
            }

            // Print the current node's value, height, and balance factor
            std.debug.print("Node Value: {d}, Height: {d}, Balance Factor: {d}\n", .{
                node.value,
                node.height,
                node.getBalanceFactor(),
            });

            if (node.children[1] != null) {
                self.printNode(node.children[1].?); // Traverse right subtree
            }
        }

        pub fn remove(self: *@This(), node: *Node) void {
            if (self.modcheck) panic("Concurrent modification detected. Tree is being modified while iterating.");
            self.modcheck = true;
            defer self.modcheck = false;

            self.validate();
            if (node.tree != self) panic("Attempt to remove a node not part of this tree.");

            var fakeRoot = zeroes(Node);
            self.root.?.parent = &fakeRoot;
            fakeRoot.tree = self;
            fakeRoot.children[0] = self.root;

            if (node.children[0] != null and node.children[1] != null) {
                const smallest = 0;
                const a = self.findRecuresively(node.children[1], smallest, .SmallestAboveOrEqual).?;
                a.swap(node);
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
                        itemIter.children[1] = itemIter.children[1].?.rotateRight();
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
                if (root.parent != &fakeRoot) panic("Root's parent mismatch after removal.");
                root.parent = null;
            }

            self.validate();
        }

        pub fn findRecursively(self: *@This(), maybeRoot: ?*Node, key: KeyDefaultType, searchMode: SearchMode) ?*Node {
            if (maybeRoot) |root| {
                const comparison = Node.compareKeys(root.key.sKey, key);

                if (comparison == 0) return root;

                switch (searchMode) {
                    .exact => return self.findRecursively(root.children[0], key, searchMode),

                    .SmallestAboveOrEqual => {
                        if (comparison > 0) {
                            return self.findRecursively(root.children[0], key, searchMode) orelse root;
                        } else {
                            return self.findRecursively(root.children[1], key, searchMode);
                        }
                    },

                    .LargestBelowOrEqual => {
                        if (comparison < 0) {
                            return self.findRecursively(root.children[1], key, searchMode) orelse root;
                        } else {
                            return self.findRecursively(root.children[0], key, searchMode);
                        }
                    },
                }
            } else {
                return null;
            }
        }

        pub fn find(self: *@This(), key: KeyDefaultType, searchMode: SearchMode) ?*Node {
            if (self.modcheck) panic("Concurrent access detected. Tree is being modified while searching.");
            self.validate();
            return self.findRecuresively(self.root, key, searchMode);
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

                const xleft = x.children[0];
                const xright = x.children[1];
                const yleft = y.children[0];
                const yright = y.children[1];

                const xleftheight = if (xleft) |left| left.height else 0;
                const xrightheight = if (xright) |right| right.height else 0;
                x.height = 1 + if (xleftheight > xrightheight) xleftheight else xrightheight;

                const yleftheight = if (yleft) |left| left.height else 0;
                const yrightheight = if (yright) |right| right.height else 0;
                y.height = 1 + if (yleftheight > yrightheight) yleftheight else yrightheight;

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

                const yleftheight = if (y.children[0]) |left| left.height else 0;
                const yrightheight = if (y.children[1]) |right| right.height else 0;
                y.height = 1 + if (yleftheight > yrightheight) yleftheight else yrightheight;

                const xleftheight = if (x.children[0]) |left| left.height else 0;
                const xrightheight = y.height; // y is now x's right child
                x.height = 1 + if (xleftheight > xrightheight) xleftheight else xrightheight;

                return x;
            }

            fn swap(self: *@This(), other: *@This()) void {
                if (self == null or other == null) {
                    return;
                }

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

            fn compareKeys(self: *@This(), otherKey: KeyDefaultType) i32 {
                if (self.key.sKey < otherKey) return -1;
                if (self.key.sKey > otherKey) return 1;
                return 0;
            }

            fn validate(self: *@This(), tree: ?*Tree, parent: ?*Node) bool {
                if (self.parent != parent) {
                    kernel.panicf("Node parent mismatch: expected {:?}, found {:?}.", .{ parent, self.parent });
                }
                if (self.tree != tree) {
                    kernel.panicf("Node tree mismatch: expected {:?}, found {:?}.", .{ tree, self.tree });
                }
                return true;
            }
        };
    };
}

pub const List = extern struct {
    prevOrLast: ?*@This(),
    nextOrFirst: ?*@This(),

    pub fn insert(self: *@This(), node: *List, start: bool) void {
        if (node.prevOrLast != null or node.nextOrFirst != null) {
            panic("bad links");
        }

        if (self.nextOrFirst == null and self.prevOrLast == null) {
            node.prevOrLast = self;
            node.nextOrFirst = self;
            self.nextOrFirst = node;
            self.prevOrLast = node;
        } else if (start) {
            node.prevOrLast = self;
            node.nextOrFirst = self.nextOrFirst;
            self.nextOrFirst.?.prevOrLast = node;
            self.nextOrFirst = node;
        } else {
            node.prevOrLast = self.prevOrLast;
            node.nextOrFirst = self;
            self.prevOrLast.?.nextOrFirst = node;
            self.prevOrLast = node;
        }
    }

    pub fn remove(self: *@This()) void {
        if (self.prevOrLast.?.nextOrFirst != self or self.nextOrFirst.?.prevOrLast != self) panic("bad links");

        if (self.prevOrLast == self.nextOrFirst) {
            self.nextOrFirst.?.nextOrFirst = null;
            self.nextOrFirst.?.prevOrLast = null;
        } else {
            self.prevOrLast.?.nextOrFirst = self.nextOrFirst;
            self.nextOrFirst.?.prevOrLast = self.prevOrLast;
        }

        self.prevOrLast = null;
        self.nextOrFirst = null;
    }
};

pub fn Bitflag(comptime EnumType: type) type {
    return extern struct {
        const IntType = std.meta.Int(.unsigned, @bitSizeOf(EnumType));
        const Enum = EnumType;

        bits: IntType,

        pub inline fn fromFlags(flags: anytype) @This() {
            const result = comptime blk: {
                const fields = std.meta.fields(@TypeOf(flags));
                if (fields.len > @bitSizeOf(EnumType)) {
                    @compileError("The number of flags exceeds the number of available bits in the EnumType.\n Ensure that the EnumType has enough bits to represent all flags.");
                }

                var bits: IntType = 0;

                for (fields) |field| {
                    const enumVal: EnumType = field.default_value.?;
                    bits |= 1 << @intFromEnum(enumVal);
                }
                break :blk bits;
            };
            return @This(){ .bits = result };
        }

        pub fn fromBits(bits: IntType) @This() {
            return @This(){ .bits = bits };
        }

        pub inline fn fromFlag(comptime flag: EnumType) @This() {
            const bits = 1 << @intFromEnum(flag);
            return @This(){ .bits = bits };
        }

        pub inline fn empty() @This() {
            return @This(){
                .bits = 0,
            };
        }

        pub inline fn all() @This() {
            const result = comptime blk: {
                var bits: IntType = 0;
                for (@typeInfo(EnumType).Enum.fields) |field| {
                    bits |= 1 << field.value;
                }
                break :blk @This(){
                    .bits = bits,
                };
            };
            return result;
        }

        pub inline fn isEmpty(self: @This()) bool {
            return self.bits == 0;
        }

        pub inline fn isAll(self: @This()) bool {
            const validBits = all().bits;
            return (self.bits & validBits) == self.bits and self.bits == validBits;
        }

        pub inline fn contains(self: @This(), comptime flag: EnumType) bool {
            return ((self.bits & (1 << @intFromEnum(flag))) >> @intFromEnum(flag)) != 0;
        }

        pub inline fn orFlag(self: @This(), comptime flag: EnumType) @This() {
            const bits = self.bits | 1 << @intFromEnum(flag);
            return @This(){ .bits = bits };
        }
        pub inline fn orFlagMut(self: *@This(), comptime flag: EnumType) void {
            self.bits |= 1 << @intFromEnum(flag);
        }
    };
}

pub const BitSet = extern struct {
    singleUsage: [*]u32,
    groupUsage: [*]u16,
    singleUsageCount: u64,
    groupdUsageCount: u64,
    const groupSize = 0x1000;
    const Self = @This();

    pub fn init(self: *Self, count: u64, mapAll: bool) void {
        self.singleUsageCount = (count + 31) & ~@as(u64, 31);
        self.groupdUsageCount = self.singleUsageCount / groupSize + 1;
        self.singleUsage = @as([*]u32, addressSpace.alloc((self.singleUsage >> 3) + (self.groupdUsageCount * 2), if (mapAll) Region.Flags.fromFlag(.fixed) else Region.Flags.empty(), 0, true));
        self.groupUsage = @as([*]u16, @intFromPtr(self.singleUsage) + ((self.singleUsageCount >> 4) * @sizeOf(u16)));
    }

    pub fn take(self: *Self, idx: u64) void {
        const group = idx / groupSize;
        self.groupUsage[group] -= 1;
        self.singleUsage[idx >> 5] &= ~(@as(u32, 1) << @as(u5, @truncate(idx)));
    }

    pub fn put(self: *Self, index: u64) void {
        self.singleUsage[index >> 5] |= @as(u32, 1) << @as(u5, @truncate(index));
        self.groupUsage[index / groupSize] += 1;
    }

    pub fn get(self: *Self, count: u64, alignment: u64, asked: u64) u64 {
        var returnVal: u64 = std.math.maxInt(u64);

        const below = blk: {
            if (asked != 0) {
                if (asked < count) return returnVal;
                break :blk asked - count;
            } else break :blk asked;
        };

        if (count == 1 and alignment == 1) {
            for (self.groupUsage[0..self.groupdUsageCount], 0..) |*grpUsage, groupIdx| {
                if (grpUsage.* != 0) {
                    var singleIdx: u64 = 0;
                    while (singleIdx < groupSize) : (singleIdx += 1) {
                        const index = groupIdx * groupSize + singleIdx;
                        if (below != 0 and index >= below) return returnVal;
                        const maskIdx = (@as(u32, 1) << @as(u5, @intCast(index)));
                        if (self.singleUsage[index >> 5] & maskIdx != 0) {
                            self.singleUsage[index >> 5] &= ~maskIdx;
                            self.groupUsage[groupIdx] -= 1;
                            return index;
                        }
                    }
                }
            }
        }
        //TODO: implement this lateeeeeeeer
        //else if (count == 16 and alignment == 16) {}
        //else if (count == 32 and alignment == 32) {}
        else {
            var found: u64 = 0;
            var start: u64 = 0;

            for (self.grpUsage[0..self.groupdUsageCount], 0..) |*grpUsage, groupIdx| {
                if (grpUsage.* == 0) {
                    found = 0;
                    continue;
                }

                var singleIdx: u64 = 0;
                while (singleIdx < groupSize) : (singleIdx += 1) {
                    const idx = groupIdx * groupSize + singleIdx;
                    const maskIdx = (@as(u32, 1) << @as(u5, @truncate(idx)));

                    if (self.singleUsage[idx >> 5] & maskIdx != 0) {
                        if (found == 0) {
                            if (idx >= below and below != 0) return returnVal;
                            if (idx % alignment != 0) continue;

                            start = idx;
                        }

                        found += 1;
                    } else {
                        found = 0;
                    }

                    if (found == count) {
                        returnVal = start;

                        var i: u64 = 0;
                        while (i < count) : (i += 1) {
                            const idxB = start + i;
                            self.singleUsage[idxB >> 5] &= ~((@as(u32, 1) << @as(u5, @truncate(idxB))));
                        }

                        return returnVal;
                    }
                }
            }
        }

        return returnVal;
    }
};

const ArrayHeader = extern struct {
    length: u64,
    allocated: u64,
};

pub export fn ArrayHeaderGet(array: ?*u64) callconv(.C) *ArrayHeader {
    return @as(*ArrayHeader, @ptrFromInt(@intFromPtr(array) - @sizeOf(ArrayHeader)));
}

pub export fn ArrayHeaderGetLength(array: ?*u64) callconv(.C) u64 {
    if (array) |arr| return ArrayHeaderGet(arr).length else return 0;
}

pub fn Array(comptime T: type, comptime heapType: HeapType) type {
    return extern struct {
        ptr: ?[*]T,
        pub fn length(self: @This()) u64 {
            return ArrayHeaderGetLength(@as(?*u64, @ptrCast(self.ptr)));
        }
        pub fn getSlice(self: @This()) []T {
            if (self.ptr) |ptr| {
                return ptr[0..self.length()];
            } else {
                panic("null array");
            }
        }

        pub fn free(self: *@This()) void {
            _ArrayFree(@as(*?*u64, @ptrCast(&self.ptr)), @sizeOf(T), getHeap());
        }

        pub fn getHeap() *Heap {
            return switch (heapType) {
                .core => &heapCore,
                .fixed => &heapFixed,
                else => unreachable,
            };
        }
        pub fn insert(self: *@This(), item: T, position: u64) ?*T {
            return @as(?*T, _ArrayInsert(@as(*?*u64, @ptrCast(self.ptr)), @intFromPtr(&item), @sizeOf(T), @as(i64, @bitCast(position)), 0, getHeap()));
        }
        pub fn getFirst(self: *@This()) *T {
            if (self.length() == 0) {
                return null;
            } else {
                return &self.ptr.?[0];
            }
        }

        pub fn getLast(self: *@This()) *T {
            if (self.length() == 0) {
                return null;
            } else {
                return &self.ptr.?[self.length() - 1];
            }
        }

        pub fn deleteMany(self: *@This(), position: u64, count: u64) void {
            _ArrayDelete(@as(?*u64, @ptrCast(self.ptr)), position, @sizeOf(T), count);
        }
    };
}

const HeapType = enum {
    core,
    fixed,
};
pub const Range = extern struct {
    from: u64,
    to: u64,
    pub const Set = extern struct {
        ranges: Array(Range, .core),
        contiguous: u64,
        const Self = @This();

        pub fn find(self: *Self, offset: u64, touching: bool) ?*Range {
            const len = self.ranges.len;
            if (len == 0) null;

            var low: i64 = 0;
            var high = @as(i64, len - 1);

            while (low <= high) {
                const idx = @divTrunc(low + (high - low), 2);
                const range = &self.ranges.ptr.?[@as(u64, @intCast(idx))];

                if (range.from <= offset and (offset < range.to or (touching and offset <= range.to))) return range else if (range.from <= offset) low = idx + 1 else high = idx - 1;
            }
            return null;
        }

        pub fn contains(self: *Self, offset: u64) bool {
            if (self.ranges.length() != 0) return self.find(offset, false) != null else return offset < self.contiguous;
        }

        pub fn normalize(self: *@This()) bool {
            if (self.contiguous != 0) {
                const oldCont = self.contiguous;
                self.contiguous = 0;

                if (!self.set(0, oldCont, null, true)) return false;
            }

            return true;
        }
        pub fn set(self: *Self, from: u64, to: u64, maybeDelta: ?*i64, modify: bool) bool {
            if (to <= from) panic("invalid range");

            const initLen = self.ranges.length();
            if (initLen == 0) {
                if (maybeDelta) |delta| {
                    if (from >= self.contiguous) delta.* = @as(i64, @intCast(to)) - @as(i64, @intCast(from)) else if (to >= self.contiguous) delta.* = @as(i64, @intCast(to)) - @as(i64, @intCast(self.contiguous)) else delta.* = 0;
                }

                if (!modify) return true;

                if (from <= self.contiguous) {
                    if (to > self.contiguous) self.contiguous = to;
                    return true;
                }

                if (!self.normalize()) return false;
            }

            const newRange = Range{
                .from = if (self.find(from, true)) |left| left.from else from,
                .to = if (self.find(to, true)) |right| right.to else to,
            };

            var i: u64 = 0;
            while (i <= self.ranges.length()) : (i += 1) {
                if (i == self.ranges.length() or self.ranges.ptr.?[i].to > newRange.from) {
                    if (modify) {
                        if (self.ranges.insert(newRange, i) == null) return false;
                        i += 1;
                    }

                    break;
                }
            }

            const deleteStart = i;
            var deleteCount: u64 = 0;
            var deleteTotal: u64 = 0;

            for (self.ranges.getSlice()[i..]) |*range| {
                const overlap = (range.from >= newRange.from and range.from <= newRange.to) or (range.to >= newRange.from and range.to <= newRange.to);

                if (overlap) {
                    deleteCount += 1;
                    deleteTotal += range.to - range.from;
                } else break;
            }

            if (modify) self.ranges.deleteMany(deleteStart, deleteCount);

            self.validate();

            if (maybeDelta) |delta| delta.* = @as(i64, @intCast(newRange.to)) - @as(i64, @intCast(newRange.from)) - @as(i64, @intCast(deleteTotal));

            return true;
        }
        pub fn clear(self: *Self, from: u64, to: u64, maybeDelta: ?*i64, modify: bool) bool {
            if (to <= from) panic("invalid range");

            if (self.ranges.length() == 0) {
                if (from < self.contiguous and self.contiguous != 0) {
                    if (to < self.contiguous) {
                        if (modify) {
                            if (!self.normalize()) return false;
                        } else {
                            if (maybeDelta) |delta| delta.* = @as(i64, @intCast(from)) - @as(i64, @intCast(to));
                            return true;
                        }
                    } else {
                        if (maybeDelta) |delta| delta.* = @as(i64, @intCast(from)) - @as(i64, @intCast(self.contiguous));
                        if (modify) self.contiguous = from;
                        return true;
                    }
                } else {
                    if (maybeDelta) |delta| delta.* = 0;
                    return true;
                }
            }

            if (self.ranges.length() == 0) {
                self.ranges.free();
                if (maybeDelta) |delta| delta.* = 0;
                return true;
            }

            if (to <= self.ranges.getFirst().from or from >= self.ranges.getLast().to) {
                if (maybeDelta) |delta| delta.* = 0;
                return true;
            }

            if (from <= self.ranges.getFirst().from and to >= self.ranges.getLast().to) {
                if (maybeDelta) |delta| {
                    var total: i64 = 0;

                    for (self.ranges.getSlice()) |range| {
                        total += @as(i64, @intCast(range.to)) - @as(i64, @intCast(range.from));
                    }

                    delta.* = -total;
                }

                if (modify) self.ranges.free();

                return true;
            }

            var overlapStart = self.ranges.length();
            var overlapCount: u64 = 0;

            for (self.ranges.getSlice(), 0..) |*range, i| {
                if (range.to > from and range.from < to) {
                    overlapStart = i;
                    overlapCount = 1;
                    break;
                }
            }

            for (self.ranges.getSlice()[overlapStart + 1 ..]) |*range| {
                if (range.to >= from and range.from < to) overlapCount += 1 else break;
            }

            const tempDelta: i64 = 0;

            if (overlapCount == 1) {
                panic("not implemented");
            } else if (overlapCount > 1) {
                panic("not implemented");
            }

            if (maybeDelta) |delta| delta.* = tempDelta;

            self.validate();
            return true;
        }

        fn validate(self: *@This()) void {
            if (self.ranges.length() == 0) return;
            var prevTo: u64 = 0;
            for (self.ranges.getSlice()) |range| {
                if (prevTo != 0 and range.from <= prevTo) panic("range in set is not placed after the prior range");
                if (range.from >= range.to) panic("Invalid range in set");

                prevTo = range.to;
            }
        }
    };
};

pub export fn _ArrayInsert(array: *?*u64, item: u64, itemSize: u64, maybePos: i64, additionalHeaderBytes: u8, heap: *Heap) callconv(.C) u64 {
    const oldArrLen = ArrayHeaderGetLength(array.*);
    const position: u64 = if (maybePos == -1) oldArrLen else @as(u64, @intCast(maybePos));

    if (maybePos < 0 or position > oldArrLen) panic("position out of bounds");

    if (!_ArraySetLength(array, oldArrLen + 1, itemSize, additionalHeaderBytes, heap)) return 0;

    const arrAddr = @intFromPtr(array.*);
    EsMemoryMove(arrAddr + itemSize * position, arrAddr + itemSize * oldArrLen, @as(i64, @intCast(itemSize)), false);
    if (item != 0) EsMemoryCopy(arrAddr + itemSize * position, item, itemSize) else EsMemoryZero(arrAddr + itemSize * position, itemSize);
    return arrAddr + itemSize * position;
}

pub export fn _ArrayMaybeInit(array: *?*u64, itemSize: u64, heap: *Heap) callconv(.C) bool {
    const newLen = 4;
    if (@as(?*ArrayHeader, @ptrFromInt(heap.allocate(@sizeOf(ArrayHeader) + itemSize * newLen, true)))) |header| {
        header.length = 0;
        header.allocated = newLen;
        array.* = @as(?*u64, @ptrFromInt(@intFromPtr(header) + @sizeOf(ArrayHeader)));
        return true;
    } else {
        return false;
    }
}
pub export fn _ArraySetLength(array: *?*u64, newLen: u64, itemSize: u64, additionalHeaderBytes: u8, heap: *Heap) callconv(.C) bool {
    if (!_ArrayMaybeInit(array, itemSize, heap)) return false;

    var header = ArrayHeaderGet(array.*);

    if (header.allocated >= newLen) {
        header.length = newLen;
        return true;
    }

    if (!_ArrayEnsureAllocated(array, if (header.allocated * 2 > newLen) header.allocated * 2 else newLen + 16, itemSize, additionalHeaderBytes, heap)) return false;

    header = ArrayHeaderGet(array.*);
    header.length = newLen;
    return true;
}

pub export fn _ArrayEnsureAllocated(array: *?*u64, minAlloc: u64, itemSize: u64, additionalHeaderBytes: u8, heap: *Heap) callconv(.C) bool {
    if (!_ArrayMaybeInit(array, itemSize, heap)) return false;

    const oldHeader = ArrayHeaderGet(array.*);

    if (oldHeader.allocated >= minAlloc) return true;

    if (@as(?*ArrayHeader, @ptrFromInt(EsHeapReallocate(@intFromPtr(oldHeader) - additionalHeaderBytes, @sizeOf(ArrayHeader) + additionalHeaderBytes + itemSize * minAlloc, false, heap)))) |newHeader| {
        newHeader.allocated = minAlloc;
        array.* = @as(?*u64, @ptrFromInt(@intFromPtr(newHeader) + @sizeOf(ArrayHeader) + additionalHeaderBytes));
        return true;
    } else {
        return false;
    }
}

pub export fn _ArrayDelete(array: ?*u64, position: u64, itemSize: u64, count: u64) callconv(.C) void {
    if (count == 0) return;

    const oldArrLen = ArrayHeaderGetLength(array);
    if (position >= oldArrLen) panic("position out of bounds");
    if (count > oldArrLen - position) panic("count out of bounds");

    ArrayHeaderGet(array).length = oldArrLen - count;
    const arrayAddress = @intFromPtr(array);

    EsMemoryMove(arrayAddress + itemSize * (position + count), arrayAddress + itemSize * oldArrLen, mMoveBackwards(u64, itemSize) * @as(i64, @intCast(count)), false);
}

pub fn mMoveBackwards(comptime T: type, addr: T) std.meta.Int(.signed, @bitSizeOf(T)) {
    return -@as(std.meta.Int(.signed, @bitSizeOf(T)), @intCast(addr));
}

pub export fn _ArrayFree(array: *?*u64, itemSize: u64, heap: *Heap) callconv(.C) void {
    if (array.* == null) return;

    heap.free(@intFromPtr(ArrayHeaderGet(array.*)), @sizeOf(ArrayHeader) + itemSize * ArrayHeaderGet(array.*).allocated);
    array.* = null;
}
