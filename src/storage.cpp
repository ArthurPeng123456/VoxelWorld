#include "terrain.hpp"

#include <openvdb/openvdb.h>

#include <cstring>
#include <stdexcept>
#include <unordered_map>

namespace voxel {

std::array<float, 8> Storage::corners(Key index) const {
    std::array<float, 8> result{};
    for (int i = 0; i < 8; ++i) {
        result[i] = get(index.x + (i & 1), index.y + ((i >> 1) & 1), index.z + (i >> 2));
    }
    return result;
}

namespace {

class Constant final : public Storage {
public:
    explicit Constant(float value) : value_(value) {}
    float get(int, int, int) const override { return value_; }
    std::array<float, 8> corners(Key) const override { std::array<float, 8> r; r.fill(value_); return r; }
    std::size_t bytes() const override { return sizeof(*this); }
private:
    float value_;
};

std::size_t linear(int n, int x, int y, int z) {
    return (std::size_t(z) * n + y) * n + x;
}

class Dense final : public Storage {
public:
    Dense(int count, const std::vector<float>& values, bool morton)
        : count_(count), tiles_((count + 3) / 4), morton_(morton) {
        if (!morton_) {
            values_ = values;
            return;
        }
        values_.resize(std::size_t(tiles_) * tiles_ * tiles_ * 64);
        for (int z = 0; z < count_; ++z) for (int y = 0; y < count_; ++y) {
            for (int x = 0; x < count_; ++x) values_[index(x, y, z)] = values[linear(count_, x, y, z)];
        }
    }
    float get(int x, int y, int z) const override { return values_[index(x, y, z)]; }
    std::array<float, 8> corners(Key p) const override {
        if (!morton_) {
            const auto first = linear(count_, p.x, p.y, p.z);
            const auto nextZ = first + std::size_t(count_) * count_;
            return {values_[first], values_[first + 1], values_[first + count_], values_[first + count_ + 1],
                    values_[nextZ], values_[nextZ + 1], values_[nextZ + count_], values_[nextZ + count_ + 1]};
        }
        if (((p.x | p.y | p.z) & 1) == 0) {
            const auto start = index(p.x, p.y, p.z);
            std::array<float, 8> result;
            std::copy_n(values_.data() + start, 8, result.data());
            return result;
        }
        return Storage::corners(p);
    }
    std::size_t bytes() const override { return sizeof(*this) + values_.capacity() * sizeof(float); }
private:
    std::size_t index(int x, int y, int z) const {
        if (!morton_) return linear(count_, x, y, z);
        const int offset = (x & 1) | ((y & 1) << 1) | ((z & 1) << 2)
                         | ((x & 2) << 2) | ((y & 2) << 3) | ((z & 2) << 4);
        return linear(tiles_, x >> 2, y >> 2, z >> 2) * 64 + offset;
    }
    int count_, tiles_;
    bool morton_;
    std::vector<float> values_;
};

using NodeId = std::uint32_t;
using Children = std::array<NodeId, 8>;

struct ChildrenHash {
    std::size_t operator()(const Children& children) const {
        std::size_t h = 0xcbf29ce484222325ULL;
        for (NodeId child : children) h = (h ^ child) * 0x100000001b3ULL;
        return h;
    }
};

class HashDag final : public Storage {
public:
    HashDag(int count, const std::vector<float>& values) {
        while (size_ < count) size_ *= 2;
        std::unordered_map<std::uint32_t, NodeId> leaves;
        std::unordered_map<Children, NodeId, ChildrenHash> branches;
        auto leaf = [&](float value) {
            if (value == 0) value = 0;  // Canonicalize negative zero.
            std::uint32_t bits;
            std::memcpy(&bits, &value, sizeof(bits));
            const auto [it, inserted] = leaves.try_emplace(bits, leafTag | NodeId(values_.size()));
            if (inserted) values_.push_back(value);
            return it->second;
        };
        const NodeId outside = leaf(background);
        auto build = [&](auto& self, int x, int y, int z, int size) -> NodeId {
            if (x >= count || y >= count || z >= count) return outside;
            if (size == 1) return leaf(values[linear(count, x, y, z)]);
            const int half = size / 2;
            Children children{};
            for (int i = 0; i < 8; ++i) {
                children[i] = self(self, x + (i & 1) * half, y + ((i >> 1) & 1) * half,
                                   z + (i >> 2) * half, half);
            }
            if ((children[0] & leafTag) && std::all_of(children.begin() + 1, children.end(),
                    [&](NodeId child) { return child == children[0]; })) return children[0];
            const auto [it, inserted] = branches.try_emplace(children, NodeId(nodes_.size()));
            if (inserted) nodes_.push_back(children);
            return it->second;
        };
        root_ = build(build, 0, 0, 0, size_);
        nodes_.shrink_to_fit();
        values_.shrink_to_fit();
    }
    float get(int x, int y, int z) const override {
        return sample(root_, size_ / 2, {x, y, z});
    }
    std::array<float, 8> corners(Key p) const override {
        const int differing = (p.x ^ (p.x + 1)) | (p.y ^ (p.y + 1)) | (p.z ^ (p.z + 1));
        NodeId node = root_;
        int bit = size_ / 2;
        for (; bit > differing && !(node & leafTag); bit /= 2) node = descend(node, bit, p);
        std::array<float, 8> result;
        if (node & leafTag) {
            result.fill(values_[node & ~leafTag]);
        } else if (bit == 1) {
            for (int i = 0; i < 8; ++i) result[i] = values_[nodes_[node][i] & ~leafTag];
        } else {
            for (int i = 0; i < 8; ++i) {
                result[i] = sample(node, bit, {p.x + (i & 1), p.y + ((i >> 1) & 1), p.z + (i >> 2)});
            }
        }
        return result;
    }
    std::size_t bytes() const override {
        return sizeof(*this) + nodes_.capacity() * sizeof(Children) + values_.capacity() * sizeof(float);
    }
private:
    NodeId descend(NodeId node, int bit, Key p) const {
        const int child = ((p.x & bit) != 0) | (((p.y & bit) != 0) << 1) | (((p.z & bit) != 0) << 2);
        return nodes_[node][child];
    }
    float sample(NodeId node, int bit, Key p) const {
        for (; !(node & leafTag); bit /= 2) node = descend(node, bit, p);
        return values_[node & ~leafTag];
    }
    static constexpr NodeId leafTag = NodeId(1) << 31;
    std::vector<Children> nodes_;
    std::vector<float> values_;
    NodeId root_{};
    int size_{1};
};

using ChunkGrid = openvdb::Grid<openvdb::tree::Tree4<float, 3, 2, 2>::Type>;

class Vdb final : public Storage {
public:
    Vdb(int count, const std::vector<float>& values, double spacing, Vec origin)
        : grid_(ChunkGrid::create(background)) {
        auto transform = openvdb::math::Transform::createLinearTransform(spacing);
        transform->postTranslate(openvdb::Vec3d(origin.x, origin.y, origin.z));
        grid_->setTransform(transform);
        {
            auto access = grid_->getAccessor();
            for (int z = 0; z < count; ++z) for (int y = 0; y < count; ++y) {
                for (int x = 0; x < count; ++x) {
                    const float value = values[linear(count, x, y, z)];
                    if (value == background) continue;
                    if (value == -background) access.setValueOff(openvdb::Coord(x, y, z), value);
                    else access.setValue(openvdb::Coord(x, y, z), value);
                }
            }
        }
        grid_->tree().prune();
        access_ = std::make_unique<ChunkGrid::ConstAccessor>(grid_->getConstAccessor());
    }
    float get(int x, int y, int z) const override { return access_->getValue(openvdb::Coord(x, y, z)); }
    std::array<float, 8> corners(Key p) const override {
        using Leaf = ChunkGrid::TreeType::LeafNodeType;
        if (p.x % Leaf::DIM != Leaf::DIM - 1 && p.y % Leaf::DIM != Leaf::DIM - 1 && p.z % Leaf::DIM != Leaf::DIM - 1) {
            if (const auto* leaf = access_->probeConstLeaf(openvdb::Coord(p.x, p.y, p.z))) {
                std::array<float, 8> result;
                const auto first = Leaf::coordToOffset(openvdb::Coord(p.x, p.y, p.z));
                for (int i = 0; i < 8; ++i) {
                    result[i] = leaf->getValue(first + (i & 1) * Leaf::DIM * Leaf::DIM +
                                               ((i >> 1) & 1) * Leaf::DIM + (i >> 2));
                }
                return result;
            }
        }
        return Storage::corners(p);
    }
    std::size_t bytes() const override { return sizeof(*this) + sizeof(*access_) + grid_->memUsage(); }
private:
    ChunkGrid::Ptr grid_;
    std::unique_ptr<ChunkGrid::ConstAccessor> access_;
};

}  // namespace

std::unique_ptr<Storage> makeStorage(Method method, int count, const std::vector<float>& values,
                                     bool morton, double spacing, Vec origin) {
    if (!values.empty() && std::all_of(values.begin(), values.end(), [&](float v) { return v == values.front(); }))
        return std::make_unique<Constant>(values.front());
    switch (method) {
        case Method::Mesh: return std::make_unique<Dense>(count, values, false);
        case Method::Grid: return std::make_unique<Dense>(count, values, morton);
        case Method::Dag: return std::make_unique<HashDag>(count, values);
        case Method::Vdb: {
            static const bool initialized = [] { openvdb::initialize(); return true; }();
            (void)initialized;
            return std::make_unique<Vdb>(count, values, spacing, origin);
        }
    }
    throw std::invalid_argument("Unknown terrain storage");
}

}  // namespace voxel
