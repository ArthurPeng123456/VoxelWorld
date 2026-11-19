#include "terrain.hpp"

#include <openvdb/openvdb.h>

#include <cstring>
#include <stdexcept>
#include <unordered_map>

namespace voxel {
namespace {

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
        NodeId node = root_;
        for (int bit = size_ / 2; !(node & leafTag); bit /= 2) {
            const int child = ((x & bit) != 0) | (((y & bit) != 0) << 1) | (((z & bit) != 0) << 2);
            node = nodes_[node][child];
        }
        return values_[node & ~leafTag];
    }
    std::size_t bytes() const override {
        return sizeof(*this) + nodes_.capacity() * sizeof(Children) + values_.capacity() * sizeof(float);
    }
private:
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
    std::size_t bytes() const override { return sizeof(*this) + sizeof(*access_) + grid_->memUsage(); }
private:
    ChunkGrid::Ptr grid_;
    std::unique_ptr<ChunkGrid::ConstAccessor> access_;
};

}  // namespace

std::unique_ptr<Storage> makeStorage(Method method, int count, const std::vector<float>& values,
                                     bool morton, double spacing, Vec origin) {
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
