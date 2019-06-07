#include "connection_box.h"
#include "vtr_assert.h"
#include "globals.h"

ConnectionBoxes::ConnectionBoxes()
    : size_(std::make_pair(0, 0)) {
}

size_t ConnectionBoxes::num_connection_box_types() const {
    return boxes_.size();
}

std::pair<size_t, size_t> ConnectionBoxes::connection_box_grid_size() const {
    return size_;
}

const ConnectionBox* ConnectionBoxes::get_connection_box(ConnectionBoxId box) const {
    if (bool(box)) {
        return nullptr;
    }

    size_t index = size_t(box);
    if (index >= boxes_.size()) {
        return nullptr;
    }

    return &boxes_.at(index);
}

bool ConnectionBoxes::find_connection_box(int inode,
                                          ConnectionBoxId* box_id,
                                          std::pair<size_t, size_t>* box_location) const {
    VTR_ASSERT(box_id != nullptr);
    VTR_ASSERT(box_location != nullptr);

    auto iter = ipin_map_.find(inode);
    if (iter == ipin_map_.end()) {
        return false;
    }

    *box_id = iter->second.box_id;
    *box_location = iter->second.box_location;
    return true;
}

// Clear IPIN map and set connection box grid size and box ids.
void ConnectionBoxes::reset_boxes(std::pair<size_t, size_t> size,
                                  const std::vector<ConnectionBox> boxes) {
    clear();

    size_ = size;
    boxes_ = boxes;
}

void ConnectionBoxes::clear() {
    ipin_map_.clear();
    size_ = std::make_pair(0, 0);
    boxes_.clear();
    canonical_loc_map_.clear();
    sink_to_ipin_.clear();
}

void ConnectionBoxes::add_connection_box(int inode, ConnectionBoxId box_id, std::pair<size_t, size_t> box_location) {
    // Ensure that box location is in bounds
    VTR_ASSERT(box_location.first < size_.first);
    VTR_ASSERT(box_location.second < size_.second);

    // Bounds check box_id
    VTR_ASSERT(bool(box_id));
    VTR_ASSERT(size_t(box_id) < boxes_.size());

    // Make sure sink map will not be invalidated upon insertion.
    VTR_ASSERT(sink_to_ipin_.size() == 0);

    auto result = ipin_map_.insert(std::make_pair(inode, ConnBoxLoc(box_location, box_id)));
    // Verify no duplicates.
    VTR_ASSERT(result.second);
}

void ConnectionBoxes::add_canonical_loc(int inode, std::pair<size_t, size_t> loc) {
    VTR_ASSERT(loc.first < size_.first);
    VTR_ASSERT(loc.second < size_.second);
    auto result = canonical_loc_map_.insert(
        std::make_pair(inode, loc));
    // Verify no duplicates.
    VTR_ASSERT(result.second);
}

const std::pair<size_t, size_t>* ConnectionBoxes::find_canonical_loc(int inode) const {
    auto iter = canonical_loc_map_.find(inode);
    if (iter == canonical_loc_map_.end()) {
        return nullptr;
    }

    return &iter->second;
}

void ConnectionBoxes::create_sink_back_ref() {
    sink_to_ipin_.clear();

    const auto& device_ctx = g_vpr_ctx.device();

    for (const auto& connection_box : ipin_map_) {
        const auto& ipin_node = device_ctx.rr_nodes[connection_box.first];
        VTR_ASSERT(ipin_node.type() == IPIN);

        for (auto edge : ipin_node.edges()) {
            int sink_inode = ipin_node.edge_sink_node(edge);
            VTR_ASSERT(device_ctx.rr_nodes[sink_inode].type() == SINK);

            sink_to_ipin_.insert(std::make_pair(sink_inode, connection_box.first));
        }
    }
}

vtr::Range<std::multimap<int, int>::const_iterator> ConnectionBoxes::find_sink_connection_boxes(
    int inode) const {
    auto pair_range = sink_to_ipin_.equal_range(inode);
    return vtr::make_range(pair_range.first, pair_range.second);
}
