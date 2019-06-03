#include "connection_box.h"
#include "vtr_assert.h"

ConnectionBoxes::ConnectionBoxes() : size_(std::make_pair(0, 0)) {
}

size_t ConnectionBoxes::num_connection_box_types() const {
    return boxes_.size();
}

std::pair<size_t, size_t> ConnectionBoxes::connection_box_grid_size() const {
    return size_;
}

const ConnectionBox *ConnectionBoxes::get_connection_box(ConnectionBoxId box) const {
    if(bool(box)) {
        return nullptr;
    }

    size_t index = size_t(box);
    if(index >= boxes_.size()) {
        return nullptr;
    }

    return &boxes_.at(index);
}

bool ConnectionBoxes::find_connection_box(int inode,
        ConnectionBoxId *box_id,
        std::pair<size_t, size_t> *box_location) const {
    VTR_ASSERT(box_id != nullptr);
    VTR_ASSERT(box_location != nullptr);

    auto iter = ipin_map_.find(inode);
    if(iter == ipin_map_.end()) {
        return false;
    }

    *box_id = iter->second.first;
    *box_location = iter->second.second;
    return true;
}

// Clear IPIN map and set connection box grid size and box ids.
void ConnectionBoxes::reset_boxes(std::pair<size_t, size_t> size,
        const std::vector<ConnectionBox> boxes) {
    ipin_map_.clear();

    size_ = size;
    boxes_ = boxes;
}

void ConnectionBoxes::clear() {
    ipin_map_.clear();
    size_ = std::make_pair(0, 0);
    boxes_.clear();
}

void ConnectionBoxes::add_connection_box(int inode, ConnectionBoxId box_id,
        std::pair<size_t, size_t> box_location) {
    VTR_ASSERT(box_location.first < size_.first);
    VTR_ASSERT(box_location.second < size_.second);
    VTR_ASSERT(bool(box_id));
    VTR_ASSERT(size_t(box_id) < boxes_.size());
    auto result = ipin_map_.insert(std::make_pair(inode, std::make_pair(box_id, box_location)));
    // Verify no duplicates.
    VTR_ASSERT(result.second);
}
