#ifndef CONNECTION_BOX_H
#define CONNECTION_BOX_H
// Some routing graphs have connectivity driven by types of connection boxes.
// This class relates IPIN rr nodes with connection box type and locations, used
// for connection box driven map lookahead.

#include <tuple>
#include "vtr_strong_id.h"
#include "vtr_flat_map.h"

struct connection_box_tag {};
typedef vtr::StrongId<connection_box_tag> ConnectionBoxId;

struct ConnectionBox {
    std::string name;
};

class ConnectionBoxes {
    public:
        ConnectionBoxes();

        size_t num_connection_box_types() const;
        std::pair<size_t, size_t> connection_box_grid_size() const;
        const ConnectionBox *get_connection_box(ConnectionBoxId box) const;

        bool find_connection_box(int inode,
                ConnectionBoxId *box_id,
                std::pair<size_t, size_t> *box_location) const;

        // Clear IPIN map and set connection box grid size and box ids.
        void clear();
        void reset_boxes(std::pair<size_t, size_t> size,
                const std::vector<ConnectionBox> boxes);

        void add_connection_box(int inode, ConnectionBoxId box_id,
                std::pair<size_t, size_t> box_location);

    private:
        std::pair<size_t, size_t> size_;
        std::vector<ConnectionBox> boxes_;
        vtr::flat_map<int, std::pair<ConnectionBoxId, std::pair<size_t, size_t>>>
            ipin_map_;
};

#endif
