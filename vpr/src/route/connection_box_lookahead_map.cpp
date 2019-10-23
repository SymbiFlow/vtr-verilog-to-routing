#include "connection_box_lookahead_map.h"

#include <vector>
#include <queue>

#include "connection_box.h"
#include "rr_node.h"
#include "router_lookahead_map_utils.h"
#include "globals.h"
#include "vtr_math.h"
#include "vtr_time.h"
#include "echo_files.h"

#include "route_timing.h"
#include "route_tree_timing.h"

#ifdef VTR_ENABLE_CAPNPROTO
#    include "capnp/serialize.h"
#    include "connection_map.capnp.h"
#    include "ndmatrix_serdes.h"
#    include "mmap_file.h"
#    include "serdes_utils.h"
#endif

/* we're profiling routing cost over many tracks for each wire type, so we'll
 * have many cost entries at each |dx|,|dy| offset. There are many ways to
 * "boil down" the many costs at each offset to a single entry for a given
 * (wire type, chan_type) combination we can take the smallest cost, the
 * average, median, etc. This define selects the method we use.
 *
 * See e_representative_entry_method */
#define REPRESENTATIVE_ENTRY_METHOD SMALLEST

#define REF_X 25
#define REF_Y 23

static int signum(int x) {
    if (x > 0) return 1;
    if (x < 0)
        return -1;
    else
        return 0;
}

/* Tuple key is: segment, connection_box, dx, dy */
typedef std::tuple<int, ConnectionBoxId, int, int> routing_data_key_t;

template<class T>
class HashTuple;

template<>
class HashTuple<routing_data_key_t> {
  public:
    std::size_t operator()(routing_data_key_t const& s) const {
        std::size_t h = std::hash<int>()(std::get<0>(s));
        vtr::hash_combine<size_t>(h, size_t(std::get<1>(s)));
        vtr::hash_combine<int>(h, std::get<2>(s));
        vtr::hash_combine<int>(h, std::get<3>(s));
        return h;
    }
};

typedef std::unordered_map<routing_data_key_t, Expansion_Cost_Entry, HashTuple<routing_data_key_t>>
    routing_data_t;

// Storage of connection box lookahead costs.
class CostMap {
  public:
    // Resize data to account for number of segments and connection boxes.
    //
    // Builds lookup between rr node index and segment.
    void set_counts(size_t seg_count, size_t box_count) {
        cost_map_.clear();
        offset_.clear();
        cost_map_.resize({seg_count, box_count});
        offset_.resize({seg_count, box_count});

        const auto& device_ctx = g_vpr_ctx.device();
        segment_map_.resize(device_ctx.rr_nodes.size());
        for (size_t i = 0; i < segment_map_.size(); ++i) {
            auto& from_node = device_ctx.rr_nodes[i];

            int from_cost_index = from_node.cost_index();
            int from_seg_index = device_ctx.rr_indexed_data[from_cost_index].seg_index;

            segment_map_[i] = from_seg_index;
        }
    }

    // Map node id to segment id.
    int node_to_segment(int from_node_ind) {
        return segment_map_[from_node_ind];
    }

    // Find the cost entry for a given segment to connection box, and distance from segment to connection box.
    Cost_Entry find_cost(int from_seg_index, ConnectionBoxId box_id, int delta_x, int delta_y) const {
        VTR_ASSERT(from_seg_index >= 0 && from_seg_index < (ssize_t)offset_.size());
        int dx = delta_x - offset_[from_seg_index][size_t(box_id)].first;
        int dy = delta_y - offset_[from_seg_index][size_t(box_id)].second;
        const auto& cost_map = cost_map_[from_seg_index][size_t(box_id)];

        if (dx < 0) {
            dx = 0;
        }
        if (dy < 0) {
            dy = 0;
        }

        if (dx >= (ssize_t)cost_map.dim_size(0)) {
            dx = cost_map.dim_size(0) - 1;
        }
        if (dy >= (ssize_t)cost_map.dim_size(1)) {
            dy = cost_map.dim_size(1) - 1;
        }

        return cost_map_[from_seg_index][size_t(box_id)][dx][dy];
    }

    // Set cost map from sparse hash table of data.
    void set_cost_map(const routing_data_t& data, e_representative_entry_method method) {
        vtr::NdMatrix<t_bb, 2> bounds({cost_map_.dim_size(0), cost_map_.dim_size(1)},
                                      {0, 0, 0, 0});

        // Find coordinate offset for this segment.
        for (const auto& entry : data) {
            t_bb& bound = bounds[std::get<0>(entry.first)][size_t(std::get<1>(entry.first))];

            bound.xmin = std::min(std::get<2>(entry.first), bound.xmin);
            bound.xmax = std::max(std::get<2>(entry.first), bound.xmax);

            bound.ymin = std::min(std::get<3>(entry.first), bound.ymin);
            bound.ymax = std::max(std::get<3>(entry.first), bound.ymax);
        }

        // Allocate cost matricies based on bounds.
        for (size_t from_seg_index = 0; from_seg_index < cost_map_.dim_size(0); ++from_seg_index) {
            for (size_t box_id = 0; box_id < cost_map_.dim_size(1); ++box_id) {
                const t_bb& bound = bounds[from_seg_index][box_id];
                offset_[from_seg_index][box_id].first = bound.xmin;
                offset_[from_seg_index][box_id].second = bound.ymin;
                size_t dim_x = bound.xmax - bound.xmin + 1;
                size_t dim_y = bound.ymax - bound.ymin + 1;

                cost_map_[from_seg_index][size_t(box_id)] = vtr::NdMatrix<Cost_Entry, 2>(
                    {dim_x, dim_y});
            }
        }

        /* set the lookahead cost map entries with a representative cost
         * entry from routing_cost_map */
        for (const auto& entry : data) {
            auto from_seg_index = std::get<0>(entry.first);
            auto box_id = std::get<1>(entry.first);
            const auto& offset = offset_[from_seg_index][size_t(box_id)];
            unsigned ix = std::get<2>(entry.first) - offset.first;
            unsigned iy = std::get<3>(entry.first) - offset.second;
            cost_map_[from_seg_index][size_t(box_id)][ix][iy] = entry.second.get_representative_cost_entry(method);
        }

        /* find missing cost entries and fill them in by copying a nearby cost entry */
        for (size_t from_seg_index = 0; from_seg_index < cost_map_.dim_size(0); ++from_seg_index) {
            for (size_t box_id = 0; box_id < cost_map_.dim_size(1); ++box_id) {
                const auto& offset = offset_[from_seg_index][size_t(box_id)];
                auto& matrix = cost_map_[from_seg_index][size_t(box_id)];
                for (unsigned ix = 0; ix < matrix.dim_size(0); ix++) {
                    for (unsigned iy = 0; iy < matrix.dim_size(1); iy++) {
                        Cost_Entry cost_entry = matrix[ix][iy];

                        if (!cost_entry.valid()) {
                            Cost_Entry copied_entry = get_nearby_cost_entry(
                                from_seg_index,
                                ConnectionBoxId(box_id),
                                offset.first + ix,
                                offset.second + iy);
                            matrix[ix][iy] = copied_entry;
                        }
                    }
                }
            }
        }
    }

    Cost_Entry get_nearby_cost_entry(int segment_index, ConnectionBoxId box_id, int x, int y) {
        /* compute the slope from x,y to 0,0 and then move towards 0,0 by one
         * unit to get the coordinates of the cost entry to be copied */

        float slope;
        int copy_x, copy_y;
        if (x == 0 || y == 0) {
            slope = std::numeric_limits<float>::infinity();
            copy_x = x - signum(x);
            copy_y = y - signum(y);
        } else {
            slope = (float)y / (float)x;
            if (slope >= 1.0) {
                copy_y = y - signum(y);
                copy_x = vtr::nint((float)y / slope);
            } else {
                copy_x = x - signum(x);
                copy_y = vtr::nint((float)x * slope);
            }
        }

        Cost_Entry copy_entry = find_cost(segment_index, box_id, copy_x, copy_y);

        /* if the entry to be copied is also empty, recurse */
        if (copy_entry.valid()) {
            return copy_entry;
        } else if (copy_x == 0 && copy_y == 0) {
            return Cost_Entry();
        }

        return get_nearby_cost_entry(segment_index, box_id, copy_x, copy_y);
    }

    void read(const std::string& file);
    void write(const std::string& file) const;

  private:
    vtr::NdMatrix<vtr::NdMatrix<Cost_Entry, 2>, 2> cost_map_;
    vtr::NdMatrix<std::pair<int, int>, 2> offset_;
    std::vector<int> segment_map_;
};

static CostMap g_cost_map;

// Search grid used for sampling segments from graph.
//
// SearchGrid divides the graph based on segment type, and within each
// segment type defines a NxN uniform search grid, where N is kTargetGridSize.
//
// SearchGrid needs to be initialized with the setup and add_node methods.
//
// Once initialized, next_node can be invoked to determine next rr node to
// expand.
//
// During expansion, visit_entry and expanded should be called to indicate
// that an expansion found data relevant to the search.
class SearchGrid {
    // Representation of one search area of the grid.
    struct GridEntry {
        GridEntry()
            : samples(0)
            , bounds(vtr::Rect<int>(-1, -1, -1, -1)) {}

        // Number of samples take from this section of the grid.
        int samples;

        // Geometric boundry of this section of the grid.
        vtr::Rect<int> bounds;

        // rr nodes that belong to this section of the grid.
        std::vector<int> nodes;
    };

    // Represent of one segment (element) type within the grid.
    struct ElementDefinition {
        // Bounding box that contains all instances of a particular segment type.
        t_bb box;

        // Stride is choosen such that dim_size(0)*x_stride and
        // dim_size(1)*y_stride encompass box (e.g. the grid includes all
        // elements).
        size_t x_stride;
        size_t y_stride;

        // Search grid for this element.
        vtr::NdMatrix<GridEntry, 2> grid;
    };

  public:
    // Initial search grid with bound boxes of each search instance, and total
    // number of nodes that could be expanded.
    void setup(const std::vector<t_bb>& grid_elements, size_t num_nodes) {
        expanded_.resize(num_nodes, false);

        constexpr int kTargetGridSize = 3;
        elements_.resize(grid_elements.size());

        for (size_t i = 0; i < grid_elements.size(); ++i) {
            elements_[i].box = grid_elements[i];
            if (grid_elements[i].xmin > grid_elements[i].xmax) {
                continue;
            }

            VTR_ASSERT(grid_elements[i].ymin <= grid_elements[i].ymax);

            size_t x_dim = std::min(kTargetGridSize, grid_elements[i].xmax - grid_elements[i].xmin + 1);
            size_t y_dim = std::min(kTargetGridSize, grid_elements[i].ymax - grid_elements[i].ymin + 1);

            // Choose strides to encompass all elements.
            size_t x_stride = (grid_elements[i].xmax - grid_elements[i].xmin + 1) / x_dim;
            while (x_stride * x_dim + grid_elements[i].xmin <= grid_elements[i].xmax) {
                x_stride += 1;
            }
            VTR_ASSERT(x_stride * x_dim + grid_elements[i].xmin > grid_elements[i].xmax);

            size_t y_stride = (grid_elements[i].ymax - grid_elements[i].ymin + 1) / y_dim;
            while (y_stride * y_dim + grid_elements[i].ymin <= grid_elements[i].ymax) {
                y_stride += 1;
            }
            VTR_ASSERT(y_stride * y_dim + grid_elements[i].ymin > grid_elements[i].ymax);

            elements_[i].x_stride = x_stride;
            elements_[i].y_stride = y_stride;
            elements_[i].grid.resize({x_dim, y_dim});

            // Initial search grid to contain all elements.
            for (size_t ix = 0; ix < x_dim; ++ix) {
                for (size_t iy = 0; iy < y_dim; ++iy) {
                    auto& entry = elements_[i].grid[ix][iy];
                    entry.bounds = vtr::Rect<int>(
                        grid_elements[i].xmin + ix * x_stride,
                        grid_elements[i].ymin + iy * y_stride,
                        grid_elements[i].xmin + (ix + 1) * x_stride,
                        grid_elements[i].ymin + (iy + 1) * y_stride);
                }
            }

            // Check that grid includes all elements.
            for (int x = grid_elements[i].xmin; x <= grid_elements[i].xmax; ++x) {
                for (int y = grid_elements[i].ymin; y <= grid_elements[i].ymax; ++y) {
                    auto grid_entry = find_grid_entry(i, x, y);
                    VTR_ASSERT(grid_entry.bounds.contains(vtr::Point<int>(x, y)));
                }
            }
        }

        reset_iteration();
    }

    // Store rr node in search grid.
    void add_node(size_t element, int x, int y, int node) {
        find_grid_entry(element, x, y).nodes.push_back(node);
    }

    // Mark that rr node has been expanded, and will not need to be expanded
    // again.
    void expanded(int node) {
        VTR_ASSERT(node < expanded_.size());
        expanded_[node] = true;
    }

    // Mark that part of the search grid has been searched.
    void visit_entry(size_t element, int x, int y) {
        auto& grid_entry = find_grid_entry(element, x, y);
        if (grid_entry.samples == 0) {
            VTR_LOG("First sample for %zu for grid at %d, %d (sampled at %d, %d)\n",
                    element, grid_entry.bounds.xmin(), grid_entry.bounds.ymin(),
                    x, y);
        }
        grid_entry.samples += 1;
    }

    // Get next rr node to visit in search pattern.
    //
    // Returns OPEN if no more nodes.
    int next_node() {
        if (done()) {
            return OPEN;
        }

        while (!is_next_node_valid() ||
               // Have samples from this grid location?
               elements_[next_element_].grid[next_ix_][next_iy_].samples > 0 ||
               // Have already expanded this node?
               expanded_[elements_[next_element_].grid[next_ix_][next_iy_].nodes[next_node_]]) {
            advance_one();
            if (done()) {
                return OPEN;
            }
        }

        VTR_ASSERT(next_element_ < elements_.size());
        VTR_ASSERT(next_ix_ < elements_[next_element_].grid.dim_size(0));
        VTR_ASSERT(next_iy_ < elements_[next_element_].grid.dim_size(1));
        VTR_ASSERT(next_node_ < elements_[next_element_].grid[next_ix_][next_iy_].nodes.size());
        int next_node = elements_[next_element_].grid[next_ix_][next_iy_].nodes[next_node_];
        advance_one();

        return next_node;
    }

  private:
    // Is next_node valid to read?
    bool is_next_node_valid() {
        return next_element_ < elements_.size() && next_ix_ < elements_[next_element_].grid.dim_size(0) && next_iy_ < elements_[next_element_].grid.dim_size(1) && next_node_ < elements_[next_element_].grid[next_ix_][next_iy_].nodes.size();
    }

    // Is done with iteration?
    bool done() {
        return next_element_ >= elements_.size();
    }

    // Reset counters so that iteration via next_node will visit all nodes.
    void reset_iteration() {
        next_element_ = 0;
        next_ix_ = 0;
        next_iy_ = 0;
        next_node_ = 0;
    }

    // Advance iteration one step.
    //
    // May be invalid, check step is valid with is_next_node_valid().
    void advance_one() {
        if (done()) {
            return;
        }

        if (next_ix_ >= elements_[next_element_].grid.dim_size(0) || next_iy_ >= elements_[next_element_].grid.dim_size(1)) {
            VTR_ASSERT(next_ix_ == 0);
            VTR_ASSERT(next_iy_ == 0);
            VTR_ASSERT(next_node_ == 0);
            next_element_ += 1;
            return;
        }

        if (next_node_ + 1 < elements_[next_element_].grid[next_ix_][next_iy_].nodes.size()) {
            next_node_ += 1;
            return;
        }

        next_node_ = 0;

        if (next_iy_ + 1 < elements_[next_element_].grid.dim_size(1)) {
            next_iy_ += 1;
            return;
        }

        next_iy_ = 0;

        if (next_ix_ + 1 < elements_[next_element_].grid.dim_size(0)) {
            next_ix_ += 1;
            return;
        }

        next_ix_ = 0;
        next_element_ += 1;
    }

    // Return grid entry for given search location.
    GridEntry& find_grid_entry(size_t element, size_t x, size_t y) {
        VTR_ASSERT(element < elements_.size());
        VTR_ASSERT(x >= elements_[element].box.xmin);
        VTR_ASSERT(x <= elements_[element].box.xmax);
        VTR_ASSERT(y >= elements_[element].box.ymin);
        VTR_ASSERT(y <= elements_[element].box.ymax);

        size_t ix = (x - elements_[element].box.xmin) / elements_[element].x_stride;
        size_t iy = (y - elements_[element].box.ymin) / elements_[element].y_stride;

        VTR_ASSERT(ix <= elements_[element].grid.dim_size(0));
        VTR_ASSERT(iy <= elements_[element].grid.dim_size(1));

        return elements_[element].grid[ix][iy];
    }

    // Search grids for each element.
    std::vector<ElementDefinition> elements_;

    // Location of next node to return, may be invalid, check with
    // is_next_node_valid() and done().
    size_t next_element_;
    size_t next_ix_;
    size_t next_iy_;
    size_t next_node_;

    std::vector<uint8_t> expanded_;
};

// Storage for expanding path during expand_from_node.
struct CostCongestion {
    int inode;
    float delay;
    float congestion_upstream;
};

// Perform expansion from src_rr_node, and store all costs from expanded nodes
// to IPINs.
void expand_from_node(int src_rr_node, SearchGrid* grid, routing_data_t* data) {
    grid->expanded(src_rr_node);
    const auto& device_ctx = g_vpr_ctx.device();
    const auto& route_ctx = g_vpr_ctx.routing();

    t_rt_node* rt_root = init_route_tree_to_source_no_net(src_rr_node);

    t_bb bounding_box;
    bounding_box.xmin = 0;
    bounding_box.xmax = device_ctx.grid.width() + 1;
    bounding_box.ymin = 0;
    bounding_box.ymax = device_ctx.grid.height() + 1;

    t_conn_cost_params cost_params;
    cost_params.criticality = 1.;
    cost_params.astar_fac = 0;
    cost_params.bend_cost = 0;

    std::vector<int> modified_rr_node_inf;
    RouterStats router_stats;

    init_heap(device_ctx.grid);

    // Perform expansion starting from src_rr_node.
    std::vector<t_heap> shortest_paths = timing_driven_find_all_shortest_paths_from_route_tree(rt_root,
                                                                                               cost_params,
                                                                                               bounding_box,
                                                                                               modified_rr_node_inf,
                                                                                               router_stats);
    free_route_tree(rt_root);

    VTR_ASSERT(shortest_paths.size() == device_ctx.rr_nodes.size());

    // Collect data from all IPIN nodes.
    std::vector<CostCongestion> path_cost;
    int num_nodes = device_ctx.rr_nodes.size();
    for (int sink_rr_node = 0; sink_rr_node < num_nodes; ++sink_rr_node) {
        if (sink_rr_node == src_rr_node) {
            continue;
        }
        if (shortest_paths[sink_rr_node].index == OPEN) {
            continue;
        }

        VTR_ASSERT(shortest_paths[sink_rr_node].index == sink_rr_node);

        // Mark that this rr node has been expanded.
        grid->expanded(sink_rr_node);

        if (device_ctx.rr_nodes[sink_rr_node].type() != IPIN) {
            continue;
        }

        ConnectionBoxId box_id;
        std::pair<size_t, size_t> box_location;
        bool found = device_ctx.connection_boxes.find_connection_box(
            sink_rr_node, &box_id, &box_location);
        if (!found) {
            VPR_THROW(VPR_ERROR_ROUTE, "No connection box for IPIN %d", sink_rr_node);
        }

        // Work from IPIN back to source.
        size_t path_length = 1;
        int current_node = sink_rr_node;
        while (current_node != src_rr_node) {
            path_length += 1;

            current_node = route_ctx.rr_node_route_inf[current_node].prev_node;
            VTR_ASSERT(current_node != OPEN);
        }

        path_cost.resize(path_length);

        current_node = sink_rr_node;
        size_t i = 0;
        while (current_node != src_rr_node) {
            i += 1;

            path_cost[path_length - i].inode = current_node;
            current_node = route_ctx.rr_node_route_inf[current_node].prev_node;
            VTR_ASSERT(current_node != OPEN);
        }

        // Now that path is defined, work from source by to IPIN and propigate
        // delays.
        VTR_ASSERT(i + 1 == path_length);
        VTR_ASSERT(current_node == src_rr_node);
        path_cost[0].inode = src_rr_node;
        path_cost[0].delay = 0.f;
        path_cost[0].congestion_upstream = 0.f;

        for (i = 1; i < path_length; ++i) {
            const auto& parent_node = device_ctx.rr_nodes[path_cost[i - 1].inode];
            int parent_edge = route_ctx.rr_node_route_inf[path_cost[i].inode].prev_edge;
            VTR_ASSERT(parent_node.edge_sink_node(parent_edge) == path_cost[i].inode);
            int switch_ind = parent_node.edge_switch(parent_edge);

            propigate_delay(
                path_cost[i].inode,
                switch_ind,
                path_cost[i - 1].delay,
                path_cost[i - 1].congestion_upstream,
                &path_cost[i].delay,
                &path_cost[i].congestion_upstream);
        }

        // For each CHANX or CHANY, add a sample.
        for (i = 0; i < path_length - 1; ++i) {
            const auto& node = device_ctx.rr_nodes[path_cost[i].inode];

            if (node.type() != CHANX && node.type() != CHANY) {
                continue;
            }

            const std::pair<size_t, size_t>* from_canonical_loc = device_ctx.connection_boxes.find_canonical_loc(path_cost[i].inode);
            if (from_canonical_loc == nullptr) {
                continue;
            }

            int delta_x = ssize_t(from_canonical_loc->first) - ssize_t(box_location.first);
            int delta_y = ssize_t(from_canonical_loc->second) - ssize_t(box_location.second);

            int node_cost_ind = device_ctx.rr_nodes[path_cost[i].inode].cost_index();
            int node_seg_ind = device_ctx.rr_indexed_data[node_cost_ind].seg_index;
            (*data)[std::make_tuple(node_seg_ind, box_id, delta_x, delta_y)].add_cost_entry(
                REPRESENTATIVE_ENTRY_METHOD,
                path_cost.back().delay - path_cost[i].delay,
                path_cost.back().congestion_upstream - path_cost[i].congestion_upstream);
            // Mark that a sample from this part of the grid was found.
            grid->visit_entry(node_seg_ind,
                              device_ctx.rr_nodes[path_cost[i].inode].xlow(),
                              device_ctx.rr_nodes[path_cost[i].inode].ylow());
        }
    }

    reset_path_costs(modified_rr_node_inf);
    empty_heap();
}

static const std::vector<int>& get_rr_node_indcies(t_rr_type rr_type, int start_x, int start_y) {
    const auto& device_ctx = g_vpr_ctx.device();
    if (rr_type == CHANX) {
        return device_ctx.rr_node_indices[rr_type][start_y][start_x][0];
    } else if (rr_type == CHANY) {
        return device_ctx.rr_node_indices[rr_type][start_x][start_y][0];
    } else {
        VPR_FATAL_ERROR(VPR_ERROR_ROUTE, "Unknown channel type %d", rr_type);
    }
}

class StartNode {
  public:
    StartNode(int start_x, int start_y, t_rr_type rr_type, int seg_index)
        : start_x_(start_x)
        , start_y_(start_y)
        , rr_type_(rr_type)
        , seg_index_(seg_index)
        , index_(0) {}
    int get_next_node() {
        const auto& device_ctx = g_vpr_ctx.device();
        const std::vector<int>& channel_node_list = get_rr_node_indcies(
            rr_type_, start_x_, start_y_);

        for (; index_ < channel_node_list.size(); index_++) {
            int node_ind = channel_node_list[index_];

            if (node_ind == OPEN || device_ctx.rr_nodes[node_ind].capacity() == 0) {
                continue;
            }

            const std::pair<size_t, size_t>* loc = device_ctx.connection_boxes.find_canonical_loc(node_ind);
            if (loc == nullptr) {
                continue;
            }

            int node_cost_ind = device_ctx.rr_nodes[node_ind].cost_index();
            int node_seg_ind = device_ctx.rr_indexed_data[node_cost_ind].seg_index;
            if (node_seg_ind == seg_index_) {
                index_ += 1;
                return node_ind;
            }
        }

        return UNDEFINED;
    }

  private:
    int start_x_;
    int start_y_;
    t_rr_type rr_type_;
    int seg_index_;
    size_t index_;
};

static void compute_connection_box_lookahead(
    const std::vector<t_segment_inf>& segment_inf,
    const std::string& /*search_locations_str*/) {
    vtr::ScopedStartFinishTimer timer("Computing connection box lookahead map");

    auto& device_ctx = g_vpr_ctx.device();

    // First find bounding boxes for all segments.
    std::vector<t_bb> segment_bb(segment_inf.size(), t_bb{INT_MAX, INT_MIN, INT_MAX, INT_MIN});

    for (const auto& node : device_ctx.rr_nodes) {
        if (node.type() != CHANX && node.type() != CHANY) {
            continue;
        }

        int node_cost_ind = node.cost_index();
        int node_seg_ind = device_ctx.rr_indexed_data[node_cost_ind].seg_index;

        segment_bb[node_seg_ind].xmin = std::min<int>(node.xlow(), segment_bb[node_seg_ind].xmin);
        segment_bb[node_seg_ind].xmax = std::max<int>(node.xhigh(), segment_bb[node_seg_ind].xmax);
        segment_bb[node_seg_ind].ymin = std::min<int>(node.ylow(), segment_bb[node_seg_ind].ymin);
        segment_bb[node_seg_ind].ymax = std::max<int>(node.yhigh(), segment_bb[node_seg_ind].ymax);
    }

    // Initial search grid.
    SearchGrid grid;
    grid.setup(segment_bb, device_ctx.rr_nodes.size());
    for (size_t inode = 0; inode < device_ctx.rr_nodes.size(); ++inode) {
        const auto& node = device_ctx.rr_nodes[inode];

        if (node.type() != CHANX && node.type() != CHANY) {
            continue;
        }

        int node_cost_ind = node.cost_index();
        int node_seg_ind = device_ctx.rr_indexed_data[node_cost_ind].seg_index;
        grid.add_node(node_seg_ind, node.xlow(), node.ylow(), inode);
    }

    routing_data_t data;
    int current_segment = -1;
    std::unique_ptr<vtr::ScopedStartFinishTimer> segment_timer;
    // Start expanding based on rr nodes given from the search grid.
    for (int inode = grid.next_node(); inode != OPEN; inode = grid.next_node()) {
        VTR_ASSERT(device_ctx.rr_nodes[inode].type() == CHANX || device_ctx.rr_nodes[inode].type() == CHANY);

        int node_cost_ind = device_ctx.rr_nodes[inode].cost_index();
        int node_seg_ind = device_ctx.rr_indexed_data[node_cost_ind].seg_index;

        // Add output whenever a new segment is being search for diagnostic
        // purposes.
        if (node_seg_ind != current_segment) {
            current_segment = node_seg_ind;
            segment_timer.reset();
            segment_timer = std::make_unique<vtr::ScopedStartFinishTimer>(
                vtr::string_fmt("Expanding %s (%d)", segment_inf[node_seg_ind].name.c_str(), node_seg_ind));
        }

        VTR_LOG("Expanding at %d,%d\n",
                device_ctx.rr_nodes[inode].xlow(),
                device_ctx.rr_nodes[inode].ylow());

        // Expand all nodes at this location to get connection variety.
        for (e_rr_type chan_type : {CHANX, CHANY}) {
            StartNode start_node(
                device_ctx.rr_nodes[inode].xlow(),
                device_ctx.rr_nodes[inode].ylow(), chan_type, node_seg_ind);
            for (int start_node_ind = start_node.get_next_node();
                 start_node_ind != UNDEFINED;
                 start_node_ind = start_node.get_next_node()) {
                expand_from_node(start_node_ind, &grid, &data);
            }
        }
    }

    segment_timer.reset();

    g_cost_map.set_counts(segment_inf.size(), device_ctx.connection_boxes.num_connection_box_types());
    g_cost_map.set_cost_map(data, REPRESENTATIVE_ENTRY_METHOD);

    return;
}

static float get_connection_box_lookahead_map_cost(int from_node_ind,
                                                   int to_node_ind,
                                                   float criticality_fac) {
    if (from_node_ind == to_node_ind) {
        return 0.f;
    }

    auto& device_ctx = g_vpr_ctx.device();

    auto to_node_type = device_ctx.rr_nodes[to_node_ind].type();

    if (to_node_type == SINK) {
        const auto& sink_to_ipin = device_ctx.connection_boxes.find_sink_connection_boxes(to_node_ind);
        if (sink_to_ipin.ipin_count > 1) {
            float cost = std::numeric_limits<float>::infinity();
            // Find cheapest cost from from_node_ind to IPINs for this SINK.
            for (int i = 0; i < sink_to_ipin.ipin_count; ++i) {
                cost = std::min(cost,
                                get_connection_box_lookahead_map_cost(
                                    from_node_ind,
                                    sink_to_ipin.ipin_nodes[i], criticality_fac));
            }

            return cost;
        } else if (sink_to_ipin.ipin_count == 1) {
            to_node_ind = sink_to_ipin.ipin_nodes[0];
            if (from_node_ind == to_node_ind) {
                return 0.f;
            }
        } else {
            return std::numeric_limits<float>::infinity();
        }
    }

    if (device_ctx.rr_nodes[to_node_ind].type() != IPIN) {
        VPR_THROW(VPR_ERROR_ROUTE, "Not an IPIN/SINK, is %d", to_node_ind);
    }
    ConnectionBoxId box_id;
    std::pair<size_t, size_t> box_location;
    bool found = device_ctx.connection_boxes.find_connection_box(
        to_node_ind, &box_id, &box_location);
    if (!found) {
        VPR_THROW(VPR_ERROR_ROUTE, "No connection box for IPIN %d", to_node_ind);
    }

    const std::pair<size_t, size_t>* from_canonical_loc = device_ctx.connection_boxes.find_canonical_loc(from_node_ind);
    if (from_canonical_loc == nullptr) {
        VPR_THROW(VPR_ERROR_ROUTE, "No canonical loc for %d (to %d)",
                  from_node_ind, to_node_ind);
    }

    ssize_t dx = ssize_t(from_canonical_loc->first) - ssize_t(box_location.first);
    ssize_t dy = ssize_t(from_canonical_loc->second) - ssize_t(box_location.second);

    int from_seg_index = g_cost_map.node_to_segment(from_node_ind);
    Cost_Entry cost_entry = g_cost_map.find_cost(from_seg_index, box_id, dx, dy);
    float expected_delay = cost_entry.delay;
    float expected_congestion = cost_entry.congestion;

    float expected_cost = criticality_fac * expected_delay + (1.0 - criticality_fac) * expected_congestion;
    return expected_cost;
}

void ConnectionBoxMapLookahead::compute(const std::vector<t_segment_inf>& segment_inf, const std::string& lookahead_search_locations) {
    compute_connection_box_lookahead(segment_inf, lookahead_search_locations);
}

float ConnectionBoxMapLookahead::get_expected_cost(
    int current_node,
    int target_node,
    const t_conn_cost_params& params,
    float /*R_upstream*/) const {
    auto& device_ctx = g_vpr_ctx.device();

    t_rr_type rr_type = device_ctx.rr_nodes[current_node].type();

    if (rr_type == CHANX || rr_type == CHANY) {
        return get_connection_box_lookahead_map_cost(
            current_node, target_node, params.criticality);
    } else if (rr_type == IPIN) { /* Change if you're allowing route-throughs */
        return (device_ctx.rr_indexed_data[SINK_COST_INDEX].base_cost);
    } else { /* Change this if you want to investigate route-throughs */
        return (0.);
    }
}

#ifndef VTR_ENABLE_CAPNPROTO

void ConnectionBoxMapLookahead::read(const std::string& file) {
    VPR_THROW(VPR_ERROR_ROUTE, "ConnectionBoxMapLookahead::read not implemented");
}
void ConnectionBoxMapLookahead::write(const std::string& file) const {
    VPR_THROW(VPR_ERROR_ROUTE, "ConnectionBoxMapLookahead::write not implemented");
}

#else

void ConnectionBoxMapLookahead::read(const std::string& file) {
    g_cost_map.read(file);
}
void ConnectionBoxMapLookahead::write(const std::string& file) const {
    g_cost_map.write(file);
}

static void ToCostEntry(Cost_Entry* out, const VprCostEntry::Reader& in) {
    out->delay = in.getDelay();
    out->congestion = in.getCongestion();
}

static void FromCostEntry(VprCostEntry::Builder* out, const Cost_Entry& in) {
    out->setDelay(in.delay);
    out->setCongestion(in.congestion);
}

static void ToVprVector2D(std::pair<int, int>* out, const VprVector2D::Reader& in) {
    *out = std::make_pair(in.getX(), in.getY());
}

static void FromVprVector2D(VprVector2D::Builder* out, const std::pair<int, int>& in) {
    out->setX(in.first);
    out->setY(in.second);
}

static void ToMatrixCostEntry(vtr::NdMatrix<Cost_Entry, 2>* out,
                              const Matrix<VprCostEntry>::Reader& in) {
    ToNdMatrix<2, VprCostEntry, Cost_Entry>(out, in, ToCostEntry);
}

static void FromMatrixCostEntry(
    Matrix<VprCostEntry>::Builder* out,
    const vtr::NdMatrix<Cost_Entry, 2>& in) {
    FromNdMatrix<2, VprCostEntry, Cost_Entry>(
        out, in, FromCostEntry);
}

void CostMap::read(const std::string& file) {
    MmapFile f(file);

    /* Increase reader limit to 1G words. */
    ::capnp::ReaderOptions opts = ::capnp::ReaderOptions();
    opts.traversalLimitInWords = 1024 * 1024 * 1024;
    ::capnp::FlatArrayMessageReader reader(f.getData(), opts);

    auto cost_map = reader.getRoot<VprCostMap>();

    {
        const auto& segment_map = cost_map.getSegmentMap();
        segment_map_.resize(segment_map.size());
        auto dst_iter = segment_map_.begin();
        for (const auto& src : segment_map) {
            *dst_iter++ = src;
        }
    }

    {
        const auto& offset = cost_map.getOffset();
        ToNdMatrix<2, VprVector2D, std::pair<int, int>>(
            &offset_, offset, ToVprVector2D);
    }

    {
        const auto& cost_maps = cost_map.getCostMap();
        ToNdMatrix<2, Matrix<VprCostEntry>, vtr::NdMatrix<Cost_Entry, 2>>(
            &cost_map_, cost_maps, ToMatrixCostEntry);
    }
}

void CostMap::write(const std::string& file) const {
    ::capnp::MallocMessageBuilder builder;

    auto cost_map = builder.initRoot<VprCostMap>();

    {
        auto segment_map = cost_map.initSegmentMap(segment_map_.size());
        for (size_t i = 0; i < segment_map_.size(); ++i) {
            segment_map.set(i, segment_map_[i]);
        }
    }

    {
        auto offset = cost_map.initOffset();
        FromNdMatrix<2, VprVector2D, std::pair<int, int>>(
            &offset, offset_, FromVprVector2D);
    }

    {
        auto cost_maps = cost_map.initCostMap();
        FromNdMatrix<2, Matrix<VprCostEntry>, vtr::NdMatrix<Cost_Entry, 2>>(
            &cost_maps, cost_map_, FromMatrixCostEntry);
    }

    writeMessageToFile(file, &builder);
}
#endif
