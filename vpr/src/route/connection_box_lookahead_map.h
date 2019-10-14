#ifndef CONNECTION_BOX_LOOKAHEAD_H_
#define CONNECTION_BOX_LOOKAHEAD_H_

#include <vector>
#include <list>
#include "physical_types.h"
#include "router_lookahead.h"
#include "router_lookahead_map_utils.h"
#include "connection_box.h"

template<typename T>
class BoundingBox {
  public:
    BoundingBox(T xmin, T ymin, T xmax, T ymax)
        : xmin_(xmin)
        , ymin_(ymin)
        , xmax_(xmax)
        , ymax_(ymax){};
    BoundingBox(T x, T y)
        : xmin_(x)
        , ymin_(y)
        , xmax_(x)
        , ymax_(y){};
    BoundingBox()
        : xmin_(std::numeric_limits<T>::max())
        , ymin_(std::numeric_limits<T>::max())
        , xmax_(std::numeric_limits<T>::min())
        , ymax_(std::numeric_limits<T>::min()){};
    BoundingBox operator|(const BoundingBox& other) const;
    BoundingBox& operator|=(const BoundingBox& other);
    std::pair<T, T> center() const;
    std::pair<T, T> sample(T x, T y, T d) const;
    bool contains(T x, T y) const;
    T width() const;
    T height() const;
    bool empty() const;

    T xmin_, ymin_, xmax_, ymax_;
};

typedef std::list<std::tuple<std::pair<int, int>,
                             std::pair<int, int>,
                             Cost_Entry,
                             ConnectionBoxId>>
    routing_cost_list;

class CostMap {
  public:
    typedef std::vector<std::tuple<std::pair<int, int>, std::pair<int, int>, Cost_Entry, ConnectionBoxId>> t_routing_cost_map;

    void set_counts(size_t seg_count, size_t box_count);
    int node_to_segment(int from_node_ind) const;
    Cost_Entry find_cost(int from_seg_index, ConnectionBoxId box_id, int delta_x, int delta_y) const;
    void set_cost_map(int from_seg_index,
                      const routing_cost_list& costs,
                      e_representative_entry_method method);
    void set_cost_map(int from_seg_index, ConnectionBoxId box_id, const routing_cost_list& costs, e_representative_entry_method method);
    Cost_Entry get_nearby_cost_entry(const vtr::NdMatrix<Cost_Entry, 2>& matrix, ssize_t cx, ssize_t cy, const BoundingBox<ssize_t>& bounds);
    void read(const std::string& file);
    void write(const std::string& file) const;
    void print(int iseg) const;
    std::list<std::pair<int, int>> list_empty() const;

  private:
    vtr::NdMatrix<vtr::NdMatrix<Cost_Entry, 2>, 2> cost_map_;
    vtr::NdMatrix<std::pair<int, int>, 2> offset_;
    std::vector<int> segment_map_;
};

class ConnectionBoxMapLookahead : public RouterLookahead {
  public:
    float get_expected_cost(int node, int target_node, const t_conn_cost_params& params, float R_upstream) const override;
    float get_map_cost(int from_node_ind, int to_node_ind, float criticality_fac) const;
    void compute(const std::vector<t_segment_inf>& segment_inf, const std::string& search_locations) override;

    void read(const std::string& file) override;
    void write(const std::string& file) const override;

    // Precompute start_inode_for_box_
    void find_start_inodes_for_segment_types(int num_segments);

    static constexpr int SAMPLE_GRID_SIZE = 8;
    typedef std::array<std::array<std::list<ssize_t>, SAMPLE_GRID_SIZE>, SAMPLE_GRID_SIZE> SampleGrid;
    std::vector<SampleGrid> start_inode_for_segment_;
    std::vector<BoundingBox<int>> bounding_box_for_segment_;
    CostMap cost_map_;
};

#endif
