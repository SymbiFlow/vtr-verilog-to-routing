#include "router_lookahead_map.h"
#include "router_lookahead_map_utils.h"
#include <cmath>
#include <vector>
#include <queue>
#include <ctime>
#include "vpr_types.h"
#include "vpr_error.h"
#include "vpr_utils.h"
#include "globals.h"
#include "vtr_math.h"
#include "vtr_log.h"
#include "vtr_assert.h"
#include "vtr_time.h"

using namespace std;

/* the cost map is computed by running a Dijkstra search from channel segment rr nodes at the specified reference coordinate */
#define REF_X 14
#define REF_Y 14

/* we will profile delay/congestion using this many tracks for each wire type */
#define MAX_TRACK_OFFSET 16

/* we're profiling routing cost over many tracks for each wire type, so we'll have many cost entries at each |dx|,|dy| offset.
 * there are many ways to "boil down" the many costs at each offset to a single entry for a given (wire type, chan_type) combination --
 * we can take the smallest cost, the average, median, etc. This define selects the method we use.
 * See e_representative_entry_method */
#define REPRESENTATIVE_ENTRY_METHOD SMALLEST

/* provides delay/congestion estimates to travel specified distances
 * in the x/y direction */
typedef vtr::NdMatrix<Cost_Entry, 4> t_cost_map; //[0..1][[0..num_seg_types-1]0..device_ctx.grid.width()-1][0..device_ctx.grid.height()-1]
                                                 //[0..1] entry distinguish between CHANX/CHANY start nodes respectively

/* used during Dijkstra expansion to store delay/congestion info lists for each relative coordinate for a given segment and channel type.
 * the list at each coordinate is later boiled down to a single representative cost entry to be stored in the final cost map */
typedef vtr::Matrix<Expansion_Cost_Entry> t_routing_cost_map; //[0..device_ctx.grid.width()-1][0..device_ctx.grid.height()-1]

/******** File-Scope Variables ********/
/* The cost map */
t_cost_map f_cost_map;

/******** File-Scope Functions ********/
static void alloc_cost_map(int num_segments);
static void free_cost_map();
/* returns index of a node from which to start routing */
static int get_start_node_ind(int start_x, int start_y, int target_x, int target_y, t_rr_type rr_type, int seg_index, int track_offset);
/* runs Dijkstra's algorithm from specified node until all nodes have been visited. Each time a pin is visited, the delay/congestion information
 * to that pin is stored is added to an entry in the routing_cost_map */
static void run_dijkstra(int start_node_ind, int start_x, int start_y, t_routing_cost_map& routing_cost_map);
/* sets the lookahead cost map entries based on representative cost entries from routing_cost_map */
static void set_lookahead_map_costs(int segment_index, e_rr_type chan_type, t_routing_cost_map& routing_cost_map);
/* fills in missing lookahead map entries by copying the cost of the closest valid entry */
static void fill_in_missing_lookahead_entries(int segment_index, e_rr_type chan_type);
/* returns a cost entry in the f_cost_map that is near the specified coordinates (and preferably towards (0,0)) */
static Cost_Entry get_nearby_cost_entry(int x, int y, int segment_index, int chan_index);
/* returns the absolute delta_x and delta_y offset required to reach to_node from from_node */
static void get_xy_deltas(int from_node_ind, int to_node_ind, int* delta_x, int* delta_y);

static void print_cost_map(const std::vector<t_segment_inf>& segment_inf);

/******** Function Definitions ********/
/* queries the lookahead_map (should have been computed prior to routing) to get the expected cost
 * from the specified source to the specified target */
float get_lookahead_map_cost(int from_node_ind, int to_node_ind, float criticality_fac) {
    auto& device_ctx = g_vpr_ctx.device();
    auto& from_node = device_ctx.rr_nodes[from_node_ind];

    e_rr_type from_type = from_node.type();
    int from_cost_index = from_node.cost_index();
    int from_seg_index = device_ctx.rr_indexed_data[from_cost_index].seg_index;

    VTR_ASSERT(from_seg_index >= 0);

    int delta_x, delta_y;
    get_xy_deltas(from_node_ind, to_node_ind, &delta_x, &delta_y);
    delta_x = abs(delta_x);
    delta_y = abs(delta_y);

    /* now get the expected cost from our lookahead map */
    int from_chan_index = 0;

    Cost_Entry cost_entry = f_cost_map[from_chan_index][from_seg_index][delta_x][delta_y];
    float expected_delay = cost_entry.delay;
    float expected_congestion = cost_entry.congestion;

    float expected_cost = criticality_fac * expected_delay + (1.0 - criticality_fac) * expected_congestion;
    return expected_cost;
}

/* Computes the lookahead map to be used by the router. If a map was computed prior to this, a new one will not be computed again.
 * The rr graph must have been built before calling this function. */
void compute_router_lookahead(const std::vector<t_segment_inf>& segment_inf) {
    size_t num_segments = segment_inf.size();
    vtr::ScopedStartFinishTimer timer("Computing router lookahead map");

    f_cost_map.clear();

    auto& device_ctx = g_vpr_ctx.device();

    /* free previous delay map and allocate new one */
    free_cost_map();
    alloc_cost_map(num_segments);

    /* run Dijkstra's algorithm for each segment type & channel type combination */
    for (int iseg = 0; iseg < num_segments; iseg++) {
        /* allocate the cost map for this iseg/chan_type */
        t_routing_cost_map routing_cost_map({device_ctx.grid.width(), device_ctx.grid.height()});
        for (e_rr_type chan_type : {CHANX, CHANY}) {
            for (int ref_inc = 0; ref_inc < 3; ref_inc++) {
                for (int track_offset = 0; track_offset < MAX_TRACK_OFFSET; track_offset += 2) {
                    /* get the rr node index from which to start routing */
                    int start_node_ind = get_start_node_ind(REF_X + ref_inc, REF_Y + ref_inc,
                                                            device_ctx.grid.width() - 2, device_ctx.grid.height() - 2, //non-corner upper right
                                                            chan_type, iseg, track_offset);

                    if (start_node_ind == UNDEFINED) {
                        continue;
                    }

                    /* run Dijkstra's algorithm */
                    run_dijkstra(start_node_ind, REF_X + ref_inc, REF_Y + ref_inc, routing_cost_map);
                }
            }
        }

        /* boil down the cost list in routing_cost_map at each coordinate to a representative cost entry and store it in the lookahead
         * cost map */
        set_lookahead_map_costs(iseg, CHANX, routing_cost_map);

        /* fill in missing entries in the lookahead cost map by copying the closest cost entries (cost map was computed based on
         * a reference coordinate > (0,0) so some entries that represent a cross-chip distance have not been computed) */
        fill_in_missing_lookahead_entries(iseg, CHANX);
    }

    if (true) print_cost_map(segment_inf);
}

/* returns index of a node from which to start routing */
static int get_start_node_ind(int start_x, int start_y, int target_x, int target_y, t_rr_type rr_type, int seg_index, int track_offset) {
    int result = UNDEFINED;

    if (rr_type != CHANX && rr_type != CHANY) {
        vpr_throw(VPR_ERROR_ROUTE, __FILE__, __LINE__, "Must start lookahead routing from CHANX or CHANY node\n");
    }

    /* determine which direction the wire should go in based on the start & target coordinates */
    e_direction direction = INC_DIRECTION;
    if ((rr_type == CHANX && target_x < start_x) || (rr_type == CHANY && target_y < start_y)) {
        direction = DEC_DIRECTION;
    }

    auto& device_ctx = g_vpr_ctx.device();

    VTR_ASSERT(rr_type == CHANX || rr_type == CHANY);

    const vector<int>& channel_node_list = device_ctx.rr_node_indices[rr_type][start_x][start_y][0];

    /* find first node in channel that has specified segment index and goes in the desired direction */
    for (unsigned itrack = 0; itrack < channel_node_list.size(); itrack++) {
        int node_ind = channel_node_list[itrack];

        e_direction node_direction = device_ctx.rr_nodes[node_ind].direction();
        int node_cost_ind = device_ctx.rr_nodes[node_ind].cost_index();
        int node_seg_ind = device_ctx.rr_indexed_data[node_cost_ind].seg_index;

        if ((node_direction == direction || node_direction == BI_DIRECTION) && node_seg_ind == seg_index) {
            /* found first track that has the specified segment index and goes in the desired direction */
            result = node_ind;
            if (track_offset == 0) {
                break;
            }
            track_offset -= 2;
        }
    }

    return result;
}

/* allocates space for cost map entries */
static void alloc_cost_map(int num_segments) {
    auto& device_ctx = g_vpr_ctx.device();

    f_cost_map = t_cost_map({2, size_t(num_segments), device_ctx.grid.width(), device_ctx.grid.height()});
}

/* frees the cost map. duh. */
static void free_cost_map() {
    f_cost_map.clear();
}

/* runs Dijkstra's algorithm from specified node until all nodes have been visited. Each time a pin is visited, the delay/congestion information
 * to that pin is stored is added to an entry in the routing_cost_map */
static void run_dijkstra(int start_node_ind, int start_x, int start_y, t_routing_cost_map& routing_cost_map) {
    auto& device_ctx = g_vpr_ctx.device();

    /* a list of boolean flags (one for each rr node) to figure out if a certain node has already been expanded */
    vector<bool> node_expanded(device_ctx.rr_nodes.size(), false);
    /* for each node keep a list of the cost with which that node has been visited (used to determine whether to push
     * a candidate node onto the expansion queue */
    vector<float> node_visited_costs(device_ctx.rr_nodes.size(), -1.0);
    /* a priority queue for expansion */
    priority_queue<PQ_Entry> pq;

    /* first entry has no upstream delay or congestion */
    PQ_Entry first_entry(start_node_ind, UNDEFINED, 0, 0, 0, true);

    pq.push(first_entry);

    /* now do routing */
    while (!pq.empty()) {
        PQ_Entry current = pq.top();
        pq.pop();

        int node_ind = current.rr_node_ind;

        /* check that we haven't already expanded from this node */
        if (node_expanded[node_ind]) {
            continue;
        }

        /* if this node is an ipin record its congestion/delay in the routing_cost_map */
        if (device_ctx.rr_nodes[node_ind].type() == IPIN) {
            int ipin_x = device_ctx.rr_nodes[node_ind].xlow();
            int ipin_y = device_ctx.rr_nodes[node_ind].ylow();

            if (ipin_x >= start_x && ipin_y >= start_y) {
                int delta_x, delta_y;
                get_xy_deltas(start_node_ind, node_ind, &delta_x, &delta_y);

                routing_cost_map[delta_x][delta_y].add_cost_entry(
                    REPRESENTATIVE_ENTRY_METHOD,
                    current.delay, current.congestion_upstream);
            }
        }

        expand_dijkstra_neighbours(current, node_visited_costs, node_expanded, pq);
        node_expanded[node_ind] = true;
    }
}

/* sets the lookahead cost map entries based on representative cost entries from routing_cost_map */
static void set_lookahead_map_costs(int segment_index, e_rr_type chan_type, t_routing_cost_map& routing_cost_map) {
    int chan_index = 0;

    /* set the lookahead cost map entries with a representative cost entry from routing_cost_map */
    for (unsigned ix = 0; ix < routing_cost_map.dim_size(0); ix++) {
        for (unsigned iy = 0; iy < routing_cost_map.dim_size(1); iy++) {
            const Expansion_Cost_Entry& expansion_cost_entry = routing_cost_map[ix][iy];

            f_cost_map[chan_index][segment_index][ix][iy] = expansion_cost_entry.get_representative_cost_entry(REPRESENTATIVE_ENTRY_METHOD);
        }
    }
}

/* fills in missing lookahead map entries by copying the cost of the closest valid entry */
static void fill_in_missing_lookahead_entries(int segment_index, e_rr_type chan_type) {
    int chan_index = 0;

    auto& device_ctx = g_vpr_ctx.device();

    /* find missing cost entries and fill them in by copying a nearby cost entry */
    for (unsigned ix = 0; ix < device_ctx.grid.width(); ix++) {
        for (unsigned iy = 0; iy < device_ctx.grid.height(); iy++) {
            Cost_Entry cost_entry = f_cost_map[chan_index][segment_index][ix][iy];

            if (!cost_entry.valid()) {
                Cost_Entry copied_entry = get_nearby_cost_entry(ix, iy, segment_index, chan_index);
                f_cost_map[chan_index][segment_index][ix][iy] = copied_entry;
            }
        }
    }
}

/* returns a cost entry in the f_cost_map that is near the specified coordinates (and preferably towards (0,0)) */
static Cost_Entry get_nearby_cost_entry(int x, int y, int segment_index, int chan_index) {
    /* compute the slope from x,y to 0,0 and then move towards 0,0 by one unit to get the coordinates
     * of the cost entry to be copied */

    VTR_ASSERT(x >= 0 && y >= 0);

    float slope;
    if (x == 0) {
        slope = 1e12; //just a really large number
    } else if (y == 0) {
        slope = 1e-12; //just a really small number
    } else {
        slope = (float)y / (float)x;
    }

    int copy_x, copy_y;
    if (slope >= 1.0) {
        copy_y = y - 1;
        copy_x = vtr::nint((float)y / slope);
    } else {
        copy_x = x - 1;
        copy_y = vtr::nint((float)x * slope);
    }

    if (copy_x < 0 || copy_y < 0) {
        return Cost_Entry();
    }

    VTR_ASSERT(copy_x >= 0 && copy_y >= 0);

    Cost_Entry copy_entry = f_cost_map[chan_index][segment_index][copy_x][copy_y];

    /* if the entry to be copied is also empty, recurse */
    if (!copy_entry.valid()) {
        copy_entry = get_nearby_cost_entry(copy_x, copy_y, segment_index, chan_index);
    }

    return copy_entry;
}

/* returns the absolute delta_x and delta_y offset required to reach to_node from from_node */
static void get_xy_deltas(int from_node_ind, int to_node_ind, int* delta_x, int* delta_y) {
    auto& device_ctx = g_vpr_ctx.device();

    auto& from = device_ctx.rr_nodes[from_node_ind];
    auto& to = device_ctx.rr_nodes[to_node_ind];

    /* get chan/seg coordinates of the from/to nodes. seg coordinate is along the wire,
     * chan coordinate is orthogonal to the wire */
    int from_seg_low = from.xlow();
    int from_seg_high = from.xhigh();
    int from_chan = from.ylow();
    int to_seg = to.xlow();
    int to_chan = to.ylow();
    if (from.type() == CHANY) {
        from_seg_low = from.ylow();
        from_seg_high = from.yhigh();
        from_chan = from.xlow();
        to_seg = to.ylow();
        to_chan = to.xlow();
    }

    /* now we want to count the minimum number of *channel segments* between the from and to nodes */
    int delta_seg, delta_chan;

    /* orthogonal to wire */
    int no_need_to_pass_by_clb = 0; //if we need orthogonal wires then we don't need to pass by the target CLB along the current wire direction
    if (to_chan > from_chan + 1) {
        /* above */
        delta_chan = to_chan - from_chan;
        no_need_to_pass_by_clb = 1;
    } else if (to_chan < from_chan) {
        /* below */
        delta_chan = from_chan - to_chan + 1;
        no_need_to_pass_by_clb = 1;
    } else {
        /* adjacent to current channel */
        delta_chan = 0;
        no_need_to_pass_by_clb = 0;
    }

    /* along same direction as wire. */
    if (to_seg > from_seg_high) {
        /* ahead */
        delta_seg = to_seg - from_seg_high - no_need_to_pass_by_clb;
    } else if (to_seg < from_seg_low) {
        /* behind */
        delta_seg = from_seg_low - to_seg - no_need_to_pass_by_clb;
    } else {
        /* along the span of the wire */
        delta_seg = 0;
    }

    /* account for wire direction. lookahead map was computed by looking up and to the right starting at INC wires. for targets
     * that are opposite of the wire direction, let's add 1 to delta_seg */
    if ((to_seg < from_seg_low && from.direction() == INC_DIRECTION) || (to_seg > from_seg_high && from.direction() == DEC_DIRECTION)) {
        delta_seg++;
    }

    *delta_x = delta_seg;
    *delta_y = delta_chan;
    if (from.type() == CHANY) {
        *delta_x = delta_chan;
        *delta_y = delta_seg;
    }
}

static void print_cost_map(const std::vector<t_segment_inf>& segment_inf) {
    auto& device_ctx = g_vpr_ctx.device();

    for (size_t chan_index = 0; chan_index < 1; chan_index++) {
        for (size_t iseg = 0; iseg < f_cost_map.dim_size(1); iseg++) {
            vtr::printf("Seg %s(%d) CHAN %d\n", segment_inf.at(iseg).name.c_str(),
                        iseg, chan_index);
            for (size_t iy = 0; iy < device_ctx.grid.height(); iy++) {
                for (size_t ix = 0; ix < device_ctx.grid.width(); ix++) {
                    vtr::printf("%d,%d: %.3g\t", ix, iy, f_cost_map[chan_index][iseg][ix][iy].delay);
                }
                vtr::printf("\n");
            }
            vtr::printf("\n\n");
        }
    }
}
