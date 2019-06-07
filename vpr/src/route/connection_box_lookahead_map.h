#ifndef CONNECTION_BOX_LOOKAHEAD_H_
#define CONNECTION_BOX_LOOKAHEAD_H_

#include <vector>
#include "physical_types.h"

void compute_connection_box_lookahead(
    const std::vector<t_segment_inf>& segment_inf);

float get_connection_box_lookahead_map_cost(int from_node_ind,
                                            int to_node_ind,
                                            float criticality_fac);

#endif
