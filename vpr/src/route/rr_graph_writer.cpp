/*
 * This file defines the writing rr graph function in XML format.
 * The rr graph is separated into channels, nodes, switches,
 * grids, edges, block types, and segments. Each tag has several
 * children tags such as timing, location, or some general
 * details. Each tag has attributes to describe them */

#include <fstream>
#include <iostream>
#include <string.h>
#include <iomanip>
#include <limits>
#include "vpr_error.h"
#include "globals.h"
#include "read_xml_arch_file.h"
#include "vtr_version.h"
#include "rr_graph_writer.h"
#include "rr_graph_generated.h"

using namespace std;

/* All values are printed with this precision value. The higher the
 * value, the more accurate the read in rr graph is. Using numeric_limits
 * max_digits10 guarentees that no values change during a sequence of
 * float -> string -> float conversions */
constexpr int FLOAT_PRECISION = std::numeric_limits<float>::max_digits10;
/*********************** Subroutines local to this module *******************/
static void write_rr_channel(fstream &fp);
static flatbuffers::Offset<vtr::vpr::Channels> write_rr_channel_flatbuffer(flatbuffers::FlatBufferBuilder &fbb);
static void write_rr_node(fstream &fp);
static flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::RrNode>>> write_rr_node_flatbuffer(flatbuffers::FlatBufferBuilder &fbb);
static void write_rr_switches(fstream &fp);
static flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::Switch>>> write_rr_switches_flatbuffer(flatbuffers::FlatBufferBuilder &fbb);
static void write_rr_grid(fstream &fp);
static flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::GridLoc>>> write_rr_grid_flatbuffer(flatbuffers::FlatBufferBuilder &fbb);
static void write_rr_edges(fstream &fp);
static flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::RrEdge>>> write_rr_edges_flatbuffer(flatbuffers::FlatBufferBuilder &fbb);
static void write_rr_block_types(fstream &fp);
static flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::BlockType>>> write_rr_block_types_flatbuffer(flatbuffers::FlatBufferBuilder &fbb);
static void write_rr_segments(fstream &fp, const t_segment_inf *segment_inf, const int num_seg_types);
static flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::Segment>>> write_rr_segments_flatbuffer(flatbuffers::FlatBufferBuilder &fbb, const t_segment_inf *segment_inf, const int num_seg_types);
static void write_rr_graph_flatbuffer(const char *file_name, const t_segment_inf *segment_inf, const int num_seg_types);
static void write_rr_graph_xml(const char *file_name, const t_segment_inf *segment_inf, const int num_seg_types);

/************************ Subroutine definitions ****************************/

/* This function is used to write the rr_graph into xml format into a a file with name: file_name */
void write_rr_graph(const char *file_name, const t_segment_inf *segment_inf, const int num_seg_types) {
    if (vtr::check_file_name_extension(file_name, ".rrgraph")) {
      write_rr_graph_flatbuffer(file_name, segment_inf, num_seg_types);
    } else {
      write_rr_graph_xml(file_name, segment_inf, num_seg_types);
    }
}

void write_rr_graph_xml(const char *file_name, const t_segment_inf *segment_inf, const int num_seg_types) {
    fstream fp;
    fp.open(file_name, fstream::out | fstream::trunc);

    /* Prints out general info for easy error checking*/
    if (!fp.is_open() || !fp.good()) {
        vpr_throw(VPR_ERROR_OTHER, __FILE__, __LINE__,
                "couldn't open file \"%s\" for generating RR graph file\n", file_name);
    }
    cout << "Writing RR graph" << endl;
    fp << "<rr_graph tool_name=\"vpr\" tool_version=\"" << vtr::VERSION <<
            "\" tool_comment=\"Generated from arch file "
            << get_arch_file_name() << "\">" << endl;

    /* Write out each individual component*/
    write_rr_channel(fp);
    write_rr_switches(fp);
    write_rr_segments(fp, segment_inf, num_seg_types);
    write_rr_block_types(fp);
    write_rr_grid(fp);
    write_rr_node(fp);
    write_rr_edges(fp);
    fp << "</rr_graph>";

    fp.close();

    cout << "Finished generating RR graph file named " << file_name << endl << endl;
}
void write_rr_graph_flatbuffer(const char *file_name, const t_segment_inf *segment_inf, const int num_seg_types) {
    cout << "Serializing RR graph" << endl;
    flatbuffers::FlatBufferBuilder fbb(1024);

    auto info_string = fbb.CreateString(vtr::VERSION);
    auto info_comment = fbb.CreateString(get_arch_file_name());
    auto channels = write_rr_channel_flatbuffer(fbb);
    auto switches = write_rr_switches_flatbuffer(fbb);
    auto segments = write_rr_segments_flatbuffer(fbb, segment_inf, num_seg_types);
    auto block_types = write_rr_block_types_flatbuffer(fbb);
    auto grid = write_rr_grid_flatbuffer(fbb);
    auto node = write_rr_node_flatbuffer(fbb);
    auto edges = write_rr_edges_flatbuffer(fbb);
    auto root = CreateRrGraph(
        fbb,
        info_string,
        info_comment,
        channels,
        switches,
        segments,
        block_types,
        grid,
        node,
        edges);
    FinishRrGraphBuffer(fbb, root);

    /* Verify generation code is correct before writing. */
    flatbuffers::Verifier verifier(
        fbb.GetBufferPointer(), fbb.GetSize(),
        /*_max_depth=*/64,
        /*_max_tables=*/std::numeric_limits<flatbuffers::uoffset_t>::max());
    if(vtr::vpr::VerifyRrGraphBuffer(verifier) == false) {
      vpr_throw(VPR_ERROR_OTHER, __FILE__, __LINE__,
                "Code failed to generate a valid rrgraph flatbuffer.");
    }

    fstream fp;
    fp.open(file_name, fstream::out | fstream::trunc | fstream::binary);

    /* Prints out general info for easy error checking*/
    if (!fp.is_open() || !fp.good()) {
        vpr_throw(VPR_ERROR_OTHER, __FILE__, __LINE__,
                "couldn't open file \"%s\" for generating RR graph file\n", file_name);
    }
    fp.write(reinterpret_cast<const char *>(fbb.GetBufferPointer()), fbb.GetSize());
    fp.close();

    cout << "Finished generating RR graph file named " << file_name << endl << endl;
}

static void add_metadata_to_xml(fstream &fp, const char *tab_prefix, const t_metadata_dict & meta) {
    fp << tab_prefix << "<metadata>" << endl;

    for(const auto &meta_elem : meta) {
        const t_offset & offset = meta_elem.first.first;
        const std::string & key = meta_elem.first.second;
        const std::vector<t_metadata_as> & values = meta_elem.second;
        for(const auto &value : values) {
          fp << tab_prefix << "\t<meta name=\"" << key << "\"";

          if(offset.x != 0 || offset.y != 0 || offset.z != 0) {
            if(offset.x) {
              fp << " x_offset=\"" << offset.x << "\"";
            }
            if(offset.y) {
              fp << " y_offset=\"" << offset.y << "\"";
            }
            if(offset.z) {
              fp << " z_offset=\"" << offset.z << "\"";
            }
          }
          fp << ">" << value.as_string() << "</meta>" << endl;
        }
    }
    fp << tab_prefix << "</metadata>" << endl;
}

/* Channel info in device_ctx.chan_width is written in xml format.
 * A general summary of the min and max values of the channels are first printed. Every
 * x and y channel list is printed out in its own attribute*/
void write_rr_channel(fstream &fp) {

    auto& device_ctx = g_vpr_ctx.device();
    fp << "\t<channels>" << endl;
    fp << "\t\t<channel chan_width_max =\"" << device_ctx.chan_width.max <<
            "\" x_min=\"" << device_ctx.chan_width.x_min <<
            "\" y_min=\"" << device_ctx.chan_width.y_min <<
            "\" x_max=\"" << device_ctx.chan_width.x_max <<
            "\" y_max=\"" << device_ctx.chan_width.y_max << "\"/>" << endl;

    int* list = device_ctx.chan_width.x_list;
    for (size_t i = 0; i < device_ctx.grid.height() - 1; i++) {
        fp << "\t\t<x_list index =\"" << i << "\" info=\"" << list[i] << "\"/>" << endl;
    }
    list = device_ctx.chan_width.y_list;
    for (size_t i = 0; i < device_ctx.grid.width() - 1; i++) {
        fp << "\t\t<y_list index =\"" << i << "\" info=\"" << list[i] << "\"/>" << endl;
    }
    fp << "\t</channels>" << endl;
}
flatbuffers::Offset<vtr::vpr::Channels> write_rr_channel_flatbuffer(flatbuffers::FlatBufferBuilder &fbb) {
    const auto& device_ctx = g_vpr_ctx.device();

    std::vector<flatbuffers::Offset<vtr::vpr::ChannelList>> x_list(device_ctx.grid.height()-1);
    for (size_t i = 0; i < device_ctx.grid.height() - 1; i++) {
        x_list[i] = vtr::vpr::CreateChannelList(fbb, i, device_ctx.chan_width.x_list[i]);
    }
    std::vector<flatbuffers::Offset<vtr::vpr::ChannelList>> y_list(device_ctx.grid.width()-1);
    for (size_t i = 0; i < device_ctx.grid.width() - 1; i++) {
        y_list[i] = vtr::vpr::CreateChannelList(fbb, i, device_ctx.chan_width.y_list[i]);
    }

    return vtr::vpr::CreateChannels(
        fbb,
        device_ctx.chan_width.max,
        device_ctx.chan_width.x_min,
        device_ctx.chan_width.y_min,
        device_ctx.chan_width.x_max,
        device_ctx.chan_width.y_max,
        fbb.CreateVector(x_list),
        fbb.CreateVector(y_list));
}

/* All relevant rr node info is written out to the graph.
 * This includes location, timing, and segment info*/
void write_rr_node(fstream &fp) {
    auto& device_ctx = g_vpr_ctx.device();

    fp << "\t<rr_nodes>" << endl;

    for (size_t inode = 0; inode < device_ctx.rr_nodes.size(); inode++) {
        auto& node = device_ctx.rr_nodes[inode];
        fp << "\t\t<node";
        fp << " id=\"" << inode;
        fp << "\" type=\"" << node.type_string();
        if (node.type() == CHANX || node.type() == CHANY) {
            fp << "\" direction=\"" << node.direction_string();
        }
        fp << "\" capacity=\"" << node.capacity();
        fp << "\">" << endl;
        fp << "\t\t\t<loc";
        fp << " xlow=\"" << node.xlow();
        fp << "\" ylow=\"" << node.ylow();
        fp << "\" xhigh=\"" << node.xhigh();
        fp << "\" yhigh=\"" << node.yhigh();
        if (node.type() == IPIN || node.type() == OPIN) {
            fp << "\" side=\"" << node.side_string() ;
        }
        fp << "\" ptc=\"" << node.ptc_num() ;
        fp << "\"/>" << endl;
        fp << "\t\t\t<timing R=\"" << setprecision(FLOAT_PRECISION)<< node.R()
                << "\" C=\"" << setprecision(FLOAT_PRECISION)<<node.C() << "\"/>" << endl;

        if (device_ctx.rr_indexed_data[node.cost_index()].seg_index != -1) {
            fp << "\t\t\t<segment segment_id=\"" << device_ctx.rr_indexed_data[node.cost_index()].seg_index << "\"/>" << endl;
        }

        const auto iter = device_ctx.rr_node_metadata.find(&node);
        if(iter != device_ctx.rr_node_metadata.end()) {
            const t_metadata_dict & meta = iter->second;
            add_metadata_to_xml(fp, "\t\t\t", meta);
        }

        fp << "\t\t</node>" << endl;
    }

    fp << "\t</rr_nodes>" << endl << endl;
}

static void add_metadata_to_flatbuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    std::vector<flatbuffers::Offset<vtr::vpr::NodeMetadata>> *metadata,
    const t_metadata_dict & meta) {
    size_t num_meta = 0;
    for(const auto &meta_elem : meta) {
      num_meta += meta_elem.second.size();
    }

    metadata->clear();
    metadata->reserve(num_meta);

    for(const auto &meta_elem : meta) {
        const t_offset & offset = meta_elem.first.first;
        const std::string & key = meta_elem.first.second;
        const std::vector<t_metadata_as> & values = meta_elem.second;
        for(const auto &value : values) {
          metadata->push_back(vtr::vpr::CreateNodeMetadata(
              fbb,
              fbb.CreateString(key),
              offset.x,
              offset.y,
              offset.z,
              fbb.CreateString(value.as_string())));
        }
    }
}
static flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::RrNode>>> write_rr_node_flatbuffer(flatbuffers::FlatBufferBuilder &fbb) {
    auto& device_ctx = g_vpr_ctx.device();

    std::vector<flatbuffers::Offset<vtr::vpr::RrNode>> nodes(device_ctx.rr_nodes.size());
    for (size_t inode = 0; inode < device_ctx.rr_nodes.size(); inode++) {
        auto& node = device_ctx.rr_nodes[inode];

        vtr::vpr::NodeType type = vtr::vpr::NodeType_INVALID_NODE_TYPE;
        vtr::vpr::NodeDirection direction = vtr::vpr::NodeDirection_NO_DIR;
        switch(node.type()) {
          case CHANX:
            type = vtr::vpr::NodeType_CHANX;
            break;
          case CHANY:
            type = vtr::vpr::NodeType_CHANY;
            break;
          case SOURCE:
            type = vtr::vpr::NodeType_SOURCE;
            break;
          case SINK:
            type = vtr::vpr::NodeType_SINK;
            break;
          case OPIN:
            type = vtr::vpr::NodeType_OPIN;
            break;
          case IPIN:
            type = vtr::vpr::NodeType_IPIN;
            break;
          default:
            vpr_throw(VPR_ERROR_OTHER, __FILE__, __LINE__,
                    "Valid inputs for class types are \"CHANX\", \"CHANY\",\"SOURCE\", \"SINK\",\"OPIN\", and \"IPIN\".");
            break;
        }

        if (node.type() == CHANX || node.type() == CHANY) {
            switch(node.direction()) {
              case INC_DIRECTION:
                direction = vtr::vpr::NodeDirection_INC_DIR;
                break;
              case DEC_DIRECTION:
                direction = vtr::vpr::NodeDirection_DEC_DIR;
                break;
              case BI_DIRECTION:
                direction = vtr::vpr::NodeDirection_BI_DIR;
                break;
              default:
                vpr_throw(VPR_ERROR_OTHER, __FILE__, __LINE__,
                        "Valid inputs for direction types are \"INC_DIRECTION\", \"DEC_DIRECTION\",\"BI_DIRECTION\".");
                break;
            }
        }

        vtr::vpr::NodeSide side = vtr::vpr::NodeSide_NO_SIDE;
        if (node.type() == IPIN || node.type() == OPIN) {
            switch(node.side()) {
            case LEFT:
              side = vtr::vpr::NodeSide_LEFT;
              break;
            case RIGHT:
              side = vtr::vpr::NodeSide_RIGHT;
              break;
            case TOP:
              side = vtr::vpr::NodeSide_TOP;
              break;
            case BOTTOM:
              side = vtr::vpr::NodeSide_BOTTOM;
              break;
            default:
              vpr_throw(VPR_ERROR_OTHER, __FILE__, __LINE__,
                      "Valid inputs for direction types are \"INC_DIRECTION\", \"DEC_DIRECTION\",\"BI_DIRECTION\".");
              break;
            }
        }

        flatbuffers::Offset<vtr::vpr::NodeSegment> segment;
        if (device_ctx.rr_indexed_data[node.cost_index()].seg_index != -1) {
          segment = vtr::vpr::CreateNodeSegment(fbb,
            device_ctx.rr_indexed_data[node.cost_index()].seg_index);
        }

        std::vector<flatbuffers::Offset<vtr::vpr::NodeMetadata>> metadata;
        const auto iter = device_ctx.rr_node_metadata.find(&node);
        if(iter != device_ctx.rr_node_metadata.end()) {
            const t_metadata_dict & meta = iter->second;
            add_metadata_to_flatbuffer(fbb, &metadata, meta);
        }

        nodes[inode] = vtr::vpr::CreateRrNode(
            fbb,
            inode,
            type,
            direction,
            node.capacity(),
            vtr::vpr::CreateNodeLoc(
                fbb,
                node.xlow(),
                node.ylow(),
                node.xhigh(),
                node.yhigh(),
                side,
                node.ptc_num()),
            vtr::vpr::CreateNodeTiming(
                fbb,
                node.R(),
                node.C()),
            fbb.CreateVector(metadata),
            segment);
    }

    return fbb.CreateVector(nodes);
}

/* Segment information in the t_segment_inf data structure is written out.
 * Information includes segment id, name, and optional timing parameters*/
void write_rr_segments(fstream &fp, const t_segment_inf *segment_inf, const int num_seg_types) {
    fp << "\t<segments>" << endl;

    for (int iseg = 0; iseg < num_seg_types; iseg++) {
        fp << "\t\t<segment id=\"" << iseg <<
                "\" name=\"" << segment_inf[iseg].name << "\">" << endl;
        fp << "\t\t\t<timing R_per_meter=\"" << setprecision(FLOAT_PRECISION) <<segment_inf[iseg].Rmetal <<
                "\" C_per_meter=\"" <<setprecision(FLOAT_PRECISION) << segment_inf[iseg].Cmetal << "\"/>" << endl;
        fp << "\t\t</segment>" << endl;
    }
    fp << "\t</segments>" << endl << endl;
}
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::Segment>>> write_rr_segments_flatbuffer(flatbuffers::FlatBufferBuilder &fbb, const t_segment_inf *segment_inf, const int num_seg_types) {
    std::vector<flatbuffers::Offset<vtr::vpr::Segment>> segments(num_seg_types);
    for (int iseg = 0; iseg < num_seg_types; iseg++) {
        segments[iseg] = vtr::vpr::CreateSegment(
            fbb,
            iseg,
            fbb.CreateString(segment_inf[iseg].name),
            vtr::vpr::CreateSegmentTiming(
              fbb,
              segment_inf[iseg].Rmetal,
              segment_inf[iseg].Cmetal));
    }

    return fbb.CreateVector(segments);
}

/* Switch info is written out into xml format. This includes
 * general, sizing, and optional timing information*/
void write_rr_switches(fstream &fp) {
    auto& device_ctx = g_vpr_ctx.device();
    fp << "\t<switches>" << endl;

    for (int iSwitch = 0; iSwitch < device_ctx.num_rr_switches; iSwitch++) {
        t_rr_switch_inf rr_switch = device_ctx.rr_switch_inf[iSwitch];

        fp << "\t\t<switch id=\"" << iSwitch;

        if (rr_switch.type() == SwitchType::TRISTATE) {
            fp << "\" type=\"tristate";
        } else if (rr_switch.type() == SwitchType::MUX) {
            fp << "\" type=\"mux";
        } else if (rr_switch.type() == SwitchType::PASS_GATE) {
            fp << "\" type=\"pass_gate";
        } else if (rr_switch.type() == SwitchType::SHORT) {
            fp << "\" type=\"short";
        } else if (rr_switch.type() == SwitchType::BUFFER) {
            fp << "\" type=\"buffer";
        } else {
            VPR_THROW(VPR_ERROR_ROUTE, "Invalid switch type %d\n", rr_switch.type());
        }
        fp << "\"";

        if (rr_switch.name) {
            fp << " name=\"" << rr_switch.name << "\"";
        }
        fp << ">" << endl;

        fp << "\t\t\t<timing R=\"" << setprecision(FLOAT_PRECISION) <<rr_switch.R <<
                "\" Cin=\"" << setprecision(FLOAT_PRECISION) <<rr_switch.Cin <<
                "\" Cout=\"" << setprecision(FLOAT_PRECISION) <<rr_switch.Cout <<
                "\" Tdel=\"" << setprecision(FLOAT_PRECISION) <<rr_switch.Tdel << "\"/>" << endl;
        fp << "\t\t\t<sizing mux_trans_size=\"" << setprecision(FLOAT_PRECISION) <<rr_switch.mux_trans_size <<
                "\" buf_size=\"" << setprecision(FLOAT_PRECISION) <<rr_switch.buf_size << "\"/>" << endl;
        fp << "\t\t</switch>" << endl;
    }
    fp << "\t</switches>" << endl << endl;
}
static flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::Switch>>> write_rr_switches_flatbuffer(flatbuffers::FlatBufferBuilder &fbb) {
    const auto& device_ctx = g_vpr_ctx.device();
    std::vector<flatbuffers::Offset<vtr::vpr::Switch>> switches(device_ctx.num_rr_switches);
    for (int iSwitch = 0; iSwitch < device_ctx.num_rr_switches; iSwitch++) {
        const t_rr_switch_inf &rr_switch = device_ctx.rr_switch_inf[iSwitch];

        vtr::vpr::SwitchType switch_type = vtr::vpr::SwitchType_INVALID_SWITCH_TYPE;
        switch(rr_switch.type()) {
        case SwitchType::TRISTATE:
          switch_type = vtr::vpr::SwitchType_TRISTATE;
          break;
        case SwitchType::MUX:
          switch_type = vtr::vpr::SwitchType_MUX;
          break;
        case SwitchType::PASS_GATE:
          switch_type = vtr::vpr::SwitchType_PASS_GATE;
          break;
        case SwitchType::SHORT:
          switch_type = vtr::vpr::SwitchType_SHORT;
          break;
        case SwitchType::BUFFER:
          switch_type = vtr::vpr::SwitchType_BUFFER;
          break;
        default:
            VPR_THROW(VPR_ERROR_ROUTE, "Invalid switch type %d\n", rr_switch.type());
        }

        flatbuffers::Offset<flatbuffers::String> name;
        if(rr_switch.name) {
          name = fbb.CreateString(rr_switch.name);
        }

        switches[iSwitch] = vtr::vpr::CreateSwitch(
            fbb,
            iSwitch,
            name,
            switch_type,
            vtr::vpr::CreateSwitchTiming(
                fbb,
                rr_switch.R,
                rr_switch.Cin,
                rr_switch.Cout,
                rr_switch.Tdel),
            vtr::vpr::CreateSwitchSizing(
                fbb,
                rr_switch.mux_trans_size,
                rr_switch.buf_size)
            );
    }

    return fbb.CreateVector(switches);
}

/* Block information is printed out in xml format. This includes general,
 * pin class, and pins */
void write_rr_block_types(fstream &fp) {
    auto& device_ctx = g_vpr_ctx.device();
    fp << "\t<block_types>" << endl;

    for (int iBlock = 0; iBlock < device_ctx.num_block_types; iBlock++) {
        auto& btype = device_ctx.block_types[iBlock];

        fp << "\t\t<block_type id=\"" << btype.index;

        /*since the < symbol is not allowed in xml format, converted to only print "EMPTY"*/
        if (btype.name) {
            fp << "\" name=\"" << btype.name;
        }

        fp << "\" width=\"" << btype.width << "\" height=\"" << btype.height << "\">" << endl;

        for (int iClass = 0; iClass < btype.num_class; iClass++) {
            auto& class_inf = btype.class_inf[iClass];

            char const* pin_type;
            switch (class_inf.type) {
                case OPEN:
                    pin_type = "OPEN";
                    break;
                case DRIVER:
                    pin_type = "OUTPUT";
                    break;
                case RECEIVER:
                    pin_type = "INPUT";
                    break;
                default:
                    pin_type = "NONE";
                    break;
            }

            //the pin list is printed out as the child values
            fp << "\t\t\t<pin_class type=\"" << pin_type << "\">\n";
            for (int iPin = 0; iPin < class_inf.num_pins; iPin++) {
                auto pin = class_inf.pinlist[iPin];
                fp << vtr::string_fmt(
                    "\t\t\t\t<pin ptc=\"%d\">%s</pin>\n",
                    pin, block_type_pin_index_to_name(&btype, pin).c_str());
            }
            fp << "\t\t\t</pin_class>" << endl;
        }
        fp << "\t\t</block_type>" << endl;

    }
    fp << "\t</block_types>" << endl << endl;
}
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::BlockType>>> write_rr_block_types_flatbuffer(flatbuffers::FlatBufferBuilder &fbb) {
    auto& device_ctx = g_vpr_ctx.device();

    std::vector<flatbuffers::Offset<vtr::vpr::BlockType>> block_types(device_ctx.num_block_types);
    for (int iBlock = 0; iBlock < device_ctx.num_block_types; iBlock++) {
        const auto& btype = device_ctx.block_types[iBlock];

        std::vector<flatbuffers::Offset<vtr::vpr::PinClass>> pin_classes(btype.num_class);
        for (int iClass = 0; iClass < btype.num_class; iClass++) {
            const auto& class_inf = btype.class_inf[iClass];

            vtr::vpr::PinType pin_type;
            switch (class_inf.type) {
                case OPEN:
                    pin_type = vtr::vpr::PinType_OPEN;
                    break;
                case DRIVER:
                    pin_type = vtr::vpr::PinType_OUTPUT;
                    break;
                case RECEIVER:
                    pin_type = vtr::vpr::PinType_INPUT;
                    break;
                default:
                    pin_type = vtr::vpr::PinType_NONE;
                    break;
            }

            //the pin list is printed out as the child values
            std::vector<flatbuffers::Offset<vtr::vpr::Pin>> pins(class_inf.num_pins);
            for (int iPin = 0; iPin < class_inf.num_pins; iPin++) {
                const auto &pin = class_inf.pinlist[iPin];
                pins[iPin] = vtr::vpr::CreatePin(
                    fbb,
                    pin,
                    fbb.CreateString(block_type_pin_index_to_name(&btype, pin)));
            }

            pin_classes[iClass] = vtr::vpr::CreatePinClass(
                fbb,
                pin_type,
                fbb.CreateVector(pins));
        }

        flatbuffers::Offset<flatbuffers::String> name;
        if(btype.name) {
          name = fbb.CreateString(btype.name);
        }

        block_types[iBlock] = vtr::vpr::CreateBlockType(
            fbb,
            btype.index,
            name,
            btype.width,
            btype.height,
            fbb.CreateVector(pin_classes));
    }

    return fbb.CreateVector(block_types);
}

/* Grid information is printed out in xml format. Each grid location
 * and its relevant information is included*/
void write_rr_grid(fstream &fp) {
    auto& device_ctx = g_vpr_ctx.device();

    fp << "\t<grid>" << endl;

    for (size_t x = 0; x < device_ctx.grid.width(); x++) {
        for (size_t y = 0; y < device_ctx.grid.height(); y++) {
            t_grid_tile grid_tile = device_ctx.grid[x][y];

            fp << "\t\t<grid_loc x=\"" << x << "\" y=\"" << y <<
                    "\" block_type_id=\"" << grid_tile.type->index <<
                    "\" width_offset=\"" << grid_tile.width_offset <<
                    "\" height_offset=\"" << grid_tile.height_offset << "\"/>" << endl;
        }
    }
    fp << "\t</grid>" << endl << endl;
}
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::GridLoc>>> write_rr_grid_flatbuffer(flatbuffers::FlatBufferBuilder &fbb) {
    auto& device_ctx = g_vpr_ctx.device();

    std::vector<flatbuffers::Offset<vtr::vpr::GridLoc>> grid;
    grid.reserve(device_ctx.grid.width()*device_ctx.grid.height());
    for (size_t x = 0; x < device_ctx.grid.width(); x++) {
        for (size_t y = 0; y < device_ctx.grid.height(); y++) {
            const t_grid_tile &grid_tile = device_ctx.grid[x][y];
            grid.push_back(vtr::vpr::CreateGridLoc(
                fbb,
                x, y,
                grid_tile.type->index,
                grid_tile.width_offset,
                grid_tile.height_offset));
        }
    }

    return fbb.CreateVector(grid);
}

/* Edges connecting to each rr node is printed out. The two nodes
 * it connects to are also printed*/
void write_rr_edges(fstream &fp) {
    auto& device_ctx = g_vpr_ctx.device();
    fp << "\t<rr_edges>" << endl;

    for (size_t inode = 0; inode < device_ctx.rr_nodes.size(); inode++) {
        auto& node = device_ctx.rr_nodes[inode];
        const auto edge_metadata_iter = device_ctx.rr_edge_metadata.find(&node);
        for (int iedge = 0; iedge < node.num_edges(); iedge++) {
            fp << "\t\t<edge src_node=\"" << inode <<
                    "\" sink_node=\"" << node.edge_sink_node(iedge) <<
                    "\" switch_id=\"" << node.edge_switch(iedge) << "\"";
            bool wrote_edge_metadata = false;
            if(edge_metadata_iter != device_ctx.rr_edge_metadata.end()) {
                auto iter = edge_metadata_iter->second.find(std::make_pair(node.edge_sink_node(iedge), node.edge_switch(iedge)));
                if(iter != edge_metadata_iter->second.end()) {
                    fp << ">" << endl;

                    const t_metadata_dict & meta = iter->second;
                    add_metadata_to_xml(fp, "\t\t\t", meta);
                    wrote_edge_metadata = true;
                }
            }

            if(wrote_edge_metadata == false) {
              fp << "/>" << endl;
            } else {
              fp << "\t\t</edge>" << endl;
            }
        }
    }
    fp << "\t</rr_edges>" << endl << endl;
}
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<vtr::vpr::RrEdge>>> write_rr_edges_flatbuffer(flatbuffers::FlatBufferBuilder &fbb) {
    auto& device_ctx = g_vpr_ctx.device();

    int num_edges = 0;
    for (size_t inode = 0; inode < device_ctx.rr_nodes.size(); inode++) {
        num_edges += device_ctx.rr_nodes[inode].num_edges();
    }

    std::vector<flatbuffers::Offset<vtr::vpr::RrEdge>> edges;
    edges.reserve(num_edges);
    for (size_t inode = 0; inode < device_ctx.rr_nodes.size(); inode++) {
        auto& node = device_ctx.rr_nodes[inode];
        const auto edge_metadata_iter = device_ctx.rr_edge_metadata.find(&node);
        for (int iedge = 0; iedge < node.num_edges(); iedge++) {
            std::vector<flatbuffers::Offset<vtr::vpr::NodeMetadata>> metadata;

            int32_t sink_node = node.edge_sink_node(iedge);
            int32_t switch_id = node.edge_switch(iedge);

            if(edge_metadata_iter != device_ctx.rr_edge_metadata.end()) {
                auto iter = edge_metadata_iter->second.find(std::make_pair(sink_node, switch_id));
                if(iter != edge_metadata_iter->second.end()) {
                  const t_metadata_dict & meta = iter->second;
                  add_metadata_to_flatbuffer(fbb, &metadata, meta);
                }
            }

            edges.push_back(vtr::vpr::CreateRrEdge(
                fbb,
                inode,
                sink_node,
                switch_id,
                fbb.CreateVector(metadata)));
        }
    }
    return fbb.CreateVector(edges);
}
