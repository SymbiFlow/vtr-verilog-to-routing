#!/bin/bash -e

VTR_ROOT=$(pwd)/../..
VPR=$VTR_ROOT/vpr/vpr
ARCH=$VTR_ROOT/vtr_flow/arch/timing/k6_frac_N10_mem32K_40nm.xml
TEST_DIR=$(mktemp -d)

cd $TEST_DIR
"$VPR" "$ARCH" "$VTR_ROOT/vpr/test/wire.eblif" --write_rr_graph test.xml --pack --place
"$VPR" "$ARCH" "$VTR_ROOT/vpr/test/wire.eblif" --write_rr_graph test.rrgraph --pack --place

# Check re-serialization to XML.
"$VPR" "$ARCH" "$VTR_ROOT/vpr/test/wire.eblif" --read_rr_graph test.xml --write_rr_graph test_xml_from_xml.xml --pack --place --route_chan_width 300
"$VPR" "$ARCH" "$VTR_ROOT/vpr/test/wire.eblif" --read_rr_graph test.rrgraph --write_rr_graph test_xml_from_flatbuffer.xml --pack --place --route_chan_width 300
diff test.xml test_xml_from_xml.xml
diff test.xml test_xml_from_flatbuffer.xml

# Check re-serialization to flatbuffer.
"$VPR" "$ARCH" "$VTR_ROOT/vpr/test/wire.eblif" --read_rr_graph test.xml --write_rr_graph test_flatbuffer_from_xml.rrgraph --pack --place --route_chan_width 300
"$VPR" "$ARCH" "$VTR_ROOT/vpr/test/wire.eblif" --read_rr_graph test.rrgraph --write_rr_graph test_flatbuffer_from_flatbuffer.rrgraph --pack --place --route_chan_width 300
diff test.rrgraph test_flatbuffer_from_xml.rrgraph
diff test.rrgraph test_flatbuffer_from_flatbuffer.rrgraph
cd -
rm -r $TEST_DIR
