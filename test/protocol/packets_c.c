/* Compiles the protocol header as C, as the touch controller does, and reports its sizes and
 * offsets so the C++ test can check C and C++ agree on the layout. */
#include <stddef.h>

#include "gait_core/protocol/packets.h"

size_t hexapod_c_control_size(void) { return sizeof(ControlPacket); }
size_t hexapod_c_hex_size(void) { return sizeof(HexPacket); }
size_t hexapod_c_control_command_offset(void) { return offsetof(ControlPacket, command); }
size_t hexapod_c_hex_flags_offset(void) { return offsetof(HexPacket, flags); }
size_t hexapod_c_hex_num_gaits_offset(void) { return offsetof(HexPacket, numGaits); }
int hexapod_c_feature_count(void) { return NUM_FEATURES; }
