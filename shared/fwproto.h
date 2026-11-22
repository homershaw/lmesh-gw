#pragma once
/*
 * Node firmware update over the mesh (LMESH UDP port 69, both directions).
 * The gateway drives; the node answers every packet with an ACK that names
 * the next byte it expects, so lost packets, duplicates and a restarted
 * node all sort themselves out from that one number.
 *
 *  gateway -> node
 *    BEGIN  [1][size u32][crc32 u32]      same size+crc as a transfer in progress = resume
 *    DATA   [2][offset u32][crc16 u16 of the bytes][bytes...]
 *    END    [3]                           node checks size + crc32, verifies the image, restarts
 *    ABORT  [4]
 *  node -> gateway
 *    ACK    [0x81][expected u32][status u8][text...]
 *
 * All integers little-endian.
 */
#define LMESH_PORT_FW   69

#define FW_BEGIN  1
#define FW_DATA   2
#define FW_END    3
#define FW_ABORT  4
#define FW_ACK    0x81

#define FW_ST_OK    0      /* receiving */
#define FW_ST_DONE  1      /* image verified, node restarts into it */
#define FW_ST_ERR   2      /* text says why; transfer is closed */

#define FW_DATA_HDR 7      /* op + offset + crc16 */
#define FW_CHUNK    182    /* data bytes per packet: fits one radio frame with the headers */
