/**
 * @file AssemblyInstr.h
 * @author Matt Chu
 * @brief Assembly Instruction Class
 * @date 2022-04-02
 * 
 */
#ifndef _MPC_ASSEMBLY_INSTR_H_
#define _MPC_ASSEMBLY_INSTR_H_

#include <cstdint>

namespace mpc::Chip8 {

/**
 * @enum OpType 
 * Enum for the different CHIP-8 opcode types indicated by the most significant nibble.
 */
enum class OpType : uint8_t {
    OP_00EN             = 0x0, /**< 0x0 */
    OP_1NNN_JP_ADDR     = 0x1, /**< 0x1 */
    OP_2NNN_CALL_ADDR   = 0x2, /**< 0x2 */
    OP_3XNN_SE_VX_NN    = 0x3, /**< 0x3 */
    OP_4XNN_SNE_VX_NN   = 0x4, /**< 0x4 */
    OP_5XY0_SE_VX_VY    = 0x5, /**< 0x5 */
    OP_6XNN_LD_VX_NN    = 0x6, /**< 0x6 */
    OP_7XNN_ADD_VX_NN   = 0x7, /**< 0x7 */
    OP_8XYN             = 0x8, /**< 0x8 */
    OP_9XY0_SNE_VX_VY   = 0x9, /**< 0x9 */
    OP_ANNN_LD_I_ADDR   = 0xA, /**< 0xA */
    OP_BNNN_JP_V0       = 0xB, /**< 0xB */
    OP_CXNN_RND_VX_NN   = 0xC, /**< 0xC */
    OP_DXYN_DRW_VX_VY_N = 0xD, /**< 0xD */
    OP_EXKN             = 0xE, /**< 0xE */
    OP_FXNN             = 0xF  /**< 0xF */
};

/**
 * @enum OpSubtype
 * Enum for CHIP-8 opcode subtypes for opcodes 00EN, 8XYN, EXKN, and FXNN. 
 * Bitwidths and bit positions are dependent on the main opcode type.
 */
enum class OpSubtype : uint8_t {
    // Opcode 00EN subtypes
    OP_00E0_CLS = 0x0, /**< 0x0 */
    OP_00EE_RET = 0xE, /**< 0xE */

    // Opcode 8XYN subtypes
    OP_8XY0_LD_VX_VY   = 0x0, /**< 0x0 */
    OP_8XY1_OR_VX_VY   = 0x1, /**< 0x1 */
    OP_8XY2_AND_VX_VY  = 0x2, /**< 0x2 */
    OP_8XY3_XOR_VX_VY  = 0x3, /**< 0x3 */
    OP_8XY4_ADD_VX_VY  = 0x4, /**< 0x4 */
    OP_8XY5_SUB_VX_VY  = 0x5, /**< 0x5 */
    OP_8XY6_SHR_VX     = 0x6, /**< 0x6 */
    OP_8XY7_SUBN_VX_VY = 0x7, /**< 0x7 */
    OP_8XYE_SHL_VX_VY  = 0xE, /**< 0xE */

    // Opcode EXKN subtypes
    OP_EXA1_SKNP_VX = 0x1, /**< 0x1 */
    OP_EX9E_SKP_VX  = 0xE, /**< 0xE */

    // Opcode FXNN subtypes
    OP_FX07_LD_VX_DT = 0x07, /**< 0x07 */
    OP_FX0A_LD_VX_K  = 0x0A, /**< 0x0A */
    OP_FX15_LD_DT_VX = 0x15, /**< 0x15 */
    OP_FX18_LD_ST_VX = 0x18, /**< 0x18 */
    OP_FX1E_ADD_I_VX = 0x1E, /**< 0x1E */
    OP_FX29_LD_F_VX  = 0x29, /**< 0x29 */
    OP_FX33_LD_B_VX  = 0x33, /**< 0x33 */
    OP_FX55_LD_I_VX  = 0x55, /**< 0x55 */
    OP_FX65_LD_VX_I  = 0x65  /**< 0x65 */
};

/** @class AssemblyInstr AssemblyInstr.h "AssemblyInstr.h"
 *  @brief Assembly Instruction Class
 */
class AssemblyInstr {
public:
    AssemblyInstr() = delete;

    AssemblyInstr(uint16_t opcode);

    virtual ~AssemblyInstr();

    OpType getOpType() const;

    uint16_t getParam(uint8_t pos) const;

    virtual int decode();

    static const uint8_t minOpType = static_cast<uint8_t>(OpType::OP_00EN); /**< Minimum opcode type */

    static const uint8_t maxOpType = static_cast<uint8_t>(OpType::OP_FXNN); /**< Maximum opcode type */

    static const uint8_t maxNumParams = 3; /**< Maximum number of parameters an opcode can take */

    static const uint8_t nibbleBits = 4; /**< Number of bits in a nibble */

    static const uint8_t byteBits = 8; /**< Number of bits in a byte */

    static const uint8_t addrBits = 12; /**< Number of bits in a CHIP-8 memory address */

private:
    static const uint8_t paramBitwidths[AssemblyInstr::maxOpType+1][AssemblyInstr::maxNumParams]; /**< Parameter bitwidths for each opcode type */

    uint16_t opcode; /**< 16-bit opcode */

    OpType opType; /**< Decoded opcode type */

    uint16_t params[AssemblyInstr::maxNumParams]; /**< Decoded opcode arguments */
};

}

#endif