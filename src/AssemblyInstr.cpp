/**
 * @file AssemblyInstr.cpp
 * @author Matt Chu
 * @brief Assembly Instruction Class
 * @date 2022-04-02
 * 
 */
#include <cmath>
#include <cstring>
#include "AssemblyInstr.h"

namespace mpc::Chip8 {

const uint8_t AssemblyInstr::paramBitwidths[AssemblyInstr::maxOpType+1][AssemblyInstr::maxNumParams] = {
        { AssemblyInstr::nibbleBits, 0, 0 },                                                 // Opcode: 00EN
        { AssemblyInstr::addrBits, 0, 0 },                                                   // Opcode: 1NNN
        { AssemblyInstr::addrBits, 0, 0 },                                                   // Opcode: 2NNN
        { AssemblyInstr::nibbleBits, AssemblyInstr::byteBits, 0 },                           // Opcode: 3XNN
        { AssemblyInstr::nibbleBits, AssemblyInstr::byteBits, 0 },                           // Opcode: 4XNN
        { AssemblyInstr::nibbleBits, AssemblyInstr::nibbleBits, 0 },                         // Opcode: 5XY0
        { AssemblyInstr::nibbleBits, AssemblyInstr::byteBits, 0 },                           // Opcode: 6XNN
        { AssemblyInstr::nibbleBits, AssemblyInstr::byteBits, 0 },                           // Opcode: 7XNN
        { AssemblyInstr::nibbleBits, AssemblyInstr::nibbleBits, AssemblyInstr::nibbleBits }, // Opcode: 8XYN
        { AssemblyInstr::nibbleBits, AssemblyInstr::nibbleBits, 0 },                         // Opcode: 9XY0
        { AssemblyInstr::addrBits, 0, 0 },                                                   // Opcode: ANNN
        { AssemblyInstr::addrBits, 0, 0 },                                                   // Opcode: BNNN
        { AssemblyInstr::nibbleBits, AssemblyInstr::byteBits, 0 },                           // Opcode: CXNN
        { AssemblyInstr::nibbleBits, AssemblyInstr::nibbleBits, AssemblyInstr::nibbleBits }, // Opcode: DXYN
        { AssemblyInstr::nibbleBits, 0, 0 },                                                 // Opcode: EXKN
        { AssemblyInstr::nibbleBits, AssemblyInstr::byteBits, 0 },                           // Opcode: FXNN
};

AssemblyInstr::AssemblyInstr(uint16_t opcode) : opcode(opcode) {
    opType = static_cast<OpType>((opcode & 0xF000) >> 12);
    memset(params, 0, sizeof(params));
}

AssemblyInstr::~AssemblyInstr() {

}

/**
 * @fn OpType AssemblyInstr::getOpType() const
 * @brief Gets the decoded opcode type.
 * @return Decoded opcode type
 */
OpType AssemblyInstr::getOpType() const {
    return opType;
}

/**
 * @fn uint16_t AssemblyInstr::getParam(uint8_t pos) const
 * @brief Gets a positional parameter decoded from the 16-bit opcode. 
 *        If pos is out of range, returns -1.
 * @param[in] pos Positional index of parameter
 * @return Decoded parameter, otherwise -1 on index out of bounds error 
 */
uint16_t AssemblyInstr::getParam(uint8_t pos) const {
    return (pos < AssemblyInstr::maxNumParams) ? params[pos] : -1;
}

/**
 * @fn int AssemblyInstr::decode()
 * @brief Decodes the 16-bit opcode this object was instantiated with.
 * @return 0 on success, negative otherwise 
 */
int AssemblyInstr::decode() {
    int status = 0;

    // Decode the parameters according to opcode type
    if (opType == OpType::OP_00EN) {
        params[0] = opcode & 0x000F;
    }
    else if (opType == OpType::OP_EXKN) {
        params[0] = (opcode & 0x0F00) >> 8;
        params[1] = opcode & 0x000F;
    }
    else {
        uint8_t startBit = 16 - AssemblyInstr::nibbleBits;
        uint8_t paramPos = 0;
        for (auto &bitwidth : paramBitwidths[static_cast<uint8_t>(opType)]) {
            uint8_t bitmask = std::pow(2, startBit) - 1;
            uint8_t endBit = startBit - bitwidth;

            // Decode the positional opcode parameter
            params[paramPos++] = (opcode & bitmask) >> endBit;

            // Move the start bit position to the next param field
            startBit = endBit;
        }
    }

    return status;
}
    
}