/**
 * @file Chip8.cpp
 * @author Matt Chu
 * @brief CHIP-8 Emulator Class
 * @date 2022-04-01
 * 
 */
#include <cstring>
#include <fstream>
#include "Chip8.h"

namespace mpc::Chip8 {

const uint8_t Chip8::fontset[Chip8::numBuiltInChars][Chip8::charSizeBytes] = {
        { 0xF0, 0x90, 0x90, 0x90, 0xF0 }, // 0
        { 0x20, 0x60, 0x20, 0x20, 0x70 }, // 1
        { 0xF0, 0x10, 0xF0, 0x80, 0xF0 }, // 2
        { 0xF0, 0x10, 0xF0, 0x10, 0xF0 }, // 3
        { 0x90, 0x90, 0xF0, 0x10, 0x10 }, // 4
        { 0xF0, 0x80, 0xF0, 0x10, 0xF0 }, // 5
        { 0xF0, 0x80, 0xF0, 0x90, 0xF0 }, // 6
        { 0xF0, 0x10, 0x20, 0x40, 0x40 }, // 7
        { 0xF0, 0x90, 0xF0, 0x90, 0xF0 }, // 8
        { 0xF0, 0x90, 0xF0, 0x10, 0xF0 }, // 9
        { 0xF0, 0x90, 0xF0, 0x90, 0x90 }, // A
        { 0xE0, 0x90, 0xE0, 0x90, 0xE0 }, // B
        { 0xF0, 0x80, 0x80, 0x80, 0xF0 }, // C
        { 0xE0, 0x90, 0x90, 0x90, 0xE0 }, // D
        { 0xF0, 0x80, 0xF0, 0x80, 0xF0 }, // E
        { 0xF0, 0x80, 0xF0, 0x80, 0x80 }  // F
};

/**
 * @fn Chip8::Chip8()
 * @brief Chip8 Class Constructor
 * @return none
 */
Chip8::Chip8() {
    indexRegister = 0;

    // 0x000 - 0x1FF are reserved memory addresses so instructions must start at 0x200
    programCounter = Chip8::instrStartAddr;
    stackPtr = 0;
    delayTimer = 0;
    soundTimer = 0;
    opcode = 0;
    memset(registers, 0, sizeof(registers));
    memset(memory, 0, sizeof(memory));
    memset(stack, 0, sizeof(stack));
    memset(keypad, 0, sizeof(keypad));
    memset(videoMem, 0, sizeof(videoMem));

    // Initialize the random number generator
    std::random_device randSeed;
    randGen = std::mt19937(randSeed());
    randByte = std::uniform_int_distribution<uint8_t>(0x00, 0xFF);

    // Load data into reserved memory
    Chip8::loadFont();

    // Initialize the opcode tables
    Chip8::initOpcodeTables();
}

/**
 * @fn Chip8::~Chip8()
 * @brief Chip8 Class Destructor
 * @return none
 */
Chip8::~Chip8() {

}

/**
 * @fn int Chip8::loadRom(std::string fileName)
 * @brief Loads a binary ROM file into the CHIP-8 memory.
 * @param[in] fileName Path of file to load
 * @return 0 on success, negative otherwise 
 */
int Chip8::loadRom(std::string fileName) {
    int status = 0;
    
    // Open the file as a binary stream and move the file pointer to the end
    std::ifstream file(fileName, std::ios::binary | std::ios::ate);
    if (file.is_open()) {
        // Get the size of the file and allocate a buffer to hold the contents
        auto size = file.tellg();
        uint8_t *buffer = new uint8_t[size];
        if (buffer == nullptr) {
            status = -1;
        } 
        else if (size > Chip8::memorySize - Chip8::instrStartAddr) {
            status = -2;
            delete[] buffer;
        } 
        else {
            // Go back to the beginning of the file and fill the buffer
            file.seekg(0, std::ios::beg);
            file.read(reinterpret_cast<char*>(buffer), size);
            file.close();

            // Load the ROM contents into the CHIP-8's memory, starting at 0x200
            if (size <= Chip8::memorySize - Chip8::instrStartAddr) {
                std::memcpy(&memory[Chip8::instrStartAddr], buffer, size);
            } 
            else {
                status = -3;
            }

            delete[] buffer;
        }
    }

    return status;
}

/**
 * @fn int Chip8::DecodeOpcode(uint16_t opcode,
 *                             uint8_t &instructionType,
 *                             uint8_t &x,
 *                             uint8_t &y,
 *                             uint8_t &lsn,
 *                             uint8_t &lsb,
 *                             uint16_t &addr)
 * @brief Decodes a 16-bit opcode instruction. Nibbles will be referred to in this order: 3333 2222 1111 0000
 * @param[in] opcode 16-bit opcode
 * @param[out] instructionType Most significant nibble, indicates instruction type
 * @param[out] x Second nibble, used to look up one of the 16-bit registers (Vx)
 * @param[out] y First nibble, used to look up one of the 16-bit registers (Vy)
 * @param[out] lsn Least significant nibble, usually used as a value argument
 * @param[out] lsb Least significant byte, usually used as a value argument
 * @param[out] addr The least significant 12 bits, a memory address
 * @return 0 on success, negative otherwise 
 */
int Chip8::DecodeOpcode(uint16_t opcode,
                        uint8_t &instructionType,
                        uint8_t &x,
                        uint8_t &y,
                        uint8_t &lsn,
                        uint8_t &lsb,
                        uint16_t &addr) {
    int status = 0;
    instructionType = (opcode & 0xF000) >> 12;
    x = (opcode & 0x0F00) >> 8;
    y = (opcode & 0x00F0) >> 4;
    lsn = opcode & 0x000F;
    lsb = opcode & 0x00FF;
    addr = opcode & 0x0FFF;
    return status;
}

/**
 * @fn int Chip8::loadFont()
 * @brief Loads CHIP-8 character font data into reserved memory.
 * @return 0 on success, negative otherwise 
 */
int Chip8::loadFont() {
    int status = 0;

    // Load character fonts into reserved memory space
    uint16_t fontMemAddr = Chip8::fontsetStartAddr;
    for (uint8_t iFont = 0; iFont < Chip8::numBuiltInChars; iFont++) {
        for (uint8_t fontByte = 0; fontByte < Chip8::charSizeBytes; fontByte++) {
            memory[fontMemAddr++] = fontset[iFont][fontByte];
        }
    }

    return status;
}

/**
 * @fn int Chip8::initOpcodeTables()
 * @brief Initializes opcode function lookup tables.
 * @return 0 on success, negative otherwise 
 */
int Chip8::initOpcodeTables() {
    int status = 0;

    // Initialize the opcode tables with a function that does nothing for safety
    for (auto &opcodeFn : mainOpcodeTable) {
        opcodeFn = &Chip8::opNULL;
    }
    for (auto &opcodeFn : opcodeTable_00En) {
        opcodeFn = &Chip8::opNULL;
    }
    for (auto &opcodeFn : opcodeTable_8xyn) {
        opcodeFn = &Chip8::opNULL;
    }
    for (auto &opcodeFn : opcodeTable_ExKn) {
        opcodeFn = &Chip8::opNULL;
    }
    for (auto &opcodeFn : opcodeTable_Fxnn) {
        opcodeFn = &Chip8::opNULL;
    }

    // Index each opcode function by their appropriate opcodes
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_00EN)            ] = &Chip8::op00En;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_1NNN_JP_ADDR)    ] = &Chip8::op1nnn_JP_addr;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_2NNN_CALL_ADDR)  ] = &Chip8::op2nnn_CALL_addr;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_3XNN_SE_VX_NN)   ] = &Chip8::op3xnn_SE_Vx_byte;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_4XNN_SNE_VX_NN)  ] = &Chip8::op4xnn_SNE_Vx_byte;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_5XY0_SE_VX_VY)   ] = &Chip8::op5xy0_SE_Vx_Vy;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_6XNN_LD_VX_NN)   ] = &Chip8::op6xnn_LD_Vx_byte;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_7XNN_ADD_VX_NN)  ] = &Chip8::op7xnn_ADD_Vx_byte;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_8XYN)            ] = &Chip8::op8xyn;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_9XY0_SNE_VX_VY)  ] = &Chip8::op9xy0_SNE_Vx_Vy;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_ANNN_LD_I_ADDR)  ] = &Chip8::opAnnn_LD_I_addr;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_BNNN_JP_V0)      ] = &Chip8::opBnnn_JP_V0;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_CXNN_RND_VX_NN)  ] = &Chip8::opCxnn_RND_Vx_byte;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_DXYN_DRW_VX_VY_N)] = &Chip8::opDxyn_DRW_Vx_Vy_nibble;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_EXKN)            ] = &Chip8::opExKn;
    mainOpcodeTable[static_cast<uint8_t>(OpType::OP_FXNN)            ] = &Chip8::opFxnn;

    // Index the below opcodes by their appropriate subtype
    // Opcodes 00EN
    opcodeTable_00En[static_cast<uint8_t>(OpSubtype::OP_00E0_CLS)] = &Chip8::op00E0_CLS;
    opcodeTable_00En[static_cast<uint8_t>(OpSubtype::OP_00EE_RET)] = &Chip8::op00EE_RET;

    // Opcodes 8XYN
    opcodeTable_8xyn[static_cast<uint8_t>(OpSubtype::OP_8XY0_LD_VX_VY)  ] = &Chip8::op8xy0_LD_Vx_Vy;
    opcodeTable_8xyn[static_cast<uint8_t>(OpSubtype::OP_8XY1_OR_VX_VY)  ] = &Chip8::op8xy1_OR_Vx_Vy;
    opcodeTable_8xyn[static_cast<uint8_t>(OpSubtype::OP_8XY2_AND_VX_VY) ] = &Chip8::op8xy2_AND_Vx_Vy;
    opcodeTable_8xyn[static_cast<uint8_t>(OpSubtype::OP_8XY3_XOR_VX_VY) ] = &Chip8::op8xy3_XOR_Vx_Vy;
    opcodeTable_8xyn[static_cast<uint8_t>(OpSubtype::OP_8XY4_ADD_VX_VY) ] = &Chip8::op8xy4_ADD_Vx_Vy;
    opcodeTable_8xyn[static_cast<uint8_t>(OpSubtype::OP_8XY5_SUB_VX_VY) ] = &Chip8::op8xy5_SUB_Vx_Vy;
    opcodeTable_8xyn[static_cast<uint8_t>(OpSubtype::OP_8XY6_SHR_VX)    ] = &Chip8::op8xy6_SHR_Vx;
    opcodeTable_8xyn[static_cast<uint8_t>(OpSubtype::OP_8XY7_SUBN_VX_VY)] = &Chip8::op8xy7_SUBN_Vx_Vy;
    opcodeTable_8xyn[static_cast<uint8_t>(OpSubtype::OP_8XYE_SHL_VX_VY) ] = &Chip8::op8xyE_SHL_Vx_Vy;

    // Opcodes EXKN
    opcodeTable_ExKn[static_cast<uint8_t>(OpSubtype::OP_EXA1_SKNP_VX)] = &Chip8::opExA1_SKNP_Vx;
    opcodeTable_ExKn[static_cast<uint8_t>(OpSubtype::OP_EX9E_SKP_VX) ] = &Chip8::opEx9E_SKP_Vx;

    // Opcodes FXNN
    opcodeTable_Fxnn[static_cast<uint8_t>(OpSubtype::OP_FX07_LD_VX_DT)] = &Chip8::opFx07_LD_Vx_DT;
    opcodeTable_Fxnn[static_cast<uint8_t>(OpSubtype::OP_FX0A_LD_VX_K) ] = &Chip8::opFx0A_LD_Vx_K;
    opcodeTable_Fxnn[static_cast<uint8_t>(OpSubtype::OP_FX15_LD_DT_VX)] = &Chip8::opFx15_LD_DT_Vx;
    opcodeTable_Fxnn[static_cast<uint8_t>(OpSubtype::OP_FX18_LD_ST_VX)] = &Chip8::opFx18_LD_ST_Vx;
    opcodeTable_Fxnn[static_cast<uint8_t>(OpSubtype::OP_FX1E_ADD_I_VX)] = &Chip8::opFx1E_ADD_I_Vx;
    opcodeTable_Fxnn[static_cast<uint8_t>(OpSubtype::OP_FX29_LD_F_VX) ] = &Chip8::opFx29_LD_F_Vx;
    opcodeTable_Fxnn[static_cast<uint8_t>(OpSubtype::OP_FX33_LD_B_VX) ] = &Chip8::opFx33_LD_B_Vx;
    opcodeTable_Fxnn[static_cast<uint8_t>(OpSubtype::OP_FX55_LD_I_VX) ] = &Chip8::opFx55_LD_I_Vx;
    opcodeTable_Fxnn[static_cast<uint8_t>(OpSubtype::OP_FX65_LD_VX_I) ] = &Chip8::opFx65_LD_Vx_I;

    return status;
}

// Opcode emulation functions
/**
 * @fn void Chip8::op00E0_CLS(const AssemblyInstr &instr)
 * @brief Opcode 00E0: Clears the screen.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op00E0_CLS(const AssemblyInstr &instr) {
    memset(videoMem, 0, sizeof(videoMem));
}

/**
 * @fn void Chip8::op00EE_RET(const AssemblyInstr &instr)
 * @brief Opcode 00EE: Returns from a subroutine.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op00EE_RET(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::op1nnn_JP_addr(const AssemblyInstr &instr)
 * @brief Opcode 1nnn: Jumps to a memory address.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op1nnn_JP_addr(const AssemblyInstr &instr) {
    uint16_t addr = instr.getParam(0);
    if (addr <= Chip8::chip8HighAddr) {
        programCounter = addr;
    }
}

/**
 * @fn void Chip8::op2nnn_CALL_addr(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op2nnn_CALL_addr(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::op3xnn_SE_Vx_byte(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op3xnn_SE_Vx_byte(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::op4xnn_SNE_Vx_byte(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op4xnn_SNE_Vx_byte(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::op5xy0_SE_Vx_Vy(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op5xy0_SE_Vx_Vy(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::op6xnn_LD_Vx_byte(const AssemblyInstr &instr)
 * @brief Opcode 6xnn: Sets register Vx with value nn.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op6xnn_LD_Vx_byte(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t nn = instr.getParam(1);
    if (x < Chip8::numRegisters) {
        registers[x] = nn;
    }
}

/**
 * @fn void Chip8::op7xnn_ADD_Vx_byte(const AssemblyInstr &instr)
 * @brief Opcode 7xnn: Adds value nn to register Vx.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op7xnn_ADD_Vx_byte(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t nn = instr.getParam(1);
    if (x < Chip8::numRegisters) {
        registers[x] += nn;
    }
}

/**
 * @fn void Chip8::op8xy0_LD_Vx_Vy(const AssemblyInstr &instr)
 * @brief Opcode 8xy0: Sets Vx to the value of Vy.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xy0_LD_Vx_Vy(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t y = instr.getParam(1);
    if (x < Chip8::numRegisters && y < Chip8::numRegisters) {
        registers[x] = registers[y];
    }
}

/**
 * @fn void Chip8::op8xy1_OR_Vx_Vy(const AssemblyInstr &instr)
 * @brief Opcode 8xy1: Sets Vx to the bitwise OR value of Vx and Vy.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xy1_OR_Vx_Vy(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t y = instr.getParam(1);
    if (x < Chip8::numRegisters && y < Chip8::numRegisters) {
        registers[x] |= registers[y];
    }
}

/**
 * @fn void Chip8::op8xy2_AND_Vx_Vy(const AssemblyInstr &instr)
 * @brief Opcode 8xy2: Sets Vx to the bitwise AND value of Vx and Vy.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xy2_AND_Vx_Vy(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t y = instr.getParam(1);
    if (x < Chip8::numRegisters && y < Chip8::numRegisters) {
        registers[x] &= registers[y];
    }
}

/**
 * @fn void Chip8::op8xy3_XOR_Vx_Vy(const AssemblyInstr &instr)
 * @brief Opcode 8xy2: Sets Vx to the bitwise XOR value of Vx and Vy.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xy3_XOR_Vx_Vy(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t y = instr.getParam(1);
    if (x < Chip8::numRegisters && y < Chip8::numRegisters) {
        registers[x] ^= registers[y];
    }
}

/**
 * @fn void Chip8::op8xy4_ADD_Vx_Vy(const AssemblyInstr &instr)
 * @brief Opcode 8xy4: Sets Vx to the sum of Vx and Vy. 
 *        This addition will affect the carry flag (VF) in the case of bit overflow.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xy4_ADD_Vx_Vy(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t y = instr.getParam(1);
    if (x < Chip8::numRegisters && y < Chip8::numRegisters) {
        uint16_t sum = registers[x] + registers[y];
        registers[x] = sum;
        registers[Chip8::opResultReg] = (sum > 0xFF);
    }
}

/**
 * @fn void Chip8::op8xy5_SUB_Vx_Vy(const AssemblyInstr &instr)
 * @brief Opcode 8xy5: Sets Vx to the value of Vx minus Vy. 
 *        If the first operand is larger than the second operand, the carry flag (VF) will be set to 1.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xy5_SUB_Vx_Vy(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t y = instr.getParam(1);
    if (x < Chip8::numRegisters && y < Chip8::numRegisters) {
        registers[Chip8::opResultReg] = (registers[x] >= registers[y]);
        registers[x] -= registers[y];
    }
}

/**
 * @fn void Chip8::op8xy6_SHR_Vx(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xy6_SHR_Vx(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::op8xy7_SUBN_Vx_Vy(const AssemblyInstr &instr)
 * @brief Opcode 8xy7: Sets Vx to the value of Vx minus Vy. 
 *        If the first operand is larger than the second operand, the carry flag (VF) will be set to 1.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xy7_SUBN_Vx_Vy(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t y = instr.getParam(1);
    if (x < Chip8::numRegisters && y < Chip8::numRegisters) {
        registers[Chip8::opResultReg] = (registers[y] >= registers[x]);
        registers[x] = (registers[y] - registers[x]);
    }
}

/**
 * @fn void Chip8::op8xyE_SHL_Vx_Vy(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xyE_SHL_Vx_Vy(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::op9xy0_SNE_Vx_Vy(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op9xy0_SNE_Vx_Vy(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opAnnn_LD_I_addr(const AssemblyInstr &instr)
 * @brief Opcode Annn: Sets index register I with address nnn.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opAnnn_LD_I_addr(const AssemblyInstr &instr) {
    uint16_t addr = instr.getParam(0);
    if (addr <= Chip8::chip8HighAddr) {
        indexRegister = addr;
    }
}

/**
 * @fn void Chip8::opBnnn_JP_V0(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opBnnn_JP_V0(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opCxnn_RND_Vx_byte(const AssemblyInstr &instr)
 * @brief Opcode Cxnn: Generates a random number, binary ANDs it with value nn, and stores the result in Vx.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opCxnn_RND_Vx_byte(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t nn = instr.getParam(1);
    if (x < Chip8::numRegisters) {
        registers[x] = (Chip8::getRandByte() & nn);
    }
}

/**
 * @fn void Chip8::opDxyn_DRW_Vx_Vy_nibble(const AssemblyInstr &instr)
 * @brief Opcode Dxyn: Display instruction
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opDxyn_DRW_Vx_Vy_nibble(const AssemblyInstr &instr) {
    uint8_t x = instr.getParam(0);
    uint8_t y = instr.getParam(1);
    uint8_t n = instr.getParam(2);

    if (x < Chip8::numRegisters && y < Chip8::numRegisters) {
        // Fetch the x and y coordinates from Vx and Vy
        uint8_t xCoord = registers[x];
        uint8_t yCoord = registers[y];

        /** @todo TODO: Implement opcode behavior */
    }
}

/**
 * @fn void Chip8::opEx9E_SKP_Vx(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opEx9E_SKP_Vx(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opExA1_SKNP_Vx(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opExA1_SKNP_Vx(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opFx07_LD_Vx_DT(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFx07_LD_Vx_DT(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opFx0A_LD_Vx_K(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFx0A_LD_Vx_K(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opFx15_LD_DT_Vx(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFx15_LD_DT_Vx(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opFx18_LD_ST_Vx(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFx18_LD_ST_Vx(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opFx1E_ADD_I_Vx(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFx1E_ADD_I_Vx(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opFx29_LD_F_Vx(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFx29_LD_F_Vx(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opFx33_LD_B_Vx(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFx33_LD_B_Vx(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opFx55_LD_I_Vx(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFx55_LD_I_Vx(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opFx65_LD_Vx_I(const AssemblyInstr &instr)
 * @brief 
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFx65_LD_Vx_I(const AssemblyInstr &instr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void Chip8::opNULL(const AssemblyInstr &instr)
 * @brief Opcode function that performs no action in case of opcode lookup table error.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opNULL(const AssemblyInstr &instr) {

}

/**
 * @fn void Chip8::op00En(const AssemblyInstr &instr)
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of 0.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op00En(const AssemblyInstr &instr) {
    uint8_t opParam = instr.getParam(0);
    if (opParam < sizeof(opcodeTable_00En)/sizeof(opcodeTable_00En[0])) {
        (this->*opcodeTable_00En[opParam])(instr);
    }
}

/**
 * @fn void Chip8::op8xyn(const AssemblyInstr &instr)
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of 8.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::op8xyn(const AssemblyInstr &instr) {
    uint8_t opParam = instr.getParam(2);
    if (opParam < sizeof(opcodeTable_8xyn)/sizeof(opcodeTable_8xyn[0])) {
       (this->*opcodeTable_8xyn[opParam])(instr);
    }
}

/**
 * @fn void Chip8::opExKn(const AssemblyInstr &instr)
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of E.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opExKn(const AssemblyInstr &instr) {
    uint8_t opParam = instr.getParam(1);
    if (opParam < sizeof(opcodeTable_ExKn)/sizeof(opcodeTable_ExKn[0])) {
        (this->*opcodeTable_ExKn[opParam])(instr);
    }
}

/**
 * @fn void Chip8::opFxnn(const AssemblyInstr &instr)
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of F.
 * @param[in] instr Decoded AssemblyInstr object
 * @return none
 */
void Chip8::opFxnn(const AssemblyInstr &instr) {
    uint8_t opParam = instr.getParam(1);
    if (opParam < sizeof(opcodeTable_Fxnn)/sizeof(opcodeTable_Fxnn[0])) {
        (this->*opcodeTable_Fxnn[opParam])(instr);
    }
}

}