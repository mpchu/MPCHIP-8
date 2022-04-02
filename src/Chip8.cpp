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

const uint8_t mpc::Chip8::fontset[mpc::Chip8::numBuiltInChars][mpc::Chip8::charSizeBytes] = {
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
 * @fn mpc::Chip8::Chip8()
 * @brief Chip8 Class Constructor
 * @return none
 */
mpc::Chip8::Chip8() {
    indexRegister = 0;

    // 0x000 - 0x1FF are reserved memory addresses so instructions must start at 0x200
    programCounter = mpc::Chip8::instrStartAddr;
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
    mpc::Chip8::loadFont();

    // Initialize the opcode tables
    mpc::Chip8::initOpcodeTables();
}

/**
 * @fn mpc::Chip8::~Chip8()
 * @brief Chip8 Class Destructor
 * @return none
 */
mpc::Chip8::~Chip8() {

}

/**
 * @fn int mpc::Chip8::loadRom(std::string fileName)
 * @brief Loads a binary ROM file into the CHIP-8 memory.
 * @param[in] fileName Path of file to load
 * @return 0 on success, negative otherwise 
 */
int mpc::Chip8::loadRom(std::string fileName) {
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
        else if (size > mpc::Chip8::memorySize - mpc::Chip8::instrStartAddr) {
            status = -2;
        } 
        else {
            // Go back to the beginning of the file and fill the buffer
            file.seekg(0, std::ios::beg);
            file.read(reinterpret_cast<char*>(buffer), size);
            file.close();

            // Load the ROM contents into the CHIP-8's memory, starting at 0x200
            if (size <= mpc::Chip8::memorySize - mpc::Chip8::instrStartAddr) {
                std::memcpy(&memory[mpc::Chip8::instrStartAddr], buffer, size);
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
 * @fn int mpc::Chip8::DecodeOpcode(uint16_t opcode,
 *                                  uint8_t &instructionType,
 *                                  uint8_t &x,
 *                                  uint8_t &y,
 *                                  uint8_t &lsn,
 *                                  uint8_t &lsb,
 *                                  uint16_t &addr)
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
int mpc::Chip8::DecodeOpcode(uint16_t opcode,
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
 * @fn int mpc::Chip8::loadFont()
 * @brief Loads CHIP-8 character font data into reserved memory.
 * @return 0 on success, negative otherwise 
 */
int mpc::Chip8::loadFont() {
    int status = 0;

    // Load character fonts into reserved memory space
    uint16_t fontMemAddr = mpc::Chip8::fontsetStartAddr;
    for (uint8_t iFont = 0; iFont < mpc::Chip8::numBuiltInChars; iFont++) {
        for (uint8_t fontByte = 0; fontByte < mpc::Chip8::charSizeBytes; fontByte++) {
            memory[fontMemAddr++] = fontset[iFont][fontByte];
        }
    }

    return status;
}

/**
 * @fn int mpc::Chip8::initOpcodeTables()
 * @brief 
 * @return 0 on success, negative otherwise 
 */
int mpc::Chip8::initOpcodeTables() {
    int status = 0;

    // Initialize the opcode tables with a function that does nothing for safety
    for (auto &opcodeFn : mainOpcodeTable) {
        opcodeFn = &mpc::Chip8::opNULL;
    }
    for (auto &opcodeFn : opcode0Table) {
        opcodeFn = &mpc::Chip8::opNULL;
    }
    for (auto &opcodeFn : opcode8Table) {
        opcodeFn = &mpc::Chip8::opNULL;
    }
    for (auto &opcodeFn : opcodeETable) {
        opcodeFn = &mpc::Chip8::opNULL;
    }
    for (auto &opcodeFn : opcodeFTable) {
        opcodeFn = &mpc::Chip8::opNULL;
    }

    // Index each opcode function by their appropriate opcodes
    mainOpcodeTable[0x0] = &mpc::Chip8::execOpcode0;
    mainOpcodeTable[0x1] = &mpc::Chip8::op1nnn_JP_addr;
    mainOpcodeTable[0x2] = &mpc::Chip8::op2nnn_CALL_addr;
    mainOpcodeTable[0x3] = &mpc::Chip8::op3xnn_SE_Vx_byte;
    mainOpcodeTable[0x4] = &mpc::Chip8::op4xnn_SNE_Vx_byte;
    mainOpcodeTable[0x5] = &mpc::Chip8::op5xy0_SE_Vx_Vy;
    mainOpcodeTable[0x6] = &mpc::Chip8::op6xnn_LD_Vx_byte;
    mainOpcodeTable[0x7] = &mpc::Chip8::op7xnn_ADD_Vx_byte;
    mainOpcodeTable[0x8] = &mpc::Chip8::execOpcode8;
    mainOpcodeTable[0x9] = &mpc::Chip8::op9xy0_SNE_Vx_Vy;
    mainOpcodeTable[0xA] = &mpc::Chip8::opAnnn_LD_I_addr;
    mainOpcodeTable[0xB] = &mpc::Chip8::opBnnn_JP_V0;
    mainOpcodeTable[0xC] = &mpc::Chip8::opCxnn_RND_Vx_byte;
    mainOpcodeTable[0xD] = &mpc::Chip8::opDxyn_DRW_Vx_Vy_nibble;
    mainOpcodeTable[0xE] = &mpc::Chip8::execOpcodeE;
    mainOpcodeTable[0xF] = &mpc::Chip8::execOpcodeF;

    opcode0Table[0x0] = &mpc::Chip8::op00E0_CLS;
    opcode0Table[0xE] = &mpc::Chip8::op00EE_RET;

    opcode8Table[0x0] = &mpc::Chip8::op8xy0_LD_Vx_Vy;
    opcode8Table[0x1] = &mpc::Chip8::op8xy1_OR_Vx_Vy;
    opcode8Table[0x2] = &mpc::Chip8::op8xy2_AND_Vx_Vy;
    opcode8Table[0x3] = &mpc::Chip8::op8xy3_XOR_Vx_Vy;
    opcode8Table[0x4] = &mpc::Chip8::op8xy4_ADD_Vx_Vy;
    opcode8Table[0x5] = &mpc::Chip8::op8xy5_SUB_Vx_Vy;
    opcode8Table[0x6] = &mpc::Chip8::op8xy6_SHR_Vx;
    opcode8Table[0x7] = &mpc::Chip8::op8xy7_SUBN_Vx_Vy;
    opcode8Table[0xE] = &mpc::Chip8::op8xyE_SHL_Vx_Vy;

    opcodeETable[0x1] = &mpc::Chip8::opExA1_SKNP_Vx;
    opcodeETable[0xE] = &mpc::Chip8::opEx9E_SKP_Vx;

    opcodeFTable[0x07] = &mpc::Chip8::opFx07_LD_Vx_DT;
    opcodeFTable[0x0A] = &mpc::Chip8::opFx0A_LD_Vx_K;
    opcodeFTable[0x15] = &mpc::Chip8::opFx15_LD_DT_Vx;
    opcodeFTable[0x18] = &mpc::Chip8::opFx18_LD_ST_Vx;
    opcodeFTable[0x1E] = &mpc::Chip8::opFx1E_ADD_I_Vx;
    opcodeFTable[0x29] = &mpc::Chip8::opFx29_LD_F_Vx;
    opcodeFTable[0x33] = &mpc::Chip8::opFx33_LD_B_Vx;
    opcodeFTable[0x55] = &mpc::Chip8::opFx55_LD_I_Vx;
    opcodeFTable[0x65] = &mpc::Chip8::opFx65_LD_Vx_I;

    return status;
}

// Opcode emulation functions
/**
 * @fn void mpc::Chip8::op00E0_CLS(uint8_t x,
 *                                 uint8_t y,
 *                                 uint8_t lsn,
 *                                 uint8_t lsb,
 *                                 uint16_t addr)
 * @brief Opcode 00E0: Clears the screen.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op00E0_CLS(uint8_t x,
                            uint8_t y,
                            uint8_t lsn,
                            uint8_t lsb,
                            uint16_t addr) {
    memset(videoMem, 0, sizeof(videoMem));
}

/**
 * @fn void mpc::Chip8::op00EE_RET(uint8_t x,
 *                                 uint8_t y,
 *                                 uint8_t lsn,
 *                                 uint8_t lsb,
 *                                 uint16_t addr)
 * @brief Opcode 00EE: Returns from a subroutine.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op00EE_RET(uint8_t x,
                            uint8_t y,
                            uint8_t lsn,
                            uint8_t lsb,
                            uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::op1nnn_JP_addr(uint8_t x,
 *                                     uint8_t y,
 *                                     uint8_t lsn,
 *                                     uint8_t lsb,
 *                                     uint16_t addr)
 * @brief Opcode 1nnn: Jumps to a memory address.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op1nnn_JP_addr(uint8_t x,
                                uint8_t y,
                                uint8_t lsn,
                                uint8_t lsb,
                                uint16_t addr) {
    programCounter = addr;
}

/**
 * @fn void mpc::Chip8::op2nnn_CALL_addr(uint8_t x,
 *                                       uint8_t y,
 *                                       uint8_t lsn,
 *                                       uint8_t lsb,
 *                                       uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op2nnn_CALL_addr(uint8_t x,
                                  uint8_t y,
                                  uint8_t lsn,
                                  uint8_t lsb,
                                  uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::op3xnn_SE_Vx_byte(uint8_t x,
 *                                        uint8_t y,
 *                                        uint8_t lsn,
 *                                        uint8_t lsb,
 *                                        uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op3xnn_SE_Vx_byte(uint8_t x,
                                   uint8_t y,
                                   uint8_t lsn,
                                   uint8_t lsb,
                                   uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::op4xnn_SNE_Vx_byte(uint8_t x,
 *                                         uint8_t y,
 *                                         uint8_t lsn,
 *                                         uint8_t lsb,
 *                                         uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op4xnn_SNE_Vx_byte(uint8_t x,
                                    uint8_t y,
                                    uint8_t lsn,
                                    uint8_t lsb,
                                    uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::op5xy0_SE_Vx_Vy(uint8_t x,
 *                                      uint8_t y,
 *                                      uint8_t lsn,
 *                                      uint8_t lsb,
 *                                      uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op5xy0_SE_Vx_Vy(uint8_t x,
                                 uint8_t y,
                                 uint8_t lsn,
                                 uint8_t lsb,
                                 uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::op6xnn_LD_Vx_byte(uint8_t x,
 *                                        uint8_t y,
 *                                        uint8_t lsn,
 *                                        uint8_t lsb,
 *                                        uint16_t addr)
 * @brief Opcode 6xnn: Sets register Vx with value nn.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op6xnn_LD_Vx_byte(uint8_t x,
                                   uint8_t y,
                                   uint8_t lsn,
                                   uint8_t lsb,
                                   uint16_t addr) {
    registers[x] = lsb;
}

/**
 * @fn void mpc::Chip8::op7xnn_ADD_Vx_byte(uint8_t x,
 *                                         uint8_t y,
 *                                         uint8_t lsn,
 *                                         uint8_t lsb,
 *                                         uint16_t addr)
 * @brief Opcode 7xnn: Adds value nn to register Vx.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op7xnn_ADD_Vx_byte(uint8_t x,
                                    uint8_t y,
                                    uint8_t lsn,
                                    uint8_t lsb,
                                    uint16_t addr) {
    registers[x] += lsb;
}

/**
 * @fn void mpc::Chip8::op8xy0_LD_Vx_Vy(uint8_t x,
 *                                      uint8_t y,
 *                                      uint8_t lsn,
 *                                      uint8_t lsb,
 *                                      uint16_t addr)
 * @brief Opcode 8xy0: Sets Vx to the value of Vy.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op8xy0_LD_Vx_Vy(uint8_t x,
                                 uint8_t y,
                                 uint8_t lsn,
                                 uint8_t lsb,
                                 uint16_t addr) {
    registers[x] = registers[y];
}

/**
 * @fn void mpc::Chip8::op8xy1_OR_Vx_Vy(uint8_t x,
 *                                      uint8_t y,
 *                                      uint8_t lsn,
 *                                      uint8_t lsb,
 *                                      uint16_t addr)
 * @brief Opcode 8xy1: Sets Vx to the bitwise OR value of Vx and Vy.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op8xy1_OR_Vx_Vy(uint8_t x,
                                 uint8_t y,
                                 uint8_t lsn,
                                 uint8_t lsb,
                                 uint16_t addr) {
    registers[x] |= registers[y];
}

/**
 * @fn void mpc::Chip8::op8xy2_AND_Vx_Vy(uint8_t x,
 *                                       uint8_t y,
 *                                       uint8_t lsn,
 *                                       uint8_t lsb,
 *                                       uint16_t addr)
 * @brief Opcode 8xy2: Sets Vx to the bitwise AND value of Vx and Vy.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op8xy2_AND_Vx_Vy(uint8_t x,
                                  uint8_t y,
                                  uint8_t lsn,
                                  uint8_t lsb,
                                  uint16_t addr) {
    registers[x] &= registers[y];
}

/**
 * @fn void mpc::Chip8::op8xy3_XOR_Vx_Vy(uint8_t x,
 *                                       uint8_t y,
 *                                       uint8_t lsn,
 *                                       uint8_t lsb,
 *                                       uint16_t addr)
 * @brief Opcode 8xy2: Sets Vx to the bitwise XOR value of Vx and Vy.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op8xy3_XOR_Vx_Vy(uint8_t x,
                                  uint8_t y,
                                  uint8_t lsn,
                                  uint8_t lsb,
                                  uint16_t addr) {
    registers[x] ^= registers[y];
}

/**
 * @fn void mpc::Chip8::op8xy4_ADD_Vx_Vy(uint8_t x,
 *                                       uint8_t y,
 *                                       uint8_t lsn,
 *                                       uint8_t lsb,
 *                                       uint16_t addr)
 * @brief Opcode 8xy4: Sets Vx to the sum of Vx and Vy. 
 *        This addition will affect the carry flag (VF) in the case of bit overflow.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op8xy4_ADD_Vx_Vy(uint8_t x,
                                  uint8_t y,
                                  uint8_t lsn,
                                  uint8_t lsb,
                                  uint16_t addr) {
    uint16_t sum = registers[x] + registers[y];
    registers[x] = sum;
    registers[mpc::Chip8::opResultReg] = (sum > 0xFF);
}

/**
 * @fn void mpc::Chip8::op8xy5_SUB_Vx_Vy(uint8_t x,
 *                                       uint8_t y,
 *                                       uint8_t lsn,
 *                                       uint8_t lsb,
 *                                       uint16_t addr)
 * @brief Opcode 8xy5: Sets Vx to the value of Vx minus Vy. 
 *        If the first operand is larger than the second operand, the carry flag (VF) will be set to 1.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op8xy5_SUB_Vx_Vy(uint8_t x,
                                  uint8_t y,
                                  uint8_t lsn,
                                  uint8_t lsb,
                                  uint16_t addr) {
    registers[mpc::Chip8::opResultReg] = (registers[x] >= registers[y]);
    registers[x] -= registers[y];
}

/**
 * @fn void mpc::Chip8::op8xy6_SHR_Vx(uint8_t x,
 *                                    uint8_t y,
 *                                    uint8_t lsn,
 *                                    uint8_t lsb,
 *                                    uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op8xy6_SHR_Vx(uint8_t x,
                               uint8_t y,
                               uint8_t lsn,
                               uint8_t lsb,
                               uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::op8xy7_SUBN_Vx_Vy(uint8_t x,
 *                                        uint8_t y,
 *                                        uint8_t lsn,
 *                                        uint8_t lsb,
 *                                        uint16_t addr)
 * @brief Opcode 8xy7: Sets Vx to the value of Vx minus Vy. 
 *        If the first operand is larger than the second operand, the carry flag (VF) will be set to 1.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op8xy7_SUBN_Vx_Vy(uint8_t x,
                                   uint8_t y,
                                   uint8_t lsn,
                                   uint8_t lsb,
                                   uint16_t addr) {
    registers[mpc::Chip8::opResultReg] = (registers[y] >= registers[x]);
    registers[x] = (registers[y] - registers[x]);
}

/**
 * @fn void mpc::Chip8::op8xyE_SHL_Vx_Vy(uint8_t x,
 *                                       uint8_t y,
 *                                       uint8_t lsn,
 *                                       uint8_t lsb,
 *                                       uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op8xyE_SHL_Vx_Vy(uint8_t x,
                                  uint8_t y,
                                  uint8_t lsn,
                                  uint8_t lsb,
                                  uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::op9xy0_SNE_Vx_Vy(uint8_t x,
 *                                       uint8_t y,
 *                                       uint8_t lsn,
 *                                       uint8_t lsb,
 *                                       uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::op9xy0_SNE_Vx_Vy(uint8_t x,
                                  uint8_t y,
                                  uint8_t lsn,
                                  uint8_t lsb,
                                  uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opAnnn_LD_I_addr(uint8_t x,
 *                                       uint8_t y,
 *                                       uint8_t lsn,
 *                                       uint8_t lsb,
 *                                       uint16_t addr) 
 * @brief Opcode Annn: Sets index register I with address nnn.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opAnnn_LD_I_addr(uint8_t x,
                                  uint8_t y,
                                  uint8_t lsn,
                                  uint8_t lsb,
                                  uint16_t addr) {
    indexRegister = addr;
}

/**
 * @fn void mpc::Chip8::opBnnn_JP_V0(uint8_t x,
 *                                   uint8_t y,
 *                                   uint8_t lsn,
 *                                   uint8_t lsb,
 *                                   uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opBnnn_JP_V0(uint8_t x,
                              uint8_t y,
                              uint8_t lsn,
                              uint8_t lsb,
                              uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opCxnn_RND_Vx_byte(uint8_t x,
 *                                         uint8_t y,
 *                                         uint8_t lsn,
 *                                         uint8_t lsb,
 *                                         uint16_t addr)
 * @brief Opcode Cxnn: Generates a random number, binary ANDs it with value nn, and stores the result in Vx.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opCxnn_RND_Vx_byte(uint8_t x,
                                    uint8_t y,
                                    uint8_t lsn,
                                    uint8_t lsb,
                                    uint16_t addr) {
    registers[x] = (mpc::Chip8::getRandByte() & lsb);
}

/**
 * @fn void mpc::Chip8::opDxyn_DRW_Vx_Vy_nibble(uint8_t x,
 *                                              uint8_t y,
 *                                              uint8_t lsn,
 *                                              uint8_t lsb,
 *                                              uint16_t addr)
 * @brief Opcode Dxyn: Display instruction
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opDxyn_DRW_Vx_Vy_nibble(uint8_t x,
                                         uint8_t y,
                                         uint8_t lsn,
                                         uint8_t lsb,
                                         uint16_t addr) {
    // Fetch the x and y coordinates from Vx and Vy
    uint8_t xCoord = registers[x];
    uint8_t yCoord = registers[y];

    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opEx9E_SKP_Vx(uint8_t x,
 *                                    uint8_t y,
 *                                    uint8_t lsn,
 *                                    uint8_t lsb,
 *                                    uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opEx9E_SKP_Vx(uint8_t x,
                               uint8_t y,
                               uint8_t lsn,
                               uint8_t lsb,
                               uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opExA1_SKNP_Vx(uint8_t x,
 *                                     uint8_t y,
 *                                     uint8_t lsn,
 *                                     uint8_t lsb,
 *                                     uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opExA1_SKNP_Vx(uint8_t x,
                                uint8_t y,
                                uint8_t lsn,
                                uint8_t lsb,
                                uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opFx07_LD_Vx_DT(uint8_t x,
 *                                      uint8_t y,
 *                                      uint8_t lsn,
 *                                      uint8_t lsb,
 *                                      uint16_t addr) 
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opFx07_LD_Vx_DT(uint8_t x,
                                 uint8_t y,
                                 uint8_t lsn,
                                 uint8_t lsb,
                                 uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opFx0A_LD_Vx_K(uint8_t x,
 *                                     uint8_t y,
 *                                     uint8_t lsn,
 *                                     uint8_t lsb,
 *                                     uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opFx0A_LD_Vx_K(uint8_t x,
                                uint8_t y,
                                uint8_t lsn,
                                uint8_t lsb,
                                uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opFx15_LD_DT_Vx(uint8_t x,
 *                                      uint8_t y,
 *                                      uint8_t lsn,
 *                                      uint8_t lsb,
 *                                      uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opFx15_LD_DT_Vx(uint8_t x,
                                 uint8_t y,
                                 uint8_t lsn,
                                 uint8_t lsb,
                                 uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opFx18_LD_ST_Vx(uint8_t x,
 *                                      uint8_t y,
 *                                      uint8_t lsn,
 *                                      uint8_t lsb,
 *                                      uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opFx18_LD_ST_Vx(uint8_t x,
                                 uint8_t y,
                                 uint8_t lsn,
                                 uint8_t lsb,
                                 uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opFx1E_ADD_I_Vx(uint8_t x,
 *                                      uint8_t y,
 *                                      uint8_t lsn,
 *                                      uint8_t lsb,
 *                                      uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opFx1E_ADD_I_Vx(uint8_t x,
                                 uint8_t y,
                                 uint8_t lsn,
                                 uint8_t lsb,
                                 uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opFx29_LD_F_Vx(uint8_t x,
 *                                     uint8_t y,
 *                                     uint8_t lsn,
 *                                     uint8_t lsb,
 *                                     uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opFx29_LD_F_Vx(uint8_t x,
                                uint8_t y,
                                uint8_t lsn,
                                uint8_t lsb,
                                uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opFx33_LD_B_Vx(uint8_t x,
 *                                     uint8_t y,
 *                                     uint8_t lsn,
 *                                     uint8_t lsb,
 *                                     uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opFx33_LD_B_Vx(uint8_t x,
                                uint8_t y,
                                uint8_t lsn,
                                uint8_t lsb,
                                uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opFx55_LD_I_Vx(uint8_t x,
 *                                     uint8_t y,
 *                                     uint8_t lsn,
 *                                     uint8_t lsb,
 *                                     uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opFx55_LD_I_Vx(uint8_t x,
                                uint8_t y,
                                uint8_t lsn,
                                uint8_t lsb,
                                uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opFx65_LD_Vx_I(uint8_t x,
 *                                     uint8_t y,
 *                                     uint8_t lsn,
 *                                     uint8_t lsb,
 *                                     uint16_t addr)
 * @brief 
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opFx65_LD_Vx_I(uint8_t x,
                                uint8_t y,
                                uint8_t lsn,
                                uint8_t lsb,
                                uint16_t addr) {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opNULL(uint8_t x,
 *                             uint8_t y,
 *                             uint8_t lsn,
 *                             uint8_t lsb,
 *                             uint16_t addr)
 * @brief Opcode function that performs no action in case of opcode lookup table error.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::opNULL(uint8_t x,
                        uint8_t y,
                        uint8_t lsn,
                        uint8_t lsb,
                        uint16_t addr) {

}

// Opcode table functions
/**
 * @fn void mpc::Chip8::execOpcode0(uint8_t x,
 *                                  uint8_t y,
 *                                  uint8_t lsn,
 *                                  uint8_t lsb,
 *                                  uint16_t addr)
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of 0.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode. Used as subtype for opcodes 0x0xxx.
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::execOpcode0(uint8_t x,
                             uint8_t y,
                             uint8_t lsn,
                             uint8_t lsb,
                             uint16_t addr) {
    if (lsn < sizeof(opcode0Table)/sizeof(opcode0Table[0])) {
        (this->*opcode0Table[lsn])(x, y, lsn, lsb, addr);
    }
}

/**
 * @fn void mpc::Chip8::execOpcode8(uint8_t x,
 *                                  uint8_t y,
 *                                  uint8_t lsn,
 *                                  uint8_t lsb,
 *                                  uint16_t addr)
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of 8.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode. Used as subtype for opcodes 0x8xxx.
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::execOpcode8(uint8_t x,
                             uint8_t y,
                             uint8_t lsn,
                             uint8_t lsb,
                             uint16_t addr) {
    if (lsn < sizeof(opcode8Table)/sizeof(opcode8Table[0])) {
       (this->*opcode8Table[lsn])(x, y, lsn, lsb, addr);
    }
}

/**
 * @fn void mpc::Chip8::execOpcodeE(uint8_t x,
 *                                  uint8_t y,
 *                                  uint8_t lsn,
 *                                  uint8_t lsb,
 *                                  uint16_t addr)
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of E.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode. Used as subtype for opcodes 0xExxx.
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::execOpcodeE(uint8_t x,
                             uint8_t y,
                             uint8_t lsn,
                             uint8_t lsb,
                             uint16_t addr) {
    if (lsn < sizeof(opcodeETable)/sizeof(opcodeETable[0])) {
        (this->*opcodeETable[lsn])(x, y, lsn, lsb, addr);
    }
}

/**
 * @fn void mpc::Chip8::execOpcodeF(uint8_t x,
 *                                  uint8_t y,
 *                                  uint8_t lsn,
 *                                  uint8_t lsb,
 *                                  uint16_t addr)
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of F.
 * @param[in] x Second nibble value as decoded from 16-bit opcode
 * @param[in] y First nibble value as decoded from 16-bit opcode
 * @param[in] lsn Least significant nibble as decoded from 16-bit opcode
 * @param[in] lsb Least significant byte as decoded from 16-bit opcode. Used as subtype for opcodes 0xFxxx.
 * @param[in] addr Least significant 12-bits as decoded from 16-bit opcode
 * @return none
 */
void mpc::Chip8::execOpcodeF(uint8_t x,
                             uint8_t y,
                             uint8_t lsn,
                             uint8_t lsb,
                             uint16_t addr) {
    if (lsb < sizeof(opcodeFTable)/sizeof(opcodeFTable[0])) {
        (this->*opcodeFTable[lsb])(x, y, lsn, lsb, addr);
    }
}