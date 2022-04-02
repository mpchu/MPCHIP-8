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
        else if (size <= mpc::Chip8::memorySize - mpc::Chip8::instrStartAddr) {
            status = -2;
        } 
        else {
            // Go back to the beginning of the file and fill the buffer
            file.seekg(0, std::ios::beg);
            file.read(reinterpret_cast<char*>(buffer), size);
            file.close();

            // Load the ROM contents into the CHIP-8's memory, starting at 0x200
            if (size <= mpc::Chip8::memorySize - mpc::Chip8::instrStartAddr) {
                std::memcpy(&memory[mpc::Chip8::instrStartAddr], buffer, sizeof(buffer));
            } 
            else {
                status = -1;
            }

            delete[] buffer;
        }
    }

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
    mainOpcodeTable[0x3] = &mpc::Chip8::op3xkk_SE_Vx_byte;
    mainOpcodeTable[0x4] = &mpc::Chip8::op4xkk_SNE_Vx_byte;
    mainOpcodeTable[0x5] = &mpc::Chip8::op5xy0_SE_Vx_Vy;
    mainOpcodeTable[0x6] = &mpc::Chip8::op6xkk_LD_Vx_byte;
    mainOpcodeTable[0x7] = &mpc::Chip8::op7xkk_ADD_Vx_byte;
    mainOpcodeTable[0x8] = &mpc::Chip8::execOpcode8;
    mainOpcodeTable[0x9] = &mpc::Chip8::op9xy0_SNE_Vx_Vy;
    mainOpcodeTable[0xA] = &mpc::Chip8::opAnnn_LD_I_addr;
    mainOpcodeTable[0xB] = &mpc::Chip8::opBnnn_JP_V0;
    mainOpcodeTable[0xC] = &mpc::Chip8::opCxkk_RND_Vx_byte;
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
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op00E0_CLS() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op00EE_RET() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op1nnn_JP_addr() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op2nnn_CALL_addr() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op3xkk_SE_Vx_byte() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op4xkk_SNE_Vx_byte() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op5xy0_SE_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op6xkk_LD_Vx_byte() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op7xkk_ADD_Vx_byte() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op8xy0_LD_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op8xy1_OR_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op8xy2_AND_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op8xy3_XOR_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op8xy4_ADD_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op8xy5_SUB_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op8xy6_SHR_Vx() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op8xy7_SUBN_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op8xyE_SHL_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::op9xy0_SNE_Vx_Vy() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opAnnn_LD_I_addr() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opBnnn_JP_V0() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opCxkk_RND_Vx_byte() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opDxyn_DRW_Vx_Vy_nibble() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opEx9E_SKP_Vx() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opExA1_SKNP_Vx() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opFx07_LD_Vx_DT() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opFx0A_LD_Vx_K() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opFx15_LD_DT_Vx() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opFx18_LD_ST_Vx() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opFx1E_ADD_I_Vx() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opFx29_LD_F_Vx() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opFx33_LD_B_Vx() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opFx55_LD_I_Vx() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn 
 * @brief 
 * @return none
 */
void mpc::Chip8::opFx65_LD_Vx_I() {
    /** @todo TODO: Implement opcode behavior */
}

/**
 * @fn void mpc::Chip8::opNULL()
 * @brief Opcode function that performs no action in case of opcode lookup table error.
 * @return none
 */
void mpc::Chip8::opNULL() {

}

// Opcode table functions
/**
 * @fn void mpc::Chip8::execOpcode0()
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of 0.
 * @return none
 */
void mpc::Chip8::execOpcode0() {
    uint8_t subcode = opcode & 0x000F;
    if (subcode < sizeof(opcode0Table)/sizeof(opcode0Table[0])) {
        (this->*opcode0Table[subcode])();
    }
}

/**
 * @fn void mpc::Chip8::execOpcode8()
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of 8.
 * @return none
 */
void mpc::Chip8::execOpcode8() {
    uint8_t subcode = opcode & 0x000F;
    if (subcode < sizeof(opcode8Table)/sizeof(opcode8Table[0])) {
       (this->*opcode8Table[subcode])();
    }
}

/**
 * @fn void mpc::Chip8::execOpcodeE()
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of E.
 * @return none
 */
void mpc::Chip8::execOpcodeE() {
    uint8_t subcode = opcode & 0x000F;
    if (subcode < sizeof(opcodeETable)/sizeof(opcodeETable[0])) {
        (this->*opcodeETable[subcode])();
    }
}

/**
 * @fn void mpc::Chip8::execOpcodeF()
 * @brief Performs a lookup and executes the correct instruction for opcodes with a most-significant nibble of F.
 * @return none
 */
void mpc::Chip8::execOpcodeF() {
    uint8_t subcode = opcode & 0x00FF;
    if (subcode < sizeof(opcodeFTable)/sizeof(opcodeFTable[0])) {
        (this->*opcodeFTable[subcode])();
    }
}