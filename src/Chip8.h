/**
 * @file Chip8.h
 * @author Matt Chu
 * @brief CHIP-8 Emulator Class
 * @date 2022-04-01
 * 
 */
#ifndef _MPC_CHIP_8_H_
#define _MPC_CHIP_8_H_

#include <cstdint>
#include <string>
#include <random>

namespace mpc {

/** @class Chip8 Chip8.h "Chip8.h"
 *  @brief CHIP-8 Emulator Class
 */
class Chip8 {

/**
 * @brief Opcode emulation function typedef
 */
typedef void (mpc::Chip8::*OpCodeFn)();

public:
    Chip8();

    ~Chip8();

    int loadRom(std::string fileName);

    static int DecodeOpcode(uint16_t opcode,
                            uint8_t &instructionType,
                            uint8_t &x,
                            uint8_t &y,
                            uint8_t &lsn,
                            uint8_t &lsb,
                            uint16_t &addr);

    static const uint8_t  numBuiltInChars = 16; /**< Number of characters built into CHIP-8 */

    static const uint8_t  charSizeBytes = 5; /**< Size of built-in characters in bytes */

    static const uint8_t  numRegisters = 16; /**< Number of 8-bit registers on a CHIP-8 which are referred to as V0 - VF */

    static const uint8_t  opResultReg = 0xF; /**< Index of the register used to store operation results */

    static const uint16_t memorySize = 4096; /**< Memory available to a CHIP-8 interpreter is 4096 bytes */

    static const uint16_t fontsetStartAddr = 0x050; /**< Start address for the 16 characters (0-F) built into CHIP-8 */

    static const uint16_t fontsetEndAddr = 0x0A0; /**< End address for the 16 characters (0-F) built into CHIP-8 */

    static const uint16_t instrStartAddr = 0x200; /**< Start address for ROM instruction storage */

    static const uint16_t chip8HighAddr = Chip8::memorySize-1; /**< Highest memory address for a CHIP-8 interpreter */

    static const uint8_t  numStacks = 16; /**< Number of stack frames available */

    static const uint8_t  numInputKeys = 16; /**< Number of input keys */

    static const uint8_t  displayPixelWidth = 64; /**< Pixel width of video display */

    static const uint8_t  displayPixelHeight = 32; /**< Pixel height of video display */

private:
    inline uint8_t getRandByte() { return randByte(randGen); }

    int loadFont();

    int initOpcodeTables();

    // Opcode emulation functions
    void op00E0_CLS();

    void op00EE_RET();

    void op1nnn_JP_addr();

    void op2nnn_CALL_addr();

    void op3xkk_SE_Vx_byte();

    void op4xkk_SNE_Vx_byte();

    void op5xy0_SE_Vx_Vy();

    void op6xkk_LD_Vx_byte();

    void op7xkk_ADD_Vx_byte();

    void op8xy0_LD_Vx_Vy();

    void op8xy1_OR_Vx_Vy();

    void op8xy2_AND_Vx_Vy();

    void op8xy3_XOR_Vx_Vy();

    void op8xy4_ADD_Vx_Vy();

    void op8xy5_SUB_Vx_Vy();

    void op8xy6_SHR_Vx();

    void op8xy7_SUBN_Vx_Vy();

    void op8xyE_SHL_Vx_Vy();

    void op9xy0_SNE_Vx_Vy();

    void opAnnn_LD_I_addr();

    void opBnnn_JP_V0();

    void opCxkk_RND_Vx_byte();

    void opDxyn_DRW_Vx_Vy_nibble();

    void opEx9E_SKP_Vx();

    void opExA1_SKNP_Vx();

    void opFx07_LD_Vx_DT();

    void opFx0A_LD_Vx_K();

    void opFx15_LD_DT_Vx();

    void opFx18_LD_ST_Vx();

    void opFx1E_ADD_I_Vx();

    void opFx29_LD_F_Vx();

    void opFx33_LD_B_Vx();

    void opFx55_LD_I_Vx();

    void opFx65_LD_Vx_I();

    void opNULL();

    // Opcode table functions
    void execOpcode0();

    void execOpcode8();

    void execOpcodeE();

    void execOpcodeF();

    static const uint8_t fontset[Chip8::numBuiltInChars][Chip8::charSizeBytes]; /**< Fontset built into each CHIP-8 */

    uint8_t registers[Chip8::numRegisters]; /**< CPU registers */

    uint8_t memory[Chip8::memorySize]; /**< Storage memory for a CHIP-8 */

    uint16_t indexRegister; /**< Register used to store memory addresses for use in operations */

    uint16_t programCounter; /**< Register used to hold the address of the next instruction to execute */

    uint16_t stack[Chip8::numStacks]; /**< CHIP-8 stack space */

    uint8_t stackPtr; /**< Stack pointer */

    uint8_t delayTimer; /**< 8-bit Delay Timer */

    uint8_t soundTimer; /**< 8-bit Sound Timer */

    uint8_t keypad[Chip8::numInputKeys]; /**< CHIP-8 input keys */

    uint64_t videoMem[Chip8::displayPixelHeight]; /**< CHIP-8 display memory */

    uint16_t opcode; /**< Current opcode to execute */

    std::mt19937 randGen; /**< Pseudo-RNG that will be seeded using std::random_device */

    std::uniform_int_distribution<uint8_t> randByte; /**< RNG distribution used to get a random byte value */

    OpCodeFn mainOpcodeTable[0xF + 1]; /**< Function pointer lookup table for opcodes */

    OpCodeFn opcode0Table[0xE + 1]; /**< Function pointer lookup table for opcodes with most significant nibble of 0x0 */

    OpCodeFn opcode8Table[0xE + 1]; /**< Function pointer lookup table for opcodes with most significant nibble of 0x8 */

    OpCodeFn opcodeETable[0xE + 1]; /**< Function pointer lookup table for opcodes with most significant nibble of 0xE */

    OpCodeFn opcodeFTable[0x65 + 1]; /**< Function pointer lookup table for opcodes with most significant nibble of 0xF */
};

}

#endif