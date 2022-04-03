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
#include "AssemblyInstr.h"

namespace mpc::Chip8 {

/** @class Chip8 Chip8.h "Chip8.h"
 *  @brief CHIP-8 Emulator Class
 */
class Chip8 {

/**
 * @brief Opcode emulation function typedef
 */
typedef void (mpc::Chip8::Chip8::*OpCodeFn)(const mpc::Chip8::AssemblyInstr &);

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
    void op00E0_CLS(const AssemblyInstr &instr);

    void op00EE_RET(const AssemblyInstr &instr);

    void op1nnn_JP_addr(const AssemblyInstr &instr);

    void op2nnn_CALL_addr(const AssemblyInstr &instr);

    void op3xnn_SE_Vx_byte(const AssemblyInstr &instr);

    void op4xnn_SNE_Vx_byte(const AssemblyInstr &instr);

    void op5xy0_SE_Vx_Vy(const AssemblyInstr &instr);

    void op6xnn_LD_Vx_byte(const AssemblyInstr &instr);

    void op7xnn_ADD_Vx_byte(const AssemblyInstr &instr);

    void op8xy0_LD_Vx_Vy(const AssemblyInstr &instr);

    void op8xy1_OR_Vx_Vy(const AssemblyInstr &instr);

    void op8xy2_AND_Vx_Vy(const AssemblyInstr &instr);

    void op8xy3_XOR_Vx_Vy(const AssemblyInstr &instr);

    void op8xy4_ADD_Vx_Vy(const AssemblyInstr &instr);

    void op8xy5_SUB_Vx_Vy(const AssemblyInstr &instr);

    void op8xy6_SHR_Vx(const AssemblyInstr &instr);

    void op8xy7_SUBN_Vx_Vy(const AssemblyInstr &instr);

    void op8xyE_SHL_Vx_Vy(const AssemblyInstr &instr);

    void op9xy0_SNE_Vx_Vy(const AssemblyInstr &instr);

    void opAnnn_LD_I_addr(const AssemblyInstr &instr);

    void opBnnn_JP_V0(const AssemblyInstr &instr);

    void opCxnn_RND_Vx_byte(const AssemblyInstr &instr);

    void opDxyn_DRW_Vx_Vy_nibble(const AssemblyInstr &instr);

    void opEx9E_SKP_Vx(const AssemblyInstr &instr);

    void opExA1_SKNP_Vx(const AssemblyInstr &instr);

    void opFx07_LD_Vx_DT(const AssemblyInstr &instr);

    void opFx0A_LD_Vx_K(const AssemblyInstr &instr);

    void opFx15_LD_DT_Vx(const AssemblyInstr &instr);

    void opFx18_LD_ST_Vx(const AssemblyInstr &instr);

    void opFx1E_ADD_I_Vx(const AssemblyInstr &instr);

    void opFx29_LD_F_Vx(const AssemblyInstr &instr);

    void opFx33_LD_B_Vx(const AssemblyInstr &instr);

    void opFx55_LD_I_Vx(const AssemblyInstr &instr);

    void opFx65_LD_Vx_I(const AssemblyInstr &instr);

    void opNULL(const AssemblyInstr &instr);

    void op00En(const AssemblyInstr &instr);

    void op8xyn(const AssemblyInstr &instr);

    void opExKn(const AssemblyInstr &instr);

    void opFxnn(const AssemblyInstr &instr);

    static const uint8_t fontset[Chip8::numBuiltInChars][Chip8::charSizeBytes]; /**< Fontset built into each CHIP-8 */

    uint8_t registers[Chip8::numRegisters]; /**< Emulated 16-bit CPU registers */

    uint8_t memory[Chip8::memorySize]; /**< Emulated storage memory for a CHIP-8 */

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

    OpCodeFn opcodeTable_00En[0xE + 1]; /**< Function pointer lookup table for opcodes with most significant nibble of 0x0 */

    OpCodeFn opcodeTable_8xyn[0xE + 1]; /**< Function pointer lookup table for opcodes with most significant nibble of 0x8 */

    OpCodeFn opcodeTable_ExKn[0xE + 1]; /**< Function pointer lookup table for opcodes with most significant nibble of 0xE */

    OpCodeFn opcodeTable_Fxnn[0x65 + 1]; /**< Function pointer lookup table for opcodes with most significant nibble of 0xF */
};

}

#endif