#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>   // For fixed-width integers (uint16_t, uint64_t, etc.)
#include <inttypes.h> // <<--- Included for printf format macros like PRIu64
#include <ctype.h>    // For isprint

// Define types using standard integer types
typedef uint16_t Word;    // 16-bit word
typedef uint16_t Address; // 12-bit address (use lower 12 bits)
typedef uint8_t Opcode;   // 3-bit opcode (bits 14-12)
typedef uint8_t Byte;     // 8-bit byte

// Memory size: 4096 words of 16 bits each
#define MEMORY_SIZE 4096
#define ADDRESS_MASK 0x0FFF // Mask for 12-bit addresses
#define WORD_MASK    0xFFFF // Mask for 16-bit words
#define BYTE_MASK    0xFF   // Mask for 8-bit bytes

// --- Hardware Constants ---

// Bus Select Lines
typedef enum {
    BUS_NONE = 0x0, BUS_AR = 0x1, BUS_PC = 0x2, BUS_DR = 0x3,
    BUS_AC = 0x4, BUS_IR = 0x5, BUS_TR = 0x6, BUS_MEM = 0x7
} BusSource;

// Control Signals (Bit Flags)
typedef enum {
    CTRL_LD_AR = 0x0001, CTRL_LD_PC = 0x0002, CTRL_LD_DR = 0x0004,
    CTRL_LD_AC = 0x0008, CTRL_LD_IR = 0x0010, CTRL_LD_TR = 0x0020,
    CTRL_INR_AR = 0x0100, CTRL_INR_PC = 0x0200, CTRL_INR_DR = 0x0400,
    CTRL_INR_AC = 0x0800, CTRL_CLR_AR = 0x1000, CTRL_CLR_PC = 0x2000,
    CTRL_CLR_AC = 0x4000, CTRL_CLR_E = 0x8000, CTRL_MEM_READ = 0x10000,
    CTRL_MEM_WRITE = 0x20000, CTRL_INR_SC = 0x40000, CTRL_CLR_SC = 0x80000,
    // Internal flags for tracking I/O this cycle
    IO_OUT = 0x100000, IO_INP = 0x200000
} ControlSignal;

// Opcodes
#define OP_AND  0x0
#define OP_ADD  0x1
#define OP_LDA  0x2
#define OP_STA  0x3
#define OP_BUN  0x4
#define OP_BSA  0x5
#define OP_ISZ  0x6
#define OP_OTHER 0x7

// Register Reference Micro-operations
#define RREG_CLA 0x800
#define RREG_CLE 0x400
#define RREG_CMA 0x200
#define RREG_CME 0x100
#define RREG_CIR 0x080
#define RREG_CIL 0x040
#define RREG_INC 0x020
#define RREG_SPA 0x010
#define RREG_SNA 0x008
#define RREG_SZA 0x004
#define RREG_SZE 0x002
#define RREG_HLT 0x001

// I/O Micro-operations
#define IO_DEF_INP 0x800
#define IO_DEF_OUT 0x400
#define IO_DEF_SKI 0x200
#define IO_DEF_SKO 0x100
#define IO_DEF_ION 0x080
#define IO_DEF_IOF 0x040

// Indirect Bit
#define I_BIT   0x8000

// --- CPU State ---

typedef struct {
    Address AR; Address PC; Word DR; Word AC; Word IR; Word TR;
    Byte INPR; Byte OUTR; Byte SC; Byte E;
    bool I; Opcode D;
    bool S; bool R; bool IEN; bool FGI; bool FGO;
} Registers;

typedef struct {
    Word memory[MEMORY_SIZE];
    Registers reg;
    bool running; bool debug_mode; uint64_t cycle_count;
    Word data_bus; BusSource bus_select; ControlSignal ctrl;
    bool input_waiting; char input_char; bool fgo_just_cleared;
} BasicComputer;

// --- Function Prototypes ---
void initialize_computer(BasicComputer *bc);
bool execute_clock_cycle(BasicComputer *bc);
void run_computer(BasicComputer *bc);
void run_interactive(BasicComputer *bc);
Word read_memory(BasicComputer *bc, Address addr);
void write_memory(BasicComputer *bc, Address addr, Word data);
void evaluate_bus(BasicComputer *bc);
void execute_control_signals(BasicComputer *bc);
void execute_instruction_fetch_t0(BasicComputer *bc);
void execute_instruction_fetch_t1(BasicComputer *bc);
void execute_decode_t2(BasicComputer *bc);
void execute_indirect_address_fetch_t3(BasicComputer *bc);
void execute_instruction_execute_t3_or_t4(BasicComputer *bc);
void execute_interrupt_t0(BasicComputer *bc);
void execute_interrupt_t1(BasicComputer *bc);
void execute_interrupt_t2(BasicComputer *bc);
void execute_memory_reference(BasicComputer *bc);
void execute_register_reference(BasicComputer *bc);
void execute_io(BasicComputer *bc);
void dump_registers(const BasicComputer *bc);
void dump_memory(const BasicComputer *bc, Address start, Address end);
bool load_program_from_file(BasicComputer *bc, const char *filename);
void print_instruction(Word instruction, Address addr);
const char* bus_source_to_string(BusSource src);
const char* control_signal_to_string(ControlSignal signal_bit);
void print_active_control_signals(ControlSignal active_ctrl);

// --- Function Implementations ---

void initialize_computer(BasicComputer *bc) {
    memset(bc->memory, 0, sizeof(bc->memory));
    memset(&bc->reg, 0, sizeof(bc->reg));
    bc->reg.PC = 0x000; bc->reg.SC = 0; bc->reg.E = 0; bc->reg.S = true;
    bc->reg.R = false; bc->reg.IEN = false; bc->reg.FGI = false; bc->reg.FGO = true;
    bc->running = false; bc->debug_mode = false; bc->cycle_count = 0;
    bc->data_bus = 0; bc->bus_select = BUS_NONE; bc->ctrl = 0;
    bc->input_waiting = false; bc->input_char = 0; bc->fgo_just_cleared = false;
}

Word read_memory(BasicComputer *bc, Address addr) {
    addr &= ADDRESS_MASK;
    if (addr >= MEMORY_SIZE) {
        if (bc->debug_mode) printf("DEBUG: Memory read attempt outside bounds: 0x%03X\n", addr);
        return 0;
    }
    return bc->memory[addr];
}

void write_memory(BasicComputer *bc, Address addr, Word data) {
    addr &= ADDRESS_MASK;
    if (addr >= MEMORY_SIZE) {
        if (bc->debug_mode) printf("DEBUG: Memory write attempt outside bounds: 0x%03X (Data: 0x%04X)\n", addr, data);
        return;
    }
    if (bc->debug_mode) {
        printf("DEBUG: Memory Write: M[0x%03X] <- 0x%04X\n", addr, data);
    }
    bc->memory[addr] = data;
}

void evaluate_bus(BasicComputer *bc) {
    switch (bc->bus_select) {
        case BUS_AR:  bc->data_bus = bc->reg.AR; break;
        case BUS_PC:  bc->data_bus = bc->reg.PC; break;
        case BUS_DR:  bc->data_bus = bc->reg.DR; break;
        case BUS_AC:  bc->data_bus = bc->reg.AC; break;
        case BUS_IR:  bc->data_bus = bc->reg.IR; break;
        case BUS_TR:  bc->data_bus = bc->reg.TR; break;
        case BUS_MEM: bc->data_bus = bc->reg.DR; break; // Assumes DR holds mem data
        case BUS_NONE: default: bc->data_bus = 0; break;
    }
    if (bc->debug_mode && bc->bus_select != BUS_NONE) {
        printf("DEBUG: Bus <- %s (Value=0x%04X)\n", bus_source_to_string(bc->bus_select), bc->data_bus);
    }
}

const char* control_signal_to_string(ControlSignal signal_bit) {
    switch (signal_bit) {
        case CTRL_LD_AR: return "LD_AR"; case CTRL_LD_PC: return "LD_PC";
        case CTRL_LD_DR: return "LD_DR"; case CTRL_LD_AC: return "LD_AC";
        case CTRL_LD_IR: return "LD_IR"; case CTRL_LD_TR: return "LD_TR";
        case CTRL_INR_AR: return "INR_AR"; case CTRL_INR_PC: return "INR_PC";
        case CTRL_INR_DR: return "INR_DR"; case CTRL_INR_AC: return "INR_AC";
        case CTRL_CLR_AR: return "CLR_AR"; case CTRL_CLR_PC: return "CLR_PC";
        case CTRL_CLR_AC: return "CLR_AC"; case CTRL_CLR_E: return "CLR_E";
        case CTRL_MEM_READ: return "MEM_READ"; case CTRL_MEM_WRITE: return "MEM_WRITE";
        case CTRL_INR_SC: return "INR_SC"; case CTRL_CLR_SC: return "CLR_SC";
        default: return "UNKNOWN_CTRL";
    }
}

void print_active_control_signals(ControlSignal active_ctrl) {
    printf("DEBUG: Active Signals: ");
    bool first = true;
    ControlSignal signals_to_check[] = {
        CTRL_LD_AR, CTRL_LD_PC, CTRL_LD_DR, CTRL_LD_AC, CTRL_LD_IR, CTRL_LD_TR,
        CTRL_INR_AR, CTRL_INR_PC, CTRL_INR_DR, CTRL_INR_AC,
        CTRL_CLR_AR, CTRL_CLR_PC, CTRL_CLR_AC, CTRL_CLR_E,
        CTRL_MEM_READ, CTRL_MEM_WRITE,
        CTRL_INR_SC, CTRL_CLR_SC
    };
    int num_signals = sizeof(signals_to_check) / sizeof(signals_to_check[0]);
    for (int i = 0; i < num_signals; ++i) {
        if (active_ctrl & signals_to_check[i]) {
            if (!first) printf(", ");
            printf("%s", control_signal_to_string(signals_to_check[i]));
            first = false;
        }
    }
    if (first) printf("None");
    printf("\n");
}


void execute_control_signals(BasicComputer *bc) {
    ControlSignal active_ctrl = bc->ctrl;
    if (bc->debug_mode) {
        print_active_control_signals(active_ctrl);
    }

    // Memory Operations
    if (active_ctrl & CTRL_MEM_READ) {
        bc->reg.DR = read_memory(bc, bc->reg.AR);
        if (bc->debug_mode) printf("DEBUG: Memory Read: DR <- M[0x%03X] (Read 0x%04X)\n", bc->reg.AR & ADDRESS_MASK, bc->reg.DR);
    }
    if (active_ctrl & CTRL_MEM_WRITE) {
        if (bc->debug_mode) printf("DEBUG: Memory Write Intent: M[0x%03X] will be written\n", bc->reg.AR & ADDRESS_MASK);
    }

    // Bus Evaluation
    evaluate_bus(bc);

    // Register Loads
    if (active_ctrl & CTRL_LD_AR) { bc->reg.AR = bc->data_bus & ADDRESS_MASK; if (bc->debug_mode) printf("DEBUG: Register Load: AR <- Bus (0x%03X)\n", bc->reg.AR); }
    if (active_ctrl & CTRL_LD_PC) { bc->reg.PC = bc->data_bus & ADDRESS_MASK; if (bc->debug_mode) printf("DEBUG: Register Load: PC <- Bus (0x%03X)\n", bc->reg.PC); }
    if (active_ctrl & CTRL_LD_DR) { bc->reg.DR = bc->data_bus; if (bc->debug_mode) printf("DEBUG: Register Load: DR <- Bus (0x%04X)\n", bc->reg.DR); }
    if (active_ctrl & CTRL_LD_AC) { bc->reg.AC = bc->data_bus; if (bc->debug_mode) printf("DEBUG: Register Load: AC <- Bus (0x%04X)\n", bc->reg.AC); }
    if (active_ctrl & CTRL_LD_IR) { bc->reg.IR = bc->data_bus; if (bc->debug_mode) printf("DEBUG: Register Load: IR <- Bus (0x%04X)\n", bc->reg.IR); }
    if (active_ctrl & CTRL_LD_TR) { bc->reg.TR = bc->data_bus; if (bc->debug_mode) printf("DEBUG: Register Load: TR <- Bus (0x%04X)\n", bc->reg.TR); }

    // Register Increments
    if (active_ctrl & CTRL_INR_AR) { bc->reg.AR = (bc->reg.AR + 1) & ADDRESS_MASK; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: AR++ -> 0x%03X\n", bc->reg.AR); }
    if (active_ctrl & CTRL_INR_PC) { bc->reg.PC = (bc->reg.PC + 1) & ADDRESS_MASK; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: PC++ -> 0x%03X\n", bc->reg.PC); }
    if (active_ctrl & CTRL_INR_DR) { bc->reg.DR = (bc->reg.DR + 1) & WORD_MASK; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: DR++ -> 0x%04X\n", bc->reg.DR); }
    if (active_ctrl & CTRL_INR_AC) { // INC Instruction
        bc->reg.AC = (bc->reg.AC + 1) & WORD_MASK;
        if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: AC++ (INC Instruction) -> 0x%04X\n", bc->reg.AC);
    }

    // Register Clears
    if (active_ctrl & CTRL_CLR_AR) { bc->reg.AR = 0; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: AR CLR\n"); }
    if (active_ctrl & CTRL_CLR_PC) { bc->reg.PC = 0; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: PC CLR\n"); }
    if (active_ctrl & CTRL_CLR_AC) { // CLA Instruction or other clears
         if (bc->debug_mode && bc->reg.AC != 0) printf("DEBUG: Register Inc/Clr: AC CLR\n");
         bc->reg.AC = 0;
    }
    if (active_ctrl & CTRL_CLR_E)  { // CLE Instruction or other clears
         if (bc->debug_mode && bc->reg.E != 0) printf("DEBUG: Flag Change: E CLR\n");
         bc->reg.E = 0;
    }

    // Update I/O Flags (Refined FGO logic)
    if (bc->fgo_just_cleared) {
        bc->reg.FGO = true;
        bc->fgo_just_cleared = false;
        if (bc->debug_mode) printf("DEBUG: Flag Change: FGO -> 1 (Output Ready)\n");
    }
    // Check if FGO was cleared this cycle (needs IO_OUT flag set temporarily by execute_io)
    if ((active_ctrl & IO_OUT) && bc->reg.FGO == false) { // If OUT happened and FGO is now false
        bc->fgo_just_cleared = true; // Mark it to be set back to true *next* cycle
    }

    // FGI handling
     if(bc->reg.FGI && (active_ctrl & IO_INP)) {
         // FGI is cleared in execute_io if input is consumed
         if (bc->debug_mode) printf("DEBUG: Flag Change: FGI -> 0 (Input Consumed by INP)\n");
    } else if (bc->input_waiting && !bc->reg.FGI) {
         bc->reg.FGI = true; // Input arrived via 'input' command
         if (bc->debug_mode) printf("DEBUG: Flag Change: FGI -> 1 (Input Ready)\n");
    }

    // Sequence Counter Update
    if (active_ctrl & CTRL_CLR_SC) {
        bc->reg.SC = 0;
        if (bc->debug_mode) printf("DEBUG: SC CLR -> T0\n");
    } else if (active_ctrl & CTRL_INR_SC) {
        bc->reg.SC = (bc->reg.SC + 1);
        if (bc->debug_mode) printf("DEBUG: SC++ -> T%d\n", bc->reg.SC);
    }

    // Reset controls for next cycle
    bc->ctrl = 0;
    bc->bus_select = BUS_NONE;
}


bool execute_clock_cycle(BasicComputer *bc) {
    if (!bc->reg.S) { return false; } // Do nothing if halted

    bc->cycle_count++;
    if (bc->debug_mode) {
        // Use PRIu64 macro for uint64_t portability
        printf("\n--- Cycle %" PRIu64 " --- SC=T%d, PC=0x%03X, R=%d, IEN=%d ---\n",
               bc->cycle_count, bc->reg.SC, bc->reg.PC, bc->reg.R, bc->reg.IEN);
    }

    bool is_interrupt_cycle = (bc->reg.R && bc->reg.IEN && bc->reg.SC == 0);

    if (is_interrupt_cycle) {
        if (bc->debug_mode) printf("DEBUG: Interrupt Cycle, T%d\n", bc->reg.SC);
        switch (bc->reg.SC) {
            case 0: execute_interrupt_t0(bc); break;
            case 1: execute_interrupt_t1(bc); break;
            case 2: execute_interrupt_t2(bc); break;
            default: printf("ERROR: Invalid SC state during interrupt cycle: %d\n", bc->reg.SC); bc->ctrl |= CTRL_CLR_SC; break;
        }
    } else {
        if (bc->debug_mode) printf("DEBUG: Instruction Cycle, T%d\n", bc->reg.SC);
        switch (bc->reg.SC) {
            case 0: execute_instruction_fetch_t0(bc); break;
            case 1: execute_instruction_fetch_t1(bc); break;
            case 2: execute_decode_t2(bc); break;
            case 3:
                if ((bc->reg.D != OP_OTHER) && bc->reg.I) { execute_indirect_address_fetch_t3(bc); }
                else { execute_instruction_execute_t3_or_t4(bc); }
                break;
            case 4: execute_instruction_execute_t3_or_t4(bc); break; // Execution phase
            // Potentially add cases for T5, T6 if needed for very complex instructions
            default:
                if (bc->debug_mode) printf("Warning: Reached unhandled SC state T%d. Resetting SC.\n", bc->reg.SC);
                bc->ctrl |= CTRL_CLR_SC;
                break;
        }
    }

    execute_control_signals(bc);
    return bc->reg.S; // Return true if still running
}

// --- T-State Helpers ---
void execute_instruction_fetch_t0(BasicComputer *bc) { // T0: AR <- PC
    bc->bus_select = BUS_PC; bc->ctrl |= CTRL_LD_AR; bc->ctrl |= CTRL_INR_SC;
}
void execute_instruction_fetch_t1(BasicComputer *bc) { // T1: IR <- M[AR], PC <- PC + 1
    bc->ctrl |= CTRL_MEM_READ; bc->bus_select = BUS_MEM; bc->ctrl |= CTRL_LD_IR;
    bc->ctrl |= CTRL_INR_PC; bc->ctrl |= CTRL_INR_SC;
}
void execute_decode_t2(BasicComputer *bc) { // T2: Decode, Set AR if MemRef
    Address addr_part = bc->reg.IR & ADDRESS_MASK;
    bc->reg.D = (bc->reg.IR >> 12) & 0x7;
    bc->reg.I = (bc->reg.IR & I_BIT) ? true : false;
    if (bc->debug_mode) {
        printf("DEBUG: T2 Decode: IR=0x%04X -> Opcode D=%d, I=%d, Addr=0x%03X\n", bc->reg.IR, bc->reg.D, bc->reg.I, addr_part);
    }
    if (bc->reg.D != OP_OTHER) { // Memory Ref
        bc->reg.AR = addr_part; // Directly load AR with address part
        if (bc->debug_mode) printf("DEBUG: T2 Decode: AR <- IR(0-11) = 0x%03X\n", bc->reg.AR);
    }
    bc->ctrl |= CTRL_INR_SC; // Always go to T3 next
}
void execute_indirect_address_fetch_t3(BasicComputer *bc) { // T3 (Indirect): AR <- M[AR]
     if (bc->debug_mode) printf("DEBUG: T3 Indirect Fetch: AR <- M[0x%03X]\n", bc->reg.AR & ADDRESS_MASK);
     bc->ctrl |= CTRL_MEM_READ; bc->bus_select = BUS_MEM; bc->ctrl |= CTRL_LD_AR;
     bc->ctrl |= CTRL_INR_SC; // Go to T4 for execution
}
void execute_instruction_execute_t3_or_t4(BasicComputer *bc) { // T3/T4: Execute
    if (bc->debug_mode) printf("DEBUG: T%d Execute Phase: D=%d, I=%d\n", bc->reg.SC, bc->reg.D, bc->reg.I);
    switch (bc->reg.D) {
        case OP_AND: case OP_ADD: case OP_LDA: case OP_STA:
        case OP_BUN: case OP_BSA: case OP_ISZ:
            execute_memory_reference(bc); break;
        case OP_OTHER:
            if (bc->reg.I == 0) { execute_register_reference(bc); }
            else { execute_io(bc); }
            break;
        default:
            printf("ERROR: Invalid decoded opcode D=%d during execution phase T%d.\n", bc->reg.D, bc->reg.SC);
            // Halt on invalid opcode for safety
            if(bc->debug_mode) printf("DEBUG: Halting due to invalid opcode.\n");
            bc->reg.S = false; bc->running = false;
            break;
    }
    // If execution is complete this cycle (most instructions)
    // This simple check assumes ISZ might take longer, prevents premature SC reset
    // More complex instructions would need more sophisticated SC handling
    if (!(bc->reg.D == OP_ISZ && bc->reg.SC < 6)) {
       bc->ctrl |= CTRL_CLR_SC; // Go back to T0
    } else {
       // For multi-cycle instructions like ISZ, don't clear SC yet if in T4/T5
       if (bc->debug_mode) printf("DEBUG: ISZ potentially needs more cycles, SC not cleared yet.\n");
       bc->ctrl |= CTRL_INR_SC; // Explicitly increment SC if needed (e.g., T5 -> T6 for ISZ)
    }
}

// --- Interrupt Cycle T-State Helpers ---
void execute_interrupt_t0(BasicComputer *bc) { // RT0: AR <- 0, TR <- PC
    bc->ctrl |= CTRL_CLR_AR; bc->bus_select = BUS_PC; bc->ctrl |= CTRL_LD_TR; bc->ctrl |= CTRL_INR_SC;
}
void execute_interrupt_t1(BasicComputer *bc) { // RT1: M[AR] <- TR, PC <- 0
    write_memory(bc, bc->reg.AR, bc->reg.TR); bc->ctrl |= CTRL_MEM_WRITE;
    bc->ctrl |= CTRL_CLR_PC; bc->ctrl |= CTRL_INR_SC;
}
void execute_interrupt_t2(BasicComputer *bc) { // RT2: PC <- PC + 1, IEN <- 0, R <- 0, SC <- 0
    bc->ctrl |= CTRL_INR_PC;
    bool ien_changed = bc->reg.IEN; bool r_changed = bc->reg.R;
    bc->reg.IEN = false; bc->reg.R = false;
    if (bc->debug_mode) {
        if(ien_changed) printf("DEBUG: Flag Change: IEN -> 0\n");
        if(r_changed) printf("DEBUG: Flag Change: R -> 0\n");
        printf("DEBUG: Interrupt Cycle Finished. Next fetch cycle will target PC=0x001\n");
    }
    bc->ctrl |= CTRL_CLR_SC;
}

// --- Instruction Execution Logic ---

void execute_memory_reference(BasicComputer *bc) {
    Address eff_addr = bc->reg.AR;
    Word operand = 0;
    const char* alu_op_str = "None";

    if (bc->debug_mode) printf("DEBUG: Executing Mem Ref D=%d, Eff Addr=0x%03X\n", bc->reg.D, eff_addr);

    // Simulate T4: Fetch operand if needed
    if (bc->reg.D != OP_STA && bc->reg.D != OP_BUN && bc->reg.D != OP_BSA) {
         bc->reg.DR = read_memory(bc, eff_addr);
         if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T4: Fetched Operand M[0x%03X] = 0x%04X into DR\n", eff_addr, bc->reg.DR);
         operand = bc->reg.DR;
    }

    // Simulate T5/T6: Perform operation
    switch (bc->reg.D) {
        case OP_AND:
            alu_op_str = "AND"; bc->reg.AC &= operand;
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: AC <- AC & DR = 0x%04X\n", bc->reg.AC);
            break;
        case OP_ADD:
            alu_op_str = "ADD"; {
                uint32_t result = (uint32_t)bc->reg.AC + (uint32_t)operand;
                Byte prev_E = bc->reg.E; bc->reg.E = (result > 0xFFFF) ? 1 : 0;
                bc->reg.AC = (Word)(result & WORD_MASK);
                if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: AC <- AC + DR = 0x%04X\n", bc->reg.AC);
                if (bc->debug_mode && prev_E != bc->reg.E) printf("DEBUG: Flag Change: E -> %d\n", bc->reg.E);
            } break;
        case OP_LDA:
            alu_op_str = "LOAD"; bc->reg.AC = operand;
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: AC <- DR = 0x%04X\n", bc->reg.AC);
            break;
        case OP_STA:
            alu_op_str = "STORE"; write_memory(bc, eff_addr, bc->reg.AC); bc->ctrl |= CTRL_MEM_WRITE;
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T4: M[0x%03X] <- AC\n", eff_addr);
            break;
        case OP_BUN:
            alu_op_str = "BRANCH"; bc->reg.PC = eff_addr;
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T4: PC <- AR = 0x%03X\n", bc->reg.PC);
            break;
        case OP_BSA:
            alu_op_str = "BRANCH_SAVE";
            write_memory(bc, eff_addr, bc->reg.PC); bc->ctrl |= CTRL_MEM_WRITE; // T4: M[AR]<-PC
            bc->reg.AR = (eff_addr + 1) & ADDRESS_MASK; // T4: AR<-AR+1
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T4: M[0x%03X] <- PC, AR <- AR+1 = 0x%03X\n", eff_addr, bc->reg.AR);
            bc->reg.PC = bc->reg.AR; // T5: PC<-AR
             if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: PC <- AR = 0x%03X\n", bc->reg.PC);
            break;
        case OP_ISZ:
            alu_op_str = "INC_MEM";
            operand = (operand + 1) & WORD_MASK; // T5: DR<-DR+1
             if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: DR <- DR+1 = 0x%04X\n", operand);
            write_memory(bc, eff_addr, operand); bc->ctrl |= CTRL_MEM_WRITE; // T6: M[AR]<-DR
             if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T6: M[0x%03X] <- DR\n", eff_addr);
            if (operand == 0) {
                bc->ctrl |= CTRL_INR_PC; // T6: If (DR=0) PC<-PC+1
                if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T6: Result is Zero. Skipping (PC will be incremented).\n");
            }
            break;
    }
    if (bc->debug_mode && strcmp(alu_op_str, "None") != 0) {
         printf("DEBUG: ALU/Execute Action: %s\n", alu_op_str);
    }
}

void execute_register_reference(BasicComputer *bc) {
    Word micro_op_bits = bc->reg.IR & 0x0FFF;
    Byte prev_E = bc->reg.E;

    if (bc->debug_mode) printf("DEBUG: Executing Register Ref IR=0x%04X\n", bc->reg.IR);

    if (micro_op_bits & RREG_CLA) { if (bc->debug_mode && bc->reg.AC != 0) printf("DEBUG: Register Inc/Clr: AC CLR\n"); bc->reg.AC = 0; }
    if (micro_op_bits & RREG_CLE) { if (bc->debug_mode && bc->reg.E != 0) printf("DEBUG: Flag Change: E CLR\n"); bc->reg.E = 0;}
    if (micro_op_bits & RREG_CMA) { if (bc->debug_mode) printf("DEBUG: ALU Action: CMA\n"); bc->reg.AC = ~bc->reg.AC; }
    if (micro_op_bits & RREG_CME) { if (bc->debug_mode) printf("DEBUG: Flag Change: E = ~E (%d -> %d)\n", prev_E, (~prev_E & 1)); bc->reg.E = ~bc->reg.E & 0x1;}
    if (micro_op_bits & RREG_CIR) {
        if (bc->debug_mode) printf("DEBUG: ALU Action: CIR\n");
        prev_E = bc->reg.E; Byte temp_lsb = bc->reg.AC & 0x0001;
        bc->reg.AC = (bc->reg.AC >> 1) | (prev_E << 15); bc->reg.E = temp_lsb;
        if (bc->debug_mode && prev_E != bc->reg.E) printf("DEBUG: Flag Change: E -> %d\n", bc->reg.E);
    }
    if (micro_op_bits & RREG_CIL) {
        if (bc->debug_mode) printf("DEBUG: ALU Action: CIL\n");
        prev_E = bc->reg.E; Byte temp_msb = (bc->reg.AC & 0x8000) ? 1 : 0;
        bc->reg.AC = (bc->reg.AC << 1) | prev_E; bc->reg.E = temp_msb;
        if (bc->debug_mode && prev_E != bc->reg.E) printf("DEBUG: Flag Change: E -> %d\n", bc->reg.E);
    }
    if (micro_op_bits & RREG_INC) {
        if (bc->debug_mode) printf("DEBUG: ALU Action: INC AC\n");
        bc->reg.AC = (bc->reg.AC + 1) & WORD_MASK;
    }
    bool skip = false;
    if ((micro_op_bits & RREG_SPA) && ( (bc->reg.AC & 0x8000) == 0) ) skip = true;
    if ((micro_op_bits & RREG_SNA) && ( (bc->reg.AC & 0x8000) != 0) ) skip = true;
    if ((micro_op_bits & RREG_SZA) && (bc->reg.AC == 0) ) skip = true;
    if ((micro_op_bits & RREG_SZE) && (bc->reg.E == 0) ) skip = true; // Use current E after potential CME/CIR/CIL
    if (skip) {
        bc->ctrl |= CTRL_INR_PC;
        if (bc->debug_mode) printf("DEBUG: Register Ref: Skip condition met. PC will be incremented.\n");
    }
    if (micro_op_bits & RREG_HLT) {
        bool prev_S = bc->reg.S;
        bc->reg.S = false; bc->running = false;
        if (bc->debug_mode) printf("DEBUG: HLT instruction encountered.\n");
        if (bc->debug_mode && prev_S) printf("DEBUG: Flag Change: S -> 0 (Halt)\n");
        printf("INFO: Program Halted by HLT instruction.\n");
    }
}

void execute_io(BasicComputer *bc) {
    Word micro_op_bits = bc->reg.IR & 0x0FFF;
    if (bc->debug_mode) printf("DEBUG: Executing I/O IR=0x%04X\n", bc->reg.IR);

    if (micro_op_bits & IO_DEF_INP) {
        bc->ctrl |= IO_INP; // Signal intent for flag handling
        if (bc->reg.FGI) {
            bc->reg.INPR = bc->input_char & BYTE_MASK;
            bc->reg.AC = (bc->reg.AC & 0xFF00) | bc->reg.INPR;
            bc->reg.FGI = false; // Clear flag AFTER reading
            bc->input_waiting = false;
            printf("INFO: Input Read: '%c' (0x%02X) loaded into AC[0-7]\n", isprint(bc->reg.INPR) ? bc->reg.INPR : '.', bc->reg.INPR);
        } else { if(bc->debug_mode) printf("DEBUG: INP attempted but FGI=0.\n"); }
    }
    if (micro_op_bits & IO_DEF_OUT) {
        bc->ctrl |= IO_OUT; // Signal intent for flag handling
        if (bc->reg.FGO) {
            bc->reg.OUTR = bc->reg.AC & BYTE_MASK;
            printf("OUTPUT: %c (0x%02X)\n", isprint(bc->reg.OUTR) ? bc->reg.OUTR : '.', bc->reg.OUTR);
            bc->reg.FGO = false; // Clear flag AFTER output
            if(bc->debug_mode) printf("DEBUG: Flag Change: FGO -> 0 (Output Busy)\n");
        } else { if(bc->debug_mode) printf("DEBUG: OUT attempted but FGO=0.\n"); }
    }
    bool skip = false;
    if ((micro_op_bits & IO_DEF_SKI) && bc->reg.FGI) skip = true;
    if ((micro_op_bits & IO_DEF_SKO) && bc->reg.FGO) skip = true;
    if (skip) {
        bc->ctrl |= CTRL_INR_PC;
        if (bc->debug_mode) printf("DEBUG: I/O: Skip condition met. PC will be incremented.\n");
    }
    if (micro_op_bits & IO_DEF_ION) {
        if (bc->debug_mode && !bc->reg.IEN) printf("DEBUG: Flag Change: IEN -> 1 (Interrupts Enabled)\n");
        bc->reg.IEN = true;
    }
    if (micro_op_bits & IO_DEF_IOF) {
         if (bc->debug_mode && bc->reg.IEN) printf("DEBUG: Flag Change: IEN -> 0 (Interrupts Disabled)\n");
        bc->reg.IEN = false;
    }
}

// --- Utility Functions ---

void dump_registers(const BasicComputer *bc) {
    printf("Registers:\n");
    printf("  PC: 0x%03X   AR: 0x%03X   IR: 0x%04X   SC: T%d\n", bc->reg.PC, bc->reg.AR, bc->reg.IR, bc->reg.SC);
    printf("  AC: 0x%04X   DR: 0x%04X   TR: 0x%04X   E: %d\n", bc->reg.AC, bc->reg.DR, bc->reg.TR, bc->reg.E);
    printf("  INPR: 0x%02X OUTR: 0x%02X\n", bc->reg.INPR, bc->reg.OUTR);
    printf("  Status: S=%d R=%d IEN=%d FGI=%d FGO=%d\n", bc->reg.S, bc->reg.R, bc->reg.IEN, bc->reg.FGI, bc->reg.FGO);
    printf("  Internal: I=%d D=%d\n", bc->reg.I, bc->reg.D);
    // Use PRIu64 macro for uint64_t portability <<--- FIX APPLIED
    printf("  VM State: running=%d, debug=%d, cycles=%" PRIu64 "\n", bc->running, bc->debug_mode, bc->cycle_count);
}

void dump_memory(const BasicComputer *bc, Address start, Address end) {
     start &= ADDRESS_MASK; end &= ADDRESS_MASK;
    if (end < start) { Address temp = start; start = end; end = temp; }
    if (end >= MEMORY_SIZE) end = MEMORY_SIZE - 1;
    printf("Memory Dump [0x%03X - 0x%03X]:\n", start, end);
    for (Address addr = start; addr <= end; ++addr) {
        if ((addr % 8 == 0) || addr == start) { if (addr != start) printf("\n"); printf("  0x%03X: ", addr); }
        printf("%04X ", bc->memory[addr]);
    }
    printf("\n");
}

bool load_program_from_file(BasicComputer *bc, const char *filename) {
    FILE *file = fopen(filename, "r"); if (!file) { perror("Error opening program file"); return false; }
    char line[100]; Address current_addr = 0; int line_num = 0; bool success = true;
    printf("Loading program from '%s'...\n", filename);
    while (fgets(line, sizeof(line), file)) {
        line_num++; line[strcspn(line, "\r\n")] = 0;
        char *trimmed_line = line; while (isspace((unsigned char)*trimmed_line)) trimmed_line++;
        if (*trimmed_line == '#' || *trimmed_line == '\0') continue;
        Word value;
        if (sscanf(trimmed_line, "%hx", &value) != 1) { // Use %hx
            fprintf(stderr, "Error parsing hex value on line %d: '%s'\n", line_num, trimmed_line);
            success = false; break;
        }
        if (current_addr >= MEMORY_SIZE) { fprintf(stderr, "Error: Program too large (line %d).\n", line_num); success = false; break; }
        write_memory(bc, current_addr, value); current_addr++;
    }
    fclose(file); if (success) printf("Program loaded successfully. %d words written.\n", current_addr);
    return success;
}

void print_instruction(Word instruction, Address addr) {
     Opcode opcode = (instruction >> 12) & 0x7; bool indirect = (instruction & I_BIT) != 0;
    Address operand = instruction & ADDRESS_MASK;
    printf("0x%03X: %04X  ", addr, instruction); printf("[%c] ", indirect ? 'I' : 'D');
    switch (opcode) {
        case OP_AND: printf("AND 0x%03X", operand); break; case OP_ADD: printf("ADD 0x%03X", operand); break;
        case OP_LDA: printf("LDA 0x%03X", operand); break; case OP_STA: printf("STA 0x%03X", operand); break;
        case OP_BUN: printf("BUN 0x%03X", operand); break; case OP_BSA: printf("BSA 0x%03X", operand); break;
        case OP_ISZ: printf("ISZ 0x%03X", operand); break;
        case OP_OTHER:
            if (!indirect) {
                printf("REG: "); Word reg_bits = instruction & 0x0FFF; if (reg_bits == 0) { printf("(NOP)"); break;}
                if (reg_bits & RREG_CLA) printf("CLA "); if (reg_bits & RREG_CLE) printf("CLE ");
                if (reg_bits & RREG_CMA) printf("CMA "); if (reg_bits & RREG_CME) printf("CME ");
                if (reg_bits & RREG_CIR) printf("CIR "); if (reg_bits & RREG_CIL) printf("CIL ");
                if (reg_bits & RREG_INC) printf("INC "); if (reg_bits & RREG_SPA) printf("SPA ");
                if (reg_bits & RREG_SNA) printf("SNA "); if (reg_bits & RREG_SZA) printf("SZA ");
                if (reg_bits & RREG_SZE) printf("SZE "); if (reg_bits & RREG_HLT) printf("HLT ");
            } else {
                printf("IO : "); Word io_bits = instruction & 0x0FFF; if (io_bits == 0) { printf("(NOP)"); break;}
                if (io_bits & IO_DEF_INP) printf("INP "); if (io_bits & IO_DEF_OUT) printf("OUT ");
                if (io_bits & IO_DEF_SKI) printf("SKI "); if (io_bits & IO_DEF_SKO) printf("SKO ");
                if (io_bits & IO_DEF_ION) printf("ION "); if (io_bits & IO_DEF_IOF) printf("IOF ");
            } break;
        default: printf("UNK 0x%03X", operand); break;
    }
    printf("\n");
}

const char* bus_source_to_string(BusSource src) {
     switch (src) {
        case BUS_AR: return "AR"; case BUS_PC: return "PC"; case BUS_DR: return "DR";
        case BUS_AC: return "AC"; case BUS_IR: return "IR"; case BUS_TR: return "TR";
        case BUS_MEM: return "MEM(DR)"; case BUS_NONE: return "NONE"; default: return "??";
    }
}

// --- Main Execution Logic ---

void run_computer(BasicComputer *bc) {
    if (bc->running) { printf("Computer is already running.\n"); return; }
    if (!bc->reg.S) { printf("Computer is Halted (S=0). Use 'reset' to run.\n"); return; }
    printf("Running computer...\n");
    bc->running = true;
    if (bc->reg.SC != 0) { if (bc->debug_mode) printf("DEBUG: Resetting SC to T0 for run command.\n"); bc->reg.SC = 0; }
    while (bc->running) {
        bc->running = execute_clock_cycle(bc);
        if (bc->cycle_count > 2000000) { // Increased safety break
             printf("\nWarning: Exceeded maximum cycle count (2,000,000). Halting simulation.\n");
             bc->running = false; bc->reg.S = false;
        }
    }
    // Use PRIu64 macro for uint64_t portability <<--- FIX APPLIED
    printf("Computer has stopped. Total cycles: %" PRIu64 "\n", bc->cycle_count);
}

void run_interactive(BasicComputer *bc) {
    char command_line[100];
    char command[20];
    Address addr1 = 0, addr2 = 0;
    printf("\nEntering interactive mode. Type 'help' for commands.\n");
    while (true) {
        printf("> ");
        if (fgets(command_line, sizeof(command_line), stdin) == NULL) { printf("\nEOF detected. Exiting.\n"); break; }

        // Use %hx for reading into uint16_t (Address) <<--- FIX APPLIED
        int items = sscanf(command_line, "%19s %hx %hx", command, &addr1, &addr2);
        if (items <= 0) continue;

        // Command Processing
        if (strcmp(command, "help") == 0) {
             printf("Commands:\n");
             printf("  run (r)          : Run until HLT or interrupt/error\n");
             printf("  step (s) [n]   : Execute N clock cycles (default 1)\n");
             printf("  regs (reg)     : Dump registers\n");
             printf("  mem <start> [end]: Dump memory from start address (hex) to end (hex, optional)\n");
             printf("  dis <start> [n]: Disassemble N instructions from start address (hex, default 10)\n");
             printf("  reset          : Reset CPU state (memory preserved), sets S=1\n");
             printf("  load <file>    : Load program from hex file (clears memory, resets CPU)\n");
             printf("  debug          : Toggle debug trace mode\n");
             printf("  input <char>   : Simulate typing a character for INP (sets FGI=1)\n");
             printf("  interrupt      : Manually trigger an interrupt request (R=1)\n");
             printf("  quit (q)       : Exit simulator\n");
        }
        else if (strcmp(command, "run") == 0 || strcmp(command, "r") == 0) { run_computer(bc); }
        else if (strcmp(command, "step") == 0 || strcmp(command, "s") == 0) {
            if (!bc->reg.S) { printf("Computer is Halted (S=0). Cannot step.\n"); continue; }
            int steps = 1;
            // Use addr1 if items > 1, but parse it as decimal int for steps
            if (items > 1 && sscanf(command_line, "%*s %d", &steps) != 1) {
                 printf("Invalid number of steps '%x'. Defaulting to 1.\n", addr1); // Show hex value if parse failed
                 steps = 1;
            }
            if (steps <= 0) steps = 1;

            printf("Executing %d cycle(s)...\n", steps);
            bc->running = true;
            for (int i = 0; i < steps && bc->reg.S; ++i) {
                 if (!execute_clock_cycle(bc)) { printf("Computer halted during step execution.\n"); break; }
            }
             if (bc->reg.S) {
                printf("\nState After Step(s):\n"); dump_registers(bc);
                if (bc->reg.SC == 0) {
                     printf("--- Next cycle is instruction fetch (T0) targeting PC=0x%03X ---\n", bc->reg.PC);
                     print_instruction(read_memory(bc, bc->reg.PC), bc->reg.PC);
                } else { printf("--- Currently in T%d of instruction/interrupt cycle ---\n", bc->reg.SC); }
            }
        }
        else if (strcmp(command, "regs") == 0 || strcmp(command, "reg") == 0) { dump_registers(bc); }
        else if (strcmp(command, "mem") == 0) {
            if (items < 2) { printf("Usage: mem <start_hex> [end_hex]\n"); }
            else {
                Address start_addr = addr1 & ADDRESS_MASK;
                Address end_addr = (items > 2) ? (addr2 & ADDRESS_MASK) : start_addr;
                if (end_addr < start_addr) { Address temp = start_addr; start_addr = end_addr; end_addr = temp; }
                dump_memory(bc, start_addr, end_addr);
            }
        }
         else if (strcmp(command, "dis") == 0) {
            if (items < 2) { printf("Usage: dis <start_hex> [num_instructions]\n"); }
            else {
                Address start_addr = addr1 & ADDRESS_MASK;
                int count = 10; // Default
                // Use addr2 if items > 2, but parse as decimal int
                 if (items > 2 && sscanf(command_line, "%*s %*s %d", &count) != 1) {
                     printf("Invalid instruction count '%x'. Defaulting to 10.\n", addr2); // Show hex if failed
                     count = 10;
                 }
                if (count <= 0) count = 10; if (count > 100) count = 100;
                printf("Disassembly from 0x%03X (%d instructions):\n", start_addr, count);
                for (int i=0; i<count; ++i) {
                    Address current_addr = (start_addr + i) & ADDRESS_MASK;
                    if (current_addr < start_addr && i > 0) { printf("Address wrap-around.\n"); break; }
                    Word instruction = read_memory(bc, current_addr);
                    print_instruction(instruction, current_addr);
                }
            }
        }
        else if (strcmp(command, "reset") == 0) {
            printf("Resetting CPU state and flags (memory preserved)...\n");
            bool old_debug = bc->debug_mode;
            Word temp_memory[MEMORY_SIZE]; memcpy(temp_memory, bc->memory, sizeof(temp_memory));
            initialize_computer(bc); // Resets S=1
            memcpy(bc->memory, temp_memory, sizeof(temp_memory));
            bc->debug_mode = old_debug;
            printf("CPU Reset. PC=0x000. S=1.\n");
        }
        else if (strcmp(command, "load") == 0) {
            char filename[80];
            if (sscanf(command_line, "%*s %79s", filename) == 1) {
                printf("Resetting computer and clearing memory before loading...\n");
                bool old_debug = bc->debug_mode; initialize_computer(bc); bc->debug_mode = old_debug;
                 if (load_program_from_file(bc, filename)) {
                     printf("Load complete. Resetting PC/SC to 0.\n");
                     bc->reg.PC = 0; bc->reg.SC = 0; bc->reg.S = true; dump_registers(bc);
                 } else { printf("Failed to load program.\n"); }
            } else { printf("Usage: load <filename>\n"); }
        }
         else if (strcmp(command, "debug") == 0) {
            bc->debug_mode = !bc->debug_mode;
            printf("Debug trace mode: %s\n", bc->debug_mode ? "ON" : "OFF");
        }
        else if (strcmp(command, "input") == 0) {
             char *input_ptr = command_line + strlen(command);
             while (*input_ptr && isspace((unsigned char)*input_ptr)) input_ptr++;
             if (*input_ptr) {
                 bc->input_char = *input_ptr; bc->input_waiting = true;
                 printf("Simulated input '%c' (0x%02X) waiting. FGI will be set next cycle if not consumed.\n",
                        isprint(bc->input_char) ? bc->input_char : '.', bc->input_char & BYTE_MASK);
             } else { printf("Usage: input <single_character>\n"); }
        }
        else if (strcmp(command, "interrupt") == 0) {
            if (!bc->reg.R) { bc->reg.R = true; printf("Manual Interrupt Request set (R=1).\n"); }
            else { printf("Interrupt already pending (R=1).\n"); }
        }
        else if (strcmp(command, "quit") == 0 || strcmp(command, "q") == 0) { printf("Exiting simulator.\n"); break; }
        else { printf("Unknown command: '%s'. Type 'help' for list.\n", command); }
    }
}

int main(int argc, char *argv[]) {
    BasicComputer bc;
    initialize_computer(&bc);
    const char *program_file = NULL;
    bool start_in_debug = false;

    // Argument Parsing
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0) { start_in_debug = true; }
        else if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--file") == 0) {
            if (i + 1 < argc) { program_file = argv[i + 1]; i++; }
            else { fprintf(stderr, "Error: -f/--file option requires a filename.\n"); return 1; }
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
             printf("Morris Mano Basic Computer Simulator\nUsage: %s [-d] [-f program.hex]\n", argv[0]);
             printf("  -d, --debug      : Enable detailed execution trace.\n");
             printf("  -f, --file <path>: Load program from hex file on startup.\n");
             printf("  -h, --help       : Show this help message.\n"); return 0;
        } else { fprintf(stderr, "Warning: Unknown argument '%s'. Ignored.\n", argv[i]); }
    }

    // Load Program if specified
    if (program_file) {
        initialize_computer(&bc); // Reset first
        if (!load_program_from_file(&bc, program_file)) {
            fprintf(stderr, "Failed to load program file '%s'. Exiting.\n", program_file); return 1;
        }
        bc.reg.PC = 0; bc.reg.SC = 0; bc.reg.S = true;
    } else {
        printf("No program file specified. Loading default HLT at 0x000.\n");
        bc.memory[0] = 0x7001; // HLT
    }

    // Apply debug mode *after* potential init/load
    bc.debug_mode = start_in_debug;
    if(bc.debug_mode) printf("Debug mode enabled.\n");

    // Start Interactive Mode
    printf("\nWelcome to the Morris Mano Basic Computer Simulator!\n");
    printf("Memory size: %d words (16-bit), Address size: 12 bits.\n", MEMORY_SIZE);
    dump_registers(&bc);
    run_interactive(&bc);

    return 0;
}
