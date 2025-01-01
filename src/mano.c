#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h> // For fixed-width integers
#include <ctype.h>  // For isprint

// Define types using standard integer types
typedef uint16_t Word;    // 16-bit word
typedef uint16_t Address; // 12-bit address (use lower 12 bits)
typedef uint8_t Opcode;   // 3-bit opcode (bits 14-12) - Use uint8_t for clarity
typedef uint8_t Byte;     // 8-bit byte

// Memory size: 4096 words of 16 bits each
#define MEMORY_SIZE 4096
#define ADDRESS_MASK 0x0FFF // Mask for 12-bit addresses
#define WORD_MASK    0xFFFF // Mask for 16-bit words
#define BYTE_MASK    0xFF   // Mask for 8-bit bytes

// --- Hardware Constants ---

// Bus Select Lines (S2, S1, S0) - Mapping to Mano's Figure 5-4
// These select which register provides data *to* the common bus.
// Using an enum for functional clarity.
typedef enum {
    BUS_NONE = 0x0, // 000: No source selected (bus = 0)
    BUS_AR   = 0x1, // 001: AR -> Bus
    BUS_PC   = 0x2, // 010: PC -> Bus
    BUS_DR   = 0x3, // 011: DR -> Bus
    BUS_AC   = 0x4, // 100: AC -> Bus
    BUS_IR   = 0x5, // 101: IR -> Bus
    BUS_TR   = 0x6, // 110: TR -> Bus
    BUS_MEM  = 0x7  // 111: Memory -> DR -> Bus (Requires Memory Read Cycle)
} BusSource;

// Control Signals - Corresponding to Load (LD), Increment (INR), Clear (CLR) lines
// Using bit flags as multiple signals can be active.
typedef enum {
    // Register Load Signals
    CTRL_LD_AR   = 0x0001, // Load AR from Bus
    CTRL_LD_PC   = 0x0002, // Load PC from Bus
    CTRL_LD_DR   = 0x0004, // Load DR from Bus
    CTRL_LD_AC   = 0x0008, // Load AC from Bus
    CTRL_LD_IR   = 0x0010, // Load IR from Bus
    CTRL_LD_TR   = 0x0020, // Load TR from Bus

    // Register Increment Signals
    CTRL_INR_AR  = 0x0100, // Increment AR
    CTRL_INR_PC  = 0x0200, // Increment PC
    CTRL_INR_DR  = 0x0400, // Increment DR (used by ISZ)
    CTRL_INR_AC  = 0x0800, // Increment AC (used by INC instruction)

    // Register Clear Signals
    CTRL_CLR_AR  = 0x1000, // Clear AR
    CTRL_CLR_PC  = 0x2000, // Clear PC
    CTRL_CLR_AC  = 0x4000, // Clear AC (used by CLA instruction)
    CTRL_CLR_E   = 0x8000, // Clear E   (used by CLE instruction)

    // Memory Control Signals
    CTRL_MEM_READ  = 0x10000, // Activate Memory Read (M[AR] -> DR)
    CTRL_MEM_WRITE = 0x20000, // Activate Memory Write (Source -> M[AR])

    // Sequence Counter Control
    CTRL_INR_SC  = 0x40000, // Increment SC
    CTRL_CLR_SC  = 0x80000, // Clear SC

    // I/O Instruction Bits (used for flag logic, though not typical 'control signals')
    IO_OUT       = 0x100000, // Flag indicating OUT occurred this cycle
    IO_INP       = 0x200000, // Flag indicating INP occurred this cycle

} ControlSignal;

// Opcode definitions (bits 14-12 of instruction)
#define OP_AND  0x0 // AND Memory to AC
#define OP_ADD  0x1 // Add Memory to AC
#define OP_LDA  0x2 // Load Memory to AC
#define OP_STA  0x3 // Store AC to Memory
#define OP_BUN  0x4 // Branch unconditionally
#define OP_BSA  0x5 // Branch and save return address
#define OP_ISZ  0x6 // Increment and skip if zero
#define OP_OTHER 0x7 // Register Reference or I/O instruction

// Register reference micro-operations (Opcode 7, I=0) - Bit definitions
#define RREG_CLA 0x800 // Clear AC
#define RREG_CLE 0x400 // Clear E
#define RREG_CMA 0x200 // Complement AC
#define RREG_CME 0x100 // Complement E
#define RREG_CIR 0x080 // Circulate right AC and E
#define RREG_CIL 0x040 // Circulate left AC and E
#define RREG_INC 0x020 // Increment AC
#define RREG_SPA 0x010 // Skip if AC positive (MSB=0)
#define RREG_SNA 0x008 // Skip if AC negative (MSB=1)
#define RREG_SZA 0x004 // Skip if AC zero
#define RREG_SZE 0x002 // Skip if E zero
#define RREG_HLT 0x001 // Halt computer

// I/O micro-operations (Opcode 7, I=1) - Bit definitions
#define IO_DEF_INP 0x800 // Input character to AC(0-7)
#define IO_DEF_OUT 0x400 // Output character from AC(0-7)
#define IO_DEF_SKI 0x200 // Skip on input flag (FGI)
#define IO_DEF_SKO 0x100 // Skip on output flag (FGO)
#define IO_DEF_ION 0x080 // Interrupt enable on
#define IO_DEF_IOF 0x040 // Interrupt enable off

// Indirect addressing bit (bit 15 of instruction)
#define I_BIT   0x8000

// --- CPU State ---

// Basic Computer Registers
typedef struct {
    Address AR;     // Address Register (12 bits)
    Address PC;     // Program Counter (12 bits)
    Word DR;        // Data Register (16 bits)
    Word AC;        // Accumulator (16 bits)
    Word IR;        // Instruction Register (16 bits)
    Word TR;        // Temporary Register (16 bits) - Used in interrupt cycle
    Byte INPR;      // Input Register (8 bits)
    Byte OUTR;      // Output Register (8 bits)
    Byte SC;        // Sequence Counter (4 bits, 0-6 typically needed)
    Byte E;         // Extended Accumulator Bit (Carry/Link) (1 bit)
    bool I;         // Indirect bit (from IR, stored for convenience during execution)
    Opcode D;       // Decoded Opcode (D0-D7, stored for convenience)
    // Status Flags
    bool S;         // Start-Stop Flip-Flop (1 = Run, 0 = Stop/Halted)
    bool R;         // Interrupt Request Flip-Flop (pending interrupt)
    bool IEN;       // Interrupt Enable Flip-Flop
    bool FGI;       // Input Flag (Input device ready)
    bool FGO;       // Output Flag (Output device ready)
} Registers;

// Complete Basic Computer VM
typedef struct {
    Word memory[MEMORY_SIZE]; // Main memory
    Registers reg;            // CPU registers
    // Simulation Control
    bool running;             // Overall VM simulation running state
    bool debug_mode;          // Print detailed execution trace
    uint64_t cycle_count;     // Track number of clock cycles executed
    // Internal state for simulation step
    Word data_bus;            // Value on the common data bus during a cycle
    BusSource bus_select;     // Which register is driving the bus
    ControlSignal ctrl;       // Active control signals for the current cycle
    // Simple I/O simulation state
    bool input_waiting;       // Simulates if input has been typed but not read by INP
    char input_char;          // Stores the waiting input character
    bool fgo_just_cleared;    // Flag to delay FGO reset by one cycle
} BasicComputer;

// --- Function Prototypes ---

// Core Simulation
void initialize_computer(BasicComputer *bc);
bool execute_clock_cycle(BasicComputer *bc); // Executes ONE clock tick (T-state)
void run_computer(BasicComputer *bc);
void run_interactive(BasicComputer *bc);

// Memory Access
Word read_memory(BasicComputer *bc, Address addr);
void write_memory(BasicComputer *bc, Address addr, Word data);

// Bus and Control Logic Simulation
void evaluate_bus(BasicComputer *bc);
void execute_control_signals(BasicComputer *bc);

// Instruction Phase Execution Helpers
void execute_instruction_fetch_t0(BasicComputer *bc);
void execute_instruction_fetch_t1(BasicComputer *bc);
void execute_decode_t2(BasicComputer *bc);
void execute_indirect_address_fetch_t3(BasicComputer *bc);
void execute_instruction_execute_t3_or_t4(BasicComputer *bc); // Handles actual execution micro-ops

// Interrupt Cycle Helpers
void execute_interrupt_t0(BasicComputer *bc);
void execute_interrupt_t1(BasicComputer *bc);
void execute_interrupt_t2(BasicComputer *bc);

// Instruction Execution Logic (Micro-operations)
void execute_memory_reference(BasicComputer *bc);
void execute_register_reference(BasicComputer *bc);
void execute_io(BasicComputer *bc);

// Utility Functions
void dump_registers(const BasicComputer *bc);
void dump_memory(const BasicComputer *bc, Address start, Address end);
bool load_program_from_file(BasicComputer *bc, const char *filename);
void print_instruction(Word instruction, Address addr);
const char* bus_source_to_string(BusSource src);
const char* control_signal_to_string(ControlSignal signal_bit); // Added
void print_active_control_signals(ControlSignal active_ctrl); // Added

// --- Function Implementations ---

// Initialize the computer state
void initialize_computer(BasicComputer *bc) {
    memset(bc->memory, 0, sizeof(bc->memory));
    memset(&bc->reg, 0, sizeof(bc->reg)); // Clear all registers

    bc->reg.PC = 0x000;    // Default start address
    bc->reg.SC = 0;
    bc->reg.E = 0;
    bc->reg.S = true;      // Computer starts in the 'on' state, ready to run
    bc->reg.R = false;     // No pending interrupt initially
    bc->reg.IEN = false;   // Interrupts disabled initially
    bc->reg.FGI = false;   // No input ready initially
    bc->reg.FGO = true;    // Output device is ready initially

    bc->running = false;   // Simulation loop not active until run command
    bc->debug_mode = false;
    bc->cycle_count = 0;
    bc->data_bus = 0;
    bc->bus_select = BUS_NONE;
    bc->ctrl = 0;
    bc->input_waiting = false;
    bc->input_char = 0;
    bc->fgo_just_cleared = false;
}

// Read from memory (handles 12-bit address masking)
Word read_memory(BasicComputer *bc, Address addr) {
    addr &= ADDRESS_MASK;
    if (addr >= MEMORY_SIZE) { // Basic bounds check
         if (bc->debug_mode) printf("DEBUG: Memory read attempt outside bounds: 0x%03X\n", addr);
         return 0; // Or handle as error
    }
    return bc->memory[addr];
}

// Write to memory (handles 12-bit address masking)
void write_memory(BasicComputer *bc, Address addr, Word data) {
    addr &= ADDRESS_MASK;
    if (addr >= MEMORY_SIZE) { // Basic bounds check
         if (bc->debug_mode) printf("DEBUG: Memory write attempt outside bounds: 0x%03X (Data: 0x%04X)\n", addr, data);
        return; // Prevent writing out of bounds
    }
    if (bc->debug_mode) {
         printf("DEBUG: Memory Write: M[0x%03X] <- 0x%04X\n", addr, data);
    }
    bc->memory[addr] = data;
}

// Evaluate the bus value based on bus select lines
void evaluate_bus(BasicComputer *bc) {
    switch (bc->bus_select) {
        case BUS_AR:  bc->data_bus = bc->reg.AR; break;
        case BUS_PC:  bc->data_bus = bc->reg.PC; break;
        case BUS_DR:  bc->data_bus = bc->reg.DR; break;
        case BUS_AC:  bc->data_bus = bc->reg.AC; break;
        case BUS_IR:  bc->data_bus = bc->reg.IR; break;
        case BUS_TR:  bc->data_bus = bc->reg.TR; break;
        case BUS_MEM: // Memory data should be in DR from a CTRL_MEM_READ
                      bc->data_bus = bc->reg.DR; break;
        case BUS_NONE:
        default:      bc->data_bus = 0; break; // Bus not driven
    }

    if (bc->debug_mode && bc->bus_select != BUS_NONE) {
        printf("DEBUG: Bus <- %s (Value=0x%04X)\n", bus_source_to_string(bc->bus_select), bc->data_bus);
    }
}

// Helper to convert control signal bit to string
const char* control_signal_to_string(ControlSignal signal_bit) {
    switch (signal_bit) {
        case CTRL_LD_AR: return "LD_AR";
        case CTRL_LD_PC: return "LD_PC";
        case CTRL_LD_DR: return "LD_DR";
        case CTRL_LD_AC: return "LD_AC";
        case CTRL_LD_IR: return "LD_IR";
        case CTRL_LD_TR: return "LD_TR";
        case CTRL_INR_AR: return "INR_AR";
        case CTRL_INR_PC: return "INR_PC";
        case CTRL_INR_DR: return "INR_DR";
        case CTRL_INR_AC: return "INR_AC";
        case CTRL_CLR_AR: return "CLR_AR";
        case CTRL_CLR_PC: return "CLR_PC";
        case CTRL_CLR_AC: return "CLR_AC";
        case CTRL_CLR_E: return "CLR_E";
        case CTRL_MEM_READ: return "MEM_READ";
        case CTRL_MEM_WRITE: return "MEM_WRITE";
        case CTRL_INR_SC: return "INR_SC";
        case CTRL_CLR_SC: return "CLR_SC";
        // Add others if needed, but IO_OUT/IO_INP aren't typical 'control signals' driven by control unit
        default: return "UNKNOWN_CTRL";
    }
}

// Helper to print active control signals
void print_active_control_signals(ControlSignal active_ctrl) {
    printf("DEBUG: Active Signals: ");
    bool first = true;
    // Check known control signals explicitly
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


// Execute actions based on active control signals for the *current* clock cycle
void execute_control_signals(BasicComputer *bc) {
    ControlSignal active_ctrl = bc->ctrl; // Work with a copy for clarity

    if (bc->debug_mode) {
        print_active_control_signals(active_ctrl); // Print active signals
    }

    // --- Memory Operations ---
    if (active_ctrl & CTRL_MEM_READ) {
        bc->reg.DR = read_memory(bc, bc->reg.AR);
        if (bc->debug_mode) printf("DEBUG: Memory Read: DR <- M[0x%03X] (Read 0x%04X)\n", bc->reg.AR & ADDRESS_MASK, bc->reg.DR);
    }
    // Note: Actual write happens in execution functions calling write_memory()
    // We can just report the *intent* here if MEM_WRITE was set
    if (active_ctrl & CTRL_MEM_WRITE) {
        if (bc->debug_mode) printf("DEBUG: Memory Write Intent: M[0x%03X] will be written\n", bc->reg.AR & ADDRESS_MASK);
    }

    // --- Bus Evaluation (Source drives the bus) ---
    evaluate_bus(bc); // Determine bc->data_bus value based on bc->bus_select

    // --- Register Load Operations (Registers load *from* the bus) ---
    if (active_ctrl & CTRL_LD_AR) { bc->reg.AR = bc->data_bus & ADDRESS_MASK; if (bc->debug_mode) printf("DEBUG: Register Load: AR <- Bus (0x%03X)\n", bc->reg.AR); }
    if (active_ctrl & CTRL_LD_PC) { bc->reg.PC = bc->data_bus & ADDRESS_MASK; if (bc->debug_mode) printf("DEBUG: Register Load: PC <- Bus (0x%03X)\n", bc->reg.PC); }
    if (active_ctrl & CTRL_LD_DR) { bc->reg.DR = bc->data_bus; if (bc->debug_mode) printf("DEBUG: Register Load: DR <- Bus (0x%04X)\n", bc->reg.DR); }
    if (active_ctrl & CTRL_LD_AC) { bc->reg.AC = bc->data_bus; if (bc->debug_mode) printf("DEBUG: Register Load: AC <- Bus (0x%04X)\n", bc->reg.AC); }
    if (active_ctrl & CTRL_LD_IR) { bc->reg.IR = bc->data_bus; if (bc->debug_mode) printf("DEBUG: Register Load: IR <- Bus (0x%04X)\n", bc->reg.IR); }
    if (active_ctrl & CTRL_LD_TR) { bc->reg.TR = bc->data_bus; if (bc->debug_mode) printf("DEBUG: Register Load: TR <- Bus (0x%04X)\n", bc->reg.TR); }

    // --- Register Increment Operations ---
    if (active_ctrl & CTRL_INR_AR) { bc->reg.AR = (bc->reg.AR + 1) & ADDRESS_MASK; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: AR++ -> 0x%03X\n", bc->reg.AR); }
    if (active_ctrl & CTRL_INR_PC) { bc->reg.PC = (bc->reg.PC + 1) & ADDRESS_MASK; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: PC++ -> 0x%03X\n", bc->reg.PC); }
    if (active_ctrl & CTRL_INR_DR) { bc->reg.DR = (bc->reg.DR + 1) & WORD_MASK; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: DR++ -> 0x%04X\n", bc->reg.DR); }
    if (active_ctrl & CTRL_INR_AC) {
        // NOTE: This is for the INC *instruction*. ALU ADD operation is separate.
        bc->reg.AC = (bc->reg.AC + 1) & WORD_MASK;
        if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: AC++ (INC Instruction) -> 0x%04X\n", bc->reg.AC);
    }

    // --- Register Clear Operations ---
    if (active_ctrl & CTRL_CLR_AR) { bc->reg.AR = 0; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: AR CLR\n"); }
    if (active_ctrl & CTRL_CLR_PC) { bc->reg.PC = 0; if (bc->debug_mode) printf("DEBUG: Register Inc/Clr: PC CLR\n"); }
    if (active_ctrl & CTRL_CLR_AC) { // Note: Also used by CLA instruction
         if (bc->debug_mode && bc->reg.AC != 0) printf("DEBUG: Register Inc/Clr: AC CLR\n");
         bc->reg.AC = 0;
    }
    if (active_ctrl & CTRL_CLR_E)  { // Note: Also used by CLE instruction
         if (bc->debug_mode && bc->reg.E != 0) printf("DEBUG: Flag Change: E CLR\n");
         bc->reg.E = 0;
    }

    // --- Update I/O Flags (Simple Simulation) ---
    // Handle FGO reset delay
    if (bc->fgo_just_cleared) {
        bc->reg.FGO = true; // Simulate output device ready again
        bc->fgo_just_cleared = false;
        if (bc->debug_mode) printf("DEBUG: Flag Change: FGO -> 1 (Output Ready)\n");
    }
    // Check if FGO was cleared this cycle (needs IO_OUT flag set temporarily by execute_io)
    if (!(active_ctrl & IO_OUT) && !bc->reg.FGO) {
        // If FGO is false, but OUT didn't happen *this cycle*, it means it happened last cycle.
        // Set the flag to reset FGO in the *next* cycle.
        bc->fgo_just_cleared = true;
    }


    // Simulate FGI becoming true if input arrives (e.g., user types)
    // FGI is set true immediately by 'input' command or potentially other simulation means.
    // FGI is cleared by INP instruction execution (active_ctrl & IO_INP)
    if(bc->reg.FGI && (active_ctrl & IO_INP)) {
         if (bc->debug_mode) printf("DEBUG: Flag Change: FGI -> 0 (Input Consumed)\n");
         // Actual FGI = false happens in execute_io
    } else if (bc->input_waiting && !bc->reg.FGI) {
         bc->reg.FGI = true; // Input command makes it ready
         if (bc->debug_mode) printf("DEBUG: Flag Change: FGI -> 1 (Input Ready)\n");
    }


    // --- Sequence Counter Update ---
    // IMPORTANT: SC is incremented/cleared for the *next* cycle.
    if (active_ctrl & CTRL_CLR_SC) {
        bc->reg.SC = 0;
        if (bc->debug_mode) printf("DEBUG: SC CLR -> T0\n");
    } else if (active_ctrl & CTRL_INR_SC) {
        bc->reg.SC = (bc->reg.SC + 1); // SC can go beyond 3
        if (bc->debug_mode) printf("DEBUG: SC++ -> T%d\n", bc->reg.SC);
    }

    // Reset control signals for the next cycle
    bc->ctrl = 0;
    bc->bus_select = BUS_NONE;
}


// Execute one clock cycle (T-state transition)
// Returns true if computer is still running, false if HLT occurred.
bool execute_clock_cycle(BasicComputer *bc) {
    if (!bc->reg.S) { // Check Halt flip-flop
        // Computer is halted, don't execute further cycles until started
        return false;
    }

    bc->cycle_count++;
    if (bc->debug_mode) {
        printf("\n--- Cycle %llu --- SC=T%d, PC=0x%03X, R=%d, IEN=%d ---\n",
               bc->cycle_count, bc->reg.SC, bc->reg.PC, bc->reg.R, bc->reg.IEN);
    }

    // Determine if Interrupt Cycle or Instruction Cycle based on R, IEN, and SC=0
    bool is_interrupt_cycle = (bc->reg.R && bc->reg.IEN && bc->reg.SC == 0);

    // Set bus source and control signals based on current state (SC, I, D, etc.)
    if (is_interrupt_cycle) {
        if (bc->debug_mode) printf("DEBUG: Interrupt Cycle, T%d\n", bc->reg.SC);
        switch (bc->reg.SC) {
            case 0: execute_interrupt_t0(bc); break; // RT0
            case 1: execute_interrupt_t1(bc); break; // RT1
            case 2: execute_interrupt_t2(bc); break; // RT2
            default: // Should not happen
                printf("ERROR: Invalid SC state during interrupt cycle: %d\n", bc->reg.SC);
                bc->ctrl |= CTRL_CLR_SC; // Reset SC
                break;
        }
    } else {
        // Instruction Cycle
        if (bc->debug_mode) printf("DEBUG: Instruction Cycle, T%d\n", bc->reg.SC);
        switch (bc->reg.SC) {
            case 0: execute_instruction_fetch_t0(bc); break; // T0: Fetch Phase 1
            case 1: execute_instruction_fetch_t1(bc); break; // T1: Fetch Phase 2
            case 2: execute_decode_t2(bc); break;          // T2: Decode Phase
            case 3:
                // T3 can be Indirect Address Fetch OR start of Execution
                if ((bc->reg.D != OP_OTHER) && bc->reg.I) {
                     execute_indirect_address_fetch_t3(bc); // Indirect Mem-Ref: T3 = Get Effective Addr
                } else {
                     execute_instruction_execute_t3_or_t4(bc); // Direct Mem-Ref, Reg-Ref, I/O: T3 = Execute
                }
                break;
            case 4: // T4 onwards are for execution phases of complex instructions or indirect mem-ref
                 execute_instruction_execute_t3_or_t4(bc); // Indirect Mem-Ref: T4 = Execute
                 break;
            // Add cases for T5, T6 if implementing multi-cycle instructions like ISZ precisely
            default:
                 // If execution finished earlier, SC should have been cleared.
                 if (bc->debug_mode) printf("Warning: Reached unhandled SC state T%d in instruction cycle. Resetting SC.\n", bc->reg.SC);
                 bc->ctrl |= CTRL_CLR_SC; // Go back to T0
                 break;
        }
    }

    // Apply the determined control signals and bus selection
    execute_control_signals(bc);

    // Check S flag again in case HLT instruction was executed in this cycle
    return bc->reg.S;
}

// --- Instruction Cycle T-State Helpers ---

// T0: AR <- PC
void execute_instruction_fetch_t0(BasicComputer *bc) {
    bc->bus_select = BUS_PC;    // Put PC onto bus
    bc->ctrl |= CTRL_LD_AR;     // Load AR from bus
    bc->ctrl |= CTRL_INR_SC;    // Go to T1
}

// T1: IR <- M[AR], PC <- PC + 1
void execute_instruction_fetch_t1(BasicComputer *bc) {
    bc->ctrl |= CTRL_MEM_READ; // Read memory pointed by AR into DR
    bc->bus_select = BUS_MEM;   // Select Memory Output (via DR)
    bc->ctrl |= CTRL_LD_IR;     // Load IR from Bus (which carries M[AR] via DR)
    bc->ctrl |= CTRL_INR_PC;    // Increment PC
    bc->ctrl |= CTRL_INR_SC;    // Go to T2
}

// T2: Decode instruction in IR, Check Indirect Bit, Prepare AR
void execute_decode_t2(BasicComputer *bc) {
    Address addr_part = bc->reg.IR & ADDRESS_MASK;
    bc->reg.D = (bc->reg.IR >> 12) & 0x7; // Opcode D0-D7
    bc->reg.I = (bc->reg.IR & I_BIT) ? true : false; // Indirect bit

    if (bc->debug_mode) {
        printf("DEBUG: T2 Decode: IR=0x%04X -> Opcode D=%d, I=%d, Addr=0x%03X\n",
               bc->reg.IR, bc->reg.D, bc->reg.I, addr_part);
    }

    if (bc->reg.D == OP_OTHER) { // Register Reference or I/O
        // No memory address needed initially.
        bc->ctrl |= CTRL_INR_SC; // Go to T3 for execution
    } else { // Memory Reference
        // Put the address part of IR into AR.
        // Simulate direct connection: Control unit decodes IR address bits -> AR input mux
        bc->reg.AR = addr_part;
        if (bc->debug_mode) printf("DEBUG: T2 Decode: AR <- IR(0-11) = 0x%03X\n", bc->reg.AR);
        // Go to T3 (for indirect fetch or execution)
        bc->ctrl |= CTRL_INR_SC;
    }
}

// T3 (Indirect Mem-Ref): AR <- M[AR] (Fetch Effective Address)
void execute_indirect_address_fetch_t3(BasicComputer *bc) {
     if (bc->debug_mode) printf("DEBUG: T3 Indirect Fetch: AR <- M[0x%03X]\n", bc->reg.AR);
     bc->ctrl |= CTRL_MEM_READ;  // Read from address specified by current AR
     bc->bus_select = BUS_MEM;   // Select memory output (via DR)
     bc->ctrl |= CTRL_LD_AR;     // Load the effective address into AR
     bc->ctrl |= CTRL_INR_SC;    // Go to T4 for execution
}


// T3 or T4: Execute the instruction based on D and I flags
// This function assumes AR holds the final effective address (if applicable)
void execute_instruction_execute_t3_or_t4(BasicComputer *bc) {
    if (bc->debug_mode) printf("DEBUG: T%d Execute Phase: D=%d, I=%d\n", bc->reg.SC, bc->reg.D, bc->reg.I);

    // Based on decoded opcode D, call the appropriate execution function
    switch (bc->reg.D) {
        case OP_AND:
        case OP_ADD:
        case OP_LDA:
        case OP_STA:
        case OP_BUN:
        case OP_BSA:
        case OP_ISZ:
            execute_memory_reference(bc); // Handles memory reference micro-ops
            break;

        case OP_OTHER: // Register Ref or I/O
            if (bc->reg.I == 0) { // Register Reference (I=0)
                execute_register_reference(bc);
            } else { // I/O (I=1)
                execute_io(bc);
            }
            break;

        default: // Should not happen
            printf("ERROR: Invalid decoded opcode D=%d during execution phase.\n", bc->reg.D);
            break;
    }

    // After execution, always return to T0 for the next instruction fetch
    bc->ctrl |= CTRL_CLR_SC;
}


// --- Interrupt Cycle T-State Helpers ---

// RT0: AR <- 0, TR <- PC
void execute_interrupt_t0(BasicComputer *bc) {
    bc->ctrl |= CTRL_CLR_AR;    // Clear AR (Set AR to 0)
    bc->bus_select = BUS_PC;    // Put PC onto bus
    bc->ctrl |= CTRL_LD_TR;     // Load TR from bus (Save PC)
    bc->ctrl |= CTRL_INR_SC;    // Go to RT1
}

// RT1: M[AR] <- TR, PC <- 0
void execute_interrupt_t1(BasicComputer *bc) {
    // Store TR (saved PC) into memory location 0 (AR is 0)
    write_memory(bc, bc->reg.AR, bc->reg.TR);
    bc->ctrl |= CTRL_MEM_WRITE; // Signal the intent for debug purposes

    bc->ctrl |= CTRL_CLR_PC;    // Clear PC (Set PC to 0)
    bc->ctrl |= CTRL_INR_SC;    // Go to RT2
}

// RT2: PC <- PC + 1, IEN <- 0, R <- 0, SC <- 0
void execute_interrupt_t2(BasicComputer *bc) {
    bc->ctrl |= CTRL_INR_PC;    // Increment PC (PC becomes 1, start of ISR)

    // Clear interrupt flags - Direct manipulation for simplicity
    bool ien_changed = bc->reg.IEN; // Check if IEN was true
    bool r_changed = bc->reg.R;     // Check if R was true
    bc->reg.IEN = false;
    bc->reg.R = false;
    if (bc->debug_mode) {
        if(ien_changed) printf("DEBUG: Flag Change: IEN -> 0\n");
        if(r_changed) printf("DEBUG: Flag Change: R -> 0\n");
        // PC increment happens via control signal later, so current PC is still 0 here
        printf("DEBUG: Interrupt Cycle Finished. Next fetch cycle will target PC=0x001\n");
    }

    bc->ctrl |= CTRL_CLR_SC;    // Clear SC, end interrupt cycle, next cycle is T0
}

// --- Instruction Execution Logic ---

// Execute Memory Reference Instruction (micro-operations within T3/T4)
// Assumes AR has the effective address.
void execute_memory_reference(BasicComputer *bc) {
    Address eff_addr = bc->reg.AR;
    Word operand = 0; // Initialize
    const char* alu_op_str = "None"; // For debug output

    if (bc->debug_mode) printf("DEBUG: Executing Mem Ref D=%d, Eff Addr=0x%03X\n", bc->reg.D, eff_addr);

    // Simulate T4: Fetch operand if needed
    if (bc->reg.D != OP_STA && bc->reg.D != OP_BUN && bc->reg.D != OP_BSA) {
         // Simulate DR <- M[AR]
         bc->reg.DR = read_memory(bc, eff_addr);
         if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T4: Fetched Operand M[0x%03X] = 0x%04X into DR\n", eff_addr, bc->reg.DR);
         operand = bc->reg.DR;
    }

    // Simulate T5 (or T4 for some): Perform operation
    switch (bc->reg.D) {
        case OP_AND:
            alu_op_str = "AND";
            bc->reg.AC &= operand; // AC <- AC & DR
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: AC <- AC & DR = 0x%04X\n", bc->reg.AC);
            break;

        case OP_ADD:
            alu_op_str = "ADD";
            {
                uint32_t result = (uint32_t)bc->reg.AC + (uint32_t)operand; // AC <- AC + DR
                Byte prev_E = bc->reg.E;
                bc->reg.E = (result > 0xFFFF) ? 1 : 0; // E <- Cout
                bc->reg.AC = (Word)(result & WORD_MASK);
                if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: AC <- AC + DR = 0x%04X\n", bc->reg.AC);
                if (bc->debug_mode && prev_E != bc->reg.E) printf("DEBUG: Flag Change: E -> %d\n", bc->reg.E);
            }
            break;

        case OP_LDA:
            alu_op_str = "LOAD";
            bc->reg.AC = operand; // AC <- DR
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: AC <- DR = 0x%04X\n", bc->reg.AC);
            break;

        case OP_STA:
            alu_op_str = "STORE";
            // T4: M[AR] <- AC
            write_memory(bc, eff_addr, bc->reg.AC);
            bc->ctrl |= CTRL_MEM_WRITE; // Signal intent for debug
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T4: M[0x%03X] <- AC\n", eff_addr);
            break;

        case OP_BUN:
            alu_op_str = "BRANCH";
            // T4: PC <- AR
            bc->reg.PC = eff_addr;
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T4: PC <- AR = 0x%03X\n", bc->reg.PC);
            break;

        case OP_BSA:
            alu_op_str = "BRANCH_SAVE";
            // T4: M[AR] <- PC, AR <- AR + 1
            write_memory(bc, eff_addr, bc->reg.PC); // Use current PC before potential increment
            bc->ctrl |= CTRL_MEM_WRITE;
            bc->reg.AR = (eff_addr + 1) & ADDRESS_MASK;
            if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T4: M[0x%03X] <- PC, AR <- AR+1 = 0x%03X\n", eff_addr, bc->reg.AR);
            // T5: PC <- AR
            bc->reg.PC = bc->reg.AR;
             if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: PC <- AR = 0x%03X\n", bc->reg.PC);
            break;

        case OP_ISZ:
            alu_op_str = "INC_MEM";
            // T4: DR <- M[AR] (Done above)
            // T5: DR <- DR + 1
            operand = (operand + 1) & WORD_MASK;
             if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T5: DR <- DR+1 = 0x%04X\n", operand);
            // T6: M[AR] <- DR, If (DR=0) then PC <- PC+1, SC <- 0
            write_memory(bc, eff_addr, operand);
            bc->ctrl |= CTRL_MEM_WRITE;
             if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T6: M[0x%03X] <- DR\n", eff_addr);
            if (operand == 0) {
                bc->ctrl |= CTRL_INR_PC; // Schedule PC increment
                if (bc->debug_mode) printf("DEBUG: Exec Mem Ref T6: Result is Zero. Skipping (PC will be incremented).\n");
            }
            break;
    }
    if (bc->debug_mode && strcmp(alu_op_str, "None") != 0) {
         printf("DEBUG: ALU/Execute Action: %s\n", alu_op_str);
    }
     // SC <- 0 is handled by the caller (execute_instruction_execute_t3_or_t4)
}

// Execute Register Reference Instruction (micro-operations within T3)
void execute_register_reference(BasicComputer *bc) {
    Word micro_op_bits = bc->reg.IR & 0x0FFF;
    bool flag_changed = false;
    Byte prev_E = bc->reg.E; // Store E before modification

    if (bc->debug_mode) printf("DEBUG: Executing Register Ref IR=0x%04X\n", bc->reg.IR);

    // Apply micro-operations based on bits. Report changes.
    if (micro_op_bits & RREG_CLA) { if (bc->debug_mode && bc->reg.AC != 0) printf("DEBUG: Register Inc/Clr: AC CLR\n"); bc->reg.AC = 0; }
    if (micro_op_bits & RREG_CLE) { if (bc->debug_mode && bc->reg.E != 0) printf("DEBUG: Flag Change: E CLR\n"); bc->reg.E = 0; flag_changed = true;}
    if (micro_op_bits & RREG_CMA) { if (bc->debug_mode) printf("DEBUG: ALU Action: CMA\n"); bc->reg.AC = ~bc->reg.AC; }
    if (micro_op_bits & RREG_CME) { if (bc->debug_mode) printf("DEBUG: Flag Change: E = ~E (%d -> %d)\n", prev_E, (~prev_E & 1)); bc->reg.E = ~bc->reg.E & 0x1; flag_changed = true;}
    if (micro_op_bits & RREG_CIR) {
        if (bc->debug_mode) printf("DEBUG: ALU Action: CIR\n");
        prev_E = bc->reg.E; // Update prev_E before change
        Byte temp_lsb = bc->reg.AC & 0x0001;
        bc->reg.AC = (bc->reg.AC >> 1) | (prev_E << 15); // Shift right, old E to MSB
        bc->reg.E = temp_lsb; // LSB of AC to E
        if (bc->debug_mode && prev_E != bc->reg.E) printf("DEBUG: Flag Change: E -> %d\n", bc->reg.E); flag_changed = true;
    }
    if (micro_op_bits & RREG_CIL) {
        if (bc->debug_mode) printf("DEBUG: ALU Action: CIL\n");
        prev_E = bc->reg.E; // Update prev_E before change
        Byte temp_msb = (bc->reg.AC & 0x8000) ? 1 : 0; // MSB of AC
        bc->reg.AC = (bc->reg.AC << 1) | prev_E; // Shift left, old E to LSB
        bc->reg.E = temp_msb; // MSB to E
        if (bc->debug_mode && prev_E != bc->reg.E) printf("DEBUG: Flag Change: E -> %d\n", bc->reg.E); flag_changed = true;
    }
    if (micro_op_bits & RREG_INC) { // Increment AC (does not affect E)
        if (bc->debug_mode) printf("DEBUG: ALU Action: INC AC\n");
        bc->reg.AC = (bc->reg.AC + 1) & WORD_MASK;
    }

    // Skip conditions - checked *after* potential modifications above
    bool skip = false;
    if ((micro_op_bits & RREG_SPA) && ( (bc->reg.AC & 0x8000) == 0) ) skip = true; // Skip if AC positive (MSB=0)
    if ((micro_op_bits & RREG_SNA) && ( (bc->reg.AC & 0x8000) != 0) ) skip = true; // Skip if AC negative (MSB=1)
    if ((micro_op_bits & RREG_SZA) && (bc->reg.AC == 0) ) skip = true; // Skip if AC zero
    if ((micro_op_bits & RREG_SZE) && (bc->reg.E == 0) ) skip = true; // Skip if E zero

    if (skip) {
        bc->ctrl |= CTRL_INR_PC; // Schedule PC increment
        if (bc->debug_mode) printf("DEBUG: Register Ref: Skip condition met. PC will be incremented.\n");
    }

    if (micro_op_bits & RREG_HLT) {
        bool prev_S = bc->reg.S;
        bc->reg.S = false; // Set Halt flip-flop
        bc->running = false; // Stop simulation loop too
        if (bc->debug_mode) printf("DEBUG: HLT instruction encountered.\n");
        if (bc->debug_mode && prev_S) printf("DEBUG: Flag Change: S -> 0 (Halt)\n");
        printf("INFO: Program Halted by HLT instruction.\n");
    }
    // SC <- 0 happens via CTRL_CLR_SC in caller
}

// Execute I/O Instruction (micro-operations within T3)
void execute_io(BasicComputer *bc) {
    Word micro_op_bits = bc->reg.IR & 0x0FFF;

    if (bc->debug_mode) printf("DEBUG: Executing I/O IR=0x%04X\n", bc->reg.IR);

    // Handle I/O operations
    if (micro_op_bits & IO_DEF_INP) {
        bc->ctrl |= IO_INP; // Signal intent for flag handling
        if (bc->reg.FGI) { // Only read if flag is set
            bc->reg.INPR = bc->input_char & BYTE_MASK;
            bc->reg.AC = (bc->reg.AC & 0xFF00) | bc->reg.INPR; // Load low byte of AC
            bc->reg.FGI = false; // Clear flag after reading
            bc->input_waiting = false; // Clear waiting flag
            printf("INFO: Input Read: '%c' (0x%02X) loaded into AC[0-7]\n",
                   isprint(bc->reg.INPR) ? bc->reg.INPR : '.', bc->reg.INPR);
            // FGI change reported by execute_control_signals
        } else {
            if(bc->debug_mode) printf("DEBUG: INP attempted but FGI=0.\n");
        }
    }
    if (micro_op_bits & IO_DEF_OUT) {
        bc->ctrl |= IO_OUT; // Signal intent for flag handling
        if (bc->reg.FGO) { // Only output if flag is set
            bc->reg.OUTR = bc->reg.AC & BYTE_MASK; // Get low byte from AC
            printf("OUTPUT: %c (0x%02X)\n", isprint(bc->reg.OUTR) ? bc->reg.OUTR : '.', bc->reg.OUTR);
            bc->reg.FGO = false; // Clear flag after output
            // FGO change reported by execute_control_signals
        } else {
             if(bc->debug_mode) printf("DEBUG: OUT attempted but FGO=0.\n");
        }
    }

    // Handle skip conditions (checked based on flags *before* potential ION/IOF)
    bool skip = false;
    if ((micro_op_bits & IO_DEF_SKI) && bc->reg.FGI) skip = true; // Skip if input ready
    if ((micro_op_bits & IO_DEF_SKO) && bc->reg.FGO) skip = true; // Skip if output ready

    if (skip) {
        bc->ctrl |= CTRL_INR_PC; // Schedule PC increment
        if (bc->debug_mode) printf("DEBUG: I/O: Skip condition met. PC will be incremented.\n");
    }

    // Handle interrupt enable/disable
    if (micro_op_bits & IO_DEF_ION) {
        if (bc->debug_mode && !bc->reg.IEN) printf("DEBUG: Flag Change: IEN -> 1 (Interrupts Enabled)\n");
        bc->reg.IEN = true;
    }
    if (micro_op_bits & IO_DEF_IOF) {
         if (bc->debug_mode && bc->reg.IEN) printf("DEBUG: Flag Change: IEN -> 0 (Interrupts Disabled)\n");
        bc->reg.IEN = false;
    }
    // SC <- 0 happens via CTRL_CLR_SC in caller
}


// --- Utility Functions ---

// Print register contents
void dump_registers(const BasicComputer *bc) {
    printf("Registers:\n");
    printf("  PC: 0x%03X   AR: 0x%03X   IR: 0x%04X   SC: T%d\n", bc->reg.PC, bc->reg.AR, bc->reg.IR, bc->reg.SC);
    printf("  AC: 0x%04X   DR: 0x%04X   TR: 0x%04X   E: %d\n", bc->reg.AC, bc->reg.DR, bc->reg.TR, bc->reg.E);
    printf("  INPR: 0x%02X OUTR: 0x%02X\n", bc->reg.INPR, bc->reg.OUTR);
    printf("  Status: S=%d R=%d IEN=%d FGI=%d FGO=%d\n", bc->reg.S, bc->reg.R, bc->reg.IEN, bc->reg.FGI, bc->reg.FGO);
    printf("  Internal: I=%d D=%d\n", bc->reg.I, bc->reg.D);
    printf("  VM State: running=%d, debug=%d, cycles=%llu\n", bc->running, bc->debug_mode, bc->cycle_count);
}

// Print memory contents within a range
void dump_memory(const BasicComputer *bc, Address start, Address end) {
    start &= ADDRESS_MASK;
    end &= ADDRESS_MASK;
    if (end < start) {
        Address temp = start;
        start = end;
        end = temp; // Swap if end < start
    }
    if (end >= MEMORY_SIZE) end = MEMORY_SIZE - 1;

    printf("Memory Dump [0x%03X - 0x%03X]:\n", start, end);
    for (Address addr = start; addr <= end; ++addr) {
        if ((addr % 8 == 0) || addr == start) { // Start newline every 8 words or at the very beginning
            if (addr != start) printf("\n");
            printf("  0x%03X: ", addr);
        }
        printf("%04X ", bc->memory[addr]);
    }
    printf("\n");
}

// Load program from a simple hex file
// Format: Each line contains one 16-bit hex word (e.g., "123A")
// Lines starting with '#' or empty lines are ignored.
// Loading starts at address 0.
bool load_program_from_file(BasicComputer *bc, const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Error opening program file");
        return false;
    }

    char line[100];
    Address current_addr = 0;
    int line_num = 0;
    bool success = true;

    printf("Loading program from '%s'...\n", filename);

    while (fgets(line, sizeof(line), file)) {
        line_num++;
        // Remove trailing newline/whitespace
        line[strcspn(line, "\r\n")] = 0;

        // Skip comments and empty lines
        char *trimmed_line = line;
        while (isspace((unsigned char)*trimmed_line)) trimmed_line++; // Skip leading whitespace
        if (*trimmed_line == '#' || *trimmed_line == '\0') {
            continue;
        }

        // Parse hex value
        Word value;
        if (sscanf(trimmed_line, "%hx", &value) != 1) { // Use %hx for unsigned short
            fprintf(stderr, "Error parsing hex value on line %d: '%s'\n", line_num, trimmed_line);
            success = false;
            break;
        }

        // Write to memory
        if (current_addr >= MEMORY_SIZE) {
            fprintf(stderr, "Error: Program too large for memory (exceeded %d words) on line %d.\n", MEMORY_SIZE, line_num);
             success = false;
            break;
        }
        // Use write_memory to ensure masking and potential future write logic
        write_memory(bc, current_addr, value);
        // printf("DEBUG: Loaded M[0x%03X] = 0x%04X\n", current_addr, value); // Optional verbose load
        current_addr++;
    }

    fclose(file);
    if (success) {
        printf("Program loaded successfully. %d words written.\n", current_addr);
    }
    return success;
}

// Print human-readable representation of an instruction
void print_instruction(Word instruction, Address addr) {
    Opcode opcode = (instruction >> 12) & 0x7;
    bool indirect = (instruction & I_BIT) != 0;
    Address operand = instruction & ADDRESS_MASK;

    printf("0x%03X: %04X  ", addr, instruction); // Show address and raw hex

    printf("[%c] ", indirect ? 'I' : 'D'); // D for Direct, I for Indirect

    switch (opcode) {
        case OP_AND: printf("AND 0x%03X", operand); break;
        case OP_ADD: printf("ADD 0x%03X", operand); break;
        case OP_LDA: printf("LDA 0x%03X", operand); break;
        case OP_STA: printf("STA 0x%03X", operand); break;
        case OP_BUN: printf("BUN 0x%03X", operand); break;
        case OP_BSA: printf("BSA 0x%03X", operand); break;
        case OP_ISZ: printf("ISZ 0x%03X", operand); break;
        case OP_OTHER: // Opcode 7
            if (!indirect) { // Register reference (I=0)
                printf("REG: ");
                Word reg_bits = instruction & 0x0FFF;
                if (reg_bits == 0) { printf("(NOP)"); break;} // Special case for 0x7000
                if (reg_bits & RREG_CLA) printf("CLA ");
                if (reg_bits & RREG_CLE) printf("CLE ");
                if (reg_bits & RREG_CMA) printf("CMA ");
                if (reg_bits & RREG_CME) printf("CME ");
                if (reg_bits & RREG_CIR) printf("CIR ");
                if (reg_bits & RREG_CIL) printf("CIL ");
                if (reg_bits & RREG_INC) printf("INC ");
                if (reg_bits & RREG_SPA) printf("SPA ");
                if (reg_bits & RREG_SNA) printf("SNA ");
                if (reg_bits & RREG_SZA) printf("SZA ");
                if (reg_bits & RREG_SZE) printf("SZE ");
                if (reg_bits & RREG_HLT) printf("HLT ");
            } else { // I/O instruction (I=1)
                printf("IO : ");
                Word io_bits = instruction & 0x0FFF;
                if (io_bits == 0) { printf("(NOP)"); break;} // Special case for 0xF000?
                if (io_bits & IO_DEF_INP) printf("INP ");
                if (io_bits & IO_DEF_OUT) printf("OUT ");
                if (io_bits & IO_DEF_SKI) printf("SKI ");
                if (io_bits & IO_DEF_SKO) printf("SKO ");
                if (io_bits & IO_DEF_ION) printf("ION ");
                if (io_bits & IO_DEF_IOF) printf("IOF ");
            }
            break;
        default: printf("UNK 0x%03X", operand); break; // Unknown opcode
    }
    // Add comments based on address or known program structure if desired
    // e.g., if (addr == 0x000) printf(" ; Start of program");
    printf("\n");
}


// Convert BusSource enum to string for debugging
const char* bus_source_to_string(BusSource src) {
    switch (src) {
        case BUS_AR: return "AR";
        case BUS_PC: return "PC";
        case BUS_DR: return "DR";
        case BUS_AC: return "AC";
        case BUS_IR: return "IR";
        case BUS_TR: return "TR";
        case BUS_MEM: return "MEM(DR)"; // Memory value comes via DR
        case BUS_NONE: return "NONE";
        default: return "??";
    }
}

// --- Main Execution Logic ---

// Run the computer continuously until HLT or error
void run_computer(BasicComputer *bc) {
    if (bc->running) {
        printf("Computer is already running.\n");
        return;
    }
     if (!bc->reg.S) {
        printf("Computer is Halted (S=0). Use 'reset' or manually set S=1 (not implemented) to run.\n");
        return;
    }
    printf("Running computer...\n");
    bc->running = true;
    // bc->reg.S = true; // Ensure S=1 to start - S is already true if not halted

    // Reset SC for a clean run start if it wasn't already T0
    if (bc->reg.SC != 0) {
        if (bc->debug_mode) printf("DEBUG: Resetting SC to T0 for run command.\n");
        bc->reg.SC = 0;
    }


    while (bc->running) {
        bc->running = execute_clock_cycle(bc);
        // Add potential checks here: max cycles, breakpoints, etc.
        if (bc->cycle_count > 1000000) { // Safety break
             printf("\nWarning: Exceeded maximum cycle count (1,000,000). Halting simulation.\n");
             bc->running = false;
             bc->reg.S = false;
        }

        // Example: Simulate an interrupt request periodically
        // if (bc->cycle_count > 10 && bc->cycle_count % 50 == 0 && !bc->reg.R && bc->reg.IEN) {
        //     if (bc->debug_mode) printf("INFO: Simulating Interrupt Request (R=1)\n");
        //     bc->reg.R = true;
        // }
    }
    printf("Computer has stopped. Total cycles: %llu\n", bc->cycle_count);
}

// Simple interactive command loop
void run_interactive(BasicComputer *bc) {
    char command_line[100];
    char command[20];
    Address addr1, addr2;

    printf("\nEntering interactive mode. Type 'help' for commands.\n");

    while (true) {
        printf("> ");
        if (fgets(command_line, sizeof(command_line), stdin) == NULL) {
            printf("\nEOF detected. Exiting.\n");
            break; // Exit on EOF or read error
        }

        // Parse the command
        int items = sscanf(command_line, "%19s %x %x", command, &addr1, &addr2);

        if (items <= 0) continue; // Empty line

        // --- Command Processing ---
        if (strcmp(command, "help") == 0) {
            printf("Commands:\n");
            printf("  run (r)          : Run until HLT or interrupt/error\n");
            printf("  step (s) [n]   : Execute N clock cycles (default 1)\n");
            printf("  regs (reg)     : Dump registers\n");
            printf("  mem <start> [end]: Dump memory from start address (hex) to end (hex, optional)\n");
            printf("  dis <start> [n]: Disassemble N instructions from start address (hex, default 10)\n");
            printf("  reset          : Reset CPU state (memory NOT cleared)\n");
            printf("  load <file>    : Load program from hex file (clears memory first!)\n");
            printf("  debug          : Toggle debug trace mode\n");
            printf("  input <char>   : Simulate typing a character for INP (sets FGI=1)\n");
            printf("  interrupt      : Manually trigger an interrupt request (R=1)\n");
            printf("  quit (q)       : Exit simulator\n");
        }
        else if (strcmp(command, "run") == 0 || strcmp(command, "r") == 0) {
            run_computer(bc);
        }
        else if (strcmp(command, "step") == 0 || strcmp(command, "s") == 0) {
            if (!bc->reg.S) {
                printf("Computer is Halted (S=0). Cannot step.\n");
                continue;
            }
            int steps = 1;
            if (items > 1) {
                 // Try parsing addr1 as decimal steps
                if (sscanf(command_line, "%*s %d", &steps) != 1 || steps <= 0) {
                    printf("Invalid number of steps. Defaulting to 1.\n");
                    steps = 1;
                }
            }
            printf("Executing %d cycle(s)...\n", steps);
            bc->running = true; // Ensure simulation can start/continue

            for (int i = 0; i < steps && bc->running; ++i) {
                 if (!execute_clock_cycle(bc)) {
                    printf("Computer halted during step execution.\n");
                    break; // Stop stepping if HLT occurred
                 }
            }
             // Show state after stepping, only if not halted during the step itself
             if (bc->reg.S) {
                printf("\nState After Step(s):\n");
                dump_registers(bc);
                if (bc->reg.SC == 0) { // If we are back at T0, show next instruction
                     printf("--- Next cycle is instruction fetch (T0) targeting PC=0x%03X ---\n", bc->reg.PC);
                     print_instruction(read_memory(bc, bc->reg.PC), bc->reg.PC);
                } else {
                     printf("--- Currently in T%d of instruction/interrupt cycle ---\n", bc->reg.SC);
                }
            }
        }
        else if (strcmp(command, "regs") == 0 || strcmp(command, "reg") == 0) {
            dump_registers(bc);
        }
        else if (strcmp(command, "mem") == 0) {
            if (items < 2) {
                printf("Usage: mem <start_hex> [end_hex]\n");
            } else {
                Address start_addr = addr1 & ADDRESS_MASK;
                Address end_addr = (items > 2) ? (addr2 & ADDRESS_MASK) : start_addr;
                if (end_addr < start_addr) { // Allow range like FF0 010
                     Address temp = start_addr;
                     start_addr = end_addr;
                     end_addr = temp;
                }
                dump_memory(bc, start_addr, end_addr);
            }
        }
         else if (strcmp(command, "dis") == 0) {
            if (items < 2) {
                printf("Usage: dis <start_hex> [num_instructions]\n");
            } else {
                Address start_addr = addr1 & ADDRESS_MASK;
                int count = (items > 2) ? (int)addr2 : 10; // Use addr2 field for count
                if (count <= 0) count = 10;
                 if (count > 100) count = 100; // Limit disassembly length
                printf("Disassembly from 0x%03X (%d instructions):\n", start_addr, count);
                for (int i=0; i<count; ++i) {
                    Address current_addr = (start_addr + i) & ADDRESS_MASK;
                    if (current_addr < start_addr && i > 0) { // Wrapped around
                        printf("Address wrap-around during disassembly.\n");
                        break;
                    }
                    Word instruction = read_memory(bc, current_addr);
                    print_instruction(instruction, current_addr);
                }
            }
        }
        else if (strcmp(command, "reset") == 0) {
            printf("Resetting CPU state and flags (memory preserved)...\n");
            bool old_debug = bc->debug_mode; // Preserve debug mode
            // Save memory content
            Word temp_memory[MEMORY_SIZE];
            memcpy(temp_memory, bc->memory, sizeof(temp_memory));
            // Initialize registers and flags
            initialize_computer(bc);
            // Restore memory
            memcpy(bc->memory, temp_memory, sizeof(temp_memory));
            bc->debug_mode = old_debug; // Restore debug
            printf("CPU Reset. PC=0x000. S=1.\n");
        }
        else if (strcmp(command, "load") == 0) {
            char filename[80];
            if (sscanf(command_line, "%*s %79s", filename) == 1) {
                printf("Resetting computer and clearing memory before loading...\n");
                initialize_computer(bc); // Clears memory and registers
                 if (load_program_from_file(bc, filename)) {
                     printf("Load complete. Resetting PC to 0.\n");
                     bc->reg.PC = 0; // Ensure PC starts at 0 after load
                     bc->reg.SC = 0; // Ensure SC starts at 0
                     dump_registers(bc); // Show state after load
                 } else {
                     printf("Failed to load program.\n");
                 }
            } else {
                printf("Usage: load <filename>\n");
            }
        }
         else if (strcmp(command, "debug") == 0) {
            bc->debug_mode = !bc->debug_mode;
            printf("Debug trace mode: %s\n", bc->debug_mode ? "ON" : "OFF");
        }
        else if (strcmp(command, "input") == 0) {
             char input_char_str[10];
             // Read the rest of the line to get the character
             if (sscanf(command_line, "%*s %c", &bc->input_char) == 1) {
                 bc->input_waiting = true;
                 bc->reg.FGI = true; // Manually set FGI since we got input
                 printf("Simulated input '%c' (0x%02X) waiting. FGI=1.\n",
                        isprint(bc->input_char) ? bc->input_char : '.', bc->input_char & BYTE_MASK);
             } else {
                 printf("Usage: input <single_character>\n");
             }
        }
        else if (strcmp(command, "interrupt") == 0) {
            if (!bc->reg.R) {
                bc->reg.R = true;
                printf("Manual Interrupt Request set (R=1). Will trigger if IEN=1 and at start of fetch cycle (T0).\n");
            } else {
                printf("Interrupt already pending (R=1).\n");
            }
        }
        else if (strcmp(command, "quit") == 0 || strcmp(command, "q") == 0) {
            printf("Exiting simulator.\n");
            break;
        }
        else {
            printf("Unknown command: '%s'. Type 'help' for list.\n", command);
        }
    }
}


int main(int argc, char *argv[]) {
    BasicComputer bc;
    initialize_computer(&bc);
    const char *program_file = NULL;

    // --- Argument Parsing ---
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0) {
            bc.debug_mode = true;
            printf("Debug mode enabled via command line.\n");
        } else if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--file") == 0) {
            if (i + 1 < argc) {
                program_file = argv[i + 1];
                i++; // Skip filename argument
            } else {
                fprintf(stderr, "Error: -f/--file option requires a filename.\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
             printf("Morris Mano Basic Computer Simulator\n");
             printf("Usage: %s [-d] [-f program.hex]\n", argv[0]);
             printf("  -d, --debug      : Enable detailed execution trace.\n");
             printf("  -f, --file <path>: Load program from hex file on startup.\n");
             printf("  -h, --help       : Show this help message.\n");
             printf("If no file is provided, enters interactive mode with empty memory.\n");
             return 0;
        } else {
             fprintf(stderr, "Warning: Unknown command line argument '%s'. Ignored.\n", argv[i]);
        }
    }

    // --- Load Program if specified ---
    if (program_file) {
        // Initialize fully before loading, ensuring memory is clear
        initialize_computer(&bc);
        // Preserve debug flag if set by argument
        bool current_debug_mode = bc.debug_mode;
        for (int i = 1; i < argc; ++i) {
             if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0) {
                 current_debug_mode = true;
                 break;
             }
        }
        bc.debug_mode = current_debug_mode;

        if (!load_program_from_file(&bc, program_file)) {
            fprintf(stderr, "Failed to load program file '%s'. Exiting.\n", program_file);
            return 1;
        }
        // Ensure PC and SC are reset after loading
        bc.reg.PC = 0;
        bc.reg.SC = 0;
    } else {
        printf("No program file specified. Starting with empty memory.\n");
        // Load a default HLT instruction at address 0 for safety
        bc.memory[0] = 0x7001; // HLT
    }


    // --- Start Interactive Mode ---
    printf("\nWelcome to the Morris Mano Basic Computer Simulator!\n");
    printf("Memory size: %d words (16-bit), Address size: 12 bits.\n", MEMORY_SIZE);
    dump_registers(&bc); // Show initial state

    run_interactive(&bc); // Enter the main command loop

    return 0;
}
