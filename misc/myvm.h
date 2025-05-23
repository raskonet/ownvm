
#define _GNU_SOURCE
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
//#include <birchutils.h>
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic push


#define NoErr       0x00    /* 00 00 */
#define SysHlt      0x01    /* 00 01 */
#define ErrMem      0x02    /* 00 10 */
#define ErrSegv     0x04    /* 01 00 */
#define NoArgs      {0x00, 0x00}


typedef unsigned char int8;
typedef unsigned short int int16;
typedef unsigned int int32;
typedef unsigned long long int int64;

#define $1 (int8 *)
#define $2 (int16)
#define $4 (int32)
#define $8 (int64)
#define $c (char *)
#define $i (int)





#define $ax ->c.r.ax
#define $bx ->c.r.bx
#define $cx ->c.r.cx
#define $dx ->c.r.dx
#define $sp ->c.r.sp
#define $ip ->c.r.ip

#define segfault(x)     error((x), ErrSegv)

/*

    16 bit
        AX
        BX
        CX
        DX
        SP
        IP
    65 KB memory
    (Serial COM port)
    (Floppy drive)

*/

typedef unsigned short int Reg;
struct s_registers {
    Reg ax;
    Reg bx;
    Reg cx;
    Reg dx;
    Reg sp;
    Reg ip;
};
typedef struct s_registers Registers;

struct s_cpu {
    Registers r;
};
typedef struct s_cpu CPU;

/*

 mov ax,0x05 // (0x01 OR 0x02)
             // 0000 0011 = mov
             // 0000 0000
             // 0000 0101 = 0x05


*/

enum e_opcode {
    mov = 0x01,
    nop = 0x02,
    hlt = 0x03
};
typedef enum e_opcode Opcode;

struct s_instrmap {
    Opcode o;
    int8 s;
};
typedef struct s_instrmap IM;

typedef int16 Args;
struct s_instruction {
    Opcode o;
    Args a[]; /* 0-2 bytes */
};
typedef struct s_instruction Instruction;
#define MEMORY_SIZE 65536
typedef unsigned char Errorcode;
typedef int8 Memory[MEMORY_SIZE];
typedef int8 Program;

struct s_vm {
    CPU c;
    Memory m;
    int16 b; /* break line */
};
typedef struct s_vm VM;
typedef Memory *Stack;

static IM instrmap[] = {
    { mov, 0x03 },
    { nop, 0x01 },
    { hlt, 0x01 }
};
#define IMs (sizeof(instrmap) / sizeof(struct s_instrmap))


void __mov(VM*,Opcode,Args,Args);
void zero(int8*,int16);
void error(VM*,Errorcode);
 Program *exampleprogram(VM*);
void copy(int8*,int8*,int16);
void execinstr(VM*,Program*);
void execute(VM*);
void printhex(int8*,int16,int8);
int8 map(Opcode);
VM *virtualmachine(void);
int main(int,char**);
