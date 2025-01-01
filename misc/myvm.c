#include "myvm.h"

void __mov(VM *vm, Opcode opcode, Args a1, Args a2) {
    vm->c.r.ax = (Reg)((a2 << 8) | a1); // Combine a1 (low byte) and a2 (high byte) into 16-bit ax
    return;
}

void zero(int8 *str, int16 size) {
    int8 *p;
    int16 n;

    for (n = 0, p = str; n < size; n++, p++)
        *p = 0;

    return;
}

void execinstr(VM* vm, Program *p) {
    Args a1 = 0, a2 = 0;
    int16 size = map(*p);

    switch (size) {
        case 1:
            break;
        case 2:
            a1 = *(p + 1);
            break;
        case 3:
            a1 = *(p + 1);
            a2 = *(p + 2); // Fixed: Fetch second byte correctly
            break;
        default:
            segfault(vm);
            break;
    }

    switch (*p) {
        case mov: 
            __mov(vm, (Opcode)*p, a1, a2);
            break;
        case nop:
            break;
        case hlt:
            error(vm, SysHlt);
            break;
    }

    return;
}

void execute(VM *vm) {
    if (!vm || !vm->m) {
        fprintf(stderr, "Failed to initialize VM memory\n");
        exit(ErrMem);
    }

    int32 brkaddr = (int32)vm->m + vm->b;
    Program *pp = (Program *)vm->m;
    int16 size = 0;

    while (true) {
        if ((int32)pp >= brkaddr) { // Fixed: >= for exact bounds checking
            segfault(vm);
        }
        size = map(*pp);
        if (*pp == hlt) { // Fixed: Check hlt before executing to ensure clean termination
            execinstr(vm, pp); // Handles exit
            break;
        }
        execinstr(vm, pp);
        vm->c.r.ip += size;
        pp += size;
    }

    return;
}

void __execute(VM *vm) { // Kept for reference, but not used in main
    int32 brkaddr = (int32)vm->m + vm->b;
    Program *pp = (Program *)vm->m;
    int16 size = 0;

    do {
        vm->c.r.ip += size;
        pp += size;

        if ((int32)pp > brkaddr)
            segfault(vm);
        size = map(*pp);
        execinstr(vm, pp);
    } while (*pp != (Opcode)hlt);

    return;
}

void error(VM* vm, Errorcode e) {
    int8 exitcode = -1;

    switch (e) {
        case ErrSegv:
            fprintf(stderr, "%s\n", "VM Segmentation fault");
            break;
        case SysHlt:
            fprintf(stderr, "%s\n", "System halted");
            exitcode = 0;
            printf("ax = %.04hx\n", (int)vm->c.r.ax);
            break;
        default:
            break;
    }
    if (vm)
        free(vm);

    exit((int)exitcode);
}

void printhex(int8 *str, int16 size, int8 delimiter) {
    int8 *p;
    int16 n;

    for (p = str, n = size; n; n--, p++) {
        printf("%.02x", *p);
        if (delimiter)
            printf("%c", delimiter);
        fflush(stdout);
    }
    printf("\n");
    return;
}

void copy(int8 *dst, int8 *src, int16 size) {
    int8 *d, *s;
    int16 n;

    for (n = size, d = dst, s = src; n; n--, d++, s++)
        *d = *s;

    return;
}

int8 map(Opcode o) {
    int8 n, ret = 0;
    IM *p;

    for (n = IMs, p = instrmap; n; n--, p++)
        if (p->o == o) {
            ret = p->s;
            break;
        }
    
    return ret;
}

VM *virtualmachine() {
    VM *p;
    int16 size = (int16)sizeof(struct s_vm);

    p = (VM *)malloc(size);
    if (!p) {
        errno = ErrMem;
        return (VM *)0;
    }
    zero((int8 *)p, size);
    zero(p->m, MEMORY_SIZE);
    return p;
}

Program *exampleprogram(VM *vm) {
    Program *p = vm->m;

    // mov ax, 0x05 (3 bytes)
    p[0] = mov;
    p[1] = 0x05; // Low byte
    p[2] = 0x00; // High byte
    // nop (1 byte)
    p[3] = nop;
    // hlt (1 byte)
    p[4] = hlt;

    vm->b = 5; // Fixed: Total program size (3 + 1 + 1)
    vm->c.r.ip = (Reg)vm->m;
    vm->c.r.sp = (Reg)-1;

    return (Program *)vm->m;
}

int main(int argc, char *argv[]) {
    Program *prog;
    VM *vm;

    vm = virtualmachine();
    printf("vm   = %p (sz: %d)\n", vm, sizeof(struct s_vm));

    prog = exampleprogram(vm);
    printf("prog = %p\n", prog);

    printhex((int8 *)prog, (map(mov) + map(nop) + map(hlt)), ' ');
    execute(vm);

    return 0;
}

#pragma GCC diagnostic pop
