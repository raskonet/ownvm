#include "myvm.h"
void __mov(VM *vm,Opcode opcode,Args a1,Args a2){
	vm $ax=(Reg)a1;

	return;
}

void zero(int8 *str, int16 size) {
    int8 *p;
    int16 n;

    for (n=0, p=str; n<size; n++, p++)
        *p = 0;

    return;
}


void execinstr(VM* vm, Program *p) {
    Args a1, a2;
    int16 size;

    a1=a2 = 0;
    size = map(*p);

    switch (size) {
        case 1:
            break;

        case 2:
            a1 = *(p+1);
            break;

        case 3:
            a1 = *(p+1);
            a2 = *(p+3);

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

    int32 brkaddr;
    Program *pp;
    int16 size;
    Instruction ip;

    size = 0;
    brkaddr = ((int32)vm->m + vm->b);
    pp = (Program *)&vm->m;

    do {
        vm->c.r.ip += size;
        pp += size;

        if ((int32)pp > brkaddr) {
            segfault(vm);
        }

        size = map(*pp);
        execinstr(vm, pp);
    } while (*pp != (Opcode)hlt);

    return;
}

void __execute(VM *vm) {
    int32 brkaddr;
    Program *pp;
    int16 size;
    Instruction ip;

   // assert(vm && *vm->m);
    size = 0;
    brkaddr = ((int32)vm->m + vm->b);
    pp = (Program *)&vm->m;

    /* instr1 arg1 instr2 instr3 */
    /* mov ax,0x05; nop; hlt; */
    /* 0x01 0x00 0x05 */
    /* 0x02 */
    /* 0x03 */

    // pp -> 3
    // brk -> 5
    // brkaddr = 0x01 0x00 0x05
    //           0x02 0x03 X

    do {
        vm $ip += size;
        pp += size;

       if ((int32)pp > brkaddr)
             segfault(vm);
            size = map(*pp);
        execinstr(vm, pp);
    } while (*pp != (Opcode)hlt);

    return;
}


void error(VM* vm, Errorcode e) {
    int8 exitcode;

    exitcode = -1;
    switch(e) {
        case ErrSegv:
            fprintf(stderr, "%s\n", "VM Segmentation fault");
            break;

        case SysHlt:
            fprintf(stderr, "%s\n", "System halted");
            exitcode = 0;
            printf("ax = %.04hx\n", $i vm $ax);

            break;

        default:
            break;
    }
    if (vm)
        free(vm);

    exit($i exitcode);
}


void printhex(int8 *str,int16 size,int8 delimter){
	int8 *p;
	int16 n;

	for(p=str,n=size;n;n--,p++){
		printf("%.02x",*p);
		if(delimter)
			printf("%c",delimter);
		fflush(stdout);
	}
	printf("\n");
	return;
}
void copy(int8 *dst,int8 *src,int16 size){
	int8 *d,*s;
	int16 n;
	for(n=size,d=dst,src=s;n;n--,d++,s++)
		*d=*s;

	return;

}

int8 map(Opcode o) {
    int8 n, ret;
    IM *p;

    ret = 0;
    for (n=IMs, p=instrmap; n; n--, p++)
        if (p->o == o) {
            ret = p->s;
            break;
        }
    
    return ret;
}


VM *virtualmachine() {
    VM *p;
    int16 size;

    size = $2 sizeof(struct s_vm);
    p = (VM *)malloc(size);
    if (!p) {
        errno = ErrMem;
        return (VM *)0;
    }
    zero($1 p, size);
	zero(p->m,MEMORY_SIZE);
     return p;
}



Program *exampleprogram(VM *vm) {
    Program *p;
    Instruction *i1, *i2, *i3;
    Args a1;
    int16 s1, s2, sa1;

    a1 = 0;
    s1 = map(mov);
    s2 = map(nop);

    i1 = (Instruction *)malloc($i s1);
    i2 = (Instruction *)malloc( s2);
    i3 = (Instruction *)malloc( s2);
    assert(i1 && i2);
    zero($1 i1, s1);
    zero($1 i2, s2);
    zero($1 i3, s2);

    i1->o = mov;
    sa1 = (s1-1);
        a1     = 0x0005;
        // 0000 0001 mov eax
        // 0000 0000
        // 0000 0005  mov eax,0x05

    p = vm->m;
    copy($1 p, $1 i1, 1);
    p++;

    if (a1) {
        copy($1 p, $1 &a1, sa1);
        p += sa1;
    }

    i2->o = nop;
    copy($1 p, $1 i2, 1);

    p++;
    i3->o = hlt;
    copy($1 p, $1 i3, 1);

    vm->b = (s1+sa1+s2+s2);
    vm $ip = (Reg)vm->m;
    vm $sp = (Reg)-1;

    free(i1);
    free(i2);

    return (Program *)&vm->m;
}

int main(int argc, char *argv[]) {
    Program *prog;
    VM *vm;

    vm = virtualmachine();
    printf("vm   = %p (sz: %d)\n", vm, sizeof(struct s_vm));

    prog = exampleprogram(vm);
    printf("prog = %p\n", prog);

   printhex($1 prog, (map(mov)+map(nop)+map(hlt)), ' ');
    execute(vm);


    return 0;
}

 
#pragma GCC diagnostic pop
