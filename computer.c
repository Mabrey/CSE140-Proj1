/*
    Go to office hours to ask about how to pass values from execute to the register write function. 
    val is returned in exe but overwritten in the mem return, which invalidates the data in val. 
    val is then passed into regwrite, but we dont know how we are supposed to use it.
    Do we access registers earlier?

*/
#include <stdio.h>
#include <stdlib.h>
//#include <winsock2.h>
#include <netinet/in.h>
#include "computer.h"
#undef mips			/* gcc already has a def for mips */

unsigned int endianSwap(unsigned int);

void PrintInfo (int changedReg, int changedMem);
unsigned int Fetch (int);
void Decode (unsigned int, DecodedInstr*, RegVals*);
int Execute (DecodedInstr*, RegVals*);
int ExecuteRFormat(DecodedInstr*, RegVals*);
int ExecuteIFormat(DecodedInstr*, RegVals*); 
int ExecuteJFormat(DecodedInstr*, RegVals*); 
int Mem(DecodedInstr*, int, int *);
void RegWrite(DecodedInstr*, int, int *);
void UpdatePC(DecodedInstr*, int);
void PrintInstruction (DecodedInstr*);
void PrintRFormat(DecodedInstr*);
void PrintIFormat(DecodedInstr*);
void PrintJFormat(DecodedInstr*);

/*Globally accessible Computer variable*/
Computer mips;
RegVals rVals;

/*
 *  Return an initialized computer with the stack pointer set to the
 *  address of the end of data memory, the remaining registers initialized
 *  to zero, and the instructions read from the given file.
 *  The other arguments govern how the program interacts with the user.
 */
void InitComputer (FILE* filein, int printingRegisters, int printingMemory,
  int debugging, int interactive) {
    int k;
    unsigned int instr;

    /* Initialize registers and memory */

    for (k=0; k<32; k++) {
        mips.registers[k] = 0;
    }
    
    /* stack pointer - Initialize to highest address of data segment */
    mips.registers[29] = 0x00400000 + (MAXNUMINSTRS+MAXNUMDATA)*4;

    for (k=0; k<MAXNUMINSTRS+MAXNUMDATA; k++) {
        mips.memory[k] = 0;
    }

    k = 0;
    while (fread(&instr, 4, 1, filein)) {
	/*swap to big endian, convert to host byte order. Ignore this.*/
        mips.memory[k] = ntohl(endianSwap(instr));
        k++;
        if (k>MAXNUMINSTRS) {
            fprintf (stderr, "Program too big.\n");
            exit (1);
        }
    }

    mips.printingRegisters = printingRegisters;
    mips.printingMemory = printingMemory;
    mips.interactive = interactive;
    mips.debugging = debugging;
}

unsigned int endianSwap(unsigned int i) {
    return (i>>24)|(i>>8&0x0000ff00)|(i<<8&0x00ff0000)|(i<<24);
}

/*
 *  Run the simulation.
 */
void Simulate () {
    char s[40];  /* used for handling interactive input */
    unsigned int instr;
    int changedReg=-1, changedMem=-1, val;
    DecodedInstr d;
    
    /* Initialize the PC to the start of the code section */
    mips.pc = 0x00400000;
    while (1) {
        if (mips.interactive) {
            printf ("> ");
            fgets (s,sizeof(s),stdin);
            if (s[0] == 'q') {
                return;
            }
        }

        /* Fetch instr at mips.pc, returning it in instr */
        instr = Fetch (mips.pc);

        printf ("Executing instruction at %8.8x: %8.8x\n", mips.pc, instr);

        /* 
	 * Decode instr, putting decoded instr in d
	 * Note that we reuse the d struct for each instruction.
	 */
        Decode (instr, &d, &rVals);

        /*Print decoded instruction*/
        PrintInstruction(&d);

        /* 
	 * Perform computation needed to execute d, returning computed value 
	 * in val 
	 */
        val = Execute(&d, &rVals);

	UpdatePC(&d,val);

        /* 
	 * Perform memory load or store. Place the
	 * address of any updated memory in *changedMem, 
	 * otherwise put -1 in *changedMem. 
	 * Return any memory value that is read, otherwise return -1.
         */
        val = Mem(&d, val, &changedMem);

        /* 
	 * Write back to register. If the instruction modified a register--
	 * (including jal, which modifies $ra) --
         * put the index of the modified register in *changedReg,
         * otherwise put -1 in *changedReg.
         */
        RegWrite(&d, val, &changedReg);

        PrintInfo (changedReg, changedMem);
    }
}

/*
 *  Print relevant information about the state of the computer.
 *  changedReg is the index of the register changed by the instruction
 *  being simulated, otherwise -1.
 *  changedMem is the address of the memory location changed by the
 *  simulated instruction, otherwise -1.
 *  Previously initialized flags indicate whether to print all the
 *  registers or just the one that changed, and whether to print
 *  all the nonzero memory or just the memory location that changed.
 */
void PrintInfo(int changedReg, int changedMem) {
	int k, addr;
	printf("New pc = %8.8x\n", mips.pc);
	if (!mips.printingRegisters && changedReg == -1)
	{
		printf("No register was updated possibly.\n");
		printf("Reg print over.\n");
	}
	else if (!mips.printingRegisters)
	{
		printf("Updated r%2.2d to %8.8x\n",
			changedReg, mips.registers[changedReg]);
	}
	else
	{
		for (k = 0; k<32; k++) {
			printf("r%2.2d: %8.8x  ", k, mips.registers[k]);
			if ((k + 1) % 4 == 0) {
				printf("\n");
			}
		}
	}

	printf("check mem update.\n");
	if (!mips.printingMemory && changedMem == -1) {
		printf("No memory location was updated.\n");
	}
	else if (!mips.printingMemory) {
		printf("Updated memory at address %8.8x to %8.8x\n",
			changedMem, Fetch(changedMem));
	}
	else {
		printf("Nonzero memory\n");
		printf("ADDR      CONTENTS\n");
		for (addr = 0x00400000 + 4 * MAXNUMINSTRS;
			addr < 0x00400000 + 4 * (MAXNUMINSTRS + MAXNUMDATA);
			addr = addr + 4) {
			if (Fetch(addr) != 0) {
				printf("%8.8x  %8.8x\n", addr, Fetch(addr));
			}
		}
	}
}

/*
 *  Return the contents of memory at the given address. Simulates
 *  instruction fetch. 
 */
unsigned int Fetch ( int addr) {
    return mips.memory[(addr-0x00400000)/4];
}

/* Decode instr, returning decoded instruction. */
void Decode ( unsigned int instr, DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
    d -> op = instr >> 26;
    if (d -> op == 0)    //Format for instruction is R
    {
        d -> type = R;

        unsigned int rs = instr << 6;    //sets rs value in decode instr and rVals
        rs = rs >> 27;
        d -> regs.r.rs = rs;
        rVals -> R_rs = mips.registers[rs];

        unsigned int rt = instr << 11;   //sets rt value in decode instr and rVals
        rt = rt >> 27;
        d -> regs.r.rt = rt;
        rVals -> R_rt = mips.registers[rt];
        
        unsigned int rd = instr << 16;   //sets rd value in decode instr and rVals
        rd = rd >> 27;
        d -> regs.r.rd = rd;
        rVals -> R_rd = mips.registers[rd];

        unsigned int shamt = instr << 21;    //sets shamt value in decode instr
        shamt = shamt >> 27;
        d -> regs.r.shamt = shamt;

        unsigned int funct = instr << 26;    //sets funct value in decode instr
        funct = funct >> 26;
        d -> regs.r.funct = funct;
    }
        
    else 
    {
        if (d -> op == 2 || d -> op == 3) //This is a j format instruction
        {
            d -> type = J;

            unsigned int target = instr << 6;        //sets the target
            target = target >> 4;
            d -> regs.j.target = target;
        }

        else                    //this is I format
        {
            d -> type = I;

            unsigned int rs = instr << 6;    //sets rs value in decode instr and rVals
            rs = rs >> 27;
            d -> regs.i.rs = rs;
            rVals -> R_rs = mips.registers[rs];

            unsigned int rt = instr << 11;   //sets rt value in decode instr and rVals
            rt = rt >> 27;
            d -> regs.i.rt = rt;
            rVals -> R_rt = mips.registers[rt];

            unsigned int unsignedImmediate;
            int signedImmediate;

            switch (d -> op){

                case 9:
                case 35:
                case 43:
                    signedImmediate = instr << 16;        //sets immediate for sign extended immediates
                    signedImmediate = signedImmediate >> 16;
                    d -> regs.i.addr_or_immed = signedImmediate;
					printf("Immediate: 0x%08x\n", signedImmediate);
                    break;

                case 15:
                    signedImmediate = instr << 16;        //sets immediate for lui
					signedImmediate = signedImmediate >> 16;
                    d -> regs.i.addr_or_immed = signedImmediate;
                    break;

                case 12:
                case 13:
                    unsignedImmediate = instr << 16;        //sets immediate for zero extended
                    unsignedImmediate = unsignedImmediate >> 16;
                    d -> regs.i.addr_or_immed = unsignedImmediate;
                    break;

                case 4:
                case 5:
                    unsignedImmediate = instr << 16;        //sets addr for beq and bne 
                    unsignedImmediate = unsignedImmediate >> 16;
                    unsignedImmediate = (unsignedImmediate * 4) + 4 + mips.pc;
                    d -> regs.i.addr_or_immed = unsignedImmediate;
                    break;
            }

        }
    }

}


void PrintInstruction (DecodedInstr* d) {
    switch (d->op) {
    case 0 :
        PrintRFormat(d);
        break;
	case 9:        //FALLTHROUGH
	case 12:    //FALLTRHOUGH
	case 13:    //FALLTRHOUGH
	case 4:        //FALLTRHOUGH
	case 5:        //FALLTRHOUGH
	case 35:    //FALLTRHOUGH
	case 43:
	case 15:
        PrintIFormat(d);
        break;
    case 2:
    case 3:
        PrintJFormat(d);
        break;
    }
}

//handles printing for R format instructions
void PrintRFormat(DecodedInstr* d) {

    int rs = d->regs.r.rs;
    int rt = d->regs.r.rt;
    int rd = d->regs.r.rd;

    switch(d->regs.r.funct) {
    case 33 :
        printf("addu\t$%d, $%d, $%d\n", rd, rs, rt);
        break;
    case 35 :
        printf("subu\t$%d, $%d, $%d\n", rd, rs, rt);
        break;
    case 0:
        printf("sll\t$%d, $%d, $%d\n", rd, rs, rt);
        break;
    case 2:
        printf("srl\t$%d, $%d, $%d\n", rd, rs, rt);
        break;
    case 36:
        printf("and\t$%d, $%d, $%d\n", rd, rs, rt);
        break;
    case 37:
        printf("or\t$%d, $%d, $%d\n", rd, rs, rt);
        break;
    case 42:
        printf("slt\t$%d, $%d, $%d\n", rd, rs, rt);
        break;
    case 8:
        printf("jr\t$%d\n", rs);
        break;
    }
}

//handles printing for I format instructions
void PrintIFormat(DecodedInstr* d) {

    int rs = d->regs.i.rs;
    int rt = d->regs.i.rt;
    int imm = d->regs.i.addr_or_immed;

    switch (d->op) {
    case 9:
        printf("addiu\t$%d, $%d, %d\n", rt, rs, imm);
        break;
    case 12:
        printf("andi\t$%d, $%d, 0x%x\n", rt, rs, imm);
        break;
    case 13:
        printf("ori\t$%d, $%d, 0x%x\n", rt, rs, imm);
        break;
    case 4:
        printf("beq\t$%d, $%d, 0x%08x\n", rs, rt, imm);
        break;
    case 5:
        printf("bne\t$%d, $%d, 0x%08x\n", rs, rt, imm);
        break;
    case 15:
        printf("lui\t$%d, 0x$%x\n", rs, imm);
        break;
    case 35:
        printf("lw\t$%d, %d($%d)\n", rt, imm, rs);
        break;
    case 43:
        printf("sw\t$%d, %d($%d)\n", rt, imm, rs);
        break;
    }
}

void PrintJFormat(DecodedInstr* d) {

    int op = d->op;
    int target = d->regs.j.target;

    switch (op) {
    case 2:
        printf("j\t0x%08x\n", target);
        break;
    case 3: 
        printf("jal\t0x%08x\n", target);
        break;
    }
}




/* Perform computation needed to execute d, returning computed value */
int Execute ( DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
	
	//MAY NEED TO CHANGE FROM UNSIGNED
	int op = d->op;
	int val = 0;

	switch (op) {
	////////////// R-FORMAT INSTRUCTIONS ///////////////
	case 0:
		val = ExecuteRFormat(d, rVals);
		break;
	////////////// I-FORMAT INSTRUCTIONS ///////////////
	case 9:			//addiu
	case 12:		//andi
	case 13:		//ori
	case 4:			//beq
	case 5:			//bne
	case 15:		//lui
	case 35:		//lw
	case 43:		//sw
		val = ExecuteIFormat(d, rVals);
		break;
	////////////// J-FORMAT INSTRUCTIONS ///////////////
	case 2:			//j
	case 3:			//jal
		val = ExecuteJFormat(d, rVals);
		break;
	}
    //printf("val in execute = %08x\n", val);
	return val;
}

int ExecuteRFormat(DecodedInstr* d, RegVals* rVals) {
	
	int funct = d->regs.r.funct;
	unsigned int rd = rVals->R_rd;
	unsigned int rt = rVals->R_rt;
	unsigned int rs = rVals->R_rs;
	
	switch (funct) {
	case 33:	//addu

		rd = rs + rt;
		break;
	case 35:	//subu

		rd = rs - rt;
		break;
	case 0:		//sll

		rd = rt << d->regs.r.shamt;
		break;
	case 2:		//srl

		rd = rt >> d->regs.r.shamt;
		break;
	case 36:	//and

		rd = rs & rt;
		break;
	case 37:	//or

		rd = rs | rt;
		break;
	case 42:	//slt

		if (rs < rt) {
			rd = 1;
		}
		else {
			rd = 0;
		}
		break;

    case 8:

        return rs;
        break;
	}

	//update register
	rVals->R_rd = rd;
	return rd;
}

int ExecuteIFormat(DecodedInstr* d, RegVals* rVals) {

    int op = d->op;
	int imm = d->regs.i.addr_or_immed;
	int val = 0;
	//unsigned int rt = rVals->R_rt;
	//unsigned int rs = rVals->R_rs;

	switch (op) {
	case 9:			//addiu
		rVals->R_rt = rVals->R_rs + imm;
		val = rVals->R_rt;
		break;
	case 12:		//andi
		rVals->R_rt = rVals->R_rs & imm;
		val = rVals->R_rt;
		break;
	case 13:		//ori
		rVals->R_rt = rVals->R_rs | imm;
		val = rVals->R_rt;
		break;
	case 4:			//beq
		val = rVals->R_rs - rVals->R_rt;
		break;
	case 5:			//bne
		val = rVals->R_rs - rVals->R_rt;
		break;
	case 15:		//lui
		rVals->R_rt = imm << 16;
		val = rVals->R_rt;
		break;
	case 35:		//lw
		rVals->R_rt = rVals->R_rs + imm;
		val = rVals->R_rt;
		break;
	case 43:		//sw
		val = rVals->R_rs + imm;
		printf("Imm: 0x%08x\n", imm);
		break;
	}

	return val;

}

int ExecuteJFormat(DecodedInstr* d, RegVals* rVals) {
	if (d -> op == 3)   //jal
	    return (mips.pc + 4);
    
    else return 0;   //j
}

/* 
 * Update the program counter based on the current instruction. For
 * instructions other than branches and jumps, for example, the PC
 * increments by 4 (which we have provided).
 */
void UpdatePC ( DecodedInstr* d, int val) {
    //printf("Val = %d\n", val);
    switch (d -> op) {

    case 4:        //beq
        if (val == 0)
            mips.pc = d -> regs.i.addr_or_immed;
        else 
            mips.pc += 4;
        break;
    case 5:        //bne
        //printf("Val = %d\n", val);
        if (val != 0)
            mips.pc = d -> regs.i.addr_or_immed;
        else 
            mips.pc += 4;
        break;
    case 2:        //j
    case 3:        //jal
        mips.pc = d -> regs.j.target;
        break;

    case 0:        //This is R format
        if (d -> regs.r.funct != 8)
            mips.pc += 4;

        else {      //this is jr
            mips.pc = val;
        }
        break;

    default:
        mips.pc += 4;
   
    }
}

/*
 * Perform memory load or store. Place the address of any updated memory 
 * in *changedMem, otherwise put -1 in *changedMem. Return any memory value 
 * that is read, otherwise return -1. 
 *
 * Remember that we're mapping MIPS addresses to indices in the mips.memory 
 * array. mips.memory[0] corresponds with address 0x00400000, mips.memory[1] 
 * with address 0x00400004, and so forth.
 *
 */
int Mem( DecodedInstr* d, int val, int *changedMem) {
    //printf("val in mem = %08x\n", val);
    if (d -> op != 35 && d -> op != 43)
    {
        *changedMem = -1;
        return val;
    }
    else
    {
		int newAddr = 4198400 + mips.registers[d->regs.i.rs] + d->regs.i.addr_or_immed * 4;
		//int newAddr = d->regs.i.rs + (val) * 4;
		printf("New Addr: 0x%08x\n", newAddr);
        if (newAddr % 4 != 0        //not word aligned
            || newAddr < 4198400    //below memory available
            || newAddr > 4210687)   //above memory available
        
        {
			printf("Memory Access Exception at 0x%08x: address 0x%08x\n", mips.pc, newAddr);
            //printf("Memory Access Exception at 0x%08x: address 0x%08x\n", mips.pc, d -> regs.i.addr_or_immed );
            exit(0);
        }

        else
        {
            int op = d->op;
			int imm = (d->regs.i.addr_or_immed) * 4;
			int rs = mips.registers[d->regs.i.rs];
			printf("rs: 0x%08x\n", rs);
            int rt = mips.registers[d->regs.i.rt];
			/*
			int rs = d->regs.i.rs;
			int rt = d->regs.i.rt;
            */
			switch (op) {
            case 35:    //lw
                d -> regs.i.rt = mips.memory[1024 + rs + imm];
                *changedMem = -1;
                return rs + imm;
                break;
            case 43:    //sw
                mips.memory[1024 + rs + imm] = mips.registers[rt];
                *changedMem = 0x00401000 + rs + imm;
                return rs + imm;
                break;
            default:
                return -1;
            }
            return -1;
        }
    }
    
}

/* 
 * Write back to register. If the instruction modified a register--
 * (including jal, which modifies $ra) --
 * put the index of the modified register in *changedReg,
 * otherwise put -1 in *changedReg.
 */
void RegWrite( DecodedInstr* d, int val, int *changedReg) {
    
    //printf("val in regwrite = %08x\n", val);
    if (d->type == I) {            //I Format

        switch (d->op) {
        case 4:        //beq
        case 5:        //bne
        case 43:    //sw
            *changedReg = -1;
            break;

        default:

            mips.registers[d->regs.i.rt] = val;
            *changedReg = d->regs.i.rt;

        }
    }
    else if (d->type == J) {    //J Format

        if (d->op == 2)        //j
            *changedReg =  -1;

        else {                //jal

            *changedReg = 31;
            mips.registers[31] = val;

        }
    }

    else {                        //R Format

        if (d->regs.r.funct == 8)    //jr
            *changedReg = -1;

        else {

            mips.registers[d->regs.r.rd] = val;
            *changedReg = d->regs.r.rd;
        }

    }
}

