
all: mips-ioe5

CFLAGS = -O0 -Wall -ggdb

mips-ioe5.o: mips-ioe5.c
	gcc $(CFLAGS) -c mips-ioe5.c

mips-ioe5: mips-ioe5.o ../elf/elf32.o
	gcc $(CFLAGS) mips-ioe5.o ../mips-arch/mips-disasm.o ../elf/elf32.o -o mips-ioe5

clean:
	rm *.o

