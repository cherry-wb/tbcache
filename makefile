CC=gcc -g
CFLAGS=-I.

tb-cache: main.o trans_all.o sys.o cpu-exec.o
  $(CC) main.o trans_all.o sys.o cpu-exec.o -o tb-cache

main.o: main.c 
	$(CC) -c main.c	

trans_all.o: trans_all.c
	$(CC) -c trans_all.c

# restriction from 'qemu/tests/tcg/cris/sys.c'
sys.o: sys.c
	$(CC) -c sys.c
	
# restriction from cpu-exec.c
cpu-exec.o: cpu-exec.c
	$(CC) -c cpu-exec.c

clean:
	rm -rf *o tb-cache
