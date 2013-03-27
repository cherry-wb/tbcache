#ifndef _MAIN_H_
#define _MAIN_H_


#include <unistd.h>
#include <sys/mman.h>

#define TB_EXIT_MASK 3
#define TB_EXIT_IDX0 0
#define TB_EXIT_IDX1 1
#define TB_EXIT_ICOUNT_EXPIRED 2
#define TB_EXIT_REQUESTED 3

/*---------added defines---------*/
#define LINE_MAX 10
#define MAX_PC 5000


uint16_t src[MAX_PC][2];

static inline void map_exec(void *addr, long size)
{
    unsigned long start, end, page_size;

    page_size = getpagesize();
    start = (unsigned long)addr;
    start &= ~(page_size - 1);

    end = (unsigned long)addr + size;
    end += page_size - 1;
    end &= ~(page_size - 1);

    mprotect((void *)start, end - start,
             PROT_READ | PROT_WRITE | PROT_EXEC);
}

static inline int des_adr(char* line)
{
	return((int)((line[2]-'0')*1000 + 
					 (line[3]-'0')*100  +
					 (line[4]-'0')*10  +
					 (line[5]-'0')) );
}
	
	

static unsigned long pc = 0;	

	
	
#endif /* _MAIN_H_ */

