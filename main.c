   /***************************************************************
	*	Optimization of Qemu translation cache policy					*
	*																					*
	*	FERJANI SABER - Tima Lab - SLS Team									*
	*																					*
	*	March 2013																	*
	*																					*
	****************************************************************/


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "trans_all.h"



tcg_target_ulong next_tb;



int main(int argc, char **argv)
{
	FILE *f;
	TranslationBlock *tb;																//see: tcg_exec_init() --max = 524272
   static CPUArchState env;															// hw to init env !?
   char line[LINE_MAX];

	if (argc<2) 
	{
		printf("Should pass source file as argument!\n");
		return -1;
	}
	



	/* Clear screen at startup */
	if (system( "clear" )) system( "cls" );
	printf("\n *** Qemu Translation Cache Model *** TIMA LAB - March 2013 ***\n\n");
		
   f=fopen(&argv[1][0],"r");
   if(f == NULL)
    {
        printf("src file not found! Exiting...\n");
        return -1;
    }

   while(fgets(line,LINE_MAX,f)!= NULL)
   {
     if (line[0]=='j') 
     {
     	src[pc][1]=des_adr(line);
     	}
     src[pc++][0]=line[0]; 
   }
   src[pc][0]=0; 
   fclose(f);
   
	 
/* unsigned long Total_Nb_Inst=pc;
  	for(pc=0;pc<Total_Nb_Inst;pc++) 
  		printf("%3d : %c  : %d \n",pc,src[pc][0],src[pc][1]); */

   
   pc=0;
      
 	code_gen_alloc(0);																	// 0 mean default size	-- this init tcg_ctx

   env.tb_jmp_cache[tb_jmp_cache_hash_func(pc)] = tb;  						//added from tb_find_slow()
   
   tcg_ctx.tb_ctx.tb_invalidated_flag = 0;					

	printf("tcg_ctx.code_gen_ptr=%ld tcg_ctx.code_gen_buffer=%d \n",(unsigned long)(tcg_ctx.code_gen_ptr), (unsigned long) (tcg_ctx.code_gen_buffer));
   
   tcg_ctx.code_gen_ptr=tcg_ctx.code_gen_buffer;				// init code_gen_ptr -- see: tcg_exec_init()
	tcg_ctx.code_gen_max_blocks  = 100;
   for(;;)
   	{
         spin_lock(&tcg_ctx.tb_ctx.tb_lock);									// why ?
         pc = tb_find_fast(env);
//         pc= (unsigned long)(tb->pc);
         /* Note: we do it here to avoid a gcc bug on Mac OS X when doing it in tb_find_slow */
//       if (tcg_ctx.tb_ctx.tb_invalidated_flag) {
         /* as some TB could have been invalidated because of memory exceptions while generating the code, we must recompute the hash index here */
//             next_tb = 0;
//             tcg_ctx.tb_ctx.tb_invalidated_flag = 0;
//              }

         /* see if we can patch the calling TB. When the TB spans two pages, we cannot safely do a direct jump. */
//       if (next_tb != 0 && tb->page_addr[1] == -1) {
//             tb_add_jump((TranslationBlock *)(next_tb & ~TB_EXIT_MASK),
//                             next_tb & TB_EXIT_MASK, tb);
//                }

         spin_unlock(&tcg_ctx.tb_ctx.tb_lock);

//       getchar();                 											//step by step

  		}
}
