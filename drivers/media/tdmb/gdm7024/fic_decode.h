#ifndef _FIC_DECODE_H
#define _FIC_DECODE_H

#include "gdm_types.h"

void 	gdm_fic_init_db(G_Int32 DAB_no);
G_Int32 gdm_fic_run_decoder(G_Int32 DAB_no,G_Uint8 *fib_buff, G_Int32 fib_buff_size);
void	 gdm_fic_check_ensemble(G_Int32 DAB_no);
#endif ///< _FIC_DECODE_H
