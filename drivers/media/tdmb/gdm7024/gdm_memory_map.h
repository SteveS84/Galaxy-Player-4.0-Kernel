/* ============================================================================================



                               G D M     M E M O R Y     M A P






 This file includes memory map file appropriate to the kind of GDM chip .







                         GCT Semiconductor Inc. All Rights Reserved.
 


============================================================================================ */



/* ============================================================================================
                                 C O D E     H I S T O R Y
===============================================================================================

-----------------------------------------------------------------------------------------------
When              Who              What                        (in reverse chronological order)
-----------------------------------------------------------------------------------------------

Feb.08.2007       James Shin       Created

-------------------------------------------------------------------------------------------- */

#ifndef _GDM_MEMORY_MAP_H
#define _GDM_MEMORY_MAP_H

/* ============================================================================================
                                I N C L U D E     F I L E S
============================================================================================ */
#include "gdm_feature.h"

#if defined(FEATURE_GDM7002)
#include "gdm7002_memory_map.h"
#elif defined(FEATURE_GDM7003)
#include "gdm7003_memory_map.h"
#elif defined(FEATURE_GDM7004)
#include "gdm7004_memory_map.h"
#elif defined(FEATURE_GDM7014)
#include "gdm7014_memory_map.h"
#elif defined(FEATURE_GDM7024)
#include "gdm7024_memory_map.h"
#endif

#endif /* _GDM_MEMORY_MAP_H */
