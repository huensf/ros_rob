#include "mbs_user_interface.h"
#include "mbs_data.h"
#include <math.h>
#include <stdlib.h>
#include "pthread.h"

typedef struct MyThreadStruct ThreadStruct;

struct MyThreadStruct
{
   void (*pointeur_give_torque_access)(); 
  
   void (*pointeur_give_pos_vit_ped_access)(); 
};
