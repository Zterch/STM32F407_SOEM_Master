#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include <stdio.h>

int ecx_config(ecx_contextt *context, uint8 usetable, void *pIOmap)
{
    int slavecnt;

    /* Scan network and init slaves */
    slavecnt = ecx_config_init(context, usetable);
    if (slavecnt <= 0)
        return 0;

    /* Map PDOs */
    ecx_config_map_group(context, pIOmap, 0);

    /* Configure DC */
    ecx_configdc(context);

    return slavecnt;
}

int ec_config(uint8 usetable, void *pIOmap)
{
    return ecx_config(&ecx_context, usetable, pIOmap);
}
