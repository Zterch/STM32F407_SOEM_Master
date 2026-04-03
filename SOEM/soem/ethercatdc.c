#include "ethercatdc.h"
#include "ethercatbase.h"
#include "osal.h"

boolean ecx_configdc(ecx_contextt *context)
{
    uint16 slavecnt = *context->slavecount;
    int32 hl = 0;
    uint16 dc_ref = 0;

    /* Read DC receive times for all slaves */
    for (uint16 i = 1; i <= slavecnt; i++) {
        uint16 configadr = context->slavelist[i].configadr;
        uint32 dctimes[4] = {0};

        ecx_FPRD(context->port, configadr, 0x0900, 16, dctimes, EC_TIMEOUTSAFE);
        context->slavelist[i].DCrtA = dctimes[0];
        context->slavelist[i].DCrtB = dctimes[1];
        context->slavelist[i].DCrtC = dctimes[2];
        context->slavelist[i].DCrtD = dctimes[3];

        if (dctimes[0] != 0) {
            context->slavelist[i].hasdc = TRUE;
            if (dc_ref == 0) {
                dc_ref = i;  /* First DC-capable slave becomes reference */
            }
        }
    }

    /* Calculate propagation delays and set DC reference */
    for (uint16 i = 1; i <= slavecnt; i++) {
        if (context->slavelist[i].hasdc) {
            if (i == 1) {
                context->slavelist[i].pdelay = 0;
            } else {
                context->slavelist[i].pdelay =
                    (int32)(context->slavelist[i].DCrtA - context->slavelist[i-1].DCrtA);
            }

            /* Write system time offset */
            hl = -context->slavelist[i].pdelay;
            ecx_FPWR(context->port, context->slavelist[i].configadr,
                     0x0920, sizeof(hl), &hl, EC_TIMEOUTSAFE);
        }
    }

    /* Set DC reference for group 0 */
    if (dc_ref > 0) {
        context->grouplist[0].hasdc = TRUE;
        context->grouplist[0].DCnext = dc_ref;
    }

    return TRUE;
}

boolean ec_configdc(void)
{
    return ecx_configdc(&ecx_context);
}

void ecx_dcsync0(ecx_contextt *context, uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift)
{
    uint16 configadr = context->slavelist[slave].configadr;
    uint8 active = 0;
    uint32 cycltime = CyclTime;
    int32 shift = CyclShift;

    /* Deactivate first */
    ecx_FPWR(context->port, configadr, ECT_REG_DCSYNCACT, 1, &active, EC_TIMEOUTSAFE);

    /* Write cycle time */
    ecx_FPWR(context->port, configadr, ECT_REG_DCCYCLE0, sizeof(cycltime), &cycltime, EC_TIMEOUTSAFE);

    /* Write start time (shift) */
    ecx_FPWR(context->port, configadr, ECT_REG_DCSTART0, sizeof(shift), &shift, EC_TIMEOUTSAFE);

    /* Activate SYNC0 */
    if (act) {
        active = 0x03; /* SYNC0 active, cyclic */
        ecx_FPWR(context->port, configadr, ECT_REG_DCSYNCACT, 1, &active, EC_TIMEOUTSAFE);
    }

    context->slavelist[slave].DCactive = active;
    context->slavelist[slave].DCcycle = CyclTime;
    context->slavelist[slave].DCshift = CyclShift;
}

void ec_dcsync0(uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift)
{
    ecx_dcsync0(&ecx_context, slave, act, CyclTime, CyclShift);
}

void ecx_dcsync01(ecx_contextt *context, uint16 slave, boolean act,
                  uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift)
{
    uint16 configadr = context->slavelist[slave].configadr;
    uint8 active = 0;

    ecx_FPWR(context->port, configadr, ECT_REG_DCSYNCACT, 1, &active, EC_TIMEOUTSAFE);

    ecx_FPWR(context->port, configadr, ECT_REG_DCCYCLE0, sizeof(CyclTime0), &CyclTime0, EC_TIMEOUTSAFE);
    ecx_FPWR(context->port, configadr, ECT_REG_DCCYCLE1, sizeof(CyclTime1), &CyclTime1, EC_TIMEOUTSAFE);

    int32 shift = CyclShift;
    ecx_FPWR(context->port, configadr, ECT_REG_DCSTART0, sizeof(shift), &shift, EC_TIMEOUTSAFE);

    if (act) {
        active = 0x07; /* SYNC0 + SYNC1 active, cyclic */
        ecx_FPWR(context->port, configadr, ECT_REG_DCSYNCACT, 1, &active, EC_TIMEOUTSAFE);
    }
}

void ec_dcsync01(uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift)
{
    ecx_dcsync01(&ecx_context, slave, act, CyclTime0, CyclTime1, CyclShift);
}
