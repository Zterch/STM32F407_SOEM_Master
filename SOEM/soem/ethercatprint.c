#include "ethercatprint.h"

const char *ec_ALstatuscode2string(uint16 ALstatuscode)
{
    switch (ALstatuscode) {
        case 0x0000: return "No error";
        case 0x0001: return "Unspecified error";
        case 0x0011: return "Invalid requested state change";
        case 0x0012: return "Unknown requested state";
        case 0x0013: return "Bootstrap not supported";
        case 0x0014: return "No valid firmware";
        case 0x0015: return "Invalid mailbox configuration";
        case 0x0016: return "Invalid mailbox configuration";
        case 0x0017: return "Invalid sync manager configuration";
        case 0x0018: return "No valid inputs available";
        case 0x0019: return "No valid outputs";
        case 0x001A: return "Synchronization error";
        case 0x001B: return "Sync manager watchdog";
        case 0x001C: return "Invalid sync manager types";
        case 0x001D: return "Invalid output configuration";
        case 0x001E: return "Invalid input configuration";
        case 0x001F: return "Invalid watchdog configuration";
        case 0x0020: return "Slave needs cold start";
        case 0x0021: return "Slave needs INIT";
        case 0x0022: return "Slave needs PREOP";
        case 0x0023: return "Slave needs SAFEOP";
        case 0x0029: return "Invalid output FMMU configuration";
        case 0x002A: return "Invalid input FMMU configuration";
        case 0x0030: return "Invalid DC SYNC configuration";
        case 0x0032: return "PLL error";
        case 0x0033: return "DC Sync IO error";
        case 0x0034: return "DC Sync timeout error";
        case 0x0036: return "DC Sync cycle time error";
        case 0x0042: return "MBX_AOE";
        case 0x0043: return "MBX_EOE";
        case 0x0044: return "MBX_COE";
        case 0x0045: return "MBX_FOE";
        case 0x0046: return "MBX_SOE";
        case 0x004F: return "MBX_VOE";
        case 0x0050: return "EEPROM no access";
        case 0x0051: return "EEPROM error";
        default:     return "Unknown AL status code";
    }
}

char *ec_elist2string(void)
{
    static char buf[128];
    buf[0] = 0;
    return buf;
}

const char *ec_sdoerror2string(uint32 sdoerrorcode)
{
    switch (sdoerrorcode) {
        case 0x05030000: return "Toggle bit not changed";
        case 0x05040000: return "SDO protocol timeout";
        case 0x05040001: return "Client/Server specifier not valid";
        case 0x05040005: return "Out of memory";
        case 0x06010000: return "Unsupported access";
        case 0x06010001: return "Read only access";
        case 0x06010002: return "Write only access";
        case 0x06020000: return "Object does not exist";
        case 0x06040041: return "Object cannot be mapped";
        case 0x06040042: return "PDO length exceeded";
        case 0x06060000: return "Access failed due to hardware error";
        case 0x06070010: return "Data type mismatch";
        case 0x06090011: return "Subindex does not exist";
        case 0x06090030: return "Value range exceeded";
        case 0x08000000: return "General error";
        case 0x08000020: return "Data cannot be transferred";
        case 0x08000021: return "Data cannot be transferred (local)";
        case 0x08000022: return "Data cannot be transferred (state)";
        default:         return "Unknown SDO error";
    }
}
