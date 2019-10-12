#include "hps.h"
#include <stdint.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

//#include "socal/socal.h"
//#include "socal/hps.h"
//#include "socal/alt_gpio.h"


#ifdef __arm__
    #include "socal/socal.h"
    #include "socal/hps.h"
    #include "socal/alt_gpio.h"
#else
    #define ALT_GPIO1_SWPORTA_DR_ADDR
    #define ALT_LWFPGASLVS_OFST
    #define ALT_STM_OFST 0
    #define alt_clrbits_word(x, y)
    #define alt_setbits_word(x, y)
    #define alt_read_word(x) 0
#endif



#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define USER_IO_DIR     (0x01000000)
#define BIT_LED         (0x01000000)

HPS::HPS()
{
    PioInit();
}

HPS::~HPS()
{
    //----- release pio resource
        // clean up our memory mapping and exit
    munmap( m_virtual_base, HW_REGS_SPAN );
    close( m_file_mem );
}



bool HPS::LedSet(bool bOn){
    if (m_file_mem == -1){
        return false;
    }

    if (!bOn){ //
        alt_clrbits_word((void *) ( (char *)m_virtual_base + ( ( uint32_t )( ALT_GPIO1_SWPORTA_DR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) )) , BIT_LED );
    }else{
        alt_setbits_word((void *) ( (char *)m_virtual_base + ( ( uint32_t )( ALT_GPIO1_SWPORTA_DR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ), BIT_LED );
    }

    return true;

}



bool HPS::PioInit(){
    m_file_mem = open( "/dev/mem", ( O_RDWR | O_SYNC ) );
    if (m_file_mem != -1){
        m_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, HW_REGS_BASE );

        // configure LED as output pin
        alt_setbits_word((void *) ( (char *)m_virtual_base + ( ( uint32_t )( ALT_GPIO1_SWPORTA_DDR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ), USER_IO_DIR );
    }else{
        return false;
    }

    return true;

}


