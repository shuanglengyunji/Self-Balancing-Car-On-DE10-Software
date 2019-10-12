#ifndef HPS_H
#define HPS_H

#include <stdint.h>


class HPS
{
public:
    HPS();
    ~HPS();

    bool LedSet(bool bOn);
protected:
    // pio
    int m_file_mem;
    void *m_virtual_base;
    bool PioInit();

};

#endif // HPS_H
 