// module ln200.cpp
// 6/29/99

#include "ln200.h"
#include "uio48.h"
#include <iostream.h>



void resetLN200(void)
    {
    extern uio48;
    cout << "Resetting LN200." << endl;       // LN200 Power On Reset
    uio48.set_bit(POR);
    sleep(2000);
    uio48.clr_bit(POR);
    cout << "LN200 reset." << endl << endl;
    }
    