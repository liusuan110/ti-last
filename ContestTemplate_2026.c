#include "ti_msp_dl_config.h"

#include "App.h"

int main(void)
{
    SYSCFG_DL_init();
    App_init();

    while (1) {
        App_loop();
    }
}
