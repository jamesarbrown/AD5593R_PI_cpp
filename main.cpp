#include <iostream>
#include "AD5593R.h"


AD5593R ad5593r("/dev/i2c-1");
bool my_DACs[8]     = {1,1,1,1,0,0,0,0};
bool my_ADCs[8]     = {0,0,0,0,1,1,0,0};
bool my_GPOs[8]     = {0,0,0,0,0,0,1,0};
bool my_GPIs[8]     = {0,0,0,0,0,0,0,1};

int main(int argc, char** argv) 
{ 
    // Init the i2c
    if(ad5593r.init() == false){
        std::cout << "Failed to initialize AD5593R.";
        return 1;
    }

    ad5593r.enable_internal_Vref();
    ad5593r.set_DAC_max_2x_Vref();
    ad5593r.set_ADC_max_2x_Vref();
    ad5593r.configure_DACs(my_DACs);
    ad5593r.write_DAC(0, 1.25);
    ad5593r.write_DAC(1, 2.5);
    ad5593r.write_DAC(2, 3.75);
    ad5593r.write_DAC(3, 5);
    ad5593r.configure_ADCs(my_ADCs);
    ad5593r.read_ADCs();
    ad5593r.configure_GPOs(my_GPOs);
    ad5593r.write_GPO(6, 1);
    ad5593r.configure_GPIs(my_GPIs);
    ad5593r.read_GPIs();
}