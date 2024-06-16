#include <iostream>
#include <fcntl.h>
#include <unistd.h>
extern "C" {
    #include <linux/i2c-dev.h>
}
#include <sys/ioctl.h>
#include <wiringPi.h>
#include "AD5593R.h"

//Definitions
#define _ADAC_NULL              0b00000000
#define _ADAC_ADC_SEQUENCE      0b00000010 // ADC sequence register - Selects ADCs for conversion
#define _ADAC_GP_CONTROL        0b00000011 // General-purpose control register - DAC and ADC control register
#define _ADAC_ADC_CONFIG        0b00000100 // ADC pin configuration - Selects which pins are ADC inputs
#define _ADAC_DAC_CONFIG        0b00000101 // DAC pin configuration - Selects which pins are DAC outputs
#define _ADAC_PULL_DOWN         0b00000110 // Pull-down configuration - Selects which pins have an 85 kO pull-down resistor to GND
#define _ADAC_LDAC_MODE         0b00000111 // LDAC mode - Selects the operation of the load DAC
#define _ADAC_GPIO_WR_CONFIG    0b00001000 // GPIO write configuration - Selects which pins are general-purpose outputs
#define _ADAC_GPIO_WR_DATA      0b00001001 // GPIO write data - Writes data to general-purpose outputs
#define _ADAC_GPIO_RD_CONFIG    0b00001010 // GPIO read configuration - Selects which pins are general-purpose inputs
#define _ADAC_POWER_REF_CTRL    0b00001011 // Power-down/reference control - Powers down the DACs and enables/disables the reference
#define _ADAC_OPEN_DRAIN_CFG    0b00001100 // Open-drain configuration - Selects open-drain or push-pull for general-purpose outputs
#define _ADAC_THREE_STATE       0b00001101 // Three-state pins - Selects which pins are three-stated
#define _ADAC_RESERVED          0b00001110 // Reserved
#define _ADAC_SOFT_RESET        0b00001111 // Software reset - Resets the AD5593R

/**
 * @name     ADAC Configuration Data Bytes
 ******************************************************************************/
 ///@{
 //write into MSB after _ADAC_POWER_REF_CTRL command to enable VREF
#define _ADAC_VREF_ON           0b00000010
#define _ADAC_SEQUENCE_ON       0b00000010

/**
 * @name   ADAC Write / Read Pointer Bytes
******************************************************************************/
///@{
#define _ADAC_DAC_WRITE         0b00010000
#define _ADAC_ADC_READ          0b01000000
#define _ADAC_DAC_READ          0b01010000
#define _ADAC_GPIO_READ         0b01110000
#define _ADAC_REG_READ          0b01100000



//Class constructor
AD5593R::AD5593R(const char* i2c_device_filepath, uint8_t i2c_addr, int a0): i2c_bus_addr(i2c_addr), i2c_file(0), i2c_filepath(i2c_device_filepath)
{
    _a0 = a0;
    _GPRC_msbs = 0x00;
    _GPRC_lsbs = 0x00;
    _PCR_msbs = 0x00;
    _PCR_lsbs = 0x00;
    //intializing the configuration struct.
    for (unsigned int i = 0; i < _num_of_channels; i++) {
        config.ADCs[i] = 0;
        config.DACs[i] = 0;
    }

    for (unsigned int i = 0; i < _num_of_channels; i++) {
        values.ADCs[i] = -1;
        values.DACs[i] = -1;
    }

    //this allows for multiple devices on the same bus, see header.
    if (_a0 > -1) {
        pinMode(_a0, OUTPUT);
        digitalWrite(_a0, HIGH);
    }
}

// Initiate the i2c device.
bool AD5593R::init() {
    //Open i2c device
    int file;
    if((file = open(i2c_filepath.data(), O_RDWR)) < 0) {
        AD5593R_PRINTLN("Can't open i2c channel");
        return false;
    }
    i2c_file = file;

    //Set address
    if(ioctl(file, I2C_SLAVE, i2c_bus_addr) < 0) {
        AD5593R_PRINTLN("IO_CTRL failed on the i2c");
        return false;
    }

    //Check its answering
    uint8_t check = ad5593r_write(_ADAC_ADC_SEQUENCE, 0x02, uint8_t(1 << 1));
    if (check != 0) {
         AD5593R_PRINTLN("Device Unresponsive Communication Error");
         return false;
    }

    return true;
}



// Custom i2c Read/Write, shamelessly stolen and modifyed from "https://github.com/wwanc/Si5351Pi_cpp"
uint8_t AD5593R::ad5593r_write(uint8_t addr, uint8_t msb, uint8_t lsb) {
    uint8_t buff[3];
    buff[0] = addr;
    buff[1] = msb;
    buff[2] = lsb;

    int result = write(i2c_file, buff, 3);
    if(result < 0){
        return result;
    }

    return 0;
}

uint16_t AD5593R::ad5593r_read(uint8_t addr) {
    int result = write(i2c_file, &addr, 1);
    if(result < 0){
        return result;
    }

    uint16_t data;
    result = read(i2c_file, &data, 2);
    if(result < 0){
        return result;
    }
    
    // LSB and MSB is switched, so switch it back here.
    uint8_t msb = data;
    uint8_t lsb = data >> 8;
    uint16_t reg_val = (msb << 8) + lsb;

	return reg_val;
}



// The print function is used in different ways.
void AD5593R::ad5593r_print(std::string data) {
    std::cout << data;
}

void AD5593R::ad5593r_print(uint8_t data) {
    printf("%02X", data);
}

void AD5593R::ad5593r_print(float data) {
    std::cout << data;
}

void AD5593R::ad5593r_println(std::string data) {
    std::cout << data << std::endl;
}



void AD5593R::enable_internal_Vref() {
    //Enable selected device for writing
    _Vref = 2.5;
    _ADC_max = _Vref;
    _DAC_max = _Vref;
    if (_a0 > -1) digitalWrite(_a0, LOW);

    //check if the on bit is already fliped on
    if ((_PCR_msbs & 0x02) != 0x02) {
        _PCR_msbs = _PCR_msbs ^ 0x02;
    }
    ad5593r_write(_ADAC_POWER_REF_CTRL, _PCR_msbs, _PCR_lsbs);

    //Disable selected device for writing
    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINTLN("Internal Reference on.");
}

void AD5593R::disable_internal_Vref() {
    //Enable selected device for writing
    _Vref = -1;
    _ADC_max = _Vref;
    _DAC_max = _Vref;
    if (_a0 > -1) digitalWrite(_a0, LOW);
    //check if the on bit is already fliped off
    if ((_PCR_msbs & 0x02) == 0x02) {
        _PCR_msbs = _PCR_msbs ^ 0x02;
    }
    ad5593r_write(_ADAC_POWER_REF_CTRL, _PCR_msbs, _PCR_lsbs);

    //Disable selected device for writing
    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINTLN("Internal Reference off.");
}

void AD5593R::set_ADC_max_2x_Vref() {
    //Enable selected device for writing
    _ADC_max = 2 * _Vref;
    if (_a0 > -1) digitalWrite(_a0, LOW);
    //check if 2x bit is on in the general purpose register
    if ((_GPRC_lsbs & 0x20) != 0x20) {
        _GPRC_lsbs = _GPRC_lsbs ^ 0x20;
    }
    ad5593r_write(_ADAC_GP_CONTROL, _GPRC_msbs, _GPRC_lsbs);

    //Disable selected device for writing
    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINTLN("ADC max voltage = 2xVref");
    _ADC_2x_mode = 1;
}

void AD5593R::set_ADC_max_1x_Vref() {
    //Enable selected device for writing
    _ADC_max = _Vref;
    if (_a0 > -1) digitalWrite(_a0, LOW);

    if ((_GPRC_lsbs & 0x20) == 0x20) {
        _GPRC_lsbs = _GPRC_lsbs ^ 0x20;
    }
    ad5593r_write(_ADAC_GP_CONTROL, _GPRC_msbs, _GPRC_lsbs);

    //Disable selected device for writing
    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINTLN("ADC max voltage = 1xVref");
    _ADC_2x_mode = 0;
}

void AD5593R::set_DAC_max_2x_Vref() {
    //Enable selected device for writing
    _DAC_max = 2 * _Vref;
    if (_a0 > -1) digitalWrite(_a0, LOW);

    if ((_GPRC_lsbs & 0x10) != 0x10) {
        _GPRC_lsbs = _GPRC_lsbs ^ 0x10;
    }
    ad5593r_write(_ADAC_GP_CONTROL, _GPRC_msbs, _GPRC_lsbs);

    //Disable selected device for writing
    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINTLN("DAC max voltage = 2xVref");
    _DAC_2x_mode = 1;
}

void AD5593R::set_DAC_max_1x_Vref() {
    //Enable selected device for writing
    _DAC_max = _Vref;
    if (_a0 > -1) digitalWrite(_a0, LOW);

    if ((_GPRC_lsbs & 0x10) == 0x10) {
        _GPRC_lsbs = _GPRC_lsbs ^ 0x10;
    }
    ad5593r_write(_ADAC_GP_CONTROL, _GPRC_msbs, _GPRC_lsbs);

    //Disable selected device for writing
    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINTLN("ADC max voltage = 1xVref");
    _DAC_2x_mode = 0;
}

void AD5593R::set_Vref(float Vref) {
    _Vref = Vref;
    if (_ADC_2x_mode == 0) {
        _ADC_max = Vref;
    }
    else {
        _ADC_max = 2 * Vref;
    }

    if (_DAC_2x_mode == 0) {
        _DAC_max = Vref;
    }
    else {
        _DAC_max = 2 * Vref;
    }
}



void AD5593R::configure_DAC(uint8_t channel) {
    if (_a0 > -1) digitalWrite(_a0, LOW);
    config.DACs[channel] = 1;
    uint8_t channel_byte = 1 << channel;
    //check to see if the channel is a DAC already
    if ((_DAC_config & channel_byte) != channel_byte) {
        _DAC_config = _DAC_config ^ channel_byte;
    }
    ad5593r_write(_ADAC_DAC_CONFIG, 0x00, _DAC_config);

    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINT("Channel ");
    AD5593R_PRINT(channel);
    AD5593R_PRINTLN(" is configured as a DAC");
}

void AD5593R::configure_DACs(bool* channels) {
    for (size_t i = 0; i < _num_of_channels; i++) {
        if (channels[i] == 1) {
            configure_DAC(i);
        }
    }
}


int AD5593R::write_DAC(uint8_t channel, float voltage) {
    //error checking
    if (config.DACs[channel] == 0) {
        AD5593R_PRINT("ERROR! Channel ");
        AD5593R_PRINT(channel);
        AD5593R_PRINTLN(" is not a DAC");
        return -1;
    }
    if (_DAC_max == -1) {
        AD5593R_PRINTLN("Vref, or DAC_max is not defined");
        return -2;
    }
    if (voltage > _DAC_max) {
        AD5593R_PRINTLN("Vref, or DAC_max is lower than set voltage");
        return -3;
    }

    if (_a0 > -1) digitalWrite(_a0, LOW);
    unsigned int data_bits = (voltage / _DAC_max) * 4095;

    //extract the 4 most signifigant bits, and move them down to the bottom
    uint8_t data_msbs = (data_bits & 0xf00) >> 8;
    uint8_t lsbs = (data_bits & 0x0ff);
    //place the channel data in the most signifigant bits
    uint8_t msbs = (0b10000000 | (channel << 4)) | data_msbs;

    ad5593r_write((_ADAC_DAC_WRITE | channel), msbs, lsbs);

    AD5593R_PRINT("Channel ");
    AD5593R_PRINT(channel);
    AD5593R_PRINT(" is set to ");
    AD5593R_PRINT(voltage);
    AD5593R_PRINTLN(" Volts");
    if (_a0 > -1) digitalWrite(_a0, HIGH);
    values.DACs[channel] = voltage;

    return 1;
}



void AD5593R::configure_ADC(uint8_t channel) {
    if (_a0 > -1) digitalWrite(_a0, LOW);
    config.ADCs[channel] = 1;
    uint8_t channel_byte = 1 << channel;
    //check to see if the channel is a ADC already
    if ((_ADC_config & channel_byte) != channel_byte) {
        _ADC_config = _ADC_config ^ channel_byte;
    }
    ad5593r_write(_ADAC_ADC_CONFIG, 0x00, _ADC_config);

    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINT("Channel ");
    AD5593R_PRINT(channel);
    AD5593R_PRINTLN(" is configured as a ADC");
}

void AD5593R::configure_ADCs(bool* channels) {
    for (size_t i = 0; i < _num_of_channels; i++) {
        if (channels[i] == 1) {
            configure_ADC(i);
        }
    }
}


float AD5593R::read_ADC(uint8_t channel) {
    if (config.ADCs[channel] == 0) {
        AD5593R_PRINT("ERROR! Channel ");
        AD5593R_PRINT(channel);
        AD5593R_PRINTLN(" is not an ADC");
        return -1;
    }
    if (_ADC_max == -1) {
        AD5593R_PRINTLN("Vref, or ADC_max is not defined");
        return -2;
    }
    if (_a0 > -1) digitalWrite(_a0, LOW);

    // Select what IO we want to read from.
    ad5593r_write(_ADAC_ADC_SEQUENCE, 0x02, uint8_t(1 << channel));

    uint16_t data_bits = ad5593r_read(_ADAC_ADC_READ);
    // We don't need the first bit, as it just informs us what IO/pin, the voltage is read from. (We read them one at a time anyway, that is why)
    data_bits &= 0x0fff;

    if (_a0 > -1) digitalWrite(_a0, HIGH);
    float data = _ADC_max * (data_bits) / 4095;

    AD5593R_PRINT("Channel ");
    AD5593R_PRINT(channel);
    AD5593R_PRINT(" reads ");
    AD5593R_PRINT(data);
    AD5593R_PRINTLN(" Volts");
    
    return data;
}

float* AD5593R::read_ADCs() {
    for (size_t i = 0; i < _num_of_channels; i++) {
        if (config.ADCs[i] == 1) {
            read_ADC(i);
        }
    }

    return values.ADCs;
}



void AD5593R::configure_GPI(uint8_t channel) {
    if (_a0 > -1) digitalWrite(_a0, LOW);
    config.DACs[channel] = 1;
    uint8_t channel_byte = 1 << channel;
    //check to see if the channel is a gpi already
    if ((_GPI_config & channel_byte) != channel_byte) {
        _GPI_config = _GPI_config ^ _GPI_config;
    }
    ad5593r_write(_ADAC_GPIO_RD_CONFIG, 0x00, _GPI_config);

    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINT("Channel ");
    AD5593R_PRINT(channel);
    AD5593R_PRINTLN(" is configured as a GPI");
}

void AD5593R::configure_GPIs(bool* channels) {
    for (size_t i = 0; i < _num_of_channels; i++) {
        if (channels[i] == 1) {
            configure_GPI(i);
        }
    }
}


void AD5593R::configure_GPO(uint8_t channel) {
    if (_a0 > -1) digitalWrite(_a0, LOW);
    config.DACs[channel] = 1;
    uint8_t channel_byte = 1 << channel;
    //check to see if the channel is a gpo already
    if ((_GPO_config & channel_byte) != channel_byte) {
        _GPO_config = _GPO_config ^ _GPO_config;
    }
    ad5593r_write(_ADAC_GPIO_WR_CONFIG, 0x00, _GPI_config);

    if (_a0 > -1) digitalWrite(_a0, HIGH);
    AD5593R_PRINT("Channel ");
    AD5593R_PRINT(channel);
    AD5593R_PRINTLN(" is configured as a GPO");
}

void AD5593R::configure_GPOs(bool* channels) {
    for (size_t i = 0; i < _num_of_channels; i++) {
        if (channels[i] == 1) {
            configure_GPO(i);
        }
    }
}


bool* AD5593R::read_GPIs() {
    if (_a0 > -1) digitalWrite(_a0, LOW);

    uint16_t data_bits = ad5593r_read(_ADAC_GPIO_READ);
    // We don't need the first bit, as it just informs us of the pin that returned a voltage.
    data_bits &= 0x0fff;
    
    if (_a0 > -1) digitalWrite(_a0, HIGH);
    for (size_t i = 0; i < _num_of_channels; i++) {
        if (config.GPIs[i] == 1) {
            values.GPI_reads[i] = bool(data_bits & 0x01);
        }
        data_bits >> 1;
    }

    return values.GPI_reads;
}

void AD5593R::write_GPOs(bool* pin_states) {
    uint8_t data_bits = 0;
    for (size_t i = 0; i < _num_of_channels; i++) {
        if (config.GPOs[i] == 1) {
            values.GPO_writes[i] = pin_states[i];
            data_bits = data_bits & pin_states[i];
        }
        data_bits << 1;
    }
    if (_a0 > -1) digitalWrite(_a0, LOW);
    ad5593r_write(_ADAC_GPIO_WR_DATA, 0x00, data_bits);
    if (_a0 > -1) digitalWrite(_a0, HIGH);
}