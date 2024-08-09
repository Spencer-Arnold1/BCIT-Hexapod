

#include "pca9685.h"


//*****************************************************************************
//! - I2C0 peripheral
//! - GPIO Port B peripheral (for I2C0 pins)
//! - I2C0SCL - PB2
//! - I2C0SDA - PB3
//*****************************************************************************


//*****************************************************************************
//
// Set the address for slave module. This is a 7-bit address sent in the
// following format:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
//
// A zero in the "RS" position of the first byte means that the master
// transmits (sends) data to the selected slave, and a one in this position
// means that the master receives data from the slave.
//
//*****************************************************************************


//*****************************************************************************
//
// Sends Data to pca9685
//
//*****************************************************************************
void pca9685sendDataFrame(uint8_t slaveAddr, uint8_t controlReg, uint8_t data) {

    // Set the slave address and indicate a write operation
    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr, false);

    // Put the control register in the data register
    I2CMasterDataPut(I2C0_BASE, controlReg);

    // Initiate the transaction (send the slave address and control register)
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait for the transaction to complete
    while (I2CMasterBusy(I2C0_BASE));

    // Check for errors
    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) {
        // Handle the error
        return;
    }

    // Put the data in the data register
    I2CMasterDataPut(I2C0_BASE, data);

    // Continue the transaction (send the data)
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    // Wait for the transaction to complete
    while (I2CMasterBusy(I2C0_BASE));

    // Check for errors
    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) {
        // Handle the error
        return;
    }

}

//*****************************************************************************
//
// Configure the I2C0 master
//
//*****************************************************************************
void i2cInit(){


    // enable I2C0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);


    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

}

//*****************************************************************************
//
// Initializes pca9685 PWM frequency and modes
//
//*****************************************************************************
void PCA9685Init(uint32_t addr){

    //todo: reset is not functioning properly --need to do a power reset. To be fixed.

    i2cInit();

    uint32_t mode1 = 0x00;
    uint32_t mode1Data_wake = 0x00;
    uint32_t mode1Data_sleep = 0x10; // Sleep mode, bit 4 set to 1

    // adjusting frequency to 50 Hz
    uint32_t pre_scale_reg = 0xFE;
    uint32_t pre_scale_value = 0x79; // 121 in decimal for 50 Hz

    // MODE1 config to sleep
    pca9685sendDataFrame(addr, mode1, mode1Data_sleep);
    pca9685sendDataFrame(addr + 1, mode1, mode1Data_sleep);


    // frequency config
    pca9685sendDataFrame(addr, pre_scale_reg, pre_scale_value);
    pca9685sendDataFrame(addr + 1, pre_scale_reg, pre_scale_value);

    // MODE1 config to wake
    pca9685sendDataFrame(addr, mode1, mode1Data_wake);
    pca9685sendDataFrame(addr + 1, mode1, mode1Data_sleep);

    // wait for the oscillator to stabilize
    SysCtlDelay(2); // small delay

    // Restart the oscillator
    uint32_t mode1_restart = 0x80; // restart bit
    pca9685sendDataFrame(addr, mode1, mode1_restart);
    pca9685sendDataFrame(addr + 1, mode1, mode1_restart);

}


// angle input from -90 to 90
int systemTransferEquation(double angle) {

    double result = 512/225 * angle + 1536/5;

    return (int)result;
}

// Wrapper function to interface with
// legBaseAddress: will indicate address of led"N"_ON_loControlReg.
void servoControl(double angle, uint32_t baseAddress, uint32_t driverAddress) {

    // 0 degree phase shift for every pulse
    uint32_t  led_ON_loControlReg_dur = 0x00;
    uint32_t  led_ON_hiControlReg_dur = 0x00;


    // Servo Initializtion
    uint32_t  led0_ON_loControlReg = baseAddress;
    uint32_t  led0_ON_hiControlReg = baseAddress + 1;

    uint32_t  led0_OFF_loControlReg = baseAddress + 2;
    uint32_t  led0_OFF_hiControlReg = baseAddress + 3;


    // PWM duration control registers
    uint32_t  led0_OFF_hiControlReg_dur;
    uint32_t  led0_OFF_loControlReg_dur;


    // Angle value is split into two registers
    led0_OFF_hiControlReg_dur = (systemTransferEquation(angle) &(0xF00)) >> 8;
    led0_OFF_loControlReg_dur = systemTransferEquation(angle) &(0x0FF);


    // sending data
    pca9685sendDataFrame(driverAddress, led0_ON_hiControlReg, led_ON_hiControlReg_dur );
    pca9685sendDataFrame(driverAddress, led0_ON_loControlReg, led_ON_loControlReg_dur );
    pca9685sendDataFrame(driverAddress, led0_OFF_hiControlReg, led0_OFF_hiControlReg_dur );
    pca9685sendDataFrame(driverAddress, led0_OFF_loControlReg, led0_OFF_loControlReg_dur );


}



