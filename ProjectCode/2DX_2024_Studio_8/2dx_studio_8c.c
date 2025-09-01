#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include <math.h>

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up

volatile unsigned long FallingEdges = 0;
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};		
	GPIO_PORTN_DIR_R |= 0x01;        								
  GPIO_PORTN_AFSEL_R &= ~0x01;     								
  GPIO_PORTN_DEN_R |= 0x01;        									
  GPIO_PORTN_AMSEL_R &= ~0x01;     								
	return;
} // n0 = led d2 = additional status LED (used for sensor issue)
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {}; 
    GPIO_PORTF_DIR_R |= 0b00010001;  // 1 = out
		GPIO_PORTF_AFSEL_R &= ~0b00010001;
    GPIO_PORTF_DEN_R |= 0b00010001;   
		GPIO_PORTF_AMSEL_R &= ~0b00010001; 
		// PF4 = LED D3 PF0 = LED D4 / PF0 (D4) = UART transmission indication / // PF4 (D3) = Measurement Status 
} 
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           												
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          											
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		
	GPIO_PORTB_AFSEL_R |= 0x0C;  // Enable alternate function for PB2 (SCL) and PB3 (SDA)
  GPIO_PORTB_ODR_R |= 0x08;    // Enable open-drain for PB3 (SDA line requires open-drain)
  GPIO_PORTB_DEN_R |= 0x0C;    // Enable digital functionality for PB2 and PB3            																	                                                       
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;   
  I2C0_MCR_R = I2C_MCR_MFE; // Enable I2C master mode                 													
  I2C0_MTPR_R = 0b0000000000000101000000000111011;  // Set clock speed for I2C communication      TPR = 59                 

}
void PortG_Init(void){ //The VL53L1X needs to be reset using XSHUT.  We will use PG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;              
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};  
    GPIO_PORTG_DIR_R &= 0x00;      // Set PG0 as an input (0 = input, 1 = output)
    GPIO_PORTG_AFSEL_R &= ~0x01;   // Disable alternate functions on PG0
    GPIO_PORTG_DEN_R |= 0x01;      // Enable digital functionality on PG0
    GPIO_PORTG_AMSEL_R &= ~0x01;   // Disable analog functionality on PG0                     
    return;
}
void PortM_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0) {}
    GPIO_PORTM_DIR_R |= 0x0F;
    GPIO_PORTM_AFSEL_R &= ~0x0F;
    GPIO_PORTM_DEN_R |= 0x0F;
    GPIO_PORTM_AMSEL_R &= ~0x0F;
}
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};		
  GPIO_PORTJ_DIR_R &= ~0x02;     // Set PJ1 as input (clearing bit 1)
  GPIO_PORTJ_DEN_R |= 0x02;      // Enable digital function for PJ1     									
	GPIO_PORTJ_PCTL_R &= ~0x000000F0; // Clear PCTL settings for PJ1 (ensure it's GPIO)
  GPIO_PORTJ_AMSEL_R &= ~0x02;      // Disable analog mode on PJ1
   GPIO_PORTJ_PUR_R |= 0x02;         // Enable internal pull-up resistor on PJ1				
}
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        
    GPIO_PORTG_DATA_R &= 0b11111110;                                
    FlashAllLEDs();
    SysTick_Wait10ms(100);
    GPIO_PORTG_DIR_R &= ~0x01;                                          
}
void rotate_CW(int delay, int steps) {
    for(int i = 0; i < steps; i++) {
        GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00000011;
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00001001;
        SysTick_Wait10ms(delay);
    }
}
void reset_CounterClockwise() {
		GPIO_PORTF_DATA_R &= ~0x10;
		GPIO_PORTN_DATA_R |= 0b01;
    for(int i = 0; i < 512; i++) {
        GPIO_PORTM_DATA_R = 0b00001001;
        SysTick_Wait1ms(6);
        GPIO_PORTM_DATA_R = 0b00000011;
        SysTick_Wait1ms(6);
        GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait1ms(6);
        GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait1ms(6);
    }
		GPIO_PORTN_DATA_R &= ~0b01;
		GPIO_PORTF_DATA_R |= 0x10;
}
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
volatile int count = 0;          // Track motor steps

int main(void) {
    uint8_t dataReady;
    uint16_t Distance;
    uint8_t previousButtonState = 1;  // Button is pulled up, so 1 is not pressed
    uint8_t isScanning = 0;  //set when button pressed
			
    PLL_Init();           
    SysTick_Init();       
    PortN_Init();         
    PortF_Init();         
    PortJ_Init();         
    PortM_Init();         
    I2C_Init();           
    PortG_Init();         
    UART_Init();          
    
    sprintf(printf_buffer, "Program Begins\r\n");
    UART_printf(printf_buffer);
    
    // Reset and initialize ToF sensor
    VL53L1X_XSHUT();
    status = VL53L1X_SensorInit(dev);
    
    if (status == 0) {
        status = VL53L1X_SetDistanceMode(dev, 2); //2 = long 4m max
        status = VL53L1X_SetTimingBudgetInMs(dev, 100);  //time per reading 100ms
        status = VL53L1X_SetInterMeasurementInMs(dev, 100); //time bw measurement
        status = VL53L1X_StartRanging(dev); //making a measurement
    }
    while (1) {
        if ((GPIO_PORTJ_DATA_R & 0x02) == 0) {
							if (isScanning == 0) {
									isScanning = 1;
									GPIO_PORTF_DATA_R |= 0x10;  // Turn on LED D3 F4
									sprintf(printf_buffer, "DATA TX STARTED\r\n");
									UART_printf(printf_buffer);
							} else {
									isScanning = 0; 
									GPIO_PORTF_DATA_R &= ~0x10;  // Turn off LED D3 F4
									sprintf(printf_buffer, "DATA TX STOPPED\r\n");
									UART_printf(printf_buffer);
							}
							while ((GPIO_PORTJ_DATA_R & 0x02) == 0) {// Wait for button release
									SysTick_Wait1ms(10);
							}
				}
        if (isScanning) {
            rotate_CW(1, 16);
						count++;
   
            dataReady = 0;
            while (dataReady == 0) {
                status = VL53L1X_CheckForDataReady(dev, &dataReady); //waiting for reutrn 1 whne data rready
								VL53L1_WaitMs(dev, 5);
               
            }
            if (isScanning && dataReady) { //if data is ready, 
                status = VL53L1X_GetDistance(dev, &Distance); //returns distance in mm
                status = VL53L1X_ClearInterrupt(dev); //clears for next event
									
                FlashLED4(3);
                sprintf(printf_buffer, "%u\r\n", Distance);
                UART_printf(printf_buffer); //sending data
								SysTick_Wait1ms(5);
                
                if(count % 32 == 0){
                    reset_CounterClockwise(); //RESET TO untangle wires
										SysTick_Wait10ms(10);
                }
            }
        } else {
            SysTick_Wait1ms(10);
        }
    }
}