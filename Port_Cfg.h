 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Hussein
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)

/* Pre-compile option for SET_PIN_DIRECTION Info API */ 
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)
/*Pre-compile option for SET_PIN_MODE Info API*/
#define SET_PIN_MODE_API                     (STD_ON)


/* Number of the configured Port Pin Channels */
#define Port_CONFIGURED_PINS             (43U)
#define  MAX_PIN_NUM                     (42U)  
#define  MIN_PIN_NUM                     (0U) 
#define  MAX_MODE                        (10U) 
#define  MIN_MODE                        (0U) 
/* Mode for port driver  */
typedef enum {
  PORT_PIN_MODE_ADC=0,PORT_PIN_MODE_UART0=1,PORT_PIN_MODE_SSI3=1,PORT_PIN_MODE_SSI2=2,PORT_PIN_MODE_UART1=2,PORT_PIN_MODE_I2C=3,PORT_PIN_MODE_M0PWM,PORT_PIN_MODE_M1PWM,PORT_PIN_MODE_QEI,PORT_PIN_MODE_GPT,PORT_PIN_MODE_CAN,PORT_PIN_MODE_USB=8,PORT_PIN_MODE_DIO=10 
}Port_MODE;
/*PORTS NUM*/
typedef enum  {
  PORT_PortA,PORT_PortB,PORT_PortC,PORT_PortD,PORT_PortE,PORT_PortF
}PORT_NUM;
/*PINS NUM*/
typedef enum  {
  PORT_Pin0,PORT_Pin1,PORT_Pin2,PORT_Pin3,PORT_Pin4,PORT_Pin5,PORT_Pin6,PORT_Pin7
}PIN_NUM;


#define PORT_PIN_LEVEL_LOW   STD_LOW 
#define PORT_PIN_LEVEL_HIGH  STD_HIGH   

#endif /* PORT_CFG_H */
