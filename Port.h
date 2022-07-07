 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Hussein
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohamed Hussein's ID = 1000 :) */
#define Port_VENDOR_ID    (1000U)

/* Port Module Id */
#define PORT_MODULE_ID    (120U)

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)
/*
 * Macros for Dio Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)
/* Standard AUTOSAR types */
#include "Std_Types.h"
/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif


/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for PORT Init */
#define PORT_INIT_SID                   (uint8)0x00

/* Service ID for PORT Set Pin Direction */
#define PORT_SET_PIN_DIRECTION_SID      (uint8)0x01

/* Service ID for PORT refresh Port Direction*/
#define PORT_REFRESH_PORT_DIR_SID       (uint8)0x02

/* Service ID for PORT Get Version Info */
#define PORT_GET_VERSION_INFO_SID       (uint8)0x03

/* Service ID for PORT Set Pin Mode */
#define PORT_SET_PIN_MODE_SID           (uint8)0x04


/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

/* DET code to repoert invalid port pin id required */ 
#define PORT_E_PARAM_PIN                (uint8)0x0A

/* Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE   (uint8)0x0B

/* API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG              (uint8)0x0C

/* API Port_SetPinMode service called when mode is invalid */
#define PORT_E_PARAM_INVALID_MODE        (uint8)0x0D
   
/* API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE         (uint8)0x0E

/* APIs called with a Null Pointer      */
#define PORT_E_PARAM_POINTER             (uint8)0x10

/* API service called without module initialization */
#define PORT_E_UNINIT                   (uint8)0x0F 



/*
 * API service used without module initialization is reported using following
 * error code (Not exist in AUTOSAR 4.0.3 DIO SWS Document.
 */

 /*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
typedef uint8  Port_PinType  ;   
typedef uint8 Initial_Val;  
typedef uint8 Pin_ModeChangable; 
typedef uint8 Pin_DirectionChangable;  
typedef uint8  Port_PinModeType;  
typedef uint8  pin_num; 
typedef uint8  port_num; 
  


  
/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirection;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 *      5. the initial value--> High,Low
 *      6. the Direction changable
 *      7. the Mode changable
 *      
 */
  typedef struct 
  {
   port_num  port_num   ; 
   pin_num    pin_num    ;  
   Port_PinDirection direction ; 
   Port_InternalResistor  resistor_state ; 
   Initial_Val value ; 
   Port_PinModeType mode ; 
   Pin_DirectionChangable Dir_changable ;  
   Pin_ModeChangable Mode_changable ; 
   } Port_ConfigChannel; 
   
/* Data Structure required for initializing the Port Driver */
typedef struct 
{
	Port_ConfigChannel Channels[Port_CONFIGURED_PINS];
} Port_ConfigType;
      

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* Function for Port Initialization API */
void Port_Init(const Port_ConfigType * ConfigPtr);

/* Function for setting up port direction */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirection direction);
#endif

/* Function to refresh port direction */
void Port_RefreshPortDirection( void );

/* Function for PORT Get Version Info API */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo);
#endif

/* Function to set up Pin Mode */
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );
#endif


/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_ConfigPtr;

#endif /* PORT_H */
