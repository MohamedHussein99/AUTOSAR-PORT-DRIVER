 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Hussein
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"
#include "Port_Regs.h"
#include "Common_Macros.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

static const Port_ConfigChannel *Port_Config = NULL_PTR ;  
static uint8 Port_status = PORT_NOT_INITIALIZED  ;


/************************************************************************************
* Service Name: Port_init
* Service ID[hex]:0X00
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr )
{
#if(PORT_DEV_ERROR_DETECT == STD_ON) 
  
  if(ConfigPtr==NULL_PTR)  /* CHECK IF THE INPUT CONGIFURATON IS NOT NULL PTR */
  {
   Det_ReportError(PORT_MODULE_ID , PORT_INSTANCE_ID , PORT_INIT_SID , PORT_E_PARAM_CONFIG ) ;
  }
  else
#endif
    Port_status=PORT_INITIALIZED;
  Port_Config=ConfigPtr->Channels;
for( uint8 i=0;i<Port_CONFIGURED_PINS;i++)
{
   volatile uint32 * PortGpio_Ptr = NULL_PTR;   /* address first place in the container */
   volatile uint32 delay = 0;
   switch (Port_Config[i].port_num)
   {
case 0: PortGpio_Ptr=(volatile uint32*)GPIO_PORTA_BASE_ADDRESS;    /*Port A base adress*/
            break;
case 1: PortGpio_Ptr=(volatile uint32*)GPIO_PORTB_BASE_ADDRESS;    /*Port B base adress*/
            break;
case 2: PortGpio_Ptr=(volatile uint32*)GPIO_PORTC_BASE_ADDRESS;    /*Port C base adress*/
            break;
case 3: PortGpio_Ptr=(volatile uint32*)GPIO_PORTD_BASE_ADDRESS;    /*Port D base adress*/
            break;
case 4: PortGpio_Ptr=(volatile uint32*)GPIO_PORTE_BASE_ADDRESS;    /*Port E base adress*/
             break;
case 5: PortGpio_Ptr=(volatile uint32*)GPIO_PORTF_BASE_ADDRESS;    /*Port F base adress*/
             break;
  SYSCTL_REGCGC2_REG |=(1<<Port_Config[i].port_num);
    delay =  SYSCTL_REGCGC2_REG ;  
   }
   if((Port_Config[i].port_num==5&&Port_Config[i].pin_num==0)||((Port_Config[i].port_num==3&&Port_Config[i].pin_num==7)))
   {
   *(volatile uint32*)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET)=MAGIC_UNLOCK_NUM;
     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Config[i].pin_num);   
     
}
else if ((Port_Config[i].port_num == 2) && (Port_Config[i].pin_num <= 3 ))
{
      
      continue ;  
      /* DO NOTHING FOR JTAG PINS */  
      
    } 
    else  {
      
      /* DO NOTHING */ 
      
    } 
   
   
if(Port_Config[i].mode==PORT_PIN_MODE_ADC)
{
  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET),Port_Config[i].pin_num); /*Disable Digital function*/
  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET),Port_Config[i].pin_num); /*Disable Alternative function*/
  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Config[i].pin_num);  /*Enable analog function*/
}


else if(Port_Config[i].mode == PORT_PIN_MODE_DIO){
       CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Config[i].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET),Port_Config[i].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Config[i].pin_num * 4));     /* Clear the PMCx bits for this pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET),Port_Config[i].pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
    }
 else{
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Config[i].pin_num);              /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((0x0000000F & Port_Config[i].mode) << (Port_Config[i].pin_num * 4));
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Config[i].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
    }         
if(Port_Config[i].direction==PORT_PIN_IN) 
{
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr +PORT_DIR_REG_OFFSET) , Port_Config[i].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */

}
           else if (Port_Config[i].direction==PORT_PIN_OUT)
           {
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr +PORT_DIR_REG_OFFSET) , Port_Config[i].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
           }

           else
           {
             /*do nothing*/
           }
}
           }
           
 
/****************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number.
*                  Direction - Port Pin direction.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the Port Pin direction.
****************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirection direction) {
#if(PORT_DEV_ERROR_DETECT == STD_ON) 
  if((Pin< MIN_PIN_NUM) || (Pin> MAX_PIN_NUM)){
    
    Det_ReportError(PORT_MODULE_ID , PORT_INSTANCE_ID , PORT_SET_PIN_DIRECTION_SID , PORT_E_PARAM_PIN  ) ; 
    
  } 
  else if (Port_Config[Pin].Dir_changable == STD_OFF) {
    
    Det_ReportError(PORT_MODULE_ID , PORT_INSTANCE_ID , PORT_SET_PIN_DIRECTION_SID , PORT_E_DIRECTION_UNCHANGEABLE ) ; 
    
  }  
  else if (Port_status == PORT_NOT_INITIALIZED) {
    Det_ReportError(PORT_MODULE_ID , PORT_INSTANCE_ID , PORT_SET_PIN_DIRECTION_SID , PORT_E_UNINIT ) ; 
    
  } 
  else
#endif
  { 
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0;
    /*
    * Switch case to determine the Base address of the Port from the attribute port_num in the struct ConfigPtr
    */
    switch(Port_Config[Pin].port_num)
    {
    case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    break;
    case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    break;
    case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    break;
    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /*PORTD Base Address */
    break;
    case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    break;
    case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    break;
    } 
    if(((Port_Config[Pin].port_num == 3) && (Port_Config[Pin].pin_num == 7)) || ((Port_Config[Pin].port_num == 5 ) && (Port_Config[Pin].pin_num == 0))) {
      
      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = MAGIC_UNLOCK_NUM;                     /* Unlock the GPIOCR register */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Config[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
      
      
    } 
    else if ( (Port_Config[Pin].port_num == 2) && (Port_Config[Pin].pin_num <= 3 ))  {
      return ; 
      /* DO NOTHING FOR JTAG PINS */  
      
    } 
    else  {
      
      /* DO NOTHING */ 
      
    }  
    if(Port_Config[Pin].direction == PORT_PIN_IN) {
      
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Config[Pin].pin_num);   
      
      
    } 
    else if (Port_Config[Pin].direction == PORT_PIN_OUT) {
      
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Config[Pin].pin_num);    
      
    } 
    else {
      
      /* DO NOTHING */ 
    }
  }
}
#endif          
  
/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]:0X02
* Sync/Async: Synchronous
* Reentrancy: nonreentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description:Refresh port direction

************************************************************************************/
 void Port_RefreshPortDirection( void )
 {
 #if(PORT_DEV_ERROR_DETECT == STD_ON) 
  if (Port_status == PORT_NOT_INITIALIZED)
  {
    Det_ReportError(PORT_MODULE_ID , PORT_INSTANCE_ID , PORT_REFRESH_PORT_DIR_SID, PORT_E_UNINIT ) ; 
    
  }    
#endif 

for( uint8 i=0;i<Port_CONFIGURED_PINS;i++)
{
   volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0; 
   switch (Port_Config[i].port_num)
   {
case 0: PortGpio_Ptr=(volatile uint32*)GPIO_PORTA_BASE_ADDRESS; 
            break;
case 1: PortGpio_Ptr=(volatile uint32*)GPIO_PORTB_BASE_ADDRESS;
            break;
case 2: PortGpio_Ptr=(volatile uint32*)GPIO_PORTC_BASE_ADDRESS;
            break;
case 3: PortGpio_Ptr=(volatile uint32*)GPIO_PORTD_BASE_ADDRESS; 
            break;
case 4: PortGpio_Ptr=(volatile uint32*)GPIO_PORTE_BASE_ADDRESS; 
             break;
case 5: PortGpio_Ptr=(volatile uint32*)GPIO_PORTF_BASE_ADDRESS; 
             break;
  SYSCTL_REGCGC2_REG |= (1<<Port_Config[i].port_num);
    delay =  SYSCTL_REGCGC2_REG ;  
   }
   if((Port_Config[i].port_num==5&&Port_Config[i].pin_num==0)||((Port_Config[i].port_num==3&&Port_Config[i].pin_num==7)))
   {
   *(volatile uint32*)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET)=MAGIC_UNLOCK_NUM;
     SET_BIT(*(volatile uint32*)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Config[i].pin_num);   
     
}
else if ((Port_Config[i].port_num == 2) && (Port_Config[i].pin_num <= 3 ))
{
      
      continue ;  
      /* DO NOTHING FOR JTAG PINS */  
      
    } 
    else  {
      
      /* DO NOTHING */ 
      
    }
   
 if(Port_Config[i].direction==PORT_PIN_IN) 
{
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr +PORT_DIR_REG_OFFSET) , Port_Config[i].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */

}
           else if (Port_Config[i].direction==PORT_PIN_OUT)
           {
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr +PORT_DIR_REG_OFFSET) , Port_Config[i].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
           }
           else
           {
             /*do nothing*/
           }  
 

}
 }
  /****************************
  * Service Name: Port_GetVersionInfo
  * Service ID[hex]: 0x03
  * Sync/Async: Synchronous
  * Reentrancy: Reentrant
  * Parameters (in): None
  * Parameters (inout): None
  * Parameters (out): VersionInfo - Pointer to where to store the version information of this module.
  * Return value: None
  * Description: Function to get the version information of this module.
  ****************************/  
#if (PORT_VERSION_INFO_API == STD_ON)
  void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
  {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is not Null pointer */
    if(NULL_PTR == versioninfo)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                      PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
    }
    else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
    {
      /* Copy the vendor Id */
      versioninfo->vendorID = (uint16)Port_VENDOR_ID;
      /* Copy the module Id */
      versioninfo->moduleID = (uint16)PORT_MODULE_ID;
      /* Copy Software Major Version */
      versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
      /* Copy Software Minor Version */
      versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
      /* Copy Software Patch Version */
      versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
  }
#endif  
  /************************************************************************************
  * Service Name: Port_SetPinMode
  * Service ID[hex]: 0x04
  * Sync/Async: Synchronous
  * Reentrancy: Reentrant
  * Parameters (in): Pin - Port Pin ID number.
  *                  Mode - New Port Pin mode to be set on port pin.
  * Parameters (inout): None
  * Parameters (out): None
  * Return value: None
  * Description: Sets the port pin mode.
  ************************************************************************************/   

  
#if (PORT_SET_PIN_MODE_API == STD_ON)
 void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode ) {
#if (PORT_DEV_ERROR_DETECT == STD_ON)  
    if((Pin< MIN_PIN_NUM) || (Pin> MAX_PIN_NUM)){
      
      Det_ReportError(PORT_MODULE_ID , PORT_INSTANCE_ID , PORT_SET_PIN_MODE_SID   , PORT_E_PARAM_PIN  ) ; 
      
    } 
    else {}
    if (Port_Config[Pin].Mode_changable == STD_OFF) {
      
      Det_ReportError(PORT_MODULE_ID , PORT_INSTANCE_ID ,  PORT_SET_PIN_MODE_SID    , PORT_E_MODE_UNCHANGEABLE ) ; 
      
    }  
    else {}
    if (Port_status == PORT_UNINITIALIZED) {
      Det_ReportError(PORT_MODULE_ID , PORT_INSTANCE_ID , PORT_SET_PIN_MODE_SID    , PORT_E_UNINIT ) ; 
      
    }
    else {}
    if (( Mode < MIN_MODE ) || (Mode>MAX_MODE)){
      Det_ReportError(PORT_MODULE_ID , PORT_INSTANCE_ID , PORT_SET_PIN_MODE_SID    , PORT_E_PARAM_INVALID_MODE  ) ; 
    }
    else {}
#endif
    
      volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
      volatile uint32 delay = 0;
      /*
      * Switch case to determine the Base address of the Port from the attribute port_num in the struct ConfigPtr
      */
      switch(Port_Config[Pin].port_num)
      {
      case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
      break;
      case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
      break;
      case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
      break;
      case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /*PORTD Base Address */
      break;
      case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
      break;
      case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
      break;
      } 
      if(((Port_Config[Pin].port_num == 3) && (Port_Config[Pin].pin_num == 7)) || ((Port_Config[Pin].port_num == 5 ) && (Port_Config[Pin].pin_num == 0))) {
        
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = MAGIC_UNLOCK_NUM;                     /* Unlock the GPIOCR register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Config[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
        
        
      } 
      else if ( (Port_Config[Pin].port_num == 2) && (Port_Config[Pin].pin_num <= 3 ))  {
        return ; 
        /* DO NOTHING FOR JTAG PINS */  
        
      } 
      else  {
        
        /* DO NOTHING */ 
        
      } 
      
      if(Port_Config[Pin].mode == PORT_PIN_MODE_ADC)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Config[Pin].pin_num);      /* SET the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Config[Pin].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Config[Pin].pin_num);             /* Disable Digitale Enable for this pin by clear the corresponding bit in GPIODEN register */
      }
      else if(Port_Config[Pin].mode == PORT_PIN_MODE_DIO){
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Config[Pin].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Config[Pin].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Config[Pin].pin_num * 4));     /* Clear the PMCx bits for this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Config[Pin].pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
      }
      else{
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Config[Pin].pin_num);              /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((0x0000000F & Port_Config[Pin].mode) << (Port_Config[Pin].pin_num * 4));
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Config[Pin].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
      }
  }
#endif
