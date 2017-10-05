/**
	\file
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions and runtime functions.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	7/09/2014
	\todo
	    Interrupts are not implemented in this API implementation.
 */
#include "MK64F12.h"
#include "GPIO.h"

static GPIO_interruptFlags_t GPIO_intrStatusFlag;

void PORTC_IRQHandler(){
	flag_c = TRUE;
	GPIO_intrStatusFlag.flagPortC = TRUE;
	GPIO_clearInterrupt(GPIO_C);
}


void PORTA_IRQHandler(){
	GPIO_intrStatusFlag.flagPortA  = TRUE;
	GPIO_clearInterrupt(GPIO_A);
}

uint8 GPIO_getIRQStatus(GPIO_portNameType gpio)
{
	switch (gpio) {
		case GPIO_A:
			return(GPIO_intrStatusFlag.flagPortA);
			break;
		case GPIO_B:
			return(GPIO_intrStatusFlag.flagPortB);
			break;
		case GPIO_C:
			return(GPIO_intrStatusFlag.flagPortC);
			break;
		case GPIO_D:
			return(GPIO_intrStatusFlag.flagPortD);
			break;
		case GPIO_E:
			return(GPIO_intrStatusFlag.flagPortE);
			break;
		default:
			return(ERROR);
			break;
	}

}

uint8 GPIO_clearIRQStatus(GPIO_portNameType gpio)
{
	switch (gpio) {
		case GPIO_A:
			GPIO_intrStatusFlag.flagPortA = FALSE;
			break;
		case GPIO_B:
			GPIO_intrStatusFlag.flagPortB = FALSE;
			break;
		case GPIO_C:
			GPIO_intrStatusFlag.flagPortC = FALSE;
			break;
		case GPIO_D:
			GPIO_intrStatusFlag.flagPortD = FALSE;
			break;
		case GPIO_E:
			GPIO_intrStatusFlag.flagPortE = FALSE;
			break;
		default:
			return(ERROR);
			break;
	}
	return(TRUE);
}


void GPIO_clearInterrupt(GPIO_portNameType portName)
{
	switch(portName)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			PORTA->ISFR=0xFFFFFFFF;
			break;
		case GPIO_B: /** GPIO B is selected*/
			PORTB->ISFR=0xFFFFFFFF;
			break;
		case GPIO_C: /** GPIO C is selected*/
			PORTC->ISFR = 0xFFFFFFFF;
			break;
		case GPIO_D: /** GPIO D is selected*/
			PORTD->ISFR=0xFFFFFFFF;
			break;
		default: /** GPIO E is selected*/
			PORTE->ISFR=0xFFFFFFFF;
			break;
	}// end switch
}

uint8 GPIO_clockGating(GPIO_portNameType portName)
{
	switch(portName)/** Selecting the GPIO for clock enabling*/
			{
				case GPIO_A: /** GPIO A is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
					break;
				case GPIO_B: /** GPIO B is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
					break;
				case GPIO_C: /** GPIO C is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
					break;
				case GPIO_D: /** GPIO D is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
					break;
				case GPIO_E: /** GPIO E is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
					break;
				default: /**If doesn't exist the option*/
					return(FALSE);
			}// end switch
	/**Successful configuration*/
	return(TRUE);
}// end function

uint8 GPIO_pinControlRegister(GPIO_portNameType portName,uint8 pin,const GPIO_pinControlRegisterType*  pinControlRegister)
{
	switch(portName)
		{
		case GPIO_A:/** GPIO A is selected*/
			PORTA->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_B:/** GPIO B is selected*/
			PORTB->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_C:/** GPIO C is selected*/
			PORTC->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_D:/** GPIO D is selected*/
			PORTD->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->PCR[pin]= *pinControlRegister;
		default:/**If doesn't exist the option*/
			return(FALSE);
		break;
		}
	/**Successful configuration*/
	return(TRUE);
}

void GPIO_writePORT(GPIO_portNameType portName, uint32 Data ){
	switch(portName){
		case GPIO_A: /** GPIO A is selected*/
			GPIOA->PDOR = Data;
			break;
		case GPIO_B: /** GPIO B is selected*/
			GPIOB->PDOR = Data;
			break;
		case GPIO_C: /** GPIO C is selected*/
			GPIOC->PDOR = Data;
			break;
		case GPIO_D: /** GPIO D is selected*/
			GPIOD->PDOR = Data;
			break;
		default: /** GPIO E is selected*/
			GPIOE->PDOR = Data;
			break;
	}
}
uint32 GPIO_readPORT(GPIO_portNameType portName){
	switch (portName) {
		case GPIO_A:
			return GPIOA->PDIR;
			break;
		case GPIO_B:
			return GPIOB->PDIR;
			break;
		case GPIO_C:
			return GPIOC->PDIR;
			break;
		case GPIO_D:
			return GPIOD->PDIR;
			break;
		default:
			return GPIOE->PDIR;
			break;
	}
}

uint8 GPIO_readPIN(GPIO_portNameType portName, uint8 pin){
	switch(portName)
			{
			case GPIO_A:/** GPIO A is selected*/
				return (((GPIOA->PDIR) >> pin) & 0x1);
				break;
			case GPIO_B:/** GPIO B is selected*/
				return (((GPIOB->PDIR) >> pin) & 0x1);
				break;
			case GPIO_C:/** GPIO C is selected*/
				return (((GPIOC->PDIR) >> pin) & 0x1);
				break;
			case GPIO_D:/** GPIO D is selected*/
				return (((GPIOD->PDIR) >> pin) & 0x1);
				break;
			case GPIO_E: /** GPIO E is selected*/
				return (((GPIOE->PDIR) >> pin) & 0x1);
			default:
				return FALSE;
				break;
			}
}

void GPIO_setPIN(GPIO_portNameType portName, uint8 pin){
	switch(portName){
		case GPIO_A:
		GPIOA->PSOR = (GPIOA->PDOR | (0x1 << pin));
			break;
		case GPIO_B:
			GPIOB->PSOR = (GPIOB->PDOR | (0x1 << pin));
			break;
		case GPIO_C:
			GPIOC->PSOR = (GPIOC->PDOR | (0x1 << pin));
			break;
		case GPIO_D:
			GPIOD->PSOR = (GPIOD->PDOR | (0x1 << pin));
			break;
		case GPIO_E:
			GPIOE->PSOR = (GPIOE->PDOR | (0x1 << pin));
			break;
		default:
			break;
	}
}

void GPIO_clearPIN(GPIO_portNameType portName, uint8 pin){
	switch(portName){
		case GPIO_A:
		GPIOA->PCOR = (GPIOA->PDOR | (0x1 << pin));
			break;
		case GPIO_B:
			GPIOB->PCOR = (GPIOB->PDOR | (0x1 << pin));
			break;
		case GPIO_C:
			GPIOC->PCOR = (GPIOC->PDOR | (0x1 << pin));
			break;
		case GPIO_D:
			GPIOD->PCOR = (GPIOD->PDOR | (0x1 << pin));
			break;
		case GPIO_E:
			GPIOE->PCOR = (GPIOE->PDOR | (0x1 << pin));
			break;
		default:
			break;
	}
}

void GPIO_tooglePIN(GPIO_portNameType portName, uint8 pin){
	switch(portName){
	case GPIO_A:
		GPIOA->PTOR = (0x1 << pin);
		break;
	case GPIO_B:
		GPIOB->PTOR =(0x1 << pin);
		break;
	case GPIO_C:
		GPIOC->PTOR = (0x1 << pin);
		break;
	case GPIO_D:
		GPIOD->PTOR= (0x1 << pin);
		break;
	case GPIO_E:
		GPIOE->PTOR = (0x1 << pin);
		break;
	default:
		break;
	}
}

void GPIO_dataDirectionPORT(GPIO_portNameType portName ,uint32 direction){
	switch(portName){
		case GPIO_A:
			GPIOA->PDDR = direction;
			break;
		case GPIO_B:
			GPIOB->PDDR = direction;
			break;
		case GPIO_C:
			GPIOC->PDDR = direction;
			break;
		case GPIO_D:
			GPIOD->PDDR = direction;
			break;
		case GPIO_E:
			GPIOE->PDDR = direction;
			break;
		default:
			break;
	}
}

void GPIO_dataDirectionPIN(GPIO_portNameType portName, uint8 State, uint8 pin){
	switch(portName){
			case GPIO_A:
				if(State){
					GPIOA->PDDR |= (0x1 << pin);
				}else{
					GPIOA->PDDR &= ~(0x1 << pin);
				}
				break;
			case GPIO_B:
				if(State){
					GPIOB->PDDR |= (0x1 << pin);
				}else{
					GPIOB->PDDR &= ~(0x1 << pin);
				}
				break;
			case GPIO_C:
				if(State){
					GPIOC->PDDR |= (0x1 << pin);
				}else{
					GPIOC->PDDR &= ~(0x1 << pin);
				}
				break;
			case GPIO_D:
				if(State){
					GPIOD->PDDR |= (0x1 << pin);
				}else{
					GPIOD->PDDR &= ~(0x1 << pin);
				}
				break;
			case GPIO_E:
				if(State){
					GPIOE->PDDR |= (0x1 << pin);
				}else{
					GPIOE->PDDR &= ~(0x1 << pin);
				}
				break;
			default:
				break;
		}
}
