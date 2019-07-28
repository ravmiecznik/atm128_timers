/*
 * timer_r.cpp
 *
 *  Created on: 18 maj 2018
 *      Author: rafal
 */
#include "timers_r.h"
#include <avr/interrupt.h>
#include "../avr_ports/avr_ports.h"
#include "../common.h"


#ifdef TIMER1
volatile uint32_t TOI_COUNTER_TIMER1 = 0;

void (*TOI_ISR_FUNCTION_POINTER_TIMER1)(void) = null_function;
void (*T1COMPA_FUNC_GLOB_PTR)(void) = null_function;
void (*T1COMPB_FUNC_GLOB_PTR)(void) = null_function;
void (*T1COMPC_FUNC_GLOB_PTR)(void) = null_function;
uint16_t COMPA_PERIOD_TIMER1=0;
uint16_t COMPB_PERIOD_TIMER1=0;
uint16_t COMPC_PERIOD_TIMER1=0;

ISR(TIMER1_COMPA_vect){
	T1COMPA_FUNC_GLOB_PTR();
	if(COMPA_PERIOD_TIMER1){
		OCR1A = TCNT1 + COMPA_PERIOD_TIMER1;
	}
}
ISR(TIMER1_COMPB_vect){
	T1COMPB_FUNC_GLOB_PTR();
	if(COMPB_PERIOD_TIMER1){
		OCR1B = TCNT1 + COMPB_PERIOD_TIMER1;
	}
}
ISR(TIMER1_COMPC_vect){
	T1COMPC_FUNC_GLOB_PTR();
	if(COMPC_PERIOD_TIMER1){
		OCR1C = TCNT1 + COMPC_PERIOD_TIMER1;
	}
}
ISR(TIMER1_OVF_vect){
	TOI_COUNTER_TIMER1 += 1;
};
#endif


#ifdef TIMER3
volatile uint32_t TOI_COUNTER_TIMER3 = 0;

void null_function(){

}

void (*TOI_ISR_FUNCTION_POINTER_TIMER3)(void) = null_function;
void (*T3COMPA_FUNC_GLOB_PTR)(void) = null_function;
void (*T3COMPB_FUNC_GLOB_PTR)(void) = null_function;
void (*T3COMPC_FUNC_GLOB_PTR)(void) = null_function;
uint16_t COMPA_PERIOD_TIMER3=0;
uint16_t COMPB_PERIOD_TIMER3=0;
uint16_t COMPC_PERIOD_TIMER3=0;

ISR(TIMER3_COMPA_vect){
	T3COMPA_FUNC_GLOB_PTR();
	if(COMPA_PERIOD_TIMER3){
		OCR3A = TCNT3 + COMPA_PERIOD_TIMER3;
	}
}
ISR(TIMER3_COMPB_vect){
	T3COMPB_FUNC_GLOB_PTR();
	if(COMPB_PERIOD_TIMER3){
		OCR3B = TCNT3 + COMPB_PERIOD_TIMER3;
	}
}
ISR(TIMER3_COMPC_vect){
	T3COMPC_FUNC_GLOB_PTR();
	if(COMPC_PERIOD_TIMER3){
		OCR3C = TCNT3 + COMPC_PERIOD_TIMER3;
	}
}
ISR(TIMER3_OVF_vect){
	TOI_COUNTER_TIMER3 += 1;
};
#endif


Timer1::Timer1(timer_prescaler prescaler, timer_mode mode):Timer(TCCR1A, TCCR1B, TCCR1C, OCR1A, OCR1B, OCR1C, ICR1, TCNT1,
		COMPA_PERIOD_TIMER1, COMPB_PERIOD_TIMER1, COMPC_PERIOD_TIMER1, TOI_COUNTER_TIMER1){
	((tccrb_register&)tccrb).cs0 = ((prescaler_bits&)prescaler).cs0;
	((tccrb_register&)tccrb).cs1 = ((prescaler_bits&)prescaler).cs1;
	((tccrb_register&)tccrb).cs2 = ((prescaler_bits&)prescaler).cs2;

	((tccra_register&)tccra).wgm0 = ((wgm_bits&)mode).wgm0;
	((tccra_register&)tccra).wgm1 = ((wgm_bits&)mode).wgm1;
	((tccrb_register&)tccrb).wgm2 = ((wgm_bits&)mode).wgm2;
	((tccrb_register&)tccrb).wgm3 = ((wgm_bits&)mode).wgm3;
	_prescaler = get_prescaler(prescaler);
	overflow_interrupt_enable();
}

void Timer1::compa_interrupt_enable(void (*compa_function)(void), uint16_t compa_value, uint16_t period_ms){
	/*
	 * (*cmpa_function): pointer to Interrupt Routine to be executed, must return void and take no args
	 * compa_value: comparator match initial value
	 * period: a period of ISR execution, 0 means execution every compa_value match
	 */
	compa_kick = period_ms;
	ocra = compa_value;
	T1COMPA_FUNC_GLOB_PTR = compa_function;
	((timsk_register&)timsk).ocie1a = 1;
}

void Timer1::compb_interrupt_enable(void (*compb_function)(void), uint16_t compb_value, uint16_t period_ms){
	/*
	 * (*cmpa_function): pointer to Interrupt Routine to be executed, must return void and take no args
	 * compa_value: comparator match initial value
	 * period: a period of ISR execution, 0 means execution every compa_value match
	 */
	compb_kick = period_ms;
	ocrb = compb_value;
	T1COMPB_FUNC_GLOB_PTR = compb_function;
	((timsk_register&)timsk).ocie1b = 1;
}


Timer3::Timer3(timer_prescaler prescaler, timer_mode mode):Timer(TCCR3A, TCCR3B, TCCR3C, OCR3A, OCR3B, OCR3C, ICR3, TCNT3,
		COMPA_PERIOD_TIMER3, COMPB_PERIOD_TIMER3, COMPC_PERIOD_TIMER3, TOI_COUNTER_TIMER3){
	((tccrb_register&)tccrb).cs0 = ((prescaler_bits&)prescaler).cs0;
	((tccrb_register&)tccrb).cs1 = ((prescaler_bits&)prescaler).cs1;
	((tccrb_register&)tccrb).cs2 = ((prescaler_bits&)prescaler).cs2;

	((tccra_register&)tccra).wgm0 = ((wgm_bits&)mode).wgm0;
	((tccra_register&)tccra).wgm1 = ((wgm_bits&)mode).wgm1;
	((tccrb_register&)tccrb).wgm2 = ((wgm_bits&)mode).wgm2;
	((tccrb_register&)tccrb).wgm3 = ((wgm_bits&)mode).wgm3;
	_prescaler = get_prescaler(prescaler);
	overflow_interrupt_enable();
}

void Timer3::compa_interrupt_enable(void (*compa_function)(void), uint16_t compa_value, uint16_t period_ms){
	/*
	 * (*cmpa_function): pointer to Interrupt Routine to be executed, must return void and take no args
	 * compa_value: comparator match initial value
	 * period: a period of ISR execution, 0 means execution every compa_value match
	 */
	compa_kick = period_ms;
	ocra = compa_value;
	T3COMPA_FUNC_GLOB_PTR = compa_function;
	((etimsk_register&)etimsk).ocie3a = 1;
}

void Timer3::compb_interrupt_enable(void (*compb_function)(void), uint16_t compb_value, uint16_t period_ms){
	/*
	 * (*cmpa_function): pointer to Interrupt Routine to be executed, must return void and take no args
	 * compa_value: comparator match initial value
	 * period: a period of ISR execution, 0 means execution every compa_value match
	 */
	compb_kick = period_ms;
	ocrb = compb_value;
	T3COMPB_FUNC_GLOB_PTR = compb_function;
	((etimsk_register&)etimsk).ocie3b = 1;
}

uint32_t cycles_to_ms(uint32_t cycles, uint16_t _prescaler){
	/*
	 * Remember about TCNT overflow !
	 * Max possible time in ms for prescalers and F_CPU=14745600:
	 * pre 1   -> 4 ms
	 * pre 8   -> 35 ms
	 * pre 64  -> 284 ms
	 * pre 256 -> 1137ms
	 * pre 1024-> 4551ms
	 *
	 */
	//uint32_t div_factor = 1000;
	uint32_t f_cpu = (F_CPU/_prescaler);
	return 1000ULL*cycles/f_cpu;
}

uint32_t cycles_to_us(uint32_t cycles, uint16_t _prescaler){
	/*
	 * Remember about TCNT overflow !
	 * Max possible time in ms for prescalers and F_CPU=14745600:
	 * pre 1   -> 4 ms
	 * pre 8   -> 35 ms
	 * pre 64  -> 284 ms
	 * pre 256 -> 1137ms
	 * pre 1024-> 4551ms
	 *
	 */
	uint32_t f_cpu = (F_CPU/_prescaler);
	return (1000000ULL*cycles/f_cpu);
}

uint32_t cycles_to_ns(uint32_t cycles, uint16_t _prescaler){
	/*
	 * Remember about TCNT overflow !
	 * Max possible time in ms for prescalers and F_CPU=14745600:
	 * pre 1   -> 4 ms
	 * pre 8   -> 35 ms
	 * pre 64  -> 284 ms
	 * pre 256 -> 1137ms
	 * pre 1024-> 4551ms
	 *
	 */
	uint32_t f_cpu = (F_CPU/_prescaler);
	return (1000000000ULL*cycles/f_cpu);
}

uint16_t ms_to_cycles(uint16_t ms, uint16_t _prescaler){
	uint32_t cycles = ms*(F_CPU/1000ULL)/_prescaler;
	assert(cycles <= 0xffffULL);
	return (uint16_t)cycles;
}

uint16_t Timer::get_prescaler(timer_prescaler prescaler){
	uint16_t prescaler_map[8];
	prescaler_map[zero] = 0;
	prescaler_map[clk_div_1] = 1;
	prescaler_map[clk_div_8] = 8;
	prescaler_map[clk_div_64] = 64;
	prescaler_map[clk_div_256] = 256;
	prescaler_map[clk_div_1024] = 1024;
	prescaler_map[external_rising] = 0;
	prescaler_map[external_falling] = 0;
	return prescaler_map[prescaler];
}

uint8_t Timer::tccrb_get(){
	return tccrb;
}

