/*
 * timers_r.h
 *
 *  Created on: 18 maj 2018
 *      Author: rafal
 */

#ifndef TIMERS_R_H_
#define TIMERS_R_H_
#include <avr/io.h>
#include <stdlib.h>


typedef volatile uint8_t& reg8b_ref;
typedef volatile uint16_t& reg16b_ref;

//Globals for interrupts and interrputs definitions
#define TIMER1
#define TIMER3

struct time_stamp{
	uint32_t cycles;
	uint32_t ovf_cnt;
	time_stamp & operator - (time_stamp tstamp){
		cycles -= tstamp.cycles;
		ovf_cnt -= tstamp.ovf_cnt;
		return *this;
	}
	operator uint32_t(){
		return ovf_cnt*65535 + cycles;
	}
};

uint32_t cycles_to_ms(uint32_t cycles, uint16_t _prescaler);
uint32_t cycles_to_us(uint32_t cycles, uint16_t _prescaler);
uint32_t cycles_to_ns(uint32_t cycles, uint16_t _prescaler);
uint32_t t_stamp_to_ms(uint16_t cycles, uint16_t _prescaler);
uint16_t ms_to_cycles(uint16_t ms, uint16_t _prescaler);

#define REG_DECODE(reg_bits, reg)	((reg_bits&)reg)

	enum{
		normal_mode_OvCnt_disconnected,
		toggle_OvCnt_on_compare_match,
		clear_OvCnt_on_compare_match,
		set_OvCnt_on_compare_match,
	}compare_match;

	typedef enum{
		zero,
		clk_div_1,
		clk_div_8,
		clk_div_64,
		clk_div_256,
		clk_div_1024,
		external_rising,
		external_falling,
	}timer_prescaler;

	typedef enum{
		mode_normal,
		pwm_phase_correct_8bit,
		pwm_phase_correct_9bit,
		pwm_phase_correct_10bit,
		mode_ctc_top_ocra ,
		fast_pwm_8bit,
		fast_pwm_9bit,
		fast_pwm_10bit,
		pwm_phase_and_freq_correct_top_icr,
		pwm_phase_and_freq_correct_top_ocra,
		pwm_phase_correct_top_icr,
		pwm_phase_correct_top_ocra,
		mode_ctc_top_icr,
		reserved,
		fast_pwm_top_icr,
		fast_pwm_top_ocra,
	}timer_mode;

	struct tccra_register{
		union{
			uint8_t tccra;
			struct {
				bool wgm0  :1;
				bool wgm1  :1;
				bool comc0 :1;
				bool comc1 :1;
				bool comb0 :1;
				bool comb1 :1;
				bool coma0 :1;
				bool coma1 :1;
			};
		};
	};

	struct timsk_register{
		union{
			uint8_t timsk;
			struct {
				bool toie0  :1; //timer overflow interrupt enable
				bool ocie0  :1; //output compare match interrupt enable
				bool toie1  :1; //timer overflow interrupt enable
				bool ocie1b :1; //output compare match interrupt enable
				bool ocie1a :1;	//output compare match interrupt enable
				bool ticie1 :1;	//input capture interrupt enable
				bool toie2  :1;	//timer overflow interrupt enable
				bool ocie2  :1;	//output compare match interrupt enable
			};
		};
	};

	struct etimsk_register{
		union{
			uint8_t etimsk;
			struct {
				bool ocie1c :1;//output compare match interrupt enable
				bool ocie3c :1;//output compare match interrupt enable
				bool toie3  :1;	//timer overflow interrupt enable
				bool ocie3b :1;	//output compare match interrupt enable
				bool ocie3a :1; //output compare match interrupt enable
				bool ticie3 :1;//input capture interrupt enable
				bool   		:1; //not used
				bool   		:1; //not used
			};
		};
	};

	struct tccrb_register{
		union{
			uint8_t tccrb;
			struct{
				bool cs0  :1;
				bool cs1  :1;
				bool cs2  :1;
				bool wgm2 :1;
				bool wgm3 :1;
				bool ices :1;
				bool icn  :1;
			};
		};
	};

	struct prescaler_bits{
		union{
			uint8_t cs;
			struct{
				bool cs0:1;
				bool cs1:1;
				bool cs2:1;
			};
		};
	};

	struct compare_match_bits{
		union{
			uint8_t cm;
			struct{
				bool cm0:1;
				bool cm1:1;
			};
		};
	};

	struct wgm_bits{
		union{
		uint8_t wgm;
		struct{
			bool wgm0:1;
			bool wgm1:1;
			bool wgm2:1;
			bool wgm3:1;
			};
		};
		wgm_bits(uint8_t w): wgm(w) {};
	};



class Timer{
public:
	reg8b_ref tccra;
	reg8b_ref tccrb;
	reg8b_ref tccrc;
	reg16b_ref ocra;
	reg16b_ref ocrb;
	reg16b_ref ocrc;
	reg16b_ref icr;
	reg16b_ref tcnt;

	//comp a b c registers and timers
	uint16_t& compa_kick;
	uint16_t& compb_kick;
	uint16_t& compc_kick;

	//common registers
	//reg8b_ref timsk = TIMSK;
	//reg8b_ref etimsk = ETIMSK;
	reg8b_ref tifr = TIFR;
	reg8b_ref etifr = ETIFR;

	volatile uint32_t& overflow_cnt;
	uint16_t _prescaler;
	time_stamp _tic;
	time_stamp t_stamp;
	Timer(reg8b_ref tccra, reg8b_ref tccrb, reg8b_ref tccrc, reg16b_ref ocra, reg16b_ref ocrb, reg16b_ref ocrc, reg16b_ref icr, reg16b_ref tcnt,
			uint16_t& compa_kick, uint16_t& compb_kick, uint16_t& compc_kick, volatile uint32_t& overflow_cnt):
		tccra(tccra), tccrb(tccrb), tccrc(tccrc), ocra(ocra), ocrb(ocrb), ocrc(ocrc), icr(icr), tcnt(tcnt), overflow_cnt(overflow_cnt),
		compa_kick(compa_kick), compb_kick(compb_kick), compc_kick(compc_kick){
		_tic = {0, 0};
		_prescaler = 0;
		t_stamp = {0, 0};
	};
	uint16_t get_prescaler(timer_prescaler);
	uint8_t tccrb_get();
	uint16_t ms(uint32_t cycles){
		return ::cycles_to_ms(cycles, _prescaler);
	}
	uint16_t get_toi_counter(){
		return overflow_cnt;
	}
	void tic(){
		_tic = {tcnt, overflow_cnt};
	}
	uint32_t toc(){
		return  (time_stamp){tcnt, overflow_cnt} - _tic;
	}
	uint16_t toc_ms(){
		return ::cycles_to_ms(toc(), _prescaler);
	}
	uint16_t toc_us(){
		return ::cycles_to_us(toc(), _prescaler);
	}
	uint16_t toc_ns(){
		return ::cycles_to_ns(toc(), _prescaler);
	}
	uint32_t tstamp_ms(){
		return ::cycles_to_ms(get_toi_counter()*65535 + tcnt, _prescaler);
	}
	uint32_t tstamp_us(){
		return ::cycles_to_us(get_toi_counter()*65535 + tcnt, _prescaler);
	}
	uint16_t tcnt_get(){
		return tcnt;
	}
	uint16_t ms_to_cycles(uint16_t ms){
		return ::ms_to_cycles(ms, _prescaler);
	}
	uint16_t cycles_to_ms(uint16_t cycles){
		return ::cycles_to_ms(cycles, _prescaler);
	}
	uint16_t cycles_to_us(uint16_t cycles){
		return ::cycles_to_us(cycles, _prescaler);
	}
	uint16_t cycles_to_ns(uint16_t cycles){
		return ::cycles_to_ns(cycles, _prescaler);
	}
};

uint32_t test32(uint32_t val);

class Timer1: Timer
{
public:
	using Timer::tccrb_get;
	using Timer::tstamp_ms;
	using Timer::tstamp_us;
	using Timer::tic;
	using Timer::toc;
	using Timer::toc_ms;
	using Timer::toc_us;
	using Timer::toc_ns;
	using Timer::tcnt_get;
	using Timer::ms_to_cycles;
	using Timer::cycles_to_ms;
	using Timer::cycles_to_us;
	using Timer::cycles_to_ns;
	reg8b_ref timsk = TIMSK;
	Timer1(timer_prescaler prescaler, timer_mode=mode_normal);
	void overflow_interrupt_enable(){
		((timsk_register&)timsk).toie1 = 1;
	}
	void overflow_interrupt_disable(){
		((timsk_register&)timsk).toie1 = 0;
	}
	void compa_interrupt_enable(void (*compa_function)(void), uint16_t compa_value, uint16_t perdiod_ms = 0);
	void compa_interrupt_disable(){
		((timsk_register&)timsk).ocie1a = 0;
	}
	void compb_interrupt_enable(void (*compb_function)(void), uint16_t compb_value, uint16_t perdiod_ms = 0);
	void compb_interrupt_disable(){
		((timsk_register&)timsk).ocie1b = 0;
	}
};

class Timer3: Timer
{
public:
	using Timer::tccrb_get;
	using Timer::tstamp_ms;
	using Timer::tstamp_us;
	using Timer::tic;
	using Timer::toc;
	using Timer::toc_ms;
	using Timer::toc_us;
	using Timer::toc_ns;
	using Timer::tcnt_get;
	using Timer::ms_to_cycles;
	using Timer::cycles_to_ms;
	using Timer::cycles_to_us;
	using Timer::cycles_to_ns;
	reg8b_ref etimsk = ETIMSK;
	Timer3(timer_prescaler prescaler, timer_mode=mode_normal);
	void overflow_interrupt_enable(){
		((etimsk_register&)etimsk).toie3 = 1;
	}
	void overflow_interrupt_disable(){
		((etimsk_register&)etimsk).toie3 = 0;
	}
	void compa_interrupt_enable(void (*compa_function)(void), uint16_t compa_value, uint16_t perdiod_ms = 0);
	void compa_interrupt_disable(){
		((etimsk_register&)etimsk).ocie3a = 0;
	}
	void compb_interrupt_enable(void (*compb_function)(void), uint16_t compb_value, uint16_t perdiod_ms = 0);
	void compb_interrupt_disable(){
		((etimsk_register&)etimsk).ocie3b = 0;
	}
};



#endif /* TIMERS_R_H_ */
