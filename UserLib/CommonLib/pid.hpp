/*
 * pid.hpp
 *
 *  Created on: Jun 26, 2024
 *      Author: gomas
 */

#ifndef PID_HPP_
#define PID_HPP_

#include <float.h>
#include <algorithm>

namespace SabaneLib{

class PID{
private:
	const float pid_freq;
	float kp;
	float ki;
	float kd;
	float error_sum = 0.0f;
	float prev_error = 0.0f;

	//anti windup
	float k_anti_windup;
	float limit_min;
	float limit_max;
public:
	PID(float _pid_freq,float _kp,float _ki,float _kd, float _k_anti_windup,float _limit_min = -FLT_MAX,float _limit_max = FLT_MAX):
		pid_freq(_pid_freq),
		kp(_kp),
		ki(_ki/pid_freq),
		kd(_kd*pid_freq),
		k_anti_windup(_k_anti_windup),
		limit_min(_limit_min),
		limit_max(_limit_max){}

	float operator()(float target,float feedback){
		float error = target - feedback;
		float p = error * kp;

		error_sum += error;
		float i = error_sum * ki;

		float d = (error - prev_error) * kd;
		prev_error = error;

		float pid_result = p+i+d;
		float pid_result_clamped = std::clamp<float>(pid_result, limit_min, limit_max);

		error_sum -= (pid_result - pid_result_clamped)*k_anti_windup;

		return pid_result_clamped;
	}

	void set_gain(float _kp,float _ki,float _kd, float _k_anti_windup){
		kp = _kp;
		ki = _ki/pid_freq;
		kd = _kd*pid_freq;
		k_anti_windup = _k_anti_windup;
	}

	void set_gain(float _kp,float _ki,float _kd){
		kp = _kp;
		ki = _ki/pid_freq;
		kd = _kd*pid_freq;
		k_anti_windup = 1.0f/kp;
	}

	void set_p_gain(float _kp){
		kp = _kp;
	}

	void set_i_gain(float _ki){
		ki = _ki/pid_freq;
	}

	void set_d_gain(float _kd){
		kd = _kd*pid_freq;
	}

	void set_limit(float _limit_max){
		limit_min = -_limit_max;
		limit_max = _limit_max;
	}

	void set_limit(float _limit_min,float _limit_max){
		limit_min = _limit_min;
		limit_max = _limit_max;
	}

	void reset(void){
		error_sum = 0;
		prev_error = 0;
	}
};

class PIDBuilder{
public:
	float freq = 1.0f;
	float kp = 0;
	float ki = 0;
	float kd = 0;
	float k_anti_windup = 0;
	float limit_max = 0.0f;
	float limit_min = 0.0f;

	PIDBuilder(float _freq = 1.0f):freq(_freq){}

	PIDBuilder& set_gain(float _kp,float _ki,float _kd){
		kp = _kp;
		ki = _ki;
		kd = _kd;
		k_anti_windup = 1.0f/kp;

		return *this;
	}
	PIDBuilder& set_gain(float _kp,float _ki,float _kd, float _k_anti_windup){
		kp = _kp;
		ki = _ki;
		kd = _kd;
		k_anti_windup = _k_anti_windup;

		return *this;
	}
	PIDBuilder& set_limit(float _limit_min,float _limit_max){
		limit_max = _limit_max;
		limit_min = _limit_min;
		return *this;
	}
	PIDBuilder& set_limit(float _limit_max){
		limit_max =  _limit_max;
		limit_min = -_limit_max;
		return *this;
	}
	PID build()const{
		return PID{freq,kp,ki,kd,k_anti_windup,limit_min,limit_max};
	}
};

}



#endif /* PID_HPP_ */
