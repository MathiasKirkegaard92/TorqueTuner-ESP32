#include "haptics.h"


HapticKnob::HapticKnob() {
	// active_mode = mode_list[0];
	num_modes = mode_list.size();
};

void HapticKnob::update() {

	update_angle();

	// Filter and gate velocity for output
	velocity_out = gate(filter(velocity), 5, 0);
	calc_acceleration(velocity_out);

	// Calculate index to transfer function
	active_mode->calc_index(this);

	// // Calculate trigger and discrete output
	float resolution = active_mode->max / stretch;

	// float fraction = angle_out - angle_discrete;
	// if (fraction >= resolution) {
	// 	angle_discrete += resolution;
	// 	// angle_discrete = angle_out;
	// 	update_trig();
	// 	printf("%i, %i \n", angle_out, angle_discrete);
	// } else if (fraction <= -resolution) {
	// 	angle_discrete -= resolution;
	// 	// angle_discrete = angle_out;
	// 	update_trig();
	// 	printf("%i, %i \n", angle_out, angle_discrete);

	// }



	angle_discrete = round(round(angle_out / resolution) * resolution);
	if (abs(angle_discrete - angle_discrete_last) >= resolution) {
		update_trig();
		angle_discrete_last = angle_discrete;
	};

	torque = static_cast<int16_t>(active_mode->calc(this) - active_mode->damping * velocity);
	// printf("%f %i \n", velocity, torque);
};

void HapticKnob::update_angle() {

	// Wrap angle
	angle_delta = angle - angle_last;
	if ((angle_delta) < -1800) {
		wrap_count += 1;
		angle_delta += 3600;
	} else if ((angle_delta) > 1800) {
		wrap_count -= 1;
		angle_delta -= 3600;
	};
	angle_last = angle;

	angle_unclipped += angle_delta;

	// Clip with parameter range
	angle_out += angle_delta;
	if (active_mode->wrap_output) {
		angle_out = mod(angle_out, 3600); // CHANGE to min, max
	} else {
		angle_out = static_cast<int32_t>(clip(angle_out, active_mode->min, active_mode->max));
		// angle_delta = angle_out - angle_out_last;
	}

	// angle_out_last = angle_out;
};


void HapticKnob::set_mode(int mode_idx) {
	if (active_mode != mode_list[mode_idx]) {
		active_mode = mode_list[mode_idx];
		active_mode->reset(angle);
		angle_last = angle;
		angle_out = 0;
		angle_unclipped = 0;
		set_defaults(active_mode);

		// Init discrete angle
		float resolution = active_mode->max / stretch;
		angle_discrete = floor(floor(angle_out / resolution) * resolution);
		print_mode(static_cast<MODE>(mode_idx));
	}
};


void HapticKnob::set_mode(MODE mode_) {
	int mode_idx = static_cast<int>(mode_);
	if (active_mode != mode_list[mode_idx]) {
		active_mode = mode_list[mode_idx];
		active_mode->reset(angle);
		angle_last = angle;
		angle_out = 0;
		angle_unclipped = 0;
		set_defaults(active_mode);


		// // Init discrete angle
		// float resolution = active_mode->max / stretch;
		// angle_discrete = floor(floor(angle_out / resolution) * resolution);

		print_mode(static_cast<MODE>(mode_idx));
	}
};

void HapticKnob::set_stretch(float stretch_) {
	if (abs(stretch_ - stretch) > 0.1) {
		stretch = stretch_;
	}
};

void HapticKnob::set_defaults(Mode * mode) {
	scale = mode->scale_default;
	stretch = mode->stretch_default;
	// printf("angle scale set to %i \n", stretch );
}

void HapticKnob::print_mode(MODE mode_) {
	printf("Switched mode to : \n");
	switch (mode_) {
	case CLICK:
		printf("Click \n");
		break;
	case INERTIA:
		printf("Inertia\n");
		break;
	case WALL:
		printf("Wall\n");
		break;
	case MAGNET:
		printf("Magnet\n");
		break;
	case LINSPRING:
		printf("Linear Spring \n");
		break;
	case EXPSPRING:
		printf("Exponential Spring \n");
		break;
	case FREE_TORQUE:
		printf("Free torque \n");
		break;
	case SPIN:
		printf("Spin \n");
		break;
	}
};

float HapticKnob::calc_acceleration(float velocity_) {
	static float last = 0;
	acceleration = last - velocity_;
	last = velocity_;
	return acceleration;
};

float HapticKnob::filter(float x) {
	// Cannonical form - https://ccrma.stanford.edu/~jos/filters/Direct_Form_II.html
	// w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
	// y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
	static float w[3];
	w[0] = x - a[1] * w[1] - a[2] * w[2];
	x = b[0] * w[0] + b[1] * w[1] + b[2] * w[2];
	w[2] = w[1];
	w[1] = w[0];
	return x;
};


float HapticKnob::gate(float val, float threshold, float floor) {
	return abs(val) > threshold ? val : floor;
};

void HapticKnob::update_trig() {
	trigger++;
	trigger = fold(trigger, 0, 1);
};

int32_t HapticKnob::getTime() {
	return esp_timer_get_time();
}

int16_t Wall::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	float val = 0;
	if (knob->angle_unclipped <= min) {
		val = 1;
	} else if (knob->angle_unclipped >= max) {
		val = -1;
	}
	val *= knob->scale;
	return static_cast<int16_t> (round(val));
};

int16_t Click::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	float val;
	if (knob->angle_out <= min) {
		val = WALL_TORQUE;
	} else if (knob->angle_out >= max) {
		val = -WALL_TORQUE;
	} else {
		val = static_cast<float>((tf_click[idx])) / TABLE_RESOLUTION * knob->scale;
	}
	return static_cast<int16_t> (round(val));
};

int16_t Magnet::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	return static_cast<float>(tf_magnet[idx]) / TABLE_RESOLUTION * knob->scale; // Magnet
};

int16_t Inertia::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	if (knob->velocity > 0) {
		return round((- (knob->angle_out / 3600.0) * knob->velocity) * knob->scale / MAX_VELOCITY);
	} else {
		return 0;
	}
};

int16_t LinSpring::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	float val = - (knob->angle_out - 1800) / 1800.0;
	if (knob->angle_unclipped <= min) {
		val = 1;
	} else if (knob->angle_unclipped >= max) {
		val = -1;
	}
	val *= knob->scale;
	return static_cast<int16_t> (round(val));
};

int16_t ExpSpring::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	float val = static_cast<float>(tf_wall[idx]) / TABLE_RESOLUTION;

	if (knob->angle_unclipped <= min) {
		val = 1;
	} else if (knob->angle_unclipped >= max) {
		val = -1;
	}

	val *= knob->scale;
	// printf("%i, %i, %i, %f \n", knob->angle_unclipped, knob->angle, idx, val);
	return static_cast<int16_t> (round(val));
};


int16_t Free_torque::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	return knob->scale;
};

int16_t Spin::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	return knob->target_velocity;
};


// Calculates an index for the look-up table based transfer functions
int16_t Mode::calc_index(void* ptr) {
	static int16_t state = 0;
	HapticKnob* knob = (HapticKnob*)ptr;

	state += static_cast<int16_t> (round(knob->angle_delta * knob->stretch));
	// printf("%i, %i\n", knob->angle_delta, state );
	// idx = knob->angle + knob->stretch * static_cast<float>(knob->angle_delta);
	if (wrap_haptics)
	{
		idx = mod(state, 3600);
	} else {
		idx = clip(state, 0,  3600);
	}
	return idx;

};

void Mode::reset(int16_t angle_) {
	idx = angle_ + offset; // apply mode specific offset to idx
};


int zero_crossing(int in) {
	static int in_last = 0;
	if (in * in_last < 0) {
		return 1;
	}
	else return 0;
};



// Hybrid mode - under construction
