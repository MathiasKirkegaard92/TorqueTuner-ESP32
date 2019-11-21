#include "haptics.h"


HapticKnob::HapticKnob() {
	set_mode(CLICK);
	num_modes = mode_list.size();
};

// idx range is [0:3600], representing angle*10
void HapticKnob::update() {


	update_angle();
	// Filter and gate velocity
	velocity = filter(velocity);
	velocity = gate(velocity, 5, 0);

	calc_acceleration();
	active_mode->angle_to_index(this);

	// // Calculate trigger and discrete output
	float resolution = active_mode->max / angle_scale;
	angle_discrete = floor(floor(angle_out / resolution) * resolution);
	if (abs(angle_discrete - angle_discrete_last) >= resolution && abs(angle_delta) > 1 && angle_scale > 0) {
		update_trig();
		// printf("%i , %i \n", angle_discrete, angle_out);
		angle_discrete_last = angle_discrete;
	};

	torque = active_mode->calc(this);
	if (zero_crossing(torque)) {
		printf("%i , %i, %i \n ", angle_discrete, angle_out, torque);
	}
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


	// angle_unclipped = (angle + (3600 * wrap_count));
	angle_unclipped += angle_delta;

	// wrap instead of clip if wrap flag is set !!

	// Clip with parameter range
	angle_out += angle_delta;
	angle_out = static_cast<int32_t>(clip(angle_out, active_mode->min, active_mode->max));
	angle_delta = angle_out - angle_out_last;
	angle_out_last = angle_out;
};


void HapticKnob::set_mode(int mode_idx) {
	if (active_mode != mode_list[mode_idx]) {
		active_mode = mode_list[mode_idx];
		active_mode->reset(angle);
		angle_out = angle;
		angle_unclipped = angle;
		set_defaults(active_mode);
		print_mode(static_cast<MODE>(mode_idx));
	}
};


void HapticKnob::set_mode(MODE mode_) {
	int mode_idx = static_cast<int>(mode_);
	if (active_mode != mode_list[mode_idx]) {
		active_mode = mode_list[mode_idx];
		active_mode->reset(angle);
		angle_out = angle;
		angle_unclipped = angle;
		set_defaults(active_mode);
		print_mode(static_cast<MODE>(mode_idx));
	}
};

void HapticKnob::set_angle_scale(float angle_scale_) {
	if (abs(angle_scale_ - angle_scale) > 0.1) {
		angle_scale = angle_scale_;
	}
};

void HapticKnob::set_defaults(Mode * mode) {
	torque_scale = mode->torque_scale_default;
	angle_scale = mode->angle_scale_default;
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
	case MOTION:
		printf("Motion \n");
		break;
	}
};

float HapticKnob::calc_acceleration() {
	static float last_velocity = 0;
	acceleration = last_velocity - velocity;
	last_velocity = velocity;
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
	val *= knob->torque_scale;
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
		val = static_cast<float>((tf_click[idx])) / TABLE_RESOLUTION * knob->torque_scale;
	}
	return static_cast<int16_t> (round(val));
};

int16_t Magnet::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	return static_cast<float>(tf_magnet[idx]) / TABLE_RESOLUTION * knob->torque_scale; // Magnet
};

int16_t Inertia::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	if (knob->velocity > 0) {
		return round((- (knob->angle_out / 3600.0) * knob->velocity) * knob->torque_scale / MAX_VELOCITY);
	} else {
		return 0;
	}
	// return knob->torque_scale * knob->acceleration / 40.0;
};

int16_t LinSpring::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	float val = - (knob->angle_out - 1800) / 1800.0;
	if (knob->angle_unclipped <= min) {
		val = 1;
	} else if (knob->angle_unclipped >= max) {
		val = -1;
	}
	val *= knob->torque_scale;
	return static_cast<int16_t> (round(val));
};

int16_t ExpSpring::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	int idx_test  = static_cast<int16_t> (round((knob->angle - min) * knob->angle_scale)) % 3600;
	float val = static_cast<float>(tf_wall[idx_test]) / TABLE_RESOLUTION; // wall
	if (knob->angle_unclipped <= min) {
		val = 1;
	} else if (knob->angle_unclipped >= max) {
		val = -1;
	}
	val *= knob->torque_scale;
	return static_cast<int16_t> (round(val));
};

int16_t Free_torque::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	return knob->torque_scale;
};

int16_t Motion::calc(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	return knob->target_velocity;
};


// Calculates an index for the look-up table based transfer functions
int16_t Mode::angle_to_index(void* ptr) {
	HapticKnob* knob = (HapticKnob*)ptr;
	// knob->angle = mod(knob->angle,3600);
	idx += static_cast<int16_t> (round((knob->angle_delta) * knob->angle_scale));
	idx = mod(idx, 3600);
	return idx;
};

void Mode::reset(int16_t angle_) {
	idx = angle_ + offset; // apply mode specific offeset to idx
}




// Hybrid mode - under construction
