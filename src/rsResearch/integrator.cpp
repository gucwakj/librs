#include <cmath>
#include <iostream>

#include <rs/Macros>
#include <rsResearch/Enum>
#include <rsResearch/Integrator>

using namespace rsResearch;

Integrator::Integrator(void) {
	_delta = 0;
	_time = 0;
}

Integrator::~Integrator(void) {
	gsl_odeiv2_driver_free(_driver);
}

/**********************************************************
	public functions
 **********************************************************/
const rs::Vec Integrator::runStep(float newtime) {
	// return vector
	short size = _params->num_robots + 1;
	rs::Vec V(size);

	// integrate
	int status = gsl_odeiv2_driver_apply(_driver, &_time, newtime, _array.data());

	// die if integration step fails and return empty vector
	if (status != GSL_SUCCESS) {
		std::cerr << "ERROR: return value = " << status << std::endl;
		return V;
	}

	// save output array
	for (short i = 0, j = 1; i < _params->num_body*3; i+=3, j++) {
		V[j] = _array[i+1]*cos(_array[i]);
	}
	// dummy output for tail robot
	V[0] = 0;
	// linearize the legs motion
	if (_params->num_legs) {
		float theta_up = -5*M_PI/6;
		float theta_down = -M_PI/6;
		float a = theta_up - M_PI;
		float b = (2*M_PI - theta_down + theta_up)/(M_PI);
		float c = (theta_down - theta_up)/M_PI;
		for (short i = _params->num_body*3, j = _params->num_body + 1; i < _params->num_robots*3; i+=3, j++) {
			// get continuous output vectors
			if (_array[i] < theta_up)
				V[j] = theta_up + (_array[i] - theta_up)*b;
			else if (_array[i] < a)
				V[j] = theta_up + (_array[i] - theta_up)*c;
			else
				V[j] = theta_down + (_array[i] - a)*b;
			// flip right side legs for linkbots
			if ( !((j - _params->num_body)%2) ) V[j] = -V[j];
			// scale
			V[j] *= 4;
		}
	}

	// turning
	if (fabs(_delta) > rs::Epsilon) {
		float a = (_delta > 0) ? -1*_params->R + _delta : -1*_params->R;
		float b = (_delta < 0) ? _params->R + _delta : _params->R;
		for (short i = 0; i < _params->num_body; i++) {
			V[i] = ((b-a)*(V[i] - -1*_params->R))/(2*_params->R) + a;
		}
	}

	// return output array
	return V;
}

void Integrator::setup(int (*function)(double, const double[], double[], void*), struct Params *params, float step) {
	// set cpg variables
	_system = {function, NULL, static_cast<size_t>(params->num_vars), (void *)params};
	_driver = gsl_odeiv2_driver_alloc_y_new(&_system, gsl_odeiv2_step_rkf45, 1e-4, 1e-4, 0);
	_array.resize(params->num_vars);
	_params = params;
}

void Integrator::turn(float val) {
	_delta = val;
}

