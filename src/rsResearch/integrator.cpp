#include <cmath>
#include <iostream>

#include <rs/Macros>
#include <rsResearch/Enum>
#include <rsResearch/Integrator>

using namespace rsResearch;

Integrator::Integrator(void) {
	_body_length = 0;
	_form = 0;
	_num_robots = 0;
	_num_vars = 0;
	_time = 0;
	_time_offset = 0;
}

Integrator::~Integrator(void) {
	gsl_odeiv2_driver_free(_driver);
}

/**********************************************************
	public functions
 **********************************************************/
void Integrator::setup(int (*function)(double, const double[], double[], void*), struct Params *params, float step) {
	// set cpg variables
	_system = {function, NULL, static_cast<size_t>(params->num_vars), (void *)params};
	_driver = gsl_odeiv2_driver_alloc_y_new(&_system, gsl_odeiv2_step_rkf45, 1e-4, 1e-4, 0);
	_array.resize(params->num_vars);
	if (params->form == Forms::Dog) {
		for (int i = 0; i < params->num_vars; i+=3) {
			_array[i] = 1;
		}
	}
	else if (params->form == Forms::Salamander) {
		for (int i = 0; i < params->num_vars; i+=3) {
			_array[i] = 1;
		}
	}
	else if (params->form == Forms::Snake) {
		for (int i = 0; i < params->num_vars; i+=6) {
			_array[i] = 1;
			_array[i+3] = -1;
		}
	}
	_body_length = params->num_body;
	_form = params->form;
	_num_robots = params->num_legs + params->num_body;
	_num_vars = params->num_vars;
}

const rs::Vec Integrator::runStep(float newtime) {
	// return vector
	rs::Vec V(_num_robots);

	// integrate
	int status = gsl_odeiv2_driver_apply(_driver, &_time, newtime + _time_offset, _array.data());

	// die if integration step fails and return empty vector
	if (status != GSL_SUCCESS) {
		std::cerr << "ERROR: return value = " << status << std::endl;
		return V;
	}

	// save output array
	if (_form == Forms::Dog) {
		for (int j = 0, k = 0; j < _num_vars; j+=3, k++) {
			V[k] = _array[j+1]*cos(_array[j]);
		}
	}
	else if (_form == Forms::Salamander) {
		float theta_up = -5*M_PI/6;
		float theta_down = -M_PI/6;
		float a = theta_up - M_PI;
		float b = (2*M_PI - theta_down + theta_up)/(M_PI);
		float c = (theta_down - theta_up)/M_PI;
		for (int i = 0, j = 0; i < _num_vars; i+=3, j++) {
			if (i < _body_length*3)
				V[j] = _array[i+1]*cos(_array[i]);
			else {
				// get continuous output vectors
				if (_array[i] < theta_up)
					V[j] = theta_up + (_array[i] - theta_up)*b;
				else if (_array[i] < a)
					V[j] = theta_up + (_array[i] - theta_up)*c;
				else
					V[j] = theta_down + (_array[i] - a)*b;
				// drop values to start at zero
				//if (i == 1) init[k - _body_length] = x[k][i];
				//V[k] -= init[k - _body_length];
				// flip right side legs for linkbots
				if ( !((j+1 - _body_length)%2) ) V[j] = -V[j];
				// scale
				V[j] *= 4;
			}
		}
	}
	else if (_form == Forms::Snake) {
		for (int j = 0, k = 0; j < _num_vars; j+=6, k++) {
			V[k] = _array[j+1] + _array[j+1]*cos(_array[j]) - _array[j+4] - _array[j+4]*cos(_array[j+3]);
		}
	}
	return V;
}

