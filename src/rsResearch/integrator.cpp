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
void Integrator::setup(int (*function)(double, const double[], double[], void*), int body, int robots, int variables, int form, double step) {
	// set cpg variables
	_system = {function, NULL, static_cast<size_t>(variables), NULL};
	_driver = gsl_odeiv2_driver_alloc_y_new(&_system, gsl_odeiv2_step_rkf45, 1e-4, 1e-4, 0);
	_array.resize(variables);
	if (form == Forms::Salamander) {
		for (int i = 0; i < variables; i+=3) {
			_array[i] = 1;
		}
	}
	else if (form == Forms::Snake) {
		for (int i = 0; i < variables; i+=6) {
			_array[i] = 1;
			_array[i+3] = -1;
		}
	}
	_body_length = body;
	_form = form;
	_num_robots = robots;
	_num_vars = variables;

	// run cpg until steady state
	rs::Vec v;
	for (int i = 0; i < 0.3/step; i++) {
		v = this->runStep(step);
		_time_offset += step;
	}
	while (v[0] < 0 && v[1] < 0) {
		v = this->runStep(step);
		_time_offset += step;
	}
}

const rs::Vec Integrator::runStep(double newtime) {
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
	if (_form == Forms::Salamander) {
		//double init[4] = {0};
		double theta_up = -5*M_PI/6;
		double theta_down = -M_PI/6;
		double a = theta_up - M_PI;
		double b = (2*M_PI - theta_down + theta_up)/(M_PI);
		double c = (theta_down - theta_up)/M_PI;
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

