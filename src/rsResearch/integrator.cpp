#include <cmath>
#include <iostream>

#include <rs/Macros>
#include <rsResearch/Enum>
#include <rsResearch/Integrator>

using namespace rsResearch;

Integrator::Integrator(void) {
	_delta = 0;
	_time = 0;
	_driver = NULL;
	_rec_on = false;
}

Integrator::~Integrator(void) {
	if (_driver != NULL) {
		gsl_odeiv2_driver_free(_driver);
	}
}

/**********************************************************
	public functions
 **********************************************************/
void Integrator::plot(std::string title) {
	if (_rec_on) {
		// store data
		FILE *temp = fopen("temp", "w");
		std::string str;
		for (unsigned int i = 0; i < _rec_t.size(); i++) {
			for (unsigned int j = 0; j < _v.size() - 1; j++) {
				str.append(std::to_string(_rec_v[i][j])).append(", ");
			}
			str.append(std::to_string(_rec_v[i][_v.size() - 1]));
			fprintf(temp, "%f, %s\n", _rec_t[i], str.c_str());
			str.clear();
		}

		// create pipe
		FILE *gnuplotPipe = popen("gnuplot -persistent", "w");

		// plot
		fprintf(gnuplotPipe, "set title '%s'\n", title.c_str());
		fprintf(gnuplotPipe, "set xlabel 'Time [s]'\n");
		fprintf(gnuplotPipe, "set ylabel 'Amplitude'\n");
		fprintf(gnuplotPipe, "plot ");
		for (unsigned int j = 0; j < _v.size() - 1; j++) {
			fprintf(gnuplotPipe, "'temp' using 1:%d with lines title 'Module %d', ", j+2, j);
		}
		fprintf(gnuplotPipe, "'temp' using 1:%d with lines title 'Module %d'\n", _v.size() + 1, _v.size() - 1);
	}
}

const rs::Vec* Integrator::runStep(float newtime) {
	// integrate
	int status = gsl_odeiv2_driver_apply(_driver, &_time, newtime, _array.data());

	// die if integration step fails and return empty vector
	if (status != GSL_SUCCESS) {
		_v[0] = -1;
		return &_v;
	}

	// save output array
	for (short i = 0, j = 0; i < _params->num_vars; i+=3, j++) {
		_v[j] = _array[i+1]*cos(_array[i]);
	}
	// linearize the legs motion
	if (_params->linearize) {
		float theta_up = -5*M_PI/6;
		float theta_down = -M_PI/6;
		float a = theta_up - M_PI;
		float b = (2*M_PI - theta_down + theta_up)/(M_PI);
		float c = (theta_down - theta_up)/M_PI;
		for (short i = _params->num_body*3, j = _params->num_body; i < _params->num_vars; i+=3, j++) {
			// get continuous output vectors
			if (_array[i] < theta_up)
				_v[j] = theta_up + (_array[i] - theta_up)*b;
			else if (_array[i] < a)
				_v[j] = theta_up + (_array[i] - theta_up)*c;
			else
				_v[j] = theta_down + (_array[i] - a)*b;
			// flip right side legs for linkbots
			if ( ((j - _params->num_body)%2) ) _v[j] = -1*_v[j];
			// scale
			_v[j] *= 0.4;
		}
	}

	// turning
	/*if (fabs(_delta) > rs::Epsilon) {
		float a = (_delta > 0) ? -1*_params->R + _delta : -1*_params->R;
		float b = (_delta < 0) ? _params->R + _delta : _params->R;
		for (short i = 0; i < _params->num_body; i++) {
			_v[i] = ((b-a)*(_v[i] - -1*_params->R))/(2*_params->R) + a;
		}
	}*/

	// record for plotting
	if (_rec_on) {
		_rec_v.push_back(_v);
		_rec_t.push_back(newtime);
	}

	// return output array
	return &_v;
}

void Integrator::setup(int (*function)(double, const double[], double[], void*), struct Params *params, float step) {
	// set cpg variables
	_system = {function, NULL, static_cast<size_t>(params->num_vars), (void *)params};
	_driver = gsl_odeiv2_driver_alloc_y_new(&_system, gsl_odeiv2_step_rkf45, 1e-4, 1e-4, 0);
	_array.resize(params->num_vars);
	_params = params;
	_v.allocate(_params->num_robots);
}

void Integrator::setRecording(bool enable) {
	_rec_on = enable;
}

void Integrator::turn(float val) {
	_delta = val;
}

