#include <chrono>
#include <thread>

#include <rs/Timer>

using namespace rs;

Timer::Timer(int units) {
	_units = units;
}

Timer::~Timer(void) {
}

/**********************************************************
	public functions
 **********************************************************/
unsigned int Timer::now(void) {
	// get time
	clock_gettime(CLOCK_REALTIME, &_time);

	// report time in proper units
	switch (_units) {
		case rs::Timer::Seconds:
			return _time.tv_sec*1 + _time.tv_nsec/1000000000;
		case rs::Timer::MilliSeconds:
			return _time.tv_sec*1000 + _time.tv_nsec/1000000;
		case rs::Timer::MicroSeconds:
			return _time.tv_sec*1000000 + _time.tv_nsec/1000;
		case rs::Timer::NanoSeconds:
			return _time.tv_sec*1000000000 + _time.tv_nsec/1;
	}

	return 0;
}

void Timer::sleep(unsigned int length) {
	// sleep in proper units
	switch (_units) {
		case rs::Timer::Seconds:
			std::this_thread::sleep_for(std::chrono::seconds(length));
			break;
		case rs::Timer::MilliSeconds:
			std::this_thread::sleep_for(std::chrono::milliseconds(length));
			break;
		case rs::Timer::MicroSeconds:
			std::this_thread::sleep_for(std::chrono::microseconds(length));
			break;
		case rs::Timer::NanoSeconds:
			std::this_thread::sleep_for(std::chrono::nanoseconds(length));
			break;
	}

	return;
}

