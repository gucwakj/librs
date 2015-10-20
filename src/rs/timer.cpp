#include <chrono>
#include <thread>

#include <rs/Timer>

using namespace rs;

Timer::Timer(int units) {
	_last = 0;
	_now = 0;
	_units = units;
}

Timer::~Timer(void) {
}

/**********************************************************
	public functions
 **********************************************************/
unsigned int Timer::delta(void) {
	this->now();
	return _now - _last;
}

unsigned int Timer::now(void) {
	// store last time
	_last = _now;

	// get time
#ifdef RS_WIN32
	DWORD time = GetTickCount();
#else
	struct timespec time;
	clock_gettime(CLOCK_REALTIME, &time);
#endif

	// report time in proper units
#ifdef RS_WIN32
	switch (_units) {
		case rs::Timer::Seconds:
			_now = time/1000;
		case rs::Timer::MilliSeconds:
			_now = time;
		case rs::Timer::MicroSeconds:
			_now = time*1000;
		case rs::Timer::NanoSeconds:
			_now = time*1000000;
	}
#else
	switch (_units) {
		case rs::Timer::Seconds:
			_now = time.tv_sec*1 + time.tv_nsec/1000000000;
		case rs::Timer::MilliSeconds:
			_now = time.tv_sec*1000 + time.tv_nsec/1000000;
		case rs::Timer::MicroSeconds:
			_now = time.tv_sec*1000000 + time.tv_nsec/1000;
		case rs::Timer::NanoSeconds:
			_now = time.tv_sec*1000000000 + time.tv_nsec/1;
	}
#endif

	return _now;
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

