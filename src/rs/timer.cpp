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
		return time/1000;
	case rs::Timer::MilliSeconds:
		return time;
	case rs::Timer::MicroSeconds:
		return time*1000;
	case rs::Timer::NanoSeconds:
		return time*1000000;
	}
#else
	switch (_units) {
		case rs::Timer::Seconds:
			return time.tv_sec*1 + time.tv_nsec/1000000000;
		case rs::Timer::MilliSeconds:
			return time.tv_sec*1000 + time.tv_nsec/1000000;
		case rs::Timer::MicroSeconds:
			return time.tv_sec*1000000 + time.tv_nsec/1000;
		case rs::Timer::NanoSeconds:
			return time.tv_sec*1000000000 + time.tv_nsec/1;
	}
#endif

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

