#pragma once
#include <spdlog/async.h>

// Debug mode
#define DEBUG_MODE 0;

#ifdef DEBUG_MODE
#define TEST(x) do { x } while(0)
#else
#define TEST(x) do { } while(0)
#endif

namespace sky
{
	namespace dbg
	{
		// Init logs
		void init();

		// Check if log folder exists
		void checkFolder();

		// Convert type_info to readable string
		std::string getType(const std::type_info& type);

		// Get Log to log message
		spdlog::logger* log();
	}
}
