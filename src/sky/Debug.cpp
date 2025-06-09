#include <sky/Debug.h>

// filesystem
#include <filesystem>

// spdlog
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

// fmt
#include <fmt/color.h>

namespace sky
{
	namespace dbg
	{
		// If init has been called
		bool initialized = false;

		// Main log, outputs to cout & file
		std::shared_ptr<spdlog::logger> mainLog;

		void init()
		{
			if (initialized) return;

			struct tm newtime;
			time_t now = time(0);
			localtime_s(&newtime, &now);
			std::ostringstream dateStream;
			dateStream << "logs/" << std::put_time(&newtime, "%d-%m-%Y %H-%M-%S") << ".log";
			std::string logName = dateStream.str();

			try
			{
				spdlog::init_thread_pool(8192, 1);

				// Create file and cout sink
				auto coutSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
				auto fileSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(logName, 5242880, 5, false);

				std::vector<spdlog::sink_ptr> sinks {coutSink, fileSink};

				mainLog = std::make_shared<spdlog::async_logger>("main", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
				spdlog::register_logger(mainLog);

				initialized = true;
			}
			catch (const spdlog::spdlog_ex& ex)
			{
				fmt::print(fmt::emphasis::bold | fmt::fg(fmt::color::crimson), "Failed to init log: {}\n", ex.what());
			}
		}

		void checkFolder()
		{
			if (!std::filesystem::exists("./Logs"))
			{
				std::filesystem::create_directory("./Logs");
				fmt::print(fmt::emphasis::bold | fmt::fg(fmt::color::gold), "Created log folder...\n");
			}
		}

		std::string getType(const std::type_info& type)
		{
			if (type == typeid(std::string)) return "string";
			return type.name();
		}

		spdlog::logger* log()
		{
			return mainLog.get();
		}
	}
}
