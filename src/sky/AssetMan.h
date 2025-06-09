#pragma once
#include <iostream>
#include <vector>
#include <filesystem>

namespace sky
{
	// Unified* io class
	// TODO: change physfs functions to utilize mutable .data from strings instead of doing dealing with char*
	namespace file
	{
		inline bool isInit;

		// Initiates the basics
		bool init();

		// Logs the latest PHYSFS error that occured
		void arc_LogLatestError();

		// Logs a std::filesystem error
		void fs_LogError(std::error_code err);

		// Mounts an archive to the virtual filesystem. "path" is the directory of the archive. "mountpoint" is where it will be "mounted"/added on the virtual filesystem
		bool arc_MountDir(std::filesystem::path path, std::string mountPoint = "");

		// Unmounts an archive from the virtual filesystem. "path" is the directory of the archive.
		bool arc_UnmountDir(std::filesystem::path path);

		// -------------------------------------- UNIFIED -----------------------------------------
		// Favors normal files over archived ones

		// Returns a vector of string file names from path "path"
		std::vector<std::string> list(std::filesystem::path dir);

		// Check if "file" exists in the filesystem. Returns true if found, else false.
		bool exists(std::filesystem::path file);

		// Returns the directory that the application was launched from
		std::filesystem::path getBaseDirectory();

		// Get file size in bytes
		bool fileSize(std::filesystem::path file);

		// Loads data from file into string
		bool readFile(std::filesystem::path file, std::string& output);

		// ----------------------  STD::FILESYSTEM ONLY (no archives) -----------------------------

		// Creates a directory
		bool createDirectory(std::filesystem::path path);

		// Removes a directory
		bool removeDirectory(std::filesystem::path path);

		// Creates a file
		bool createFile(std::filesystem::path file);

		// Removes a file
		bool removeFile(std::filesystem::path file);

		// Writes data from string into a file
		bool writeFile(std::filesystem::path file, const std::string& data, bool append = false);
	};
}
