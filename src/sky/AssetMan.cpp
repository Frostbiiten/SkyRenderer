// std
#include <filesystem>
#include <fstream>
#include <mutex>

// physfs
#include <physfs.h>

// SKY
#include <sky/AssetMan.h>
#include <sky/Debug.h>

#define SPDLOG_FMT_EXTERNAL
namespace sky
{
	namespace file
	{
        std::mutex ioMutex;

		bool init()
		{
            const std::lock_guard<std::mutex> lock {ioMutex};

			if (PHYSFS_isInit()) return false;
			if (!PHYSFS_init(NULL) && isInit) arc_LogLatestError();
			PHYSFS_permitSymbolicLinks(1);

			//Mount all archives in subdirectories.
			for (const auto& entry : std::filesystem::recursive_directory_iterator("./"))
			{
				if (entry.path().extension() == ".zip" ||
					entry.path().extension() == ".7z" ||
					entry.path().extension() == ".pak" ||
					entry.path().extension() == ".pkg")
				{
					dbg::log()->info("Mounting {}", entry.path().string());
					arc_MountDir(entry.path());
				}
			}

			isInit = true;
			return true;
		}

		void arc_LogLatestError()
		{
			if (!isInit) init();
			PHYSFS_ErrorCode err = PHYSFS_getLastErrorCode();
			dbg::log()->error("PHYSFS FILESYSTEM ERROR {}: {}", std::to_string(err), PHYSFS_getErrorByCode(err));
		}

		void fs_LogError(std::error_code err)
		{
			if (!isInit) init();
			dbg::log()->error("STANDARD FILESYSTEM ERROR {}: {}", err.value(), err.message());
		}

		bool arc_MountDir(std::filesystem::path path, std::string mountPoint)
		{
			if (!isInit) init();

			//runs and returns 0 on fail
			if (!PHYSFS_mount(path.string().c_str(), mountPoint.c_str(), 1))
			{
				arc_LogLatestError();
				return false;
			}
			else return true;
		}

		bool arc_UnmountDir(std::filesystem::path path)
		{
			if (!isInit) init();

			//runs and returns 0 on fail
			if (!PHYSFS_unmount(path.string().c_str()))
			{
				arc_LogLatestError();
				return false;
			}
			else return true;
		}

		std::vector<std::string> list(std::filesystem::path path)
		{
			if (!isInit) init();

			std::vector<std::string> list;

			//STD::FILESYSTEM SECTION

			//Iterates in the specified dierectory and adds files to list
			for (const auto& entry : std::filesystem::directory_iterator(path))
				list.push_back(entry.path().string());


			//PHYSFS SECTION

			//returns a NULL terminated array of NULL terminated strings
			char** listBuf = PHYSFS_enumerateFiles(path.string().c_str());

			//returns empty list if the buffer is NULL
			if (listBuf == NULL) return list;

			//Iterates through each element in the array, stops at NULL because it is NULL terminated
			for (std::size_t x = 0; listBuf[x] != NULL; ++x)
			{
				//Adds constructed NULL terminated strings to the vector
				list.push_back(std::string(listBuf[x], listBuf[x] + strlen(listBuf[x])));
			}

			//returns filled list
			return list;
		}

		bool exists(std::filesystem::path file)
		{
			if (!isInit) init();

			//Mounts current directory and unmounts after checking for existence
			arc_MountDir("./");
			bool exists = PHYSFS_exists(file.string().c_str()) || std::filesystem::exists(file);
			arc_UnmountDir("./");
			return exists;
		}

		std::filesystem::path getBaseDirectory()
		{
			return std::filesystem::current_path();
		}

		bool fileSize(std::filesystem::path file)
		{
			//STD::FILESYSTEM SECTION

			//Only run std::filesytem code if it exists in the current path and is a normal file
			if (std::filesystem::exists(file) && std::filesystem::is_regular_file(file))
			{
				std::error_code error;
				int fileSize = std::filesystem::file_size(file, error);
				if (error.value() == 0) return fileSize;
				else
				{
					fs_LogError(error);
					return -1;
				}
			}

			//PHYSFS SECTION

			PHYSFS_file* fileData = PHYSFS_openRead(file.string().c_str());

			if (fileData != nullptr)
			{
				//Attempt to close file handle
				if (!PHYSFS_close(fileData)) arc_LogLatestError();

				//Return file size in bytes
				return PHYSFS_fileLength(fileData);
			}
			else arc_LogLatestError();

			//defults to failiure
			return -1;
		}

		bool readFile(std::filesystem::path file, std::string& output)
		{
            const std::lock_guard<std::mutex> lock {ioMutex};
			if (!isInit) init();

			//STD::FILESYSTEM SECTION

			//Only run std::filesytem code if it exists in the current path and is a normal file
			if (std::filesystem::exists(file) && std::filesystem::is_regular_file(file))
			{
				std::ifstream stream(file, std::ios::binary);
				output.resize(std::filesystem::file_size(file));
				stream.unsetf(std::ios::skipws);
				if (!stream.is_open()) return false;
				stream.read(output.data(), output.size());
				return true;
			}

			//PHYSFS SECTION

			PHYSFS_file* fileData = PHYSFS_openRead(file.string().c_str());

			if (fileData != nullptr)
			{
				//Gets filesize
				PHYSFS_sint64 fileSize = PHYSFS_fileLength(fileData);

				//If the size is -1, return failure
				if (fileSize == -1) return false;

				// Note: termination byte is already present, output[output.size()] should be '\0'
				output.resize(fileSize);

				//Attempts to read data into buffer
				PHYSFS_sint64 dataRead = PHYSFS_readBytes(fileData, output.data(), fileSize);

				bool fail = dataRead != fileSize;
				if (fail)
				{
					//Data has not been read properly, clear buffer memory
					output.clear();
					arc_LogLatestError();
				}

				//Attempt to close file handle
				if (!PHYSFS_close(fileData)) arc_LogLatestError();
				return !fail;
			}
			else arc_LogLatestError();

			//Attempt to close file handle
			if (!PHYSFS_close(fileData)) arc_LogLatestError();

			//defults to failiure
			return false;
		}

		bool createDirectory(std::filesystem::path path)
		{
			std::error_code err;
			bool success = std::filesystem::create_directory(path, err);

			if (success) return success;

			fs_LogError(err);

			return false;
		}

		bool removeDirectory(std::filesystem::path path)
		{
			//Does not exist
			if (!std::filesystem::exists(path) || !std::filesystem::is_directory(path))
				return false;

			std::error_code err;
			bool success = std::filesystem::remove_all(path, err);

			if (success) return success;

			fs_LogError(err);

			return false;
		}

		bool createFile(std::filesystem::path file)
		{
			std::ofstream f{ file };
			return std::filesystem::exists(file);
		}

		bool removeFile(std::filesystem::path file)
		{
			//Does not exist or not a regular file
			if (!std::filesystem::exists(file) || !std::filesystem::is_regular_file(file))
				return false;

			std::error_code err;
			if (std::filesystem::remove(file, err)) return true;

			// Log error and return false if no success
			fs_LogError(err);
			return false;
		}

		bool writeFile(std::filesystem::path file, const std::string& data, bool append)
		{
			// Lock IO
            const std::lock_guard<std::mutex> lock {ioMutex};

			// Open file
			std::fstream fileStream;
			fileStream.open(file, std::ofstream::out | (append ? std::ios::app : std::ofstream::trunc));

			// If open failed, return false
			if (!fileStream.is_open()) return false;

			// Write data and return true
			fileStream << data;
			return true;
		}
	}
}
