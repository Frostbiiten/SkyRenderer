#pragma once

// std
#include <filesystem>
#include <unordered_map>
#include <any>

// TODO: remove SFML
// #include <SFML/Graphics.hpp>
// #include <SFML/Audio.hpp>

// sky
#include <sky/AssetMan.h>
#include <sky/Debug.h>

namespace sky
{
	namespace res
	{
		template <class T>
		class ResourceMap
		{
			// should this be a pointer?
			std::vector<T> resourceMap;

			// maps paths to id: intended for use upon initial access
			std::unordered_map<std::filesystem::path, std::size_t> idMap;

			// load new resource
			std::size_t load(std::filesystem::path path);

		public:

			// Constructor
			ResourceMap(std::size_t reserve = 1024)
			{
				resourceMap.reserve(reserve);
				resourceMap.resize(1);
			}

			// Intended for first access, 0 is null
			std::size_t getId(std::filesystem::path path)
			{
				auto find = idMap.find(path);
				if (find == idMap.end()) return (idMap[path] = load(path));
				return find->second;
			}

			T& getResource(std::size_t id)
			{
				return resourceMap[id];
			}
		};

		// ResourceMap<sf::Texture>& textures();
	}
}
