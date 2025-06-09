#include <sky/ResourceMan.h>
#include <cstddef>

namespace sky
{
	namespace res
	{

        /* Type-specified template - TODO: replace with raylib
		template<class T>
		std::size_t ResourceMap<T>::load(std::filesystem::path path)
		{
			dbg::log()->error("Please specify a type-specific override");
			return 0;
		}

		template<>
		std::size_t ResourceMap<sf::Texture>::load(std::filesystem::path path)
		{
			std::string data;
			file::readFile(path, data);
			resourceMap.push_back(sf::Texture{});
			if (!resourceMap[resourceMap.size() - 1].loadFromMemory(data.data(), data.size())) return 0;
			return resourceMap.size() - 1;
		}

		template<>
		ResourceMap<sf::Texture>::ResourceMap(std::size_t reserve)
		{
			resourceMap.reserve(reserve);
			load("common/null.png"); // default 0 (null)
		}

		ResourceMap<sf::Texture> _textures { 1024 };
		ResourceMap<sf::Texture>& textures() { return _textures; };

        */
	}
}