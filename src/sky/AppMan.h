#pragma once

namespace sky
{
	namespace render
	{
		constexpr int pixelWidth = 1280;
		constexpr int pixelHeight = 720;
		constexpr int scaleFactor = 1;
		constexpr int aaLevel = 0; // set to 8x or 16x for aa
	}

	void init();
}
