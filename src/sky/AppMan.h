#pragma once

namespace sky
{
	namespace render
	{
		constexpr int pixelWidth = 424;
		constexpr int pixelHeight = 260;
		constexpr int scaleFactor = 3;
		constexpr int aaLevel = 0; // set to 8x or 16x for aa
	}

	void init();
}
