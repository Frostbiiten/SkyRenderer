#pragma once
#include <flecs.h>

// sky
#include <sky/Components.h>

// game
#include <game/Enemy.h>
#include <game/Player.h>

namespace sky
{
	namespace ecs
	{
		namespace filter
		{
			inline flecs::query<com::enemy, com::hp, com::position, com::rectBounds> enemyHitFilter;
			inline flecs::query<player, com::hp, com::position, com::rectBounds> playerHitFilter;
		}
	}
}
