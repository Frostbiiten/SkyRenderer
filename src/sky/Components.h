#pragma once
#include <SFML/Graphics.hpp>
#include <functional>
#include <flecs.h>
#include <array>

namespace sky
{
	namespace com
	{
		// COMPONENTS SHOULD ONLY BE DATA

		struct position
		{
			float x, y;
		};

		struct velocity
		{
			float x, y;
		};

		struct teleport
		{
			float x, y;
		};

		struct setVel
		{
			float x, y;
		};

		struct rectBounds
		{
			float width;
			float height;
			float xOffset = 0.f;
			float yOffset = 0.f;
		};

		struct drawBounds
		{
			sf::Color color;
			std::array<sf::Vertex, 5> verts;
		};

		struct circBounds
		{
			float radius;
			float xOffset = 0.f;
			float yOffset = 0.f;
		};

		struct drawCircBounds
		{
			sf::Color color;
			std::array<sf::Vertex, 10> verts;
		};

		struct drawPoint
		{
			sf::Color color;
		};

		struct boxCollider {};

		struct loading {};
		struct debugDrawer {};

		struct timedCallback
		{
			float remaining;
			std::function<void(flecs::entity)> callback;
		};

		// Game
		struct dmg { float dmg; };
		struct hp { float hp; };
		struct dead { float overkill = 0.f; }; // reserved for future use

		enum class HitTarget
		{
			player,
			enemy,
			any
		};

		struct hit
		{
			HitTarget target = HitTarget::any;
			float time = 0.1f;
			float dmg = 1.f;
			sf::Vector2f kb {};
		};

		struct follow
		{
			flecs::entity target;
			bool followPos = true;
			float lookahead = 0.f;
			bool adoptHitstop = true;
		};

		struct drag
		{
			float magnitude = 0.01f;
		};

		struct force
		{
			sf::Vector2f force;
		};

		struct hitstop
		{
			float time = 0.06f;
		};

		struct killClock
		{
			float time;
		};

		struct invulnerable
		{
			float time = 0.2f;
		};

		struct sinOffsetJitter
		{
			sf::Vector2f dir { 1.f, 0.f };
			float freq = 100;
			float time = 0.f;
		};
	}
}