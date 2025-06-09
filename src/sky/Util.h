#pragma once
#include <SFML/System/Vector2.hpp>
#include <numbers>
#include <cmath>

namespace sky
{
	namespace util
	{
		constexpr float radToDeg = 180.f * std::numbers::inv_pi;
		constexpr float degToRad = std::numbers::pi / 180.f;

		namespace vec
		{
			template <typename T>
			inline sf::Vector2<T> lerp(const sf::Vector2<T>& a, const sf::Vector2<T>& b, float t)
			{
				return a + (b - a) * t;
			}

			template <typename T>
			inline float sqrMagnitude(const sf::Vector2<T>& vector)
			{
				return (vector.x * vector.x) + (vector.y * vector.y);
			}

			template <typename T>
			inline float magnitude(const sf::Vector2<T>& vector)
			{
				return std::sqrtf(sqrMagnitude);
			}

			inline float angle(float x, float y)
			{
				return atan2(x, y) * util::radToDeg;
			}

			inline float angle(const sf::Vector2f& vector)
			{
				return atan2(vector.x, vector.y) * util::radToDeg;
			}

			// from https://forum.unity.com/threads/whats-the-best-way-to-rotate-a-vector2-in-unity.729605/
			inline sf::Vector2f rotate(float x, float y, float delta)
			{
				return sf::Vector2f
				(
					x * std::cosf(delta) - y * std::sinf(delta),
					x * std::sinf(delta) + y * std::cosf(delta)
				);
			}

		}

		namespace f
		{
			inline float sign(float f) { return f >= 0.f ? 1.f : -1.f ; }
			inline float moveTowards(float current, float target, float maxDelta)
			{
				if (std::fabsf(target - current) <= maxDelta) return target;
				return current + sign(target - current) * maxDelta;
			}

		}

		namespace col
		{
			// https://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection
			// positions are based on center, not top-left
			inline bool collide(float circX, float circY, float circRad, float rectX, float rectY, float rectW, float rectH)
			{
				float circleDistX = abs(circX - rectX);
				float circleDistY = abs(circY - rectY);

				if (circleDistX > (rectW / 2 + circRad)) return false;
				if (circleDistY > (rectH / 2 + circRad)) return false;

				if (circleDistX <= (rectW / 2)) return true;
				if (circleDistY <= (rectH / 2)) return true; 

				float distX = (circleDistX - rectW / 2);
				float distY = (circleDistY - rectH / 2);
				float cornerDist_sq = distX * distX + distY * distY;

				return (cornerDist_sq <= (circRad * circRad));
			}
		}
	}
}

