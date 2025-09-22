#pragma once
#include <cmath>
#include <iostream>

class Vec2 {
public:
	float x, y;

	Vec2(float xpos, float ypos);

	Vec2 operator+(const Vec2& other) const;

	Vec2 operator-(const Vec2& other) const;

	Vec2 operator*(const float& other) const;

	Vec2 operator/(const float& other) const;

	float DistanceTo(const Vec2& v) const;

	Vec2 DirectionTo(const Vec2& v) const;

	float Dot(const Vec2& v) const;
};