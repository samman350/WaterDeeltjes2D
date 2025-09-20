#pragma once
#include <cmath>
#include <iostream>

class Vec2 {
public:
	float x, y;

	Vec2(float xpos, float ypos);

	Vec2 operator+(const Vec2& other);

	Vec2 operator-(const Vec2& other);

	Vec2 operator*(const float& other);

	Vec2 operator/(const float& other);

	float Distance(const Vec2& u, const Vec2& v);
	
	float DistanceTo(const Vec2& v);

	Vec2 Direction(const Vec2& u, const Vec2& v);

	float Dot(const Vec2& u, const Vec2& v);
};