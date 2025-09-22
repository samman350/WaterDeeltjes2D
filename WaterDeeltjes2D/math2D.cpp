#include "math2D.h"

Vec2::Vec2(float xpos, float ypos) {
	x = xpos;
	y = ypos;
}

Vec2 Vec2::operator+(const Vec2& other) const {
	return Vec2(x + other.x, y + other.y);
}

Vec2 Vec2::operator-(const Vec2& other) const {
	return Vec2(x - other.x, y - other.y);
}

Vec2 Vec2::operator*(const float& other) const {
	return Vec2(other * x, other * y);
}

Vec2 Vec2::operator/(const float& other) const {
	return Vec2(x / other, y / other);
}

float Vec2::DistanceTo(const Vec2& u) const {
	return std::sqrt((x - u.x) * (x - u.x) + (y - u.y) * (y - u.y));
}

Vec2 Vec2::DirectionTo(const Vec2& u) const {
	return Vec2((x - u.x) / std::sqrt((x - u.x) * (x - u.x) + (y - u.y) * (y - u.y)),
		(y - u.y) / std::sqrt((x - u.x) * (x - u.x) + (y - u.y) * (y - u.y))); 
}

float Vec2::Dot(const Vec2& v) const {
	return x * v.x + y * v.y;
}