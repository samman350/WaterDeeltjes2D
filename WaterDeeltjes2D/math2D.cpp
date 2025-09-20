#include "math2D.h"

Vec2::Vec2(float xpos, float ypos) {
	x = xpos;
	y = ypos;
}

Vec2 Vec2::operator+(const Vec2& other) {
	return Vec2(x + other.x, y + other.y);
}

Vec2 Vec2::operator-(const Vec2& other) {
	return Vec2(x - other.x, y - other.y);
}

Vec2 Vec2::operator*(const float& other) {
	return Vec2(other * x, other * y);
}

Vec2 Vec2::operator/(const float& other) {
	return Vec2(x / other, y / other);
}

float Vec2::Distance(const Vec2& u, const Vec2& v) {
	return std::sqrt((u.x - v.x) * (u.x - v.x) + (u.y - v.y) * (u.y - v.y));
}

float Vec2::DistanceTo(const Vec2& u) {
	return std::sqrt((x - u.x) * (x - u.x) + (y - u.y) * (y - u.y));
}

Vec2 Vec2::Direction(const Vec2& u, const Vec2& v) {
	return Vec2((u.x - v.x) / Distance(u, v), (u.y - v.y) / Distance(u, v));
}

float Vec2::Dot(const Vec2& u, const Vec2& v) {
	return u.x * v.x + u.y * v.y;
}