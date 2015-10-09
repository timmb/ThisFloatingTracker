#pragma once

class Sketch
{
public:
	Sketch(char const* name_) : isActive(false), name(name_) {}
	virtual void setup() {}
	virtual void update() {}
	virtual void draw() {}
	virtual void recompileShaders() {}

	bool isActive;

	char const* name;
};