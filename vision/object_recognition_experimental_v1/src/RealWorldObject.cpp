// RealWorldObject.cpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================

#include <object_recognition_experimental_v1/RealWorldObject.hpp>

void RealWorldObject::setPosition(Position position) 
{
	this->position = position;
}

void RealWorldObject::setPicture(Mat& picture)
{
	this->picture = picture;
}

void RealWorldObject::setName(string name) 
{
	this->name = name;
}

void RealWorldObject::setName(char* name)
{
	string s(name);
	this->name = s;
}

bool RealWorldObject::equals(RealWorldObject& rwo )
{
 	return (this->name == rwo.name);
}

RealWorldObject::Position RealWorldObject::getPosition()
{
	return this->position;
}

string RealWorldObject::getName()
{
	return this->name;
}

string RealWorldObject::getDescription()
{
	return this->description;
}

Mat& RealWorldObject::getPicture()
{
	return (this->picture);
}

RealWorldObject::~RealWorldObject()
{

}

RealWorldObject::RealWorldObject()
{

}

RealWorldObject::RealWorldObject(Mat& picture, string name)
{
	this->picture = picture;
	this->name = name;
}


