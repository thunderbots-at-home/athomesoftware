// RealWorldObject.cpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================
// include guard

#ifndef __REALWORLDOBJECT_H_INCLUDED__
#define __REALWORLDOBJECT_H_INCLUDED__

#include "thunderbots_vision.hpp"

using namespace std;
using namespace cv;


class RealWorldObject
{

	struct Position 
	{
			int x;
			int y;
			int z;
	};

	private:
		Position position;
		string name; // Textual representation of object
		string description; // Textual description of object
		Mat picture; // Image of object 
		bool general_object;
	public:

		void setPosition(Position);
		void setPicture(Mat&);
		void setName(string);
		void setName(char*);
		void setIsGeneral(bool);
		bool equals(RealWorldObject&);
		bool isGeneral();
		bool isSpecific();
		Position getPosition();
		string getName();
		string getDescription();
		Mat& getPicture();		

		~RealWorldObject();
		RealWorldObject();
		RealWorldObject(Mat&, string); // Image, name, (x,y,z)


};

#endif
