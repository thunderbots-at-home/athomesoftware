// Scene.cpp
// Written by Devon Ash
// Copyright of Thunderbots@HomeLeague, UBC
//=======================================================
// include guard

#include <object_recognition_experimental_v1/Scene.hpp>
#include <object_recognition_experimental_v1/RealWorldObject.hpp>
#include <vector>

vector<RealWorldObject> Scene::objects_in_scene;
vector<RealWorldObject> Scene::objects_of_interest;

bool Scene::isInScene(RealWorldObject& object)
{
	for (int i = 0; i < objects_in_scene.size(); i++)
	{
		if (objects_in_scene[i].equals(object))
		{
			return true;
		}
	}
	
	return false;
}

void Scene::loadObjectsOfInterest()
{
	// Get current dir
	// look in /data/objects_of_interest
	char buf[PATH_MAX];
	string dir(getwd(buf));
	dir = dir + "/data/objects_of_interest";
	ROS_INFO("Loading objects of interest from %s", dir.c_str());
	
	DIR *pDIR;
	pDIR = opendir(dir.c_str());
	struct dirent *entry;
	string current = dir;
	current.append("/");

	if (pDIR)
	{
			while ((pDIR != NULL) && (entry = readdir(pDIR)))
			{
				if ( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
				{
					char* filetype = getFileType(entry->d_name);
					if ((filetype != NULL) && strcmp(filetype, "jpg") == 0)
					{
						string copy = current;
						
						Mat image;
						image = imread(copy.append(entry->d_name)+".jpg");
						if (image.data)
						{
							RealWorldObject object(image, entry->d_name);
							objects_of_interest.push_back(object);
	
						}						
					}
				}
			}
			ROS_INFO("Loaded all objects of interest");	
	} 
	else
	{
		ROS_INFO("Unable to open directory %s", dir.c_str());
	}

}

char* Scene::getFileType(char* filename)
{
	// Returns the file type of a given filename.
	char* stf = strtok(filename, ".");
	return strtok(NULL, ".");
}
