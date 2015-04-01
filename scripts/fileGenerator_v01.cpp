#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;

string xacro_include(struct sensorData &, int);
void urdfGenerator(struct sensorData &, int);
string descriptionGenerator(string);
string intToString(int);
string sensorSelector(struct sensorData &, int, int, bool &);
void sensorLocation(int, bool &, struct sensorData &);

struct sensorData{
	string parent;
	string child;
	string xyz;
	string rpy;
	string material;
	string rgba;
	string stlFileName;
		};

int main()
{
	struct sensorData sensorData;
	//std::string ok("make this");
	// to turn into char * do this function
	// ok.c_str();
	int sensorPosition[9] = { };
	int i;
	bool is300Arch = false;
	string projectName = "hackweek";
	string fileName, filePathUrdf;
/*
Sensor Position defines the possible mounts point for the different sensor
sensorPosition(1) = RESERVED for Top Plate
sensorPosition(2) = Top Plate Left
sensorPosition(3) = Top Plate Center
sensorPosition(4) = Top Plate Right
sensorPosition(5) = RESERVED for Sensor Arch
sensorPosition(6) = Sensor Arch Front
sensorPosition(7) = Sensor Arch Left
sensorPosition(8) = Sensor Arch Center
sensorPosition(9) = Sensor Arch Right
*/

/*
sensor type
1: Sick LMS111
2: Sick LMS151
3: Sick LMS511
4: Top Plate
5: Sensor Arch 300
6: StereoCam
7: Velodyne
8: MicroStrain
9: Novatel GPS
10: Axis PTZ
11: Axis FixedCam
12: PTU
13: Sensor Arch 510
 */
	fileName = projectName + ".urdf.xacro";
	filePathUrdf = "urdf/" + fileName;
	
	sensorPosition[0] = 4; //RESERVED for Top Plate
	sensorPosition[1] = 0; //Top Plate Left
	sensorPosition[2] = 1; //Top Plate Center
	sensorPosition[3] = 0; //Top Plate Right
	sensorPosition[4] = 5; //RESERVED for Sensor Arch
	sensorPosition[5] = 0; //Sensor Arch Front
	sensorPosition[6] = 0; //Sensor Arch Left
	sensorPosition[7] = 0; //Sensor Arch Center
	sensorPosition[8] = 0; //Sensor Arch Right
	
	mkdir("urdf",0777);
	ofstream xacroID;
	xacroID.open (filePathUrdf.c_str());
	
	xacroID << "<?xml version=\"1.0\"?>" << endl;
	xacroID << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"" << projectName << "\">\n\n";
	xacroID << "<xacro:include filename=\"$(find husky_description)/urdf/base.urdf.xacro\" />\n";
	for (i = 0; i < 9; i++){
		if(sensorPosition[i] != 0) {
			xacroID << sensorSelector(sensorData,sensorPosition[i],i, is300Arch);
		}
	}
	descriptionGenerator(filePathUrdf);
	xacroID << "\n</robot>";
	xacroID.close();
return 0;
}

string xacro_include(struct sensorData &sensorData, int sensorPosition){
	return "<xacro:include filename=\"$(find robot_mod_rviz_plugin)/urdf/" + sensorData.child + ".urdf\" />\n";
}

void urdfGenerator(struct sensorData &sensorData, int sensorPosition){
	//Write a urdf file for a given sensor defined in sensorData.child at moment of call
	string sensorUrdfFile = "urdf/"+sensorData.child+".urdf";
	ofstream sensorID;
	sensorID.open (sensorUrdfFile.c_str());
	
	sensorID << "<?xml version=\"1.0\"?>\n";
	sensorID << "<robot name=\"" << sensorData.child << "\">\n";
	
	sensorID << "\t<joint name=\"" << sensorData.parent << "_to_" << sensorData.child  << "\" type=\"fixed\">\n";
	cout << sensorData.child << endl;
	sensorID << "\t\t<parent link=\"" << sensorData.parent << "\"/>\n";
	sensorID << "\t\t<child link=\"" << sensorData.child << "\"/>\n";
	sensorID << "\t\t<origin xyz=\"" << sensorData.xyz <<"\" rpy=\"" << sensorData.rpy << "\"/>\n";
	sensorID << "\t</joint>\n\n";
	
	sensorID << "\t<link name=\"" << sensorData.child << "\">\n";
	sensorID << "\t\t<visual>\n";
	sensorID << "\t\t\t<geometry>\n";
	sensorID << "\t\t\t\t<mesh filename=\"package://robot_mod_rviz_plugin/meshes/" << sensorData.stlFileName << ".STL\"/>\n";
	sensorID << "\t\t\t</geometry>\n";
	sensorID << "\t\t\t<material name=\"" << sensorData.material << "\">\n";
	sensorID << "\t\t\t\t<color rgba=\"" << sensorData.rgba << "\"/>\n";
	sensorID << "\t\t\t</material>\n";
	sensorID << "\t\t</visual>\n";
	sensorID << "\t</link>\n";
	sensorID << "</robot>\n";
	
	sensorID.close();
	
}

string descriptionGenerator(string filePathUrdf){
	mkdir("launch",0777);
	string descriptionFile = "launch/description.launch";
	ofstream descriptionID;
	descriptionID.open (descriptionFile.c_str());
	
	descriptionID << "<?xml version=\"1.0\"?>\n";
	descriptionID << "<launch>\n";
	descriptionID << "\t<param name=\"robot_description\" command=\"$(find xacro)/xacro.py '$(find robot_mod_rviz_plugin)/" << filePathUrdf << "\" />\n";
	descriptionID << "\t\t<node name=\"robot_state_publisher\" pkg=\"robot_state_publisher\" type=\"state_publisher\" />\n";
	descriptionID << "</launch>\n";
	
	return "launch/description.launch";
}

void sensorLocation(int sensorPosition, bool &is300Arch, struct sensorData &sensorData){
	switch(sensorPosition){
		case 1 : {
			//Mount point at the left of Top Plate
			sensorData.parent = "topPlate";
			sensorData.xyz = "0.29 0.21 ";
			break;
		}
		case 2 : {
			//Mount point at the center of Top Plate
			sensorData.parent = "topPlate";
			sensorData.xyz = "0.29 0.0 ";
			break;
		}
		case 3 : {
			//Mount point at the right of Top Plate
			sensorData.parent = "topPlate";
			sensorData.xyz = "0.29 -0.21 0.0";
			break;
		}
		case 4 : {
			if(is300Arch){
				//Mount point at the front of 300mm Sensor Arch
				sensorData.parent = "300Arch";
				sensorData.xyz = "0.02 0.0 -0.01";
			}
			else{
				//Mount point at the front of 510mm Sensor Arch
				sensorData.parent = "510Arch";
				sensorData.xyz = "0.02 0.0 -0.01";
			}
			break;
		}
		case 6 : {
			if(is300Arch){
				//Mount point at the left of 300mm Sensor Arch
				sensorData.parent = "300Arch";
				sensorData.xyz = "0.0 0.265 0.0";
			}
			else{
				//Mount point at the left of 510 Sensor Arch
				sensorData.parent = "510Arch";
				sensorData.xyz = "0.0 0.265 0.0";
			}
			break;
		}
		case 7 : {
			if(is300Arch){
				//Mount point at the center of 300mm Sensor Arch
				sensorData.parent = "300Arch";
				sensorData.xyz = "0.0 0.0 ";
			}
			else{
				//Mount point at the center of 510 Sensor Arch
				sensorData.parent = "510Arch";
				sensorData.xyz = "0.0 0.0 ";
			}
			break;
		}
		case 8 : {
			if(is300Arch){
				//Mount point at the right of 300mm Sensor Arch
				sensorData.parent = "300Arch";
				sensorData.xyz = "0.0 -0.265 0.0";
			}
			else{
				//Mount point at the right of 510 Sensor Arch
				sensorData.parent = "510Arch";
				sensorData.xyz = "0.0 -0.265 0.0";
			}
			break;
		}
	}
}

string intToString(int integer){
	ostringstream convert;
	convert << integer;
	return convert.str();
}

string sensorSelector(struct sensorData &sensorData, int sensorType, int sensorPosition, bool &is300Arch){
	
	switch(sensorType){
		case 1 : {
			sensorLocation(sensorPosition,is300Arch,sensorData);
			sensorData.child = "lms1xx_"+intToString(sensorPosition);
			sensorData.stlFileName = "lms1xx";
			cout << sensorData.child << endl;
			sensorData.xyz += "0.04";
			
			sensorData.rpy = "3.14159265359 0.0 0.0";
			sensorData.material = "velodyneGray";
			sensorData.rgba = "0.81 0.81 0.81 1";
			urdfGenerator(sensorData,sensorPosition);
			break;
		}
		case 2 : {
			sensorData.parent = "topPlate";
            sensorData.child = "lms1xx";
            sensorData.xyz = "0.29 0.21 0.0";
            sensorData.rpy = "3.14159265359 0.0 0.0";
            sensorData.material = "velodyneGray";
            sensorData.rgba = "0.81 0.81 0.81 1";
            urdfGenerator(sensorData,sensorPosition);
            break;
        }
        case 3 : {
            sensorData.parent = "topPlate";
            sensorData.child = "lms1xx";
            sensorData.xyz = "0.29 -0.21 0.0";
            sensorData.rpy = "3.14159265359 0.0 0.0";
            sensorData.material = "velodyneGray";
            sensorData.rgba = "0.81 0.81 0.81 1";
            urdfGenerator(sensorData,sensorPosition);
            break;
        }
        case 4 : {
            sensorData.parent = "base_link";
            sensorData.child = "topPlate";
            sensorData.stlFileName = "topPlate";
            sensorData.xyz = "0.0 0.0 0.2503";
            sensorData.rpy = "0.0 0.0 0.0";
            sensorData.material = "black";
            sensorData.rgba = "0.6 0.6 0.6 1";
            urdfGenerator(sensorData,sensorPosition); //Create the xacro file for the given sensor
            break;
		}
		case 5 : {
			is300Arch = true;
			sensorData.parent = "topPlate";
			sensorData.child = "300Arch";
			sensorData.stlFileName = "300Arch";
			sensorData.xyz = "-0.03 0.0 0.3"; 
			sensorData.rpy = "0.0 0.0 0.0";
			sensorData.material = "black";
			sensorData.rgba = "0.6 0.6 0.6 1";
			urdfGenerator(sensorData,sensorPosition);
			break;
		}
		case 6 : {
            sensorData.parent = "topPlate";
            sensorData.child = "lms1xx";
            sensorData.xyz = "0.0 0.0 0.0";
            sensorData.rpy = "3.14159265359 0.0 0.0";
            sensorData.material = "velodyneGray";
            sensorData.rgba = "0.81 0.81 0.81 1";
            urdfGenerator(sensorData,sensorPosition);
            break;
        }
        case 7 : {
            sensorData.parent = "topPlate";
            sensorData.child = "lms1xx";
            sensorData.xyz = "0.0 0.0 0.0";
            sensorData.rpy = "3.14159265359 0.0 0.0";
            sensorData.material = "velodyneGray";
            sensorData.rgba = "0.81 0.81 0.81 1";
            urdfGenerator(sensorData,sensorPosition);
            break;
        }
        case 8 : {
            sensorData.parent = "topPlate";
            sensorData.child = "lms1xx";
            sensorData.xyz = "0.0 0.0 0.0";
            sensorData.rpy = "3.14159265359 0.0 0.0";
            sensorData.material = "velodyneGray";
            sensorData.rgba = "0.81 0.81 0.81 1";
            urdfGenerator(sensorData,sensorPosition);
            break;
        }
        case 9 : {
            sensorData.parent = "topPlate";
            sensorData.child = "lms1xx";
            sensorData.xyz = "0.0 0.0 0.0";
            sensorData.rpy = "3.14159265359 0.0 0.0";
            sensorData.material = "velodyneGray";
            sensorData.rgba = "0.81 0.81 0.81 1";
            urdfGenerator(sensorData,sensorPosition);
            break;
        }
        case 10 : {
            sensorData.parent = "topPlate";
            sensorData.child = "lms1xx";
            sensorData.xyz = "0.0 0.0 0.0";
            sensorData.rpy = "3.14159265359 0.0 0.0";
            sensorData.material = "velodyneGray";
            sensorData.rgba = "0.81 0.81 0.81 1";
            urdfGenerator(sensorData,sensorPosition);
            break;
        }
        case 11 : {
            sensorData.parent = "topPlate";
            sensorData.child = "lms1xx";
            sensorData.xyz = "0.0 0.0 0.0";
            sensorData.rpy = "3.14159265359 0.0 0.0";
            sensorData.material = "velodyneGray";
            sensorData.rgba = "0.81 0.81 0.81 1";
            urdfGenerator(sensorData,sensorPosition);
            break;
        }
        case 12 : {
            sensorData.parent = "topPlate";
            sensorData.child = "lms1xx";
            sensorData.xyz = "0.0 0.0 0.0";
            sensorData.rpy = "3.14159265359 0.0 0.0";
            sensorData.material = "velodyneGray";
            sensorData.rgba = "0.81 0.81 0.81 1";
            urdfGenerator(sensorData,sensorPosition);
            break;
        }
	}
	return xacro_include(sensorData,sensorPosition);
}
