//Nikolay Radaev project 1 cs410

#include <string>
#include <math.h>
#include <vector>
#include <map>
#include <cstring> 
#include <iostream>
#include <sstream> 
#include <fstream>
#include <stdlib.h>

using namespace std;


// model array
vector<string> modelArray;

map<string, int> objCount;

// struct model to operate
class Model {
	public:
		double rotateWX;
		double rotateWY;
		double rotateWZ;
		double rotateTHETA;
		double scale;
		double translateTX;
		double translateTY;
		double translateTZ;
		string modelName;
};

class FileObj {
	public:
		string pathToSave;
		vector <double> vertX;
		vector <double> vertY;
		vector <double> vertZ;
		vector <string> face1;
		vector <string> face2;
		vector <string> face3;
};

void splitModelLine(string line, char delimiter) {
	stringstream ss(line); // Turn the string into a stream.
	string token;

	while (getline(ss, token, delimiter)) {
		modelArray.push_back(token);
	}
}

FileObj transformModel(Model modelObject, FileObj objectFile) {

	// let's rotate
	//create a W
	double W[3] = {modelObject.rotateWX, modelObject.rotateWY, modelObject.rotateWZ};

	// normalize W
	double s = sqrt(modelObject.rotateWX * modelObject.rotateWX + modelObject.rotateWY * modelObject.rotateWY + modelObject.rotateWZ * modelObject.rotateWZ);
	W[0] = modelObject.rotateWX / s; W[1] = modelObject.rotateWY / s; W[2] = modelObject.rotateWZ / s;

	//make axis M NOT parrallel to W from W
	double M[3];
	M[0] = W[0]; M[1] = W[1]; M[2] = W[2];

	int minIndex = 0;

	for (int i = 0; i < 3; i++) {
		// chose minimum
		if (abs(M[i]) <= abs(M[minIndex])) minIndex = i;
	}
	// assign min to one
	M[minIndex] = 1;

	cout << "M after changed min to 1: " << '\n';

	for (int i = 0; i < 3; i++) {
			cout << "| " << to_string(M[i]) << " |";
		}
	cout << '\n';

	// normalize M
	s = sqrt(M[0] * M[0] + M[1] * M[1] + M[2] * M[2]);
	M[0] = M[0] / s; M[1] = M[1] / s; M[2] = M[2] / s;

	cout << "M normalized: " << '\n';


	for (int i = 0; i < 3; i++) {
			cout << "| " << to_string(M[i]) << " |";
		}
		cout << '\n';

	// create U = W x M; cross product
	double U[3] = {W[1] * M[2] - W[2] * M[1], 
					W[2] * M[0] - W[0] * M[2],
					W[0] * M[1] - W[1] * M[0]};

	// normalize U
	s = sqrt(U[0] * U[0] + U[1] * U[1] + U[2] * U[2]);
	U[0] = U[0] / s; U[1] = U[1] / s; U[2] = U[2] / s;
	
	// create V = W x U;
	double V[3] = {W[1] * U[2] - W[2] * U[1],
					W[2] * U[0] - W[0] * U[2],
					W[0] * U[1] - W[1] * U[0]};

	// normalize V
	s = sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
	V[0] = V[0] / s; V[1] = V[1] / s; V[2] = V[2] / s;

	// now create Rotational Rw Matrix
	double Rw[3][3];

	Rw[0][0] = U[0]; Rw[0][1] = U[1]; Rw[0][2] = U[2];
	Rw[1][0] = V[0]; Rw[1][1] = V[1]; Rw[1][2] = V[2];
	Rw[2][0] = W[0]; Rw[2][1] = W[1]; Rw[2][2] = W[2];


	cout << "Rw matrix: " << '\n';

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cout << "| " << to_string(Rw[i][j]) << " |";
		}
		cout << '\n';
	}

	// let's transpose Rotatonal Matrix
	double Rt[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Rt[j][i] = Rw[i][j];
		}
	}

	cout << "Rt matrix: " << '\n';

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cout << "| " << to_string(Rt[i][j]) << " |";
		}
		cout << '\n';
	}

	// now create Rotational Matrix about Z
	double Rz[3][3];
	// convert theta to radians
	double radians = modelObject.rotateTHETA * 0.0174533;

	Rz[0][0] = cos(radians);	Rz[0][1] = (-1) * sin(radians);	Rz[0][2] = 0;
	Rz[1][0] = sin(radians);	Rz[1][1] = cos(radians);		Rz[1][2] = 0;
	Rz[2][0] = 0;				Rz[2][1] = 0;					Rz[2][2] = 1;


	cout << "Rz matrix: " << '\n';

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cout << "| " << to_string(Rz[i][j]) << " |";
		}
		cout << '\n';
	}

	//let's multiply 3 x 3 matrices

	double RzRw[3][3];
	double temp = 0;

	for (int c = 0; c < 3; c++) {
		for (int b = 0; b < 3; b++) {
			temp = 0;
			for (int a = 0; a < 3; a++) {
				temp = temp + Rz[c][a] * Rw[a][b];
			}
			RzRw[c][b] = temp;
		}
	}

	cout << "what ? " << '\n';

	cout << "RzRw matrix: " << '\n';

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cout << "| " << to_string(RzRw[i][j]) << " |";
		}
		cout << '\n';
	}


	double RtRzRw[3][3];
	temp = 0;

	for (int c = 0; c < 3; c++) {
		for (int b = 0; b < 3; b++) {
			for (int a = 0; a < 3; a++) {
				temp = temp + Rt[c][a] * RzRw[a][b];
			}
			RtRzRw[c][b] = temp;
			temp = 0;
		}
	}

	cout << "RtRzRw matrix: " << '\n';

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cout << "| " << to_string(RtRzRw[i][j]) << " |";
		}
		cout << '\n';
	}
	//let's add missing numbers and print 4x4


	double R[4][4];

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R[i][j] = RtRzRw[i][j];
		}
	}

													R[0][3] = 0;
													R[1][3] = 0;
													R[2][3] = 0;
	R[3][0] = 0;	R[3][1] = 0;	R[3][2] = 0;	R[3][3] = 1;

	cout << "R matrix: " << '\n';

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cout << "| " << to_string(R[i][j]) << " |";
		}
		cout << '\n';
	}

	// let's create 4x4 scale matrix, we have uniform scale, so we're good

	double sc = modelObject.scale;

	double S[4][4];

	S[0][0] = sc;	S[0][1] = 0;	S[0][2] = 0;	S[0][3] = 0; 
	S[1][0] = 0;	S[1][1] = sc;	S[1][2] = 0;	S[1][3] = 0;
	S[2][0] = 0;	S[2][1] = 0;	S[2][2] = sc;	S[2][3] = 0;
	S[3][0] = 0;	S[3][1] = 0;	S[3][2] = 0;	S[3][3] = 1;
	
	cout << "S matrix: " << '\n';

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cout << "| " << to_string(S[i][j]) << " |";
		}
		cout << '\n';
	}

	// let's create 4x4 translate matrix

	double tx = modelObject.translateTX;
	double ty = modelObject.translateTY;
	double tz = modelObject.translateTZ;

	double T[4][4];

	T[0][0] = 1;	T[0][1] = 0;	T[0][2] = 0;	T[0][3] = tx;
	T[1][0] = 0;	T[1][1] = 1;	T[1][2] = 0;	T[1][3] = ty;
	T[2][0] = 0;	T[2][1] = 0;	T[2][2] = 1;	T[2][3] = tz;
	T[3][0] = 0;	T[3][1] = 0;	T[3][2] = 0;	T[3][3] = 1;

	cout << "T matrix: " << '\n';

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cout << "| " << to_string(T[i][j]) << " |";
		}
		cout << '\n';
	}

	// lets multiply TxS 4x4 matrices

	double TS[4][4];
	temp = 0;

	for (int c = 0; c < 4; c++) {
		for (int b = 0; b < 4; b++) {
			for (int a = 0; a < 4; a++) {
				temp = temp + T[c][a] * S[a][b];
			}
			TS[c][b] = temp;
			temp = 0;
		}
	}

	cout << "TS matrix: " << '\n';

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cout << "| " << to_string(TS[i][j]) << " |";
		}
		cout << '\n';
	}

	// lets multiply TxSR 4x4 matrices

	double TSR[4][4];
	temp = 0;

	for (int c = 0; c < 4; c++) {
		for (int b = 0; b < 4; b++) {
			for (int a = 0; a < 4; a++) {
				temp = temp + TS[c][a] * R[a][b];
			}
			TSR[c][b] = temp;
			temp = 0;
		}
	}

	cout << "Final transformational TSR matrix: " << '\n';

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cout << "| " << to_string(TSR[i][j]) << " |";
		}
		cout << '\n';
	}

	// let's multiply it by vertex coordinates

	for (size_t i = 0; i != objectFile.vertX.size(); i++) { // i am going each row of verticies

		double vertex[4]; // making a copy to array, so I can use them
		vertex[0] = objectFile.vertX[i];
		vertex[1] = objectFile.vertY[i];
		vertex[2] = objectFile.vertZ[i];
		vertex[3] = 1;

		double vertexTemp[4] = {0, 0, 0, 0} ; // having a temp vertex array, so I can produce new vertices and use them without interfering

		for (int j = 0; j < 4; j++) {
			vertexTemp[j] = TSR[j][0] * vertex[0] + TSR[j][1] * vertex[1] + TSR[j][2] * vertex[2] + TSR[j][3] * vertex[3];
		}

		objectFile.vertX[i] = vertexTemp[0]; // assigning results
		objectFile.vertY[i] = vertexTemp[1]; // assigning results
		objectFile.vertZ[i] = vertexTemp[2]; // assigning results
	}

	return objectFile;
}


int main(int argc, char* argv[]) {

	Model modelObject;

	if (argc != 2) {
		cout << "Incorrect number of arguments" << '\n';
		exit(1);
 	}

	ifstream driverFile (argv[1]);

	if (!driverFile.is_open()) {
		cout << "Error opening the driver file" << '\n';
		exit(1);
	}

	string line = "";
	
	while (getline(driverFile, line)) {
		if (line.substr(0, 6) == "model ") {

			modelArray.clear(); // zero model array

			memset(&modelObject, 0, sizeof(Model)); // zero class

			splitModelLine(line.substr(6, line.length()), ' ');

			// converting vector to beautiful string
			modelObject.rotateWX = stod(modelArray[0]);
			modelObject.rotateWY = stod(modelArray[1]);
			modelObject.rotateWZ = stod(modelArray[2]);
			modelObject.rotateTHETA = stod(modelArray[3]);
			modelObject.scale = stod(modelArray[4]);
			modelObject.translateTX = stod(modelArray[5]);
			modelObject.translateTY = stod(modelArray[6]);
			modelObject.translateTZ = stod(modelArray[7]);
			modelObject.modelName = modelArray[8];


			// remove newline character
			if (modelObject.modelName[modelObject.modelName.length() - 1] == '\r') {
				modelObject.modelName = modelObject.modelName.substr(0, modelObject.modelName.length() - 5);
			}
			else {
				modelObject.modelName = modelObject.modelName.substr(0, modelObject.modelName.length() - 4);
			}

			// printing structure for help
			cout << "rotateWX: " << modelObject.rotateWX << '\n';
			cout << "rotateWY: " << modelObject.rotateWY << '\n';
			cout << "rotateWZ: " << modelObject.rotateWZ << '\n';
			cout << "rotate THETA: " << modelObject.rotateTHETA << '\n';
			cout << "scale: " << modelObject.scale << '\n';
			cout << "translateTX: " << modelObject.translateTX << '\n';
			cout << "translateTY: " << modelObject.translateTY << '\n';
			cout << "translateTZ: " << modelObject.translateTZ << '\n';
			cout << "modelName: " << modelObject.modelName << '\n';
			
			string originFile = modelObject.modelName + ".obj";

			cout << "Origin file: " << originFile << '\n';

			objCount[modelObject.modelName] = objCount[modelObject.modelName] + 1;

			string x = to_string(objCount[modelObject.modelName] - 1);

			string count = to_string(objCount[modelObject.modelName]);

			string saveFilename = modelObject.modelName.append("_mw0");
			saveFilename = saveFilename.append(x);
			saveFilename.append(".obj");

			cout << "Proper filename to save to: " << saveFilename << '\n';
			// your program should write cube_mw00.obj, ellelltri_mw00.obj, and cube_mw01.obj
			// to the disk under a folder named driver00.

			string saveFoldername = argv[1];

			saveFoldername = saveFoldername.substr(0, saveFoldername.length() - 4);

			cout << "Proper folder name: " << saveFoldername << '\n';

			FileObj objectFile;


			// now stream the obj 

			ifstream objFileStream(originFile);

			if (!objFileStream.is_open()) {
				cout << "Error opening the object file" << '\n';
				exit(1);
			}

			memset(&objectFile, 0, sizeof(FileObj)); // zero class

			objectFile.pathToSave = saveFoldername + '/' + saveFilename; // complete path to save the obect!

			string line = "";

			while (getline(objFileStream, line)) {
				if (line.substr(0, 2) == "v ") {

					string line1 = line.substr(2, line.length());

					stringstream ss(line1); // Turn the string into a stream.
					string token;

					getline(ss, token, ' ');
					objectFile.vertX.push_back(stod(token)); // save to x

					getline(ss, token, ' '); //getline again
					objectFile.vertY.push_back(stod(token)); // save to x

					getline(ss, token, ' '); //getline again
					objectFile.vertZ.push_back(stod(token)); // save to x
				}

				else if (line.substr(0, 2) == "f ") {

					string line1 = line.substr(2, line.length());

					stringstream ss(line1); // Turn the string into a stream.
					string token;

					getline(ss, token, ' ');
					objectFile.face1.push_back(token); // save to x

					getline(ss, token, ' '); //getline again
					objectFile.face2.push_back(token); // save to x

					getline(ss, token, ' '); //getline again
					objectFile.face3.push_back(token); // save to x
				}
			}

				// lets print the obect data
				cout << "Path to save object file: " << objectFile.pathToSave << '\n';

				for (size_t i = 0; i != objectFile.vertX.size(); i++) {
					cout << "Vertex: " << objectFile.vertX[i] << ' ' << objectFile.vertY[i] << ' ' << objectFile.vertZ[i] << '\n';
				}

				for (size_t i = 0; i != objectFile.face1.size(); i++) {
					cout << "Face: " << objectFile.face1[i] << ' ' << objectFile.face2[i] << ' ' << objectFile.face3[i] << '\n';
				}
				
				// now rotate, scale and translate
				FileObj newObjectFile = transformModel(modelObject, objectFile);
				
				// after the transformations
				cout << "After transformations: " << '\n';

				for (size_t i = 0; i != newObjectFile.vertX.size(); i++) {
					cout << "v " << newObjectFile.vertX[i] << ' ' << newObjectFile.vertY[i] << ' ' << newObjectFile.vertZ[i] << '\n';
				}

				for (size_t i = 0; i != newObjectFile.face1.size(); i++) {
					cout << "f " << newObjectFile.face1[i] << ' ' << newObjectFile.face2[i] << ' ' << newObjectFile.face3[i] << '\n';
				}

				// now save

				
				cout << "Saving now..." << '\n';

				// let's create a folder

				string systemCommand = "mkdir " + saveFoldername;

				system(systemCommand.c_str());

				ofstream savingFile;

				savingFile.open(objectFile.pathToSave);

				for (size_t i = 0; i != newObjectFile.vertX.size(); i++) {
					savingFile << "v " << newObjectFile.vertX[i] << ' ' << newObjectFile.vertY[i] << ' ' << newObjectFile.vertZ[i] << '\n';
				}

				for (size_t i = 0; i != newObjectFile.face1.size(); i++) {
					savingFile << "f " << newObjectFile.face1[i] << ' ' << newObjectFile.face2[i] << ' ' << newObjectFile.face3[i] << '\n';
				}

				savingFile.close();

				cout << "Saved successfully..." << '\n';

				objFileStream.close();
		}
	}

	driverFile.close();

	return 0;
}
