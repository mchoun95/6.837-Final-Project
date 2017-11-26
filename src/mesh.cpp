#include "mesh.h"

#include "vertexrecorder.h"

using namespace std;

void Mesh::load( const char* filename )
{
	// 4.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.
	// Load the mesh from file here.
	ifstream mesh_file(filename);			//create input file stream

											//extraction variables
	char obj_type;
	Vector3f v;
	Tuple3u face;

	//extracting data line by line
	while (mesh_file.good()) {
		//Load object type
		mesh_file >> obj_type;

		//If it is a vertex
		if (obj_type == 'v') {
			//load data into a vertex vector
			mesh_file >> v[0];
			mesh_file >> v[1];
			mesh_file >> v[2];

			//add the vertex vector to the vecotr of vertices
			bindVertices.push_back(v);
		}
		//else if it is a face
		else if (obj_type == 'f') {
			//extraction variables
			int a, d, g;

			//load data
			mesh_file >> a;

			mesh_file >> d;

			mesh_file >> g;

			//place data in tuple and add to list of faces
			face[0] = a - 1;
			face[1] = d - 1;
			face[2] = g - 1;

			faces.push_back(face);

		}
	}

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// 4.2 Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".
	VertexRecorder rec;

	for (unsigned i = 0; i < faces.size(); i++) {
		//take a face
		Tuple3u face = faces[i];
		int m = face[0];
		//generate a normal
		Vector3f n;
		n = Vector3f(face[0], face[1], face[2]);
		Vector3f edge1 = currentVertices[face[0]] - currentVertices[face[1]];
		Vector3f edge2 = currentVertices[face[0]] - currentVertices[face[2]];
		n = Vector3f::cross(edge1, edge2).normalized();


		//load the vertices and generated normal
		rec.record(currentVertices[faces[i][0]], n);
		rec.record(currentVertices[faces[i][1]], n);
		rec.record(currentVertices[faces[i][2]], n);
		
	}
	rec.draw();
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 4.3. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
	ifstream attach_file(filename);			//create input file stream

	//extraction variables
	float attach_joint;
	std::vector<float> attach_joints;

	//extracting data line by line
	while (attach_file.good()) {
		for (unsigned i = 0; i < numJoints - 1; i++) {
			attach_file >> attach_joint;
			attach_joints.push_back(attach_joint);
		}
		attachments.push_back(attach_joints);
		attach_joints.clear();
	}
}
