#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool Threshold(Vertex* vertices, float& edge_1, float& edge_2, float& edge_3, float threshold)
{
	if(edge_1<=threshold && edge_2<=threshold && edge_3<=threshold)
	{
		return true;
	}
	return false;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.1f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;

	unsigned int MAX = 2 *(width-1)*(height-1);

	std::vector<std::string> vTriangle(MAX);

	for(unsigned int row=0; row < height-1; ++row)
	{
		for(unsigned int col=0; col<width-1; ++col)
		{
			unsigned int idx_1 = row * width + col;
			unsigned int idx_2 = row * width + col + 1;
			unsigned int idx_3 = (row + 1) * width + col;
			unsigned int idx_4 = (row + 1)* width + col + 1;

			float edge_1 = (vertices[idx_1].position - vertices[idx_2].position).norm();
			float edge_2 = (vertices[idx_2].position - vertices[idx_4].position).norm();
			float edge_3 = (vertices[idx_4].position - vertices[idx_3].position).norm();
			float edge_4 = (vertices[idx_3].position - vertices[idx_1].position).norm();
			float edge_5 = (vertices[idx_1].position - vertices[idx_4].position).norm();

			if (Threshold(vertices,	edge_1, edge_2, edge_5, edgeThreshold))
			{
				vTriangle[nFaces] = std::to_string(idx_1) + " " + std::to_string(idx_2) + " " + std::to_string(idx_4);
				nFaces++;
			}
			if (Threshold(vertices,	edge_3, edge_4, edge_5, edgeThreshold))
			{
				vTriangle[nFaces] = std::to_string(idx_1) + " " + std::to_string(idx_3) + " " + std::to_string(idx_4);
				nFaces++;
			}
		}
	}	


	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for (unsigned int idx = 0; idx < height*width; ++idx)
	{
		if (vertices[idx].position[0]==MINF)
		{
			outFile << 0.0 << " "  << 0.0 << " " << 0.0 << " "
					<< vertices[idx].color[0] <<" "
					<< vertices[idx].color[1] <<" "
					<< vertices[idx].color[2] << std::endl;
		}
		else
		{
			outFile << vertices[idx].position[0] << " "
					<< vertices[idx].position[1] << " "
					<< vertices[idx].position[2] << " "
					<< (int)vertices[idx].color[0] <<" "
					<< (int)vertices[idx].color[1] <<" "
					<< (int)vertices[idx].color[2] <<" "
					<< (int)vertices[idx].color[3] << std::endl;
		}
	}


	// TODO: save valid faces
	std::cout << "# list of faces" << std::endl;
	std::cout << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
	for (std::vector<std::string>::iterator v = vTriangle.begin(); v != vTriangle.end(); ++v)
	{
		outFile << "3" <<" "<< *v << std::endl;
	}


	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];


		
		for(unsigned int x=0; x < sensor.GetDepthImageWidth(); ++x)
		{
			for(unsigned int y=0; y < sensor.GetDepthImageHeight(); ++y)
			{
				unsigned int idx = y * sensor.GetDepthImageWidth() + x;
				if (depthMap[idx] == MINF)
				{
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF); 
					vertices[idx].color = Vector4uc(0,0,0,0);
				}
				else
				{
					float Z = depthMap[idx];
					float X = (x - cX) * Z / fX;
					float Y = (y - cY) * Z / fY;

					Vector4f Homo_P = Vector4f(X, Y, Z, 1.0f);

					vertices[idx].position = trajectoryInv * depthExtrinsicsInv * Homo_P;
					vertices[idx].color[0] = colorMap[idx*4 + 0];
					vertices[idx].color[1] = colorMap[idx*4 + 1];
					vertices[idx].color[2] = colorMap[idx*4 + 2];
					vertices[idx].color[3] = colorMap[idx*4 + 3];
					

				}
			}
		}
	

	

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	
	}
	
	return 0;
}