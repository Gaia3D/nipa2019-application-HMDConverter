

#include "stdafx.h"

#include "utility.h"

#include "../geometry/TrianglePolyhedron.h"

#define EdgeLengthComparisonTolerance 10E-5
#define RotationAxisVectorMagnitudeTolerance 10E-7
#define PointCoincidenceTolerance 10E-4
#define AngleComparisonTolerance 10E-9


namespace gaia3d
{
	void GeometryUtility::calculatePlaneNormal(double& x0, double& y0, double& z0,
												double& x1, double& y1, double& z1,
												double& x2, double& y2, double& z2,
												double& nX, double& nY, double& nZ,
												bool bNormalize)
	{
		double edgeX1 = x1 - x0, edgeY1 = y1 - y0, edgeZ1 = z1 - z0;
		double edgeX2 = x2 - x0, edgeY2 = y2 - y0, edgeZ2 = z2 - z0;

		crossProduct(edgeX1, edgeY1, edgeZ1, edgeX2, edgeY2, edgeZ2, nX, nY, nZ);

		if(bNormalize)
		{
			double magnitude = sqrt(nX*nX + nY*nY + nZ*nZ);
			nX /= magnitude;
			nY /= magnitude;
			nZ /= magnitude;
		}
	}

	void GeometryUtility::crossProduct(double x1, double y1, double z1,
										double x2, double y2, double z2,
										double& nX, double& nY, double& nZ)
	{
		nX = y1*z2 - z1*y2;
		nY = z1*x2 - x1*z2;
		nZ = x1*y2 - y1*x2;
	}

	bool GeometryUtility::areTwoCongruentWithEachOther(void* geom1, void* geom2, void* transform, double tolerance, GeomType geomType)
	{
		// TODO(khj 20170317) : 나중에 geometry 타입의 종속성이 없도록 제거해야 한다.
		switch(geomType)
		{
		case POLYHEDRON:
			{
				TrianglePolyhedron* candidate = (gaia3d::TrianglePolyhedron*)geom1;
				TrianglePolyhedron* original = (gaia3d::TrianglePolyhedron*)geom2;

				// check if both vertex counts are same with each other
				size_t candidateVertexCount = candidate->getVertices().size();
				size_t originalVertexCount = original->getVertices().size();
				if(candidateVertexCount != originalVertexCount)
					return false;

				// check if two triangles composed of first three vertices in both polyhedrons respectively are congruent with each other
				Triangle tr1, tr2;
				Vertex v11, v12, v13, v21, v22, v23;
				v11.position = candidate->getVertices()[0]->position;
				v12.position = candidate->getVertices()[1]->position;
				v13.position = candidate->getVertices()[2]->position;
				v21.position = original->getVertices()[0]->position;
				v22.position = original->getVertices()[1]->position;
				v23.position = original->getVertices()[2]->position;
				tr1.setVertices(&v11, &v12, &v13);
				tr2.setVertices(&v21, &v22, &v23);
				if(!areTwoCongruentWithEachOther(&tr1, &tr2, NULL, EdgeLengthComparisonTolerance, TRIANGLE))
					return false;

				// check if we can make the first triangle of first geom coincide with that of second geom
				// by two rotational transforms and one translational transform
				Point3D rotAxis1, rotAxis2;
				double angle1, angle2; // rotational angles
				bool isFirstAxisZero = true, isSecondAxisZero = true;
				Matrix4 rotMat1, rotMat2;

				// first rotation axis for first rotational transform is made by cross product between first edges in both triangles.
				Point3D firstEdgeVec1, firstEdgeVec2;
				firstEdgeVec1.set(tr1.getVertices()[1]->position.x - tr1.getVertices()[0]->position.x,
								tr1.getVertices()[1]->position.y - tr1.getVertices()[0]->position.y,
								tr1.getVertices()[1]->position.z - tr1.getVertices()[0]->position.z);

				firstEdgeVec2.set(tr2.getVertices()[1]->position.x - tr2.getVertices()[0]->position.x,
								tr2.getVertices()[1]->position.y - tr2.getVertices()[0]->position.y,
								tr2.getVertices()[1]->position.z - tr2.getVertices()[0]->position.z);

				rotAxis1 = firstEdgeVec1 ^ firstEdgeVec2;
				double magnitude1 = rotAxis1.magnitude();
				if( magnitude1 >= RotationAxisVectorMagnitudeTolerance || magnitude1 <= -RotationAxisVectorMagnitudeTolerance)
					isFirstAxisZero = false;

				if (isFirstAxisZero)
				{
					// check if angle is zero or 180 degrees
					if (firstEdgeVec1.x * firstEdgeVec2.x + firstEdgeVec1.y*firstEdgeVec2.y + firstEdgeVec1.z*firstEdgeVec2.z > 0)
						rotMat1.identity();
					else
					{
						// at this case(angle == 180), set a vector which is parallel to triangle1's normal and goes through vertex1 of triangle1 as rot axis
						Point3D tri1Normal;
						calculatePlaneNormal(v11.position.x, v11.position.y, v11.position.z,
											v12.position.x, v12.position.y, v12.position.z,
											v13.position.x, v13.position.y, v13.position.z,
											tri1Normal.x, tri1Normal.y, tri1Normal.z, true);
						rotMat1.rotation(M_PI, &tri1Normal);
					}
				}
				else
				{
					rotAxis1.normalize();
					angle1 = angleBetweenTwoVectors(firstEdgeVec2.x, firstEdgeVec2.y, firstEdgeVec2.z, firstEdgeVec1.x, firstEdgeVec1.y, firstEdgeVec1.z);
					rotMat1.rotation(angle1, &rotAxis1);
				}

				// rotate second triangle. then first edges in both triangles will be parallel to each other.
				tr2.getVertices()[0]->position = rotMat1 * tr2.getVertices()[0]->position;
				tr2.getVertices()[1]->position = rotMat1 * tr2.getVertices()[1]->position;
				tr2.getVertices()[2]->position = rotMat1 * tr2.getVertices()[2]->position;

				// second rotation axis for second rotational transform is made by cross product between normal vectors of both triangles.
				calculatePlaneNormal(tr1.getVertices()[0]->position.x, tr1.getVertices()[0]->position.y, tr1.getVertices()[0]->position.z,
									tr1.getVertices()[1]->position.x, tr1.getVertices()[1]->position.y, tr1.getVertices()[1]->position.z,
									tr1.getVertices()[2]->position.x, tr1.getVertices()[2]->position.y, tr1.getVertices()[2]->position.z,
									tr1.getNormal()->x, tr1.getNormal()->y, tr1.getNormal()->z,
									true);
				calculatePlaneNormal(tr2.getVertices()[0]->position.x, tr2.getVertices()[0]->position.y, tr2.getVertices()[0]->position.z,
									tr2.getVertices()[1]->position.x, tr2.getVertices()[1]->position.y, tr2.getVertices()[1]->position.z,
									tr2.getVertices()[2]->position.x, tr2.getVertices()[2]->position.y, tr2.getVertices()[2]->position.z,
									tr2.getNormal()->x, tr2.getNormal()->y, tr2.getNormal()->z,
									true);
				Point3D* normal1 = tr1.getNormal();
				Point3D* normal2 = tr2.getNormal();
				rotAxis2 = *normal1 ^ *normal2;

				double magnitude2 = rotAxis2.magnitude();
				if(magnitude2 >= RotationAxisVectorMagnitudeTolerance || magnitude2 <= -RotationAxisVectorMagnitudeTolerance)
					isSecondAxisZero = false;

				if (isSecondAxisZero)
				{
					// check if angle is zero or 180 degrees
					if(normal1->x * normal2->x + normal1->y * normal2->y + normal1->z * normal2->z > 0)
						rotMat2.identity();
					else
					{
						// at this case(angle = 180), set the first edge of triangle1 as a rot axis
						firstEdgeVec1.normalize();
						rotMat2.rotation(M_PI, &firstEdgeVec1);
					}
				}
				else
				{
					rotAxis2.normalize();
					angle2 = angleBetweenTwoVectors(normal2->x, normal2->y, normal2->z, normal1->x, normal1->y, normal1->z);
					rotMat2.rotation(angle2, &rotAxis2);
				}
					

				// rotate second triangle. then the plane of second triangle will be parallel to that of first triangle.
				tr2.getVertices()[0]->position = rotMat2 * tr2.getVertices()[0]->position;
				tr2.getVertices()[1]->position = rotMat2 * tr2.getVertices()[1]->position;
				tr2.getVertices()[2]->position = rotMat2 * tr2.getVertices()[2]->position;

				// at this time, two triangles are parallel to each other.
				// let's find translational transform to make second trangle coincide with first triangle.
				Point3D transVec = tr1.getVertices()[0]->position - tr2.getVertices()[0]->position;
				Matrix4 transMat, *finalTransform;
				transMat.translation(&transVec);
				finalTransform = (Matrix4*)transform;

				*finalTransform = (rotMat1 * rotMat2) * transMat; // final transform matrix

				// finally, check if all original mesh vertices multiplied by final transform coincide with all candidate mesh vertices
				Point3D translatedPoint;
				for(size_t i = 3; i < originalVertexCount; i++)
				{
					translatedPoint = *finalTransform * (original->getVertices()[i]->position);
					if(!areTwoCongruentWithEachOther(&(candidate->getVertices()[i]->position), &translatedPoint, NULL, PointCoincidenceTolerance, POINT))
						return false;
				}
			}
			break;
		case TRIANGLE:
			{
				gaia3d::Triangle* tr1 = (gaia3d::Triangle*)geom1;
				gaia3d::Triangle* tr2 = (gaia3d::Triangle*)geom2;

				double edgeLength1[3], edgeLength2[3];
				edgeLength1[0] = sqrt(tr1->getVertices()[0]->position.squaredDistanceTo(tr1->getVertices()[1]->position));
				edgeLength1[1] = sqrt(tr1->getVertices()[1]->position.squaredDistanceTo(tr1->getVertices()[2]->position));
				edgeLength1[2] = sqrt(tr1->getVertices()[2]->position.squaredDistanceTo(tr1->getVertices()[0]->position));
				edgeLength2[0] = sqrt(tr2->getVertices()[0]->position.squaredDistanceTo(tr2->getVertices()[1]->position));
				edgeLength2[1] = sqrt(tr2->getVertices()[1]->position.squaredDistanceTo(tr2->getVertices()[2]->position));
				edgeLength2[2] = sqrt(tr2->getVertices()[2]->position.squaredDistanceTo(tr2->getVertices()[0]->position));

				if( edgeLength1[0] >= edgeLength2[0] + tolerance || edgeLength1[0] <= edgeLength2[0] - tolerance ||
					edgeLength1[1] >= edgeLength2[1] + tolerance || edgeLength1[1] <= edgeLength2[1] - tolerance ||
					edgeLength1[2] >= edgeLength2[2] + tolerance || edgeLength1[2] <= edgeLength2[2] - tolerance )
					return false;
			}
			break;
		case POINT:
			{
				Point3D* point1 = (Point3D*)geom1;
				Point3D* point2 = (Point3D*)geom2;

				if( point1->x >= point2->x + tolerance || point1->x <= point2->x - tolerance ||
					point1->y >= point2->y + tolerance || point1->y <= point2->y - tolerance ||
					point1->z >= point2->z + tolerance || point1->z <= point2->z - tolerance )
					return false;

			}
			break;
		}

		return true;
	}

	double GeometryUtility::angleBetweenTwoVectors(double x1, double y1, double z1, double x2, double y2, double z2)
	{
		double angle;
		double dotProduct, dotProductMax;

		dotProductMax= sqrt(x1*x1 + y1*y1 + z1*z1)*sqrt(x2*x2 + y2*y2 + z2*z2);
		angle=0.0;
		if(dotProductMax >= AngleComparisonTolerance || dotProductMax <= -AngleComparisonTolerance)
		{
			dotProduct= x1*x2 + y1*y2+ z1*z2;
			if(dotProduct/dotProductMax<=-1.0) angle=M_PI;
			else
			{
				if(dotProduct/dotProductMax>=1.0) angle=0.0;
				else angle = acos(dotProduct/dotProductMax);
			}
		}

		return angle;
	}

	bool GeometryUtility::isInsideBox(double x, double y, double z,
									double minX, double minY, double minZ,
									double maxX, double maxY, double maxZ)
	{
		if( x <= minX || x >= maxX || y <= minY || y >= maxY || z <= minZ || z >= maxZ)
			return false;

		return true;
	}

	bool GeometryUtility::doesTriangleIntersectWithBox(double& x0, double& y0, double& z0,
														double& x1, double& y1, double& z1,
														double& x2, double& y2, double& z2,
														double& minX, double& minY, double& minZ,
														double& maxX, double& maxY, double& maxZ)
	{
		// SAT(Separating Axis Test)

		// 1. 3 axes projection test
		double triMaxX = (x0 > x1) ? ((x0 > x2) ? x0 : x2) : ((x1 > x2) ? x1 : x2);
		double triMaxY = (y0 > y1) ? ((y0 > y2) ? y0 : y2) : ((y1 > y2) ? y1 : y2);
		double triMaxZ = (z0 > z1) ? ((z0 > z2) ? z0 : z2) : ((z1 > z2) ? z1 : z2);
		double triMinX = (x0 > x1) ? ((x1 > x2) ? x2 : x1) : ((x0 > x2) ? x2 : x0);
		double triMinY = (y0 > y1) ? ((y1 > y2) ? y2 : y1) : ((y0 > y2) ? y2 : y0);
		double triMinZ = (z0 > z1) ? ((z1 > z2) ? z2 : z1) : ((z0 > z2) ? z2 : z0);

		double tolerance = 0.000001;
		if(triMaxX < minX - tolerance || triMinX > maxX + tolerance ||
			triMaxY < minY - tolerance || triMinY > maxY + tolerance ||
			triMaxZ < minZ - tolerance || triMinZ > maxZ + tolerance)
			return false;

		// 2. box projection on triangle normal test
		// 2-1. plane equation of triangle(ax + by + cz + d = 0)
		double a, b, c, d;
		calculatePlaneNormal(x0, y0, z0, x1, y1, z1, x2, y2, z2, a, b, c, true);
		d = -a*x1 -b*y1 -c*z1;
		// 2-2. plane equation tests for 8 corners of box
		double test[8];
		test[0] = a*minX + b*minY + c*minZ + d;
		test[1] = a*maxX + b*minY + c*minZ + d;
		test[2] = a*maxX + b*maxY + c*minZ + d;
		test[3] = a*minX + b*maxY + c*minZ + d;
		test[4] = a*minX + b*minY + c*maxZ + d;
		test[5] = a*maxX + b*minY + c*maxZ + d;
		test[6] = a*maxX + b*maxY + c*maxZ + d;
		test[7] = a*minX + b*maxY + c*maxZ + d;
		bool testPassed = false;
		char countTouchedPoint = 0;
		if (test[0] >= -10E-6 && test[0] <= 10E-6)
			countTouchedPoint++;

		for(int i = 1; i < 8; i++)
		{
			if(test[0]*test[i] < -10E-8)
			{
				testPassed = true;
				break;
			}

			if (test[i] >= -10E-6 && test[i] <= 10E-6)
			{
				countTouchedPoint++;
				if (countTouchedPoint > 2)
				{
					testPassed = true;
					break;
				}
			}
		}

		if(!testPassed)
			return false;

		// 3. test on 9 axes made by cross products between 3 axis vectors and 3 edge vectors of triangle
		// 3-1. edge vectors of triangle
		double edge[3][3];
		edge[0][0] = x1-x0, edge[0][1] = y1-y0, edge[0][2] = z1-z0;
		edge[1][0] = x2-x1, edge[1][1] = y2-y1, edge[1][2] = z2-z1;
		edge[2][0] = x0-x2, edge[2][1] = y0-y2, edge[2][2] = z0-z2;

		// 3-2. 9 axes made by cross product of 3 global axis vectors and 3 edge vectors
		double axis[3][3][3];
		/*
		axis[0][0][0] = 0.0, axis[0][0][1] = -edge[0][2], axis[0][0][2] = edge[0][1];
		axis[0][1][0] = 0.0, axis[0][1][1] = -edge[1][2], axis[0][1][2] = edge[1][1];
		axis[0][2][0] = 0.0, axis[0][2][1] = -edge[2][2], axis[0][2][2] = edge[2][1];

		axis[1][0][0] = edge[0][2], axis[1][0][1] = 0.0, axis[1][0][2] = -edge[0][0];
		axis[1][1][0] = edge[1][2], axis[1][1][1] = 0.0, axis[1][1][2] = -edge[1][0];
		axis[1][2][0] = edge[2][2], axis[1][2][1] = 0.0, axis[1][2][2] = -edge[2][0];

		axis[2][0][0] = -edge[0][1], axis[2][0][1] = edge[0][0], axis[2][0][2] = 0.0;
		axis[2][1][0] = -edge[1][1], axis[2][1][1] = edge[1][0], axis[2][1][2] = 0.0;
		axis[2][2][0] = -edge[2][1], axis[2][2][1] = edge[2][0], axis[2][2][2] = 0.0;
		*/

		// 3 global axis vector loop
		for(int i = 0; i < 3; i++)
		{
			// 3 edge vectors loop
			for(int j = 0; j < 3; j++)
			{
				// x, y, z loop
				for(int k = 0; k < 3; k++)
				{
					if(i == k)
						axis[i][j][k] = 0.0;
					else
					{
						if((i+1)%3 == k)
							axis[i][j][k] = -edge[j][(k+1)%3];
						else
							axis[i][j][k] = edge[j][(i+1)%3];
					}
				}
			}
		}

		// 3-3. center of box
		double cobX = (maxX+minX)/2.0, cobY = (maxY+minY)/2.0, cobZ = (maxZ+minZ)/2.0;

		// 3-4. translated 3 vertices of triangle by vector which translate cob into origin
		double v[3][3];
		v[0][0] = x0 - cobX, v[0][1] = y0 - cobY, v[0][2] = z0 - cobZ;
		v[1][0] = x1 - cobX, v[1][1] = y1 - cobY, v[1][2] = z1 - cobZ;
		v[2][0] = x2 - cobX, v[2][1] = y2 - cobY, v[2][2] = z2 - cobZ;

		// 3-5. half diagonal vector of box
		double hd[3];
		hd[0] = (maxX-minX)/2.0, hd[1] = (maxY-minY)/2.0, hd[2] = (maxZ-minZ)/2.0;

		double radius;
		// first 3 loop of 9 axes
		double vProj1, vProj2;
		for(int i = 0; i < 3; i++)
		{
			// second 3 loop of 9 axes
			for(int j = 0; j < 3; j++)
			{
				radius = hd[(i+1)%3]*fabs(axis[i][j][(i+1)%3]) + hd[(i+2)%3]*fabs(axis[i][j][(i+2)%3]);

				vProj1 = v[j][(i+2)%3]*v[(j+1)%3][(i+1)%3] - v[j][(i+1)%3]*v[(j+1)%3][(i+2)%3];
				vProj2 = (v[(j+1)%3][(i+1)%3] - v[j][(i+1)%3])*v[(j+2)%3][(i+2)%3] - (v[(j+1)%3][(i+2)%3] -v[j][(i+2)%3])*v[(j+2)%3][(i+1)%3];

				if( (vProj1 > vProj2) ? (vProj1 < -radius -10E-8 || vProj2 > radius + 10E-8) : (vProj2 < -radius -10E-8 || vProj1 > radius + 10E-8) )
					return false;
			}
		}

		return true;
	}

	void GeometryUtility::wgs84ToAbsolutePosition(double&lon, double& lat, double& alt, double* result)
	{
		//WGS 84.***************************************************
		// Extracted from WikiPedia "Geodetic datum".
		// WGS 84 Defining Parameters
		// semi-major axis	a	6378137.0 m
		// Reciprocal of flattening	1/f	298.257223563

		// WGS 84 derived geometric constants
		// Semi-minor axis	b = a(1 − f)	6356752.3142 m
		// First eccentricity squared	e2 = (1 − b2/a2 = 2f − f2) =	6.69437999014 x 10−3
		// Second eccentricity squared	e′2	= (a2/b2 − 1 = f(2 − f)/(1 − f)2) = 6.73949674228 x 10−3
		//----------------------------------------------------------

		// defined in the LINZ standard LINZS25000 (Standard for New Zealand Geodetic Datum 2000)
		// https://www.linz.govt.nz/data/geodetic-system/coordinate-conversion/geodetic-datum-conversions/equations-used-datum
		// a = semi-major axis.
		// e2 = firstEccentricitySquared.
		// v = a / sqrt(1 - e2 * sin2(lat)).
		// x = (v+h)*cos(lat)*cos(lon).
		// y = (v+h)*cos(lat)*sin(lon).
		// z = [v*(1-e2)+h]*sin(lat).

		double degToRadFactor = 0.017453292519943296; // 3.141592653589793 / 180.0;
		double equatorialRadius = 6378137.0;
		double firstEccentricitySquared = 6.69437999014E-3;
		double lonRad = lon *degToRadFactor;
		double latRad = lat * degToRadFactor;
		double cosLon = cos(lonRad);
		double cosLat = cos(latRad);
		double sinLon = sin(lonRad);
		double sinLat = sin(latRad);
		double a = equatorialRadius;
		double e2 = firstEccentricitySquared;
		double v = a / sqrt(1.0 - e2 * sinLat * sinLat);
		double h = alt;

		result[0] = (v + h)*cosLat*cosLon;
		result[1] = (v + h)*cosLat*sinLon;
		result[2] = (v*(1.0 - e2) + h)*sinLat;
	}

	void GeometryUtility::normalAtAbsolutePosition(double& x, double& y, double& z, double* result)
	{
		gaia3d::Point3D normal;

		double equatorialRadiusSquared = 40680631590769.0;
		double polarRadiusSquared = 40408299984087.05552164;

		normal.set(x / equatorialRadiusSquared, y / equatorialRadiusSquared, z / polarRadiusSquared);
		normal.normalize();

		result[0] = normal.x;
		result[1] = normal.y;
		result[2] = normal.z;
	}

	void GeometryUtility::transformMatrixAtAbsolutePosition(double& x, double& y, double& z, double* m)
	{
		gaia3d::Matrix4 matrix;

		gaia3d::Point3D xAxis, yAxis, zAxis;

		double normal[3];
		normalAtAbsolutePosition(x, y, z, normal);
		zAxis.set(normal[0], normal[1], normal[2]);

		xAxis.set(-y, x, 0.0);
		xAxis.normalize();

		yAxis = zAxis ^ xAxis;
		yAxis.normalize();

		matrix.set(	xAxis.x,	xAxis.y,	xAxis.z,	0.0,
					yAxis.x,	yAxis.y,	yAxis.z,	0.0,
					zAxis.x,	zAxis.y,	zAxis.z,	0.0,
					x,			y,			z,			1.0);

		matrix.getDoubleArray(m);
	}

	void findConcavePointsAndNormal(double* pxs, double* pys,
									std::vector<size_t>& indices,
									std::vector<size_t>& concavePointIndicesOnAllPoints,
									std::vector<size_t>& concavePointIndicesOnThisPolygon,
									int& normal)
	{
		double crossProd, dotProd, angle;
		size_t count = indices.size();
		size_t prevIndex, nextIndex;
		gaia3d::Point3D prevVector, nextVector;
		double lfNormal = 0.0;
		for (size_t i = 0; i < count; i++)
		{
			prevIndex = (i == 0) ? count - 1 : i - 1;
			nextIndex = (i == count - 1) ? 0 : i + 1;

			prevVector.set(pxs[indices[i]] - pxs[indices[prevIndex]], pys[indices[i]] - pys[indices[prevIndex]], 0.0);
			nextVector.set(pxs[indices[nextIndex]] - pxs[indices[i]], pys[indices[nextIndex]] - pys[indices[i]], 0.0);

			prevVector.normalize();
			nextVector.normalize();

			crossProd = prevVector.x*nextVector.y - prevVector.y*nextVector.x;
			dotProd = prevVector.x * nextVector.x + prevVector.y * nextVector.y;
			if (crossProd > 0.0)
			{
				crossProd = 1.0;
			}
			else if (crossProd < 0.0)
			{
				crossProd = -1;
				concavePointIndicesOnAllPoints.push_back(indices[i]);
				concavePointIndicesOnThisPolygon.push_back(i);
			}
			else
				continue;

			if (dotProd > 1.0)
				dotProd = 1.0;
			
			if (dotProd < -1.0)
				dotProd = -1.0;

			angle = acos(dotProd);
			
			lfNormal += (crossProd * angle);
		}

		normal = (lfNormal > 0.0) ? 1 : -1;
	}

	void tessellateIntoSubPolygons(double* pxs, double* pys,
									std::vector<size_t>& polygonVertexIndices,
									std::vector<size_t>& concavePointIndicesOnAllPoints,
									std::vector<size_t>& concavePointIndicesOnThisPolygon,
									int thisPolygonNormal,
									std::vector<std::vector<size_t>>& subPolygons)
	{
		// 0. use only 1st concave point.
		size_t concavePointIndexOnAllPoints = concavePointIndicesOnAllPoints[0];
		size_t concavePointIndexOnThisPolygon = concavePointIndicesOnThisPolygon[0];
		size_t polygonPointCount = polygonVertexIndices.size();
		gaia3d::Point3D concavePoint;
		concavePoint.set(pxs[concavePointIndexOnAllPoints], pys[concavePointIndexOnAllPoints], 0.0);

		// 1. sort all points by acsending distance from the concave point except the concave point and its 2 neighbor points.
		std::map<double, size_t> sortedPointIndicesOnAllPointsMap, sortedPointIndicesOnThisPolygonMap;
		size_t prevIndexOfConcavePointOnThisPolygon = (concavePointIndexOnThisPolygon == 0) ? polygonPointCount - 1 : concavePointIndexOnThisPolygon - 1;
		size_t nextIndexOfConcavePointOnThisPolygon = (concavePointIndexOnThisPolygon == polygonPointCount - 1) ? 0 : concavePointIndexOnThisPolygon + 1;
		gaia3d::Point3D targetPoint;
		for (size_t i = 0; i < polygonVertexIndices.size(); i++)
		{
			if (i == concavePointIndexOnThisPolygon ||
				i == prevIndexOfConcavePointOnThisPolygon ||
				i == nextIndexOfConcavePointOnThisPolygon)
				continue;

			targetPoint.set(pxs[polygonVertexIndices[i]], pys[polygonVertexIndices[i]], 0.0);
			sortedPointIndicesOnAllPointsMap[(targetPoint.squaredDistanceTo(concavePoint))] = polygonVertexIndices[i];
			sortedPointIndicesOnThisPolygonMap[(targetPoint.squaredDistanceTo(concavePoint))] = i;
		}
		
		std::vector<size_t> sortedPointIndicesOnAllPoints;
		std::vector<size_t> sortedPointIndicesOnThisPolygon;
		for (std::map<double, size_t>::iterator itr1 = sortedPointIndicesOnAllPointsMap.begin(), itr2 = sortedPointIndicesOnThisPolygonMap.begin();
			itr1 != sortedPointIndicesOnAllPointsMap.end();
			itr1++, itr2++)
		{
			sortedPointIndicesOnAllPoints.push_back(itr1->second);
			sortedPointIndicesOnThisPolygon.push_back(itr2->second);
		}
			

		// 2. find a point of this polygon where line segment composed of the point and concave point can slices this polygon
		double slicerStartX = pxs[concavePointIndexOnAllPoints], slicerStartY = pys[concavePointIndexOnAllPoints];
		double slicerEndX, slicerEndY;
		double targetStartX, targetStartY, targetEndX, targetEndY;
		gaia3d::Point3D normalVector;
		double dotProdLine1, dotProdLine2Start, dotProdLine2End;
		double dotProdLine2, dotProdLine1Start, dotProdLine1End;
		double tolerance = 1E-6;
		for (size_t i = 0; i < sortedPointIndicesOnAllPoints.size(); i++)
		{
			// 2.1 line interseciton test between the slicer and each edge of polygon
			slicerEndX = pxs[sortedPointIndicesOnAllPoints[i]];
			slicerEndY = pys[sortedPointIndicesOnAllPoints[i]];
			bool bIntersected = false, bIntersectedCase1, bIntersectedCase2;
			for (size_t j = 0; j < polygonPointCount-1; j++)
			{
				targetStartX = pxs[polygonVertexIndices[j]];
				targetStartY = pys[polygonVertexIndices[j]];
				targetEndX = pxs[polygonVertexIndices[j + 1]];
				targetEndY = pys[polygonVertexIndices[j + 1]];
				
				// find vector normal to line 1
				normalVector.set(-(slicerEndY - slicerStartY), slicerEndX - slicerStartX, 0.0);
				normalVector.normalize();

				// dot products of 4 nodes in 2 lines on this normal vector
				// (dot products of 2 nodes in line 1 on this normal vector are same)
				dotProdLine1 = normalVector.x * slicerStartX + normalVector.y * slicerStartY;
				dotProdLine2Start = normalVector.x * targetStartX + normalVector.y * targetStartY;
				dotProdLine2End = normalVector.x * targetEndX + normalVector.y * targetEndY;
				bIntersectedCase1 = true;
				if ((dotProdLine1 > dotProdLine2Start - tolerance && dotProdLine1 > dotProdLine2End - tolerance) ||
					(dotProdLine1 < dotProdLine2Start + tolerance && dotProdLine1 < dotProdLine2End + tolerance))
					bIntersectedCase1 = false;

				// find vector normal to line 2
				normalVector.set(-(targetEndY - targetStartY), targetEndX - targetStartY, 0.0);
				normalVector.normalize();

				// dot products of 4 nodes in 2 lines on this normal vector
				// (dot products of 2 nodes in line 2 on this normal vector are same)
				dotProdLine2 = normalVector.x * targetStartX + normalVector.y * targetStartY;
				dotProdLine1Start = normalVector.x * slicerStartX + normalVector.y * slicerStartY;
				dotProdLine1End = normalVector.x * slicerEndX + normalVector.y * slicerEndY;
				bIntersectedCase2 = true;
				if ((dotProdLine2 > dotProdLine1Start - tolerance && dotProdLine2 > dotProdLine1End - tolerance) ||
					(dotProdLine2 < dotProdLine1Start + tolerance && dotProdLine2 < dotProdLine1End + tolerance))
					bIntersectedCase2 = false;

				if (bIntersectedCase1 && bIntersectedCase2)
				{
					bIntersected = true;
					break;
				}
			}

			if (bIntersected)
				continue;

			// 2.2 test if the sliced 2 sub-polygons have same plane normal directions with original polygon
			std::vector<size_t> firstSubPolygonIndicesOnAllPoints;
			firstSubPolygonIndicesOnAllPoints.push_back(concavePointIndexOnAllPoints);
			firstSubPolygonIndicesOnAllPoints.push_back(polygonVertexIndices[sortedPointIndicesOnThisPolygon[i]]);
			for (size_t j = 1; j < polygonPointCount; j++)
			{
				firstSubPolygonIndicesOnAllPoints.push_back(polygonVertexIndices[(sortedPointIndicesOnThisPolygon[i] + j) % polygonPointCount]);
				if ((sortedPointIndicesOnThisPolygon[i] + j) % polygonPointCount == prevIndexOfConcavePointOnThisPolygon)
					break;
			}
			std::vector<size_t> secondSubPolygonIndicesOnAllPoints;
			secondSubPolygonIndicesOnAllPoints.push_back(polygonVertexIndices[sortedPointIndicesOnThisPolygon[i]]);
			secondSubPolygonIndicesOnAllPoints.push_back(concavePointIndexOnAllPoints);
			for (size_t j = 1; j < polygonPointCount; j++)
			{
				secondSubPolygonIndicesOnAllPoints.push_back(polygonVertexIndices[(concavePointIndexOnThisPolygon + j) % polygonPointCount]);
				if ((concavePointIndexOnThisPolygon + j) % polygonPointCount ==
					(sortedPointIndicesOnThisPolygon[i] + polygonPointCount - 1) % polygonPointCount)
					break;
			}
	
			int firstPolygonNormal, secondPolygonNormal;
			std::vector<size_t> concavePointsOfSubPolygon1IndicesOnAllPoints, concavePointsOfSubPolygon1IndicesOnSubPolygon1;
			std::vector<size_t> concavePointsOfSubPolygon2IndicesOnAllPoints, concavePointsOfSubPolygon2IndicesOnSubPolygon2;

			findConcavePointsAndNormal(pxs, pys,
										firstSubPolygonIndicesOnAllPoints,
										concavePointsOfSubPolygon1IndicesOnAllPoints,
										concavePointsOfSubPolygon1IndicesOnSubPolygon1,
										firstPolygonNormal);
			findConcavePointsAndNormal(pxs, pys,
										secondSubPolygonIndicesOnAllPoints,
										concavePointsOfSubPolygon2IndicesOnAllPoints,
										concavePointsOfSubPolygon2IndicesOnSubPolygon2,
										secondPolygonNormal);
			if (thisPolygonNormal != firstPolygonNormal || thisPolygonNormal != secondPolygonNormal)
				continue;

			// 2.3 make sub-polygons or tessellate more
			if (concavePointsOfSubPolygon1IndicesOnAllPoints.empty())
				subPolygons.push_back(firstSubPolygonIndicesOnAllPoints);
			else
				tessellateIntoSubPolygons(pxs, pys,
										firstSubPolygonIndicesOnAllPoints,
										concavePointsOfSubPolygon1IndicesOnAllPoints,
										concavePointsOfSubPolygon1IndicesOnSubPolygon1,
										firstPolygonNormal,
										subPolygons);

			if (concavePointsOfSubPolygon2IndicesOnAllPoints.empty())
				subPolygons.push_back(secondSubPolygonIndicesOnAllPoints);
			else
				tessellateIntoSubPolygons(pxs, pys,
										secondSubPolygonIndicesOnAllPoints,
										concavePointsOfSubPolygon2IndicesOnAllPoints,
										concavePointsOfSubPolygon2IndicesOnSubPolygon2,
										secondPolygonNormal,
										subPolygons);
			break;
		}
	}

	void GeometryUtility::tessellate(double* xs, double* ys, double* zs, size_t count, std::vector<size_t>& indices)
	{
		// 0. basic validation
		if (count < 3)
			return;

		if (count == 3)
		{
			indices.push_back(0);
			indices.push_back(1);
			indices.push_back(2);
			return;
		}

		// 1. calculate normal of this polygon
		gaia3d::Point3D normal, crossProd, prevVector, nextVector;
		double dotProd, angle;
		size_t prevIndex, nextIndex;
		normal.set(0.0, 0.0, 0.0);
		for (size_t i = 0; i < count; i++)
		{
			prevIndex = (i == 0) ? count - 1 : i - 1;
			nextIndex = (i == count - 1) ? 0 : i + 1;

			prevVector.set(xs[i] - xs[prevIndex], ys[i] - ys[prevIndex], zs[i] - zs[prevIndex]);
			nextVector.set(xs[nextIndex] - xs[i], ys[nextIndex] - ys[i], zs[nextIndex] - zs[i]);

			prevVector.normalize();
			nextVector.normalize();

			crossProd = prevVector ^ nextVector;
			crossProd.normalize();

			dotProd = prevVector.x * nextVector.x + prevVector.y * nextVector.y + prevVector.z * nextVector.z;
			angle = acos(dotProd);

			normal += (crossProd * angle);
		}
		normal.normalize();

		// 2. make projected polygon
		unsigned char projectionType; // 0 : onto x-y plane, 1 : onto y-z plane, 2 : onto z-x plane
		double nx = abs(normal.x);
		double ny = abs(normal.y);
		double nz = abs(normal.z);

		projectionType = (nz > nx) ? ((nz > ny) ? 0 : 2) : ((nx > ny) ? 1 : 2);
		double* pxs = new double[count];
		memset(pxs, 0x00, sizeof(double)*count);
		double* pys = new double[count];
		memset(pys, 0x00, sizeof(double)*count);

		switch (projectionType)
		{
		case 0:
		{
			if (normal.z > 0)
			{
				for (size_t i = 0; i < count; i++)
				{
					pxs[i] = xs[i];
					pys[i] = ys[i];
				}
			}
			else
			{
				for (size_t i = 0; i < count; i++)
				{
					pxs[i] = xs[i];
					pys[i] = -ys[i];
				}
			}
		}
		break;
		case 1:
		{
			if (normal.x > 0)
			{
				for (size_t i = 0; i < count; i++)
				{
					pxs[i] = ys[i];
					pys[i] = zs[i];
				}
			}
			else
			{
				for (size_t i = 0; i < count; i++)
				{
					pxs[i] = -ys[i];
					pys[i] = zs[i];
				}
			}
		}
		break;
		case 2:
		{
			if (normal.y > 0)
			{
				for (size_t i = 0; i < count; i++)
				{
					pxs[i] = -xs[i];
					pys[i] = zs[i];
				}
			}
			else
			{
				for (size_t i = 0; i < count; i++)
				{
					pxs[i] = xs[i];
					pys[i] = zs[i];
				}
			}
		}
		break;
		}

		// 3. find concave points and normal of this projected polygon
		std::vector<size_t> concavePointIndicesOnAllPoints;
		std::vector<size_t> concavePointIndicesOnThisPolygon;
		int projectedPolygonNormal;
		std::vector<size_t> polygonVertexIndices;
		for (size_t i = 0; i < count; i++)
			polygonVertexIndices.push_back(i);

		findConcavePointsAndNormal(pxs, pys, polygonVertexIndices, concavePointIndicesOnAllPoints, concavePointIndicesOnThisPolygon, projectedPolygonNormal);

		if (concavePointIndicesOnAllPoints.empty())
		{
			for (size_t i = 1; i < count - 1; i++)
			{
				indices.push_back(0);
				indices.push_back(i);
				indices.push_back(i + 1);
			}

			delete[] pxs;
			delete[] pys;
			return;
		}

		// 4. split this polygon into convex sub-polygons
		std::vector<std::vector<size_t>> subPolygons;
		tessellateIntoSubPolygons(pxs, pys, polygonVertexIndices, concavePointIndicesOnAllPoints, concavePointIndicesOnThisPolygon, projectedPolygonNormal, subPolygons);

		delete[] pxs;
		delete[] pys;

		// 5. split sub-polygons into triangles
		for (size_t i = 0; i < subPolygons.size(); i++)
		{
			for (size_t j = 1; j < (subPolygons[i]).size() - 1; j++)
			{
				indices.push_back((subPolygons[i])[0]);
				indices.push_back((subPolygons[i])[j]);
				indices.push_back((subPolygons[i])[j+1]);
			}
		}
	}

#ifdef _WIN32
#include <Windows.h>
#endif
	std::string StringUtility::convertWideStringToUtf8(std::wstring& sourceString)
	{
#ifdef _WIN32
		int neededLength = WideCharToMultiByte(CP_UTF8, 0, sourceString.c_str(), (int)sourceString.size(), NULL, 0, NULL, NULL);
		char* receiver = new char[neededLength + 1];
		memset(receiver, 0x00, sizeof(char)*(neededLength + 1));
		WideCharToMultiByte(CP_UTF8, 0, sourceString.c_str(), (int)sourceString.size(), receiver, neededLength, NULL, NULL);
		std::string newString(receiver);
		delete[] receiver;
		return newString;
#else
		std::string newString(sourceString.begin(), sourceString.end());
		return newString;
#endif
	}

	std::wstring StringUtility::convertUtf8ToWideString(std::string& sourceString)
	{
#ifdef _WIN32
		int neededLength = 0;
		neededLength = MultiByteToWideChar(CP_UTF8, 0, sourceString.c_str(), -1, NULL, 0);
		wchar_t* receiver = new wchar_t[neededLength+1];
		memset(receiver, 0x00, sizeof(wchar_t)*(neededLength+1));
		MultiByteToWideChar(CP_UTF8, 0, sourceString.c_str(), -1, receiver, neededLength);
		std::wstring newString(receiver);
		delete[] receiver;
		return newString;
#else
		std::wstring newString(sourceString.begin(), sourceString.end());
		return newString;
#endif
	}

#ifdef _WIN32
	std::string StringUtility::convertMultibyteToUtf8(std::string& sourceString)
	{
		int neededLength = MultiByteToWideChar(CP_ACP, 0, sourceString.c_str(), (int)sourceString.size(), NULL, 0);
		wchar_t* receiver = new wchar_t[neededLength + 1];
		memset(receiver, 0x00, sizeof(wchar_t)*(neededLength + 1));
		MultiByteToWideChar(CP_ACP, 0, sourceString.c_str(), (int)sourceString.size(), receiver, neededLength);
		std::wstring wideString(receiver);
		delete[] receiver;
		return convertWideStringToUtf8(wideString);
	}
#endif

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../util/stb_image_write.h"
	void ImageUtility::writeMemoryImageToFile(unsigned char* buffer, int width, int height, const char* fullPath)
	{
		stbi_write_jpg(fullPath, width, height, 4, buffer, 0);
	}
}


