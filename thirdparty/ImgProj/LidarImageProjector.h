/*
 * LidarImageProjector.h
 *
 *  Created on: Jun 10, 2010
 *      Author: moosmann
 */

#ifndef LIDARIMAGEPROJECTOR_H_
#define LIDARIMAGEPROJECTOR_H_

/*!
 * \class LidarImageProjector
 * \brief This class serves as abstract base class for transforming a 3D point cloud into a 2D distance image and back
 *
 * Mathematical counter-clockwise angles are used everywhere
 */
class LidarImageProjector {
public:
	virtual ~LidarImageProjector() {};

	virtual unsigned int getImgHorizSize() const = 0;
	virtual unsigned int getImgVertSize() const = 0;
	virtual double getImgHorizAngleRAD() const = 0;
	virtual double getImgVertAngleRAD() const = 0;

	        bool getImageIndexRel(double x, double y, double z, int &hi, int &vi, double &dist) const; // can be used to project relative 3D points onto the image. returns false if resulting vi points outside of image!
	virtual bool getImageIndexYP(double yawRAD, double pitchRAD, int &hi, int &vi) const = 0; // can be used to get the image indices from the angles. returns false if resulting vi points outside of image!
	virtual void get3DCoordRel(int hi, int vi, double dist, double &x, double &y, double &z) const = 0; // can be used to project image into 3D, relative to Ego
};

#endif /* LIDARIMAGEPROJECTOR_H_ */
