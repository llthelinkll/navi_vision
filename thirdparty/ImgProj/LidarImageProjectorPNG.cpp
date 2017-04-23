#include "LidarImageProjectorPNG.h"

#include <cfloat>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "PngDistanceImage.hpp"
#include "CsvReader.h"

using namespace std;
//using namespace matrixTools;
using namespace boost::filesystem;

typedef boost::numeric::ublas::vector<double>   DVector;
typedef boost::numeric::ublas::matrix<double>   DMatrix;

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
////////////////////         PNGImageProjector         ///////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
/*!
 * \brief constructor
 * \param cfgFile file holding all needed configuration settings.
 *                either: imgHSize; imgVSize; horizStartAngle(grad); horizStopAngle(grad); vertStartAngle(grad); vertStopAngle(grad)
 *                or: imgHSize; imgVSize; horizStartAngle(grad); horizStopAngle(grad); vertAngle_1; vertAngle_2; ...; vertAngle_n
 * \throws runtime_error if the config file is invalid
 */
PNGImageProjector::PNGImageProjector(string cfgFile)
{
	CsvReader config(cfgFile, ";", "#");
	if (config.lineCount() < 1)
		throw runtime_error("PNGImageProjector: configuration file has no valid line");
	unsigned int nbEntries = config.fieldCount(0);
	if (nbEntries < 6)
		throw runtime_error("PNGImageProjector: configuration file has not enough valid entries");
	double toRAD = M_PI/180.0;
	imgHSize = config.get<unsigned int>(0,0);
	imgVSize = config.get<unsigned int>(0,1);
	hSpacing = NULL;
	vSpacing = NULL;
//	cout << "entries: " << nbEntries;
//	for (unsigned int i=0; i<nbEntries; ++i) cout << " " << i << ":" << config.get(0,i) << flush;
	if (nbEntries == 2+2+2) {
		double horizStartAngleRAD_ = config.get<double>(0,2)*toRAD;
		double horizStopAngleRAD_ = config.get<double>(0,3)*toRAD;
		if (horizStopAngleRAD_ >= horizStartAngleRAD_) throw invalid_argument("PNGImageProjector: horizStopAngleRAD_ >= horizStartAngleRAD_");
		hSpacing = new PNGImageProjector::RegularSpacing(horizStartAngleRAD_, horizStopAngleRAD_, imgHSize);
		double vertStartAngleRAD_ = config.get<double>(0,4)*toRAD;
		double vertStopAngleRAD_ = config.get<double>(0,5)*toRAD;
		if (vertStopAngleRAD_ <= vertStartAngleRAD_) throw invalid_argument("PNGImageProjector: vertStopAngleRAD_ <= vertStartAngleRAD_");
		vSpacing = new PNGImageProjector::RegularSpacing(vertStartAngleRAD_, vertStopAngleRAD_, imgVSize);
	}
	if (nbEntries == 2+2+imgVSize) {
		double horizStartAngleRAD_ = config.get<double>(0,2)*toRAD;
		double horizStopAngleRAD_ = config.get<double>(0,3)*toRAD;
		if (horizStopAngleRAD_ >= horizStartAngleRAD_)
			throw invalid_argument("PNGImageProjector: horizStopAngleRAD_ >= horizStartAngleRAD_");
		horizOpeningAngleRAD = horizStartAngleRAD_ - horizStopAngleRAD_;
		hSpacing = new PNGImageProjector::RegularSpacing(horizStartAngleRAD_, horizStopAngleRAD_, imgHSize);
		vector<double> vertAngles; vertAngles.reserve(imgVSize);
		for (unsigned int i=4; i<nbEntries; ++i)
			vertAngles.push_back(config.get<double>(0,i)*toRAD);
		vertOpeningAngleRAD = vertAngles[0] - vertAngles[nbEntries-1];
		vSpacing = new PNGImageProjector::IrregularSpacing(vertAngles.begin(), vertAngles.end(), imgVSize);
	}
	if (hSpacing == NULL || vSpacing == NULL)
		throw runtime_error("PNGImageProjector: configuration file has unexpected number of entries");

	setupSphere();
//	horizStartAngleRAD = config.get<double>(0,2)*toRAD;
	//horizStartAngle(grad); horizStopAngle(grad); vertStartAngle(grad); vertStopAngle(grad)
}

/*!
 * \brief constructor for a regular lattice.

		horizontal angle increases to the left, i.e. NOT according to horizontal indexing => horizStopAngleRAD_ < horizStartAngleRAD
		vertical angle increases downward, i.e. according to vertical indexing => vertStopAngleRAD_ > vertStartAngleRAD

 * \param horizStartAngleRAD_ yaw-angle (mathematical ccw angle in radian) of the left border of the first=leftmost image column.
 * \param horizStopAngleRAD_ yaw-angle (mathematical ccw angle in radian) of the right border of the last=rightmost image column.
 * \param vertStartAngleRAD_ pitch-angle (mathematical ccw angle in radian) of the upper border of the first=uppermost image row.
 * \param vertStopAngleRAD_ pitch-angle (mathematical ccw angle in radian) of the lower border of the last=lowermost image row.
 * \param imgHSize_ horizontal image size in pixel
 * \param imgVSize_ vertical image size in pixel
 * \throws invalid_argument if (horizStopAngleRAD_ >= horizStartAngleRAD_) or (vertStopAngleRAD_ <= vertStartAngleRAD_)
 */
PNGImageProjector::PNGImageProjector(double horizStartAngleRAD_, double horizStopAngleRAD_, double vertStartAngleRAD_, double vertStopAngleRAD_, unsigned int imgHSize_, unsigned int imgVSize_)
	: imgHSize(imgHSize_)
	 ,imgVSize(imgVSize_)
{
	if (horizStopAngleRAD_ >= horizStartAngleRAD_) throw invalid_argument("PNGImageProjector: horizStopAngleRAD_ >= horizStartAngleRAD_");
	if (vertStopAngleRAD_ <= vertStartAngleRAD_) throw invalid_argument("PNGImageProjector: vertStopAngleRAD_ <= vertStartAngleRAD_");

	horizOpeningAngleRAD = horizStartAngleRAD_ - horizStopAngleRAD_;
	vertOpeningAngleRAD = vertStartAngleRAD_ - vertStopAngleRAD_;
	hSpacing = new PNGImageProjector::RegularSpacing(horizStartAngleRAD_, horizStopAngleRAD_, imgHSize);
	vSpacing = new PNGImageProjector::RegularSpacing(vertStartAngleRAD_, vertStopAngleRAD_, imgVSize);
	setupSphere();
}

void YawPitch_2_Rot( const double yaw, const double pitch, DMatrix &R)
{
  double c_a = cos( yaw );
  double s_a = sin( yaw );
  double c_b = cos( pitch );
  double s_b = sin( pitch );

  R.resize( 3, 3 ); //(row,col)

  R(0, 0) = c_a * c_b;
  R(1, 0) = s_a * c_b;
  R(2, 0) = -s_b;

  R(0, 1) = -s_a;
  R(1, 1) = c_a;
  R(2, 1) = 0.0;

  R(0, 2) = c_a * s_b;
  R(1, 2) = s_a * s_b;
  R(2, 2) = c_b;
}

void PNGImageProjector::setupSphere()
{
	cout << "  allocating 3x" << imgHSize*imgVSize*sizeof(double)/1024 << "kB..." << flush;
	xUnitSphere = new double[imgHSize*imgVSize];
	yUnitSphere = new double[imgHSize*imgVSize];
	zUnitSphere = new double[imgHSize*imgVSize];
	rotMat = new double[imgHSize*imgVSize*9];

	cout << "initializing..." << flush;
	DVector unity(3,0.); unity[0] = 1.;
	DMatrix Rot;
	DVector pt;
	double yaw,pitch;
	for (unsigned int row=0; row<imgVSize; ++row) {
		for (unsigned int col=0; col<imgHSize; ++col) {
			yaw = hSpacing->getAngleRAD(col);
			pitch = vSpacing->getAngleRAD(row);
			YawPitch_2_Rot(yaw,pitch,Rot);
			pt = boost::numeric::ublas::prod(Rot, unity);
			unsigned int idx = row*imgHSize + col;
			xUnitSphere[idx] = pt[0];
			yUnitSphere[idx] = pt[1];
			zUnitSphere[idx] = pt[2];
			memcpy( &(rotMat[9*idx]), &(Rot.data()[0]), 9*sizeof(double) );
		}
	}
	cout << "done" << endl;
}

PNGImageProjector::~PNGImageProjector()
{
  delete [] rotMat;
	delete [] zUnitSphere;
	delete [] yUnitSphere;
	delete [] xUnitSphere;
	delete vSpacing;
	delete hSpacing;
}

unsigned int PNGImageProjector::getImgHorizSize() const
{
  return imgHSize;
}

unsigned int PNGImageProjector::getImgVertSize() const
{
  return imgVSize;
}

double PNGImageProjector::getImgHorizAngleRAD() const
{
	return horizOpeningAngleRAD;
}

double PNGImageProjector::getImgVertAngleRAD() const
{
	return vertOpeningAngleRAD;
}

bool PNGImageProjector::getImageIndexYP(double yawRAD, double pitchRAD, int &hi, int &vi) const
{
	return (hSpacing->getImageIndex(yawRAD, hi) && hSpacing->getImageIndex(pitchRAD, vi));
}


void PNGImageProjector::get3DCoordRel(int hi, int vi, double dist, double &x, double &y, double &z) const
{
  unsigned int idx = vi*imgHSize + hi;
  x = dist * xUnitSphere[idx];
  y = dist * yUnitSphere[idx];
  z = dist * zUnitSphere[idx];
}


//////////////////////////////////////////////////////////////////////////////
//////////////////  Inner Class: Regular Spacing  ////////////////////////////
//////////////////////////////////////////////////////////////////////////////

PNGImageProjector::RegularSpacing::RegularSpacing(double startAngleRAD_, double stopAngleRAD_, unsigned int nbPix_)
{
	cout << "  set up regular spacing with " << startAngleRAD_ << ".." << stopAngleRAD_ << ", pix " << nbPix_ << endl;
	// normalize starting angle to [0,2pi)
	while (startAngleRAD_ >= 2*M_PI) { startAngleRAD_ -= 2*M_PI; stopAngleRAD_ -= 2*M_PI;}
	while (startAngleRAD_ < 0.0) { startAngleRAD_ += 2*M_PI; stopAngleRAD_ += 2*M_PI;}
	double rangeRAD_ = stopAngleRAD_-startAngleRAD_;

	startAngleRAD = startAngleRAD_;
	rangeRAD = fabs(rangeRAD_);
	rangeSign = (rangeRAD_ < 0) ? -1.0 : 1.0;
	pixSizeRAD = rangeRAD_/(double)nbPix_;
}

bool PNGImageProjector::RegularSpacing::getImageIndex(double angleRAD, int &idx)
{
	// 1) normalize angle
	while (angleRAD < 0) angleRAD += 2*M_PI;
	while (angleRAD >= 2*M_PI) angleRAD -= 2*M_PI;
	// 2) make angle relative
	angleRAD -= startAngleRAD;
	// 3) check angle
	if (angleRAD*rangeSign < 0) return false;
	if (angleRAD*rangeSign > rangeRAD) return false;
	// 4) calculate index
	idx = angleRAD / pixSizeRAD;
	return true;
}

double PNGImageProjector::RegularSpacing::getAngleRAD(int idx)
{
	return startAngleRAD + pixSizeRAD*((double)idx+0.5); // center of pixel
}

//////////////////////////////////////////////////////////////////////////////
//////////////////  Inner Class: Irregular Spacing  //////////////////////////
//////////////////////////////////////////////////////////////////////////////

bool PNGImageProjector::IrregularSpacing::getImageIndex(double angleRAD, int &idx)
{
	if (angleRAD*pixSizeSign < (vAnglesRAD.front()-avgPixSizeRAD)*pixSizeSign) return false;
	if (angleRAD*pixSizeSign > (vAnglesRAD.back()+avgPixSizeRAD)*pixSizeSign) return false;
	idx = 0;
	double oldDst = DBL_MAX;
	while (idx < (int)vAnglesRAD.size()) {
		double cDst = fabs(angleRAD-vAnglesRAD[idx]);
		if (cDst < oldDst) {
			oldDst = cDst;
			++idx;
		}	else
			break;
	}
	idx--;
	return true;
}

double PNGImageProjector::IrregularSpacing::getAngleRAD(int idx)
{
	return vAnglesRAD[idx];
}

