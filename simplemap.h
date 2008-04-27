#ifndef MAPAC_H
#define MAPAC_H

#include <iostream>
#include <vector>

#include <fig/fig.h>
#include <rdk2/geometry/point.h>
#include <rdk2/geometry/viewport.h>

#include <rdk2/sensordata/laserdata.h>

namespace PPU {
	using namespace RDK2::Geometry;
	using RDK2::SensorData::LaserData;
	
	
	// Layer organization:
	
	// 99       bounding box
	// 100-199  Both sensor and robot obstacles
	// 200-299  Only robot

	#define L_DUMMY 99
	#define L_BOTH_MIN 100
	#define L_BOTH_MAX 199
	#define L_ONLYROBOT_MIN 200
	#define L_ONLYROBOT_MAX 299
	
	
	// How the segment behaves with lasers
	enum LaserType {
		LaserIgnore = 0,
		LaserSolid = 1,     // Good readings
	//	LaserFail  = 2  // zone of noisy readings; ignore
	};
	
	// How the segment behaves as an obstacle
	enum ObstType {
		ObstIgnore = 0,
		ObstSolid  = 1,     // is an obstacle
	};
	
	struct Segment {
		Point2d p[2];
		
		LaserType laserType;
		ObstType  obstType;
		
		// Distance of q to the line containing this segment
		double distToLine(const Point2d& q) const;
		// A unitary vector perpendicular to this segment
		Point2d normal() const;
	};
	
	struct Map : public std::vector<Segment> {
		
		/** Ray tracing with incidence. */
		bool laserIncidence(
			double& dist, double& alpha,
			const Point2od& eye, double maxRange=0) const ;
			
		LaserData rayTracing(
			Point2od pose, double fov_rad, int nrays, double horizon = INFINITY);
		
		void ray_tracing(
			Point2od pose, double fov_rad, int nrays, double maxReading,
			double*reading, double*theta, double*alpha, int*valid) ;
		
		void loadFig(FIG::Figure&fig);
		
		bool isCellObst(const Viewport& cellBoundingBox) const;
		
		Viewport getBoundingBox(); 
	};
	

	/// With default settings, "1 inch" on the XFig screen is represented
	/// as 1000 internal units.
	/// The convention is to set "1 inch" = 1 real "meter", so
	/// that scale=0.001
	/// 
	/// Also note that FIG flips the y coordinate.

	#define DEFAULT_FIG_SCALEX (10.0/4500.0)
	#define DEFAULT_FIG_SCALEY (-10.0/4500.0)
	extern double fig_scalex; // =  0.001
	extern double fig_scaley; // = -0.001
	
	
	// Conversione coordinate
	FIG::FIGPoint world2fig(const Point2d& w);
	Point2d fig2world(const FIG::FIGPoint& f);
	void fig2world(const FIG::FIGPoint& f, Point2d& p);
	
	// Conversione lunghezze
	int world2fig(double);
	double fig2world(int);
	
/// Vecchia roba ancora non utilizzata 
	#define OUT_OP(c) std::ostream& operator <<	(std::ostream&stream,const c&ob)
	#define  IN_OP(c) std::istream& operator >> (std::istream&stream,      c&ob)
	
	OUT_OP(Point2d  ); IN_OP(Point2d  );
	OUT_OP(Segment); IN_OP(Segment);
	OUT_OP(Map    ); IN_OP(Map    );
	
}

#endif
