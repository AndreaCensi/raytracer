#include <float.h>
#include <cmath>
#include "simplemap.h"

namespace PPU {
	using namespace std;
	
	// Distance of q to the line containing this segment
	double Segment::distToLine(const Point2d& q) const {
		return fabs(   (q-p[0]) * normal()  );
	}
	
	// A unitary vector perpendicular to this segment
	Point2d Segment::normal() const {
		return (p[1]-p[0]).perp().vers();
	}

	
	bool Map::laserIncidence(
		double& dist, double& alpha, 
		const Point2od& eye, double maxRange) const  
	{
		// beam's directional vector
		Point2d N(cos(eye.theta), sin(eye.theta));
	
		double mindist = 0;
		Point2d Q; bool found = false;;
		for(size_type i=0;i<size();i++) {
			const Segment &s = (*this)[i];
			
//			if(ObstIgnore == s.obstType) continue;
			if(s.laserType == LaserIgnore) continue;
			
			
			// Normal to segment line
			Point2d S = (s.p[1] - s.p[0]).perp();
			// Check whether they can cross
			if(S*N == 0) continue; // don't cross
			// Black magic
			double dist = (S*(s.p[0]-eye.point2())) / (S*N);
			if(dist<=0) continue;
			// don't cross
			//cerr << "Segment: " << s << " Normal: "<< S << " Dist: " << dist << endl;
			// Now we check whether the crossing point
			// with the line lies within the segment
			Point2d crossingPoint = eye.point2() + N * dist;
			// distance from segment center to crossing point
			double rad = crossingPoint.distTo( (s.p[0]+s.p[1])*0.5 );
			if( rad > S.abs()/2 ) continue; // crossing point out of segment
			if(mindist==0||dist<mindist) {
				mindist = dist;
				Q=s.p[1]-s.p[0];
				found = true;
			}
		}
		
		if(!found) {
			dist=NAN;
			alpha=NAN;
			return false;
		}
		
		if (maxRange>0 && mindist>maxRange){
			dist=NAN;
			alpha=NAN;
			return false;
		} 
		
		dist=mindist;
		//incidence=(N*Q)/sqrt(Q*Q);
		alpha = M_PI/2 + atan2(Q.y,Q.x);
		
		if( N * Point2d::vers(alpha) > 0 ) {
			alpha = alpha + M_PI;
		}
		
		return true;
	
	}
	
	//////////////// Cambi di coordinate
	
	double fig_scalex = DEFAULT_FIG_SCALEX;
	double fig_scaley = DEFAULT_FIG_SCALEY;
	
	FIG::FIGPoint world2fig(const Point2d& w) {
		return FIG::FIGPoint( (int)( w.x/fig_scalex), (int)( w.y/fig_scaley));
	}
	
	Point2d fig2world(const FIG::FIGPoint& f) {
		return Point2d(f.x*fig_scalex, f.y*fig_scaley);
	}
	
	void fig2world(const FIG::FIGPoint& f, Point2d& p) {
		p.x = f.x * fig_scalex;
		p.y = f.y * fig_scaley;
	}
	
	
	int world2fig(double d) {
		return (int)(d/fig_scalex);
	}

	double fig2world(int i) {
		return i*fig_scalex;
	}
	///////////
	
	void Map::loadFig(FIG::Figure&fig) {
		clear();
	
		// Number of added segments
		int nsegs = 0;
		
		for(FIG::Figure::FigureIterator i = fig.beginTree(); i != fig.endTree(); ++i) {
			FIG::Poly * poly;
			if((poly=dynamic_cast<FIG::Poly*> (*i)))
				for(unsigned int a=0;a<poly->size()-1;a++) {
					Segment s;
					s.p[0] = fig2world((*poly)[a]);
					s.p[1] = fig2world((*poly)[a+1]);

/*					if(poly->depth >= L_DECORATIVE_MIN && poly->depth <= L_DECORATIVE_MAX) {
						s.obstType = ObstIgnore;
						s.laserType = LaserIgnore;
						continue;
					} */
					
					if(poly->depth >= L_BOTH_MIN && poly->depth <= L_BOTH_MAX) {
						s.obstType = ObstSolid;
						s.laserType = LaserSolid;
					}
					else
					if(poly->depth >= L_ONLYROBOT_MIN && poly->depth <= L_ONLYROBOT_MAX) {
						s.obstType = ObstSolid;
						s.laserType = LaserIgnore;
					}
					else
					if(poly->depth == L_DUMMY) {
						s.obstType = ObstIgnore;
						s.laserType = LaserIgnore;
					}
					else continue;
					
					cerr << "Segment " << s.p[0].toString() << " -> "
						<< s.p[1].toString() << endl; 
					//cerr << (*poly)[a].x << (*poly)[a].y << endl;
					push_back(s);
					nsegs++;
				}
		}
			
		std::cerr << "Loaded map with " << nsegs << " segments." << std::endl;
	}
	
	
	LaserData Map::rayTracing(
		Point2od pose, double fov_rad, int nrays, double maxReading) 
	{
		LaserData ld;
		ld.odometry = pose;
		ld.estimate = pose;
		ld.laserPose = Point2od(0,0,0);
		ld.minReading = 0;
		ld.maxReading = maxReading;
		ld.minTheta = -fov_rad/2;
		ld.maxTheta = +fov_rad/2;
		
		//cout << "Ray tracing: maxreading = " << maxReading << endl;
		
		for(int i=0;i<nrays;i++) {
			LaserData::LaserPoint lp;
			lp.theta = ld.minTheta + ((ld.maxTheta-ld.minTheta)*i)/nrays;
			
			bool valid = laserIncidence(
				lp.reading, lp.trueAlpha,
				Point2od(pose.x,pose.y,pose.theta+lp.theta), maxReading);
			
			if(!valid) 
				lp.markInvalid();
			
			//cout << "ray " << i << " valid = " << lp.isValid() <<
			//	" alpha = " << lp.trueAlpha << " reading = " << lp.reading << endl;
			ld.points.push_back(lp);
		}
		return ld;
	}
	
	void Map::ray_tracing(
		Point2od pose, double fov_rad, int nrays, double maxReading,
		double*reading, double*theta, double *alpha, int*valid) 
	{	
		double minTheta = -fov_rad/2;
		double maxTheta = +fov_rad/2;
		
		for(int i=0;i<nrays;i++) {
			theta[i] = minTheta + ((maxTheta-minTheta)*i)/nrays;
			
			bool is_valid = laserIncidence(
				reading[i], alpha[i],
				Point2od(pose.x, pose.y, pose.theta + theta[i]), 
				maxReading);
			
			if(!is_valid) {
				valid[i] = 0;
				reading[i] = alpha[i] = NAN;
			} else {
				valid[i] = 1;
			}
			
		}
	}
	
		
	Viewport Map::getBoundingBox() {  
		Point2d rmin(DBL_MAX,DBL_MAX), rmax(-DBL_MAX,-DBL_MAX);
		
		for (unsigned int i = 0; i < size(); i++) {
			rmin = Point2d::min(rmin, at(i).p[0]);
			rmax = Point2d::max(rmax, at(i).p[0]);
			rmin = Point2d::min(rmin, at(i).p[1]);
			rmax = Point2d::max(rmax, at(i).p[1]);
		}
		
		return Viewport(rmin,rmax);
	}
	
	OUT_OP(Point2d  ) { return stream << ob.x << " " << ob.y;                        }
	OUT_OP(Segment) { return stream << ob.p[0] << endl << ob.p[1] << endl << endl; }
	OUT_OP(Map    ) {
		for(Map::const_iterator i=ob.begin();i!=ob.end();i++) stream << *i;
		return stream;
	}
	
	IN_OP(Point2d  ) { return stream >> ob.x >> ob.y; }
	IN_OP(Segment) { return stream >> ob.p[0] >> ob.p[1]; }
	IN_OP(Map    ) {
		while(stream) {
			Segment s; stream >> s; ob.push_back(s);
		}
		return stream;
	}


} // namespace SimpleMap
