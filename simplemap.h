#ifndef MAPAC_H
#define MAPAC_H

#include <iostream>
#include <vector>
#include <string>

namespace RayTracer {
	
	using namespace std;
	
	struct Stuff;
	struct Environment  {
		std::vector<Stuff*> stuff;
		

		bool ray_tracing(const double p[2], const double direction,  double& out_distance, double &out_alpha, int& surface, int&region, double&coord) const ;
		
		/** Checks whether the specified circle intersects something */
		bool check_circle_intersection(const double center[2], const double radius, int&surface_id) const;
		
		Stuff* find_surface(int surface_id);
		bool remove(int surface_id);
	};
	
	struct Stuff {
		int surface_id;
		 
		/** Ray tracing with incidence. */
		virtual bool ray_tracing(const double p[2], const double direction, double& out_distance, double &out_alpha, int&region, double&coord) const = 0; 	
	
		virtual bool check_circle_intersection(
			const double center[2],
			const double radius, 
			double &distance,
			double &angle) const  = 0;
			
		virtual ~Stuff() { }
	}; 
	
	struct Segment: public Stuff {
		int region_id;
		double p0[2], p1[2];
		double t0,t1;
		
		Segment() {}
		Segment(double x0,double y0,double x1,double y1) {
			p0[0] = x0; p0[1] = y0;
			p1[0] = x1; p1[1] = y1;
			
			t0=0; t1=1; region_id=-1;
		}
		
		bool ray_tracing(const double p[2], const double direction, double& out_distance, double &out_alpha, int&region, double&coord) const;
		bool check_circle_intersection(
				const double center[2],
				const double radius, 
				double &distance_to_impact,
				double &angle) const;
		
		double getSegmentLength();
		virtual ~Segment() {};
	};

	struct Circle: public Stuff {
		double center[2], radius;
		int solid_inside;
		
		Circle() {}
		
		bool ray_tracing(const double p[2], const double direction,  double& out_distance, double &out_alpha, int&region, double&coord) const;
		bool check_circle_intersection(
				const double center[2],
				const double radius, 
				double &distance_to_impact,
				double &angle) const;
		
		
		virtual ~Circle() {};
	};
	
}


#endif
