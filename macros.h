#ifndef H_RAYTRACER_MACROS
#define H_RAYTRACER_MACROS

// XXX make it right
#define sm_error printf  

#define jo_expect_array(a) (a!=0 && json_object_is_type(a, json_type_array))
#define jo_expect_object(a) (a!=0 &&  json_object_is_type(a, json_type_object))
#define jo_expect_string(a) (a!=0 && json_object_is_type(a, json_type_string))
#define jo_expect_double(a) (a!=0 && json_object_is_type(a, json_type_double))
#define jo_expect_array_size(a,n) ( (a!=0) && (json_object_is_type(a, json_type_array)&& (jo_array_length(a)==n)))
#define jo_expect_array_size_min(a,n) ( (a!=0) && (json_object_is_type(a, json_type_array)&& (jo_array_length(a)>=n)))


#define expect(a) if(!a) { \
		sm_error("Invalid format: \n\t %s \n", #a); \
			return -3; \
		}

#define expect_s(a, s) if(!a) { \
		sm_error("Invalid format: %s \n\t %s \n", s, #a); \
			return -2; \
		}
		


#endif