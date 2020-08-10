/*
   raytracer.hh - Header file for raytracer.cc

   Contains types and other math information for raytracer.cc
*/
#ifndef RAYTRACER_HH
#define RAYTRACER_HH

#include <stdint.h>
#include <float.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <memory>

using u8   = uint8_t;
using u16  = uint16_t;
using u32  = uint32_t;
using u64  = uint64_t;

using i8   = int8_t;
using i16  = int16_t;
using i32  = int32_t;
using i64  = int32_t;

using b32x = int_least32_t;

using r32 = float;
using r64 = double;

#define Assert assert

#define internal      static
#define global        static
#define local_persist static


//////////////////////////////////////////////////////////////////////////
//
// Struct definitions
//
//////////////////////////////////////////////////////////////////////////

struct bitmap32
{
	u32 width;
	u32 height;
	u32 *data;
};

union vec3
{
	struct
	{
		r64 x;
		r64 y;
		r64 z;
	};
	struct
	{
		r64 r;
		r64 g;
		r64 b;
	};
	r64 e[3];

	inline const r64&
	operator[](int i) const {
		return this->e[i];
	}
};

using color3 = vec3;
using point3 = vec3;

struct ray3
{
	point3 orig;
	vec3 dir;
	r64 time;
};

struct camera
{
	point3 origin;
	point3 llc;
	vec3   horizontal;
	vec3   vertical;
	vec3   u, v, w;
	r64    lens_radius;
	r64	   t0, t1;
};

struct material;

struct intersect
{
	point3	 point;
	r64    	 t;
	vec3   	 normal;
	material *mat;
	r64		 u,v;
};

struct material
{
	virtual b32x Scatter(const ray3& incoming_ray, const intersect& inter, color3& attenuation,
			ray3& scattered_ray) const = 0;
};

struct aabb
{
	point3 min;
	point3 max;
} ;

struct surface
{
	virtual b32x Hit(const ray3& r, r64 min, r64 max, intersect& inter) const = 0;
	virtual b32x BoundingBox(r64 t0, r64 t1, aabb &out) const = 0;
};

struct sphere : surface
{
	point3	 center;
	r64    	 radius;
	material *mat;

	virtual b32x Hit(const ray3& r, r64 min, r64 max, intersect& inter) const override;
	virtual b32x BoundingBox(r64 t0, r64 t1, aabb &out) const override;
};

struct moving_sphere : surface
{
	point3	 center0, center1;
	r64    	 radius;
	material *mat;
	r64		 t0, t1;

	virtual b32x Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const override;
	virtual b32x BoundingBox(r64 t0, r64 t1, aabb &out) const override;
};

struct bvh : surface
{
	std::shared_ptr<bvh> left;
	std::shared_ptr<bvh> right;
	surface *value;
	aabb box;

	virtual b32x Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const override;
	virtual b32x BoundingBox(r64 t0, r64 t1, aabb &out) const override;
};

struct world : surface
{
	std::vector<surface*> surfaces;
	std::vector<material*> materials;
	camera cam;
	std::shared_ptr<bvh> tree;

	virtual b32x Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const override;
	virtual b32x BoundingBox(r64 t0, r64 t1, aabb &out) const override;
};

struct task
{
	world    *scene;
	bitmap32 *bmp;
	i32      xmin;
	i32      xmax;
	i32      ymin;
	i32      ymax;
};

struct task_queue
{
	std::vector<task> tasks;

	volatile u64 next_index;
	volatile u64 ray_count;
	volatile u64 retired_tiles;
};

namespace constants
{
	const r64 TOLERANCE = 0.001;
	const r64 INFIN     = DBL_MAX;
	const r64 PI        = M_PI;
	const u64 U64Max	= UINT64_MAX;
};

struct texture
{
	virtual color3 Value(r64 u, r64 v, const point3 &p) const = 0;
};

struct solid_color : texture
{
	color3 color_value;

	virtual color3 Value(r64 u, r64 v, const point3 &p) const override
	{
		return color_value;
	}
};

struct checker_texture : texture
{
	std::shared_ptr<texture> odd;
	std::shared_ptr<texture> even;

	virtual color3 Value(r64 u, r64 v, const point3 &p) const override
	{
		auto sines = sin(10*p.x)*sin(10*p.y)*sin(10*p.z);

		if (sines < 0)
			return odd->Value(u, v, p);
		else
			return even->Value(u, v, p);
	}
};

//////////////////////////////////////////////////////////////////////////
//
// Vectors
//
//////////////////////////////////////////////////////////////////////////

inline vec3
Hadamard(const vec3& a, const vec3& b)
{
	return {a.e[0] * b.e[0], a.e[1] * b.e[1], a.e[2] * b.e[2]};
}

inline r64
Inner(const vec3& a, const vec3& b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline vec3
Cross(const vec3& u, const vec3& v)
{
	return vec3{u.e[1] * v.e[2] - u.e[2] * v.e[1],
                u.e[2] * v.e[0] - u.e[0] * v.e[2],
                u.e[0] * v.e[1] - u.e[1] * v.e[0]};
}

inline vec3
operator-(const vec3& a)
{
	return {-a.e[0], -a.e[1], -a.e[2]};
}

inline vec3
operator+(const vec3& a, const vec3& b)
{
	return {a.e[0] + b.e[0], a.e[1] + b.e[1], a.e[2] + b.e[2]};
}

inline vec3
operator+(const vec3& a, r64 b)
{
	return {a.e[0] + b, a.e[1] + b, a.e[2] + b};
}

inline vec3
operator+(r64 a, const vec3& b)
{
	return b + a;
}

inline vec3
operator-(const vec3& a, const vec3& b)
{
	return a + -b;
}

inline vec3
operator*(const vec3& a, const vec3& b)
{
	return Hadamard(a, b);
}

inline vec3
operator*(r64 t, const vec3& v)
{
	return {t*v.e[0], t*v.e[1], t*v.e[2]};
}

inline vec3
operator*(const vec3 &v, r64 t)
{
	return t*v;
}

inline vec3
operator/(const vec3& v, r64 t)
{
	return (1/t) * v;
}

inline vec3
operator+=(vec3& a, const vec3& b)
{
	a = a + b;
	return a;
}

inline vec3
operator*=(vec3& a, r64 b)
{
	a = a * b;
	return a;
}

inline vec3
operator/=(vec3& a, r64 b)
{
	a = a * 1/b;
	return a;
}

inline r64
Length2(const vec3 &a)
{
	return a.e[0] * a.e[0] + a.e[1] * a.e[1] + a.e[2] * a.e[2];
}

inline r64
Length(const vec3 &a)
{
	return sqrt(Length2(a));
}

inline vec3
Norm(const vec3 &a)
{
	return (1/Length(a)) * a;
}

inline vec3
FromPointAToB(const point3 &a, const point3 &b)
{
	return b - a;
}

inline vec3
ReflectAboutNormal(const vec3 &v, const vec3 &n)
{
	return v - 2*Inner(v, n) * n;
}

inline vec3
RefractAboutNormal(const vec3& incoming, const vec3& normal, r64 refraction_ratio)
{
	auto cos_theta = Inner(-incoming, normal);

	vec3 out_perp = refraction_ratio * (incoming + cos_theta*normal);
	vec3 out_para = -sqrt(fabs(1.0 - Length2(out_perp))) * normal;

	return out_perp + out_para;
}

inline vec3 SampleRandomVectorFromLambertian();
inline r64 RandomUnilateral(b32x seed = false);
inline r64 RandomUnilateral(r64, r64);
inline i32 RandomInteger(i32, i32);
inline r64 SchlickApproximation(r64 cosine, r64 index_of_refraction);

//////////////////////////////////////////////////////////////////////////
//
// Color
//
//////////////////////////////////////////////////////////////////////////

inline color3
Lerp(const color3 &a, const color3 &b, r64 t)
{
	return (1.0-t)*a + t*b;
}

//////////////////////////////////////////////////////////////////////////
//
// Rays
//
//////////////////////////////////////////////////////////////////////////

inline point3
RayAt(const ray3 &a, r64 t)
{
	return a.orig + a.dir*t;
}

//////////////////////////////////////////////////////////////////////////
//
// Camera
//
//////////////////////////////////////////////////////////////////////////

inline r64 DegreesToRadians(r64 degrees);
inline vec3 SampleRandomVectorFromUnitDisk();

inline ray3
GetRay(const camera& cam, r64 s, r64 t)
{
	vec3 rd     = cam.lens_radius * SampleRandomVectorFromUnitDisk();
	vec3 offset = cam.u * rd.x + cam.v * rd.y;

	vec3 ray_origin   = cam.origin + offset;
	point3 pixel_proj = cam.llc + s*cam.horizontal + t*cam.vertical;

	vec3 dir = Norm(FromPointAToB(ray_origin, pixel_proj));

	return ray3{ray_origin, dir, RandomUnilateral(cam.t0, cam.t1)};
}

inline camera
CreateCamera(point3 lookfrom, point3 lookat, const vec3& up,
		r64 vfov, r64 aspect_ratio, r64 aperture, r64 focus_distance,
		r64 t0 = 0, r64 t1 = 0)
{
	camera cam = {};
	auto theta = DegreesToRadians(vfov);
	auto height = tan(theta/2);

	auto viewport_height = 2.0 * height;
	auto viewport_width = aspect_ratio * viewport_height;

	cam.w = Norm(-FromPointAToB(lookfrom, lookat));
	cam.u = Norm(Cross(up, cam.w));
	cam.v = Cross(cam.w, cam.u);

	cam.origin     = lookfrom;
	cam.horizontal = focus_distance * viewport_width * cam.u;
	cam.vertical   = focus_distance * viewport_height * cam.v;
	cam.llc        = cam.origin - cam.horizontal/2 - cam.vertical/2 - focus_distance * cam.w;

	cam.lens_radius = aperture / 2;

	cam.t0 = t0;
	cam.t1 = t1;

	return cam;
}

//////////////////////////////////////////////////////////////////////////
//
// Materials
//
//////////////////////////////////////////////////////////////////////////

inline void
GetSphereUVCoords(const point3 &p, double &u, double &v)
{
	auto phi = atan2(p.z, p.x);
	auto theta = asin(p.y);
	u = 1-(phi+constants::PI) / 2*constants::PI;
	v = (theta + constants::PI/2) / constants::PI;
}

struct lambertian : material
{
	std::shared_ptr<texture> albedo;

	inline virtual b32x
	Scatter(const ray3& incoming_ray, const intersect& inter, color3& attenuation,
			ray3& scattered_ray) const override
	{
		// Choose the correct normal depending on the angle between the ray and the outward normal
		vec3 correct_normal = Inner(incoming_ray.dir, inter.normal) > 0.0 ? -inter.normal : inter.normal;
		vec3 scatter_dir = correct_normal + SampleRandomVectorFromLambertian();

		scattered_ray = ray3{inter.point, Norm(scatter_dir), incoming_ray.time};
		attenuation = albedo->Value(inter.u, inter.v, inter.point);
		return true;
	}
};

struct metal : material
{
	color3 albedo;
	r64 fuzz;

	inline virtual b32x
	Scatter(const ray3& incoming_ray, const intersect& inter, color3& attenuation,
			ray3& scattered_ray) const override
	{
		// Choose the correct normal depending on the angle between the ray and the outward normal
		vec3 correct_normal = Inner(incoming_ray.dir, inter.normal) > 0.0 ? -inter.normal : inter.normal;

		vec3 reflected = ReflectAboutNormal(incoming_ray.dir, correct_normal);

		scattered_ray = ray3{
			inter.point,
			Norm(reflected + SampleRandomVectorFromLambertian()*fuzz),
			incoming_ray.time
		};

		attenuation = albedo;
		return Inner(scattered_ray.dir, correct_normal) > 0.0;
	}
};

struct dielectric : material
{
	r64 index_of_refraction;

	inline virtual b32x
	Scatter(const ray3& incoming_ray, const intersect& inter, color3& attenuation,
			ray3& scattered_ray) const override
	{
		attenuation = color3{1.0, 1.0, 1.0};

		b32x front_face = Inner(incoming_ray.dir, inter.normal) < 0.0;

		// Choose the correct normal depending on the angle between the ray and the outward normal
		vec3 correct_normal = front_face ? inter.normal : -inter.normal;

		r64 refractive_ratio = front_face ? (1.0 / index_of_refraction) : index_of_refraction;

		r64 cos_theta = fmin(Inner(-incoming_ray.dir, correct_normal), 1.0);
		r64 sin_theta = sqrt(1.0 - cos_theta*cos_theta);

		// Total internal reflection
		if (refractive_ratio * sin_theta > 1.0)
		{
			vec3 reflected = ReflectAboutNormal(incoming_ray.dir, correct_normal);
			scattered_ray = ray3{inter.point, reflected, incoming_ray.time};
			return true;
		}

		r64 reflect_probability = SchlickApproximation(cos_theta, refractive_ratio);

		if (RandomUnilateral() < reflect_probability)
		{
			vec3 reflected = ReflectAboutNormal(incoming_ray.dir, correct_normal);
			scattered_ray = ray3{inter.point, reflected, incoming_ray.time};
			return true;
		}

		vec3 refracted = RefractAboutNormal(incoming_ray.dir, correct_normal, refractive_ratio);
		scattered_ray = ray3{inter.point, refracted, incoming_ray.time};
		return true;
	}
};

//////////////////////////////////////////////////////////////////////////
//
// Surfaces
//
//////////////////////////////////////////////////////////////////////////

inline aabb SurroundingBox(const aabb& a, const aabb& b);
inline b32x AABBIntersect(const aabb& box, const ray3 &r, r64 tmin, r64 tmax);

inline b32x
sphere::Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const
{
	vec3 oc = FromPointAToB(this->center, r.orig);
	const auto a = Inner(r.dir, r.dir);
	const auto b = 2 * (Inner(r.orig, r.dir) - Inner(r.dir, this->center));
	const auto c = Inner(oc, oc) - this->radius*this->radius;

	const auto discriminant = b*b - 4*a*c;

	if (discriminant > 0.0) {
		const auto near = (-b - sqrt(discriminant)) / 2*a;

		if (near > min && near < max) {
			inter.t      = near;
			inter.point  = RayAt(r, inter.t);
			inter.normal = FromPointAToB(this->center, inter.point) / this->radius;
			GetSphereUVCoords(inter.normal, inter.u, inter.v);
			inter.mat    = this->mat;
			return true;
		}

		const auto far = (-b + sqrt(discriminant)) / 2*a;

		if (far > min && far < max) {
			inter.t      = far;
			inter.point  = RayAt(r, inter.t);
			inter.normal = FromPointAToB(this->center, inter.point) / this->radius;
			GetSphereUVCoords(inter.normal, inter.u, inter.v);
			inter.mat    = this->mat;
			return true;
		}
	}

	return false;
}

inline b32x
sphere::BoundingBox(r64 t0, r64 t1, aabb &out) const
{
	aabb r = {};

	r.min = this->center - vec3{radius, radius, radius};
	r.max = this->center + vec3{radius, radius, radius};

	out = r;

	return true;
}

inline internal point3
MovingSphereCenterAtTime(const moving_sphere* ms, r64 t)
{
	r64 scaled_time = (t - ms->t0) / (ms->t1 - ms->t0);
	return Lerp(ms->center0, ms->center1, scaled_time);
}

inline b32x
moving_sphere::Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const
{
	point3 center = MovingSphereCenterAtTime(this, r.time);

	vec3 oc = FromPointAToB(center, r.orig);

	const auto a = Inner(r.dir, r.dir);
	const auto b = 2 * (Inner(r.orig, r.dir) - Inner(r.dir, center));
	const auto c = Inner(oc, oc) - this->radius*this->radius;

	const auto discriminant = b*b - 4*a*c;

	if (discriminant > 0.0) {
		const auto near = (-b - sqrt(discriminant)) / 2*a;

		if (near > min && near < max) {
			inter.t      = near;
			inter.point  = RayAt(r, inter.t);
			inter.normal = FromPointAToB(center, inter.point) / this->radius;
			inter.mat    = this->mat;
			return true;
		}

		const auto far = (-b + sqrt(discriminant)) / 2*a;

		if (far > min && far < max) {
			inter.t      = far;
			inter.point  = RayAt(r, inter.t);
			inter.normal = FromPointAToB(center, inter.point) / this->radius;
			inter.mat    = this->mat;
			return true;
		}
	}

	return false;
}

inline b32x
moving_sphere::BoundingBox(r64 t0, r64 t1, aabb &out) const
{
	point3 center0 = MovingSphereCenterAtTime(this, t0);
	point3 center1 = MovingSphereCenterAtTime(this, t1);

	aabb box0 = {center0 - vec3{this->radius, this->radius, this->radius},
		center0 + vec3{this->radius, this->radius, this->radius}};

	aabb box1 = {center1 - vec3{this->radius, this->radius, this->radius},
		center1 + vec3{this->radius, this->radius, this->radius}};

	out = SurroundingBox(box0, box1);

	return true;
}

inline b32x
bvh::Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const
{
	if (this->value)
		return this->value->Hit(r, min, max, inter);

	if (!AABBIntersect(this->box, r, min, max))
		return false;

	b32x hit_left = this->left->Hit(r, min, max, inter);
	b32x hit_right = this->right->Hit(r, min, hit_left ? inter.t : max, inter);

	return hit_left || hit_right;
}

inline b32x
bvh::BoundingBox(r64 t0, r64 t1, aabb &out) const
{
	out = this->box;
	return true;
}

inline b32x
world::Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const
{
	b32x hit_anything = false;
	auto closest = max;

	for (const auto& surface : this->surfaces) {
		intersect last_in;
		if (surface->Hit(r, min, closest, last_in)) {
			hit_anything = true;
			closest = last_in.t;
			inter = last_in;
		}
	}

	return hit_anything;
}

inline b32x
world::BoundingBox(r64 t0, r64 t1, aabb &out) const
{
	if (surfaces.empty()) return false;

	aabb temp;
	b32x first = true;

	for (const auto& surface : surfaces)
	{
		if (!surface->BoundingBox(t0, t1, temp)) return false;

		out = first ? temp : SurroundingBox(temp, out);
		first = false;
	}

	return true;
}

template<int T>
inline internal b32x
BoxAxisCompare(const surface *a, const surface *b)
{
	aabb a_box;
	aabb b_box;

	if (!a->BoundingBox(0, 0, a_box) || !b->BoundingBox(0, 0, b_box)) {
		fprintf(stderr, "[ERROR] No bounding box found.\n");
		return false;
	}

	return a_box.min[T] < b_box.min[T];
}

inline std::shared_ptr<bvh>
ConstructBVH(std::vector<surface *> surfaces, i32 start, i32 end, r64 t0, r64 t1)
{
	auto r = std::make_shared<bvh>();
	r->left = r->right = nullptr;
	r->value = nullptr;


	i32 axis = RandomInteger(0, 2);

	auto comparator = (axis == 0) ? BoxAxisCompare<0>
					: (axis == 1) ? BoxAxisCompare<1> :
					BoxAxisCompare<2>;

	i32 len = end - start;

	if (len == 1) {
		r->value = surfaces[start];
		aabb temp;
		r->value->BoundingBox(t0, t1, temp);
		r->box = temp;
		return r;
	} else if (len == 2) {
		if (comparator(surfaces[start], surfaces[start+1]) > 0) {
			r->left = ConstructBVH(surfaces, start, start + 1, t0, t1);
			r->right = ConstructBVH(surfaces, start + 1, start + 2, t0, t1);
		} else {
			r->right = ConstructBVH(surfaces, start, start + 1, t0, t1);
			r->left = ConstructBVH(surfaces, start + 1, start + 2, t0, t1);
		}
	} else {
		// TODO(kosi): Replace with non C++ standard library bloat.
		std::sort(surfaces.begin() + start, surfaces.begin() + end, comparator);

		auto mid = start + len/2;

		r->left = ConstructBVH(surfaces, start, mid, t0, t1);
		r->right = ConstructBVH(surfaces, mid, end, t0, t1);
	}

	aabb box_left, box_right;

	if (!r->left->BoundingBox(t0, t1, box_left) || !r->right->BoundingBox(t0, t1, box_right)) {
		fprintf(stderr, "[ERROR] No bounding box found.\n");
	}

	r->box = SurroundingBox(box_left, box_right);

	return r;
}

inline void
CalculateBVH(world &scene, r64 t0, r64 t1)
{
	scene.tree = ConstructBVH(scene.surfaces, 0, scene.surfaces.size(), t0, t1);
}

inline sphere*
AddSphere(world& scene, const point3& center, r64 radius, material* mat)
{
	sphere *s = new sphere;

	s->center = center;
	s->radius = radius;
	s->mat    = mat;

	scene.surfaces.push_back(static_cast<surface*>(s));

	return s;
}

inline moving_sphere*
AddMovingSphere(world& scene, const point3& center0,
		const point3& center1, r64 t0, r64 t1, r64 radius, material* mat)
{
	moving_sphere *s = new moving_sphere;

	s->center0 = center0;
	s->center1 = center1;
	s->t0      = t0;
	s->t1      = t1;
	s->radius  = radius;
	s->mat     = mat;

	scene.surfaces.push_back(static_cast<surface*>(s));

	return s;
}

inline lambertian*
AddLambertian(world &scene, const color3& albedo)
{
	lambertian *l = new lambertian;

	auto sc = std::make_shared<solid_color>();
	sc->color_value = albedo;

	l->albedo = sc;

	scene.materials.push_back(static_cast<material*>(l));

	return l;
}

inline lambertian*
AddLambertian(world &scene, std::shared_ptr<texture> albedo)
{
	lambertian *l = new lambertian;

	l->albedo = albedo;

	scene.materials.push_back(static_cast<material*>(l));

	return l;
}

inline metal*
AddMetal(world &scene, const color3& albedo, r64 fuzz)
{
	metal *l = new metal;

	l->albedo = albedo;
	l->fuzz   = fuzz;

	scene.materials.push_back(static_cast<material*>(l));

	return l;
}

inline material*
AddDielectric(world &scene, r64 index_of_refraction)
{
	dielectric *l = new dielectric;

	l->index_of_refraction = index_of_refraction;

	scene.materials.push_back(static_cast<material*>(l));

	return l;
}

inline void
DestroyWorld(world& scene)
{
	for (const auto &surface : scene.surfaces) {
		free(surface);
	}

	for (const auto &material : scene.materials) {
		free(material);
	}

	scene.surfaces.clear();
	scene.materials.clear();
}

//////////////////////////////////////////////////////////////////////////
//
// Utility
//
//////////////////////////////////////////////////////////////////////////

inline r64
DegreesToRadians(r64 degrees)
{
	return degrees * constants::PI / 180.0;
}

// TODO(kosi): Make this thread-safe somehow...
inline r64
RandomUnilateral(b32x seed) {
	local_persist u16 randstate[3] = {0xDEAD, 0xBEEF, 0xAABB};

	if (seed)
	{
		// setup unbuffered urandom
		FILE* urandom = fopen ("/dev/urandom", "r");
		setvbuf (urandom, NULL, _IONBF, 0);  // turn off buffering

		// fgetc() returns a `char`, we need to fill a `short`
		randstate[0] = (fgetc (urandom) << 8) | fgetc (urandom);
		randstate[1] = (fgetc (urandom) << 8) | fgetc (urandom);
		randstate[2] = (fgetc (urandom) << 8) | fgetc (urandom);

		// cleanup urandom
		fclose (urandom);
	}

	return erand48(randstate);
}

inline i32
RandomInteger(i32 min, i32 max) {
	return static_cast<i32>(RandomUnilateral(min, max+1));
}

inline r64
RandomUnilateral(r64 min, r64 max)
{
	return min + (max-min)*RandomUnilateral();
}

inline vec3
SampleRandomVector() {
	return vec3{RandomUnilateral(), RandomUnilateral(), RandomUnilateral()};
}

inline vec3
SampleRandomVector(r64 a, r64 b) {
	return vec3{RandomUnilateral(a, b), RandomUnilateral(a, b), RandomUnilateral(a, b)};
}

inline vec3
SampleRandomVectorFromUnitSphere() {
	while (true) {
		vec3 p = SampleRandomVector();
		if (Length2(p) >= 1) continue;
		return p;
	}
}

inline vec3
SampleRandomVectorFromUnitDisk() {
	while (true) {
		auto v = vec3{RandomUnilateral(-1, 1), RandomUnilateral(-1, 1), 0};
		if (Length2(v) >= 1) continue;
		return v;
	}
}

inline vec3
SampleRandomVectorFromLambertian() {
	auto a = RandomUnilateral(0, 2 * constants::PI);
	auto z = RandomUnilateral(-1, 1);
	auto r = sqrt(1 - z*z);

	return {r*cos(a), r*sin(a), z};
}

// NOTE(kosi): What the hell does this do???
inline r64
SchlickApproximation(r64 cosine, r64 refractive_ratio)
{
	r64 r0 = (1-refractive_ratio) / (1+refractive_ratio);
	r0 = r0*r0;
	auto inv_cos = 1 - cosine;
	return r0 + (1-r0)*inv_cos*inv_cos*inv_cos*inv_cos*inv_cos;
}

inline r64
Clamp(r64 x, r64 min, r64 max)
{
	if (x < min) return min;
	if (x > max) return max;
	return x;
}

//////////////////////////////////////////////////////////////////////////
//
// Multithreading stuff
//
//////////////////////////////////////////////////////////////////////////

u64 LockedAdd(volatile u64 *, u64);

inline void
EnqueueTask(task_queue *queue, task order)
{
	queue->tasks.push_back(order);
}

inline i32
QueueSize(task_queue *queue) {
	return queue->tasks.size();
}

inline b32x
QueueEmpty(task_queue *queue) {
	return QueueSize(queue) <= queue->next_index;
}

inline u64
RetireTask(task_queue *queue) {
	return LockedAdd(&queue->retired_tiles, 1);
}

inline b32x
AllTasksRetired(task_queue *queue) {
	return QueueSize(queue) <= queue->retired_tiles;
}

inline b32x
DequeueTask(task_queue *queue, task *order)
{
	if (QueueEmpty(queue)) {
		return false;
	}

	*order = queue->tasks[LockedAdd(&queue->next_index, 1)];

	return true;
}

//////////////////////////////////////////////////////////////////////////
//
// axis-aligned bounding box
//
//////////////////////////////////////////////////////////////////////////

inline b32x
AABBIntersect(const aabb& box, const ray3 &r, r64 tmin, r64 tmax)
{
	for (i32 i = 0; i < 3; ++i)
	{
		auto invB = 1.0 / r.dir[i];

		auto t0 = (box.min[i] - r.orig[i]) * invB;
		auto t1 = (box.max[i] - r.orig[i]) * invB;

		if (invB < 0)
			std::swap(t0, t1);

		tmin = t0 > tmin ? t0 : tmin;
		tmax = t1 < tmax ? t1 : tmax;

		if (tmax <= tmin)
			return false;
	}

	return true;
}

inline aabb
SurroundingBox(const aabb& a, const aabb& b)
{
	point3 small = {
		fmin(a.min.x, b.min.x),
		fmin(a.min.y, b.min.y),
		fmin(a.min.z, b.min.z)
	};

	point3 big = {
		fmax(a.max.x, b.max.x),
		fmax(a.max.y, b.max.y),
		fmax(a.max.z, b.max.z)
	};

	return {small, big};
}

#endif // RAYTRACER_HH
