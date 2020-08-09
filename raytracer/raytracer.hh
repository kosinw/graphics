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
#include <random>
#include <vector>

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
// Images
//
//////////////////////////////////////////////////////////////////////////

struct bitmap32 {
	u32 width;
	u32 height;
	u32 *data;
};

//////////////////////////////////////////////////////////////////////////
//
// Vectors
//
//////////////////////////////////////////////////////////////////////////

union vec3 {
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
};

using color3 = vec3;
using point3 = vec3;

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
inline r64 RandomBilateral();
inline r64 RandomBilateral(r64, r64);
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

struct ray3
{
	point3 orig;
	vec3 dir;
	r64 time;
};

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

inline ray3
GetRay(const camera& cam, r64 s, r64 t)
{
	vec3 rd     = cam.lens_radius * SampleRandomVectorFromUnitDisk();
	vec3 offset = cam.u * rd.x + cam.v * rd.y;

	vec3 ray_origin   = cam.origin + offset;
	point3 pixel_proj = cam.llc + s*cam.horizontal + t*cam.vertical;

	vec3 dir = Norm(FromPointAToB(ray_origin, pixel_proj));

	return ray3{ray_origin, dir, RandomBilateral(cam.t0, cam.t1)};
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
// Intersect record
//
//////////////////////////////////////////////////////////////////////////

struct material;

struct intersect
{
	point3	 point;
	r64    	 t;
	vec3   	 normal;
	material *mat;
};

//////////////////////////////////////////////////////////////////////////
//
// Materials
//
//////////////////////////////////////////////////////////////////////////


struct material
{
	virtual b32x Scatter(const ray3& incoming_ray, const intersect& inter, color3& attenuation,
			ray3& scattered_ray) const = 0;
};

struct lambertian : material
{
	color3 albedo;

	inline virtual b32x
	Scatter(const ray3& incoming_ray, const intersect& inter, color3& attenuation,
			ray3& scattered_ray) const override
	{
		// Choose the correct normal depending on the angle between the ray and the outward normal
		vec3 correct_normal = Inner(incoming_ray.dir, inter.normal) > 0.0 ? -inter.normal : inter.normal;
		vec3 scatter_dir = correct_normal + SampleRandomVectorFromLambertian();

		scattered_ray = ray3{inter.point, Norm(scatter_dir), incoming_ray.time};
		attenuation = albedo;
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

		if (RandomBilateral() < reflect_probability)
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

struct surface
{
	virtual b32x Hit(const ray3& r, r64 min, r64 max, intersect& inter) const = 0;
};

struct sphere : surface
{
	point3	 center;
	r64    	 radius;
	material *mat;

	virtual b32x Hit(const ray3& r, r64 min, r64 max, intersect& inter) const override;
};

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
			inter.mat    = this->mat;
			return true;
		}

		const auto far = (-b + sqrt(discriminant)) / 2*a;

		if (far > min && far < max) {
			inter.t      = far;
			inter.point  = RayAt(r, inter.t);
			inter.normal = FromPointAToB(this->center, inter.point) / this->radius;
			inter.mat    = this->mat;
			return true;
		}
	}

	return false;
}

struct moving_sphere : surface
{
	point3	 center0, center1;
	r64    	 radius;
	material *mat;
	r64		 t0, t1;

	virtual b32x Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const override;
};

inline b32x
moving_sphere::Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const
{
	r64 scaled_time = (r.time - this->t0) / (this->t1 -this->t0);
	point3 center = Lerp(this->center0, this->center1, scaled_time);

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

struct world : surface
{
	std::vector<surface*> surfaces;
	std::vector<material*> materials;

	camera cam;

	virtual b32x Hit(const ray3 &r, r64 min, r64 max, intersect& inter) const override;
};

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
		delete surface;
	}

	for (const auto &material : scene.materials) {
		delete material;
	}

	scene.surfaces.clear();
	scene.materials.clear();
}

//////////////////////////////////////////////////////////////////////////
//
// Utility
//
//////////////////////////////////////////////////////////////////////////

namespace constants
{
	const r64 TOLERANCE = 0.001;
	const r64 INFIN     = DBL_MAX;
	const r64 PI        = M_PI;
};

inline r64
DegreesToRadians(r64 degrees)
{
	return degrees * constants::PI / 180.0;
}

inline r64
RandomBilateral() {
	local_persist std::uniform_real_distribution<r64> distribution(0.0, 1.0);
	local_persist std::mt19937 generator;
	return distribution(generator);
}

inline r64
RandomBilateral(r64 min, r64 max)
{
	return min + (max-min)*RandomBilateral();
}

inline vec3
SampleRandomVector() {
	return vec3{RandomBilateral(), RandomBilateral(), RandomBilateral()};
}

inline vec3
SampleRandomVector(r64 a, r64 b) {
	return vec3{RandomBilateral(a, b), RandomBilateral(a, b), RandomBilateral(a, b)};
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
		auto v = vec3{RandomBilateral(-1, 1), RandomBilateral(-1, 1), 0};
		if (Length2(v) >= 1) continue;
		return v;
	}
}

inline vec3
SampleRandomVectorFromLambertian() {
	auto a = RandomBilateral(0, 2 * constants::PI);
	auto z = RandomBilateral(-1, 1);
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

#endif // RAYTRACER_HH
