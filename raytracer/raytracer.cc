/*
   raytracer.cc - Small, parallel raytracer written in C++.
*/

/*
   TODO(kosi): Todo list

   - Add SIMD instructions.
   - Add windowing (probably use own platform specific code).
		- Add dear imgui debugging information.
   - Use BRDF sampling.
   - Incremental increase of sample quality. Draw scene w/ one sample per pixel and increase.
*/

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#include "raytracer.hh"
#include "platform.hh"
#include "stb_image_write.h"

inline u32
GetTotalBitmapSize(bitmap32 *bmp)
{
	return static_cast<u32>(bmp->width * bmp->height * sizeof(u32));
}

inline u32
Color3ToABGR(color3 packed)
{
	u32 result = (255                                                << 24 |
				  static_cast<u8>(Clamp(packed.b, 0.0, 0.999) * 256) << 16 |
				  static_cast<u8>(Clamp(packed.g, 0.0, 0.999) * 256) << 8  |
				  static_cast<u8>(Clamp(packed.r, 0.0, 0.999) * 256));

	return result;
}

inline color3
ApproximateGammaCorrection(color3 input, i32 gamma)
{
	return {pow(input.r, 1.0/gamma), pow(input.g, 1.0/gamma), pow(input.b, 1.0/gamma)};
}

b32x
AllocateBitmap(bitmap32 *bmp, u32 image_width, u32 image_height)
{
	bmp->width = image_width;
	bmp->height = image_height;

	u32 total_size = GetTotalBitmapSize(bmp);

	bmp->data = static_cast<u32*>(malloc(total_size));
	memset(bmp->data, Color3ToABGR(color3{0, 0, 0}), total_size);

	if (!bmp->data) {
		return false;
	}

	return true;
}

void
DestroyBitmap(bitmap32* bmp)
{
	free(bmp->data);
}


b32x
WriteBitmapToFile(const char *filename, bitmap32 *image)
{
	b32x result = stbi_write_bmp(filename,
								 image->width,
								 image->height,
								 4,
								 image->data);

	return result != 0;

}

inline color3
CastRays(world &scene, volatile u64 *ray_count, const ray3& r, i32 depth)
{
	intersect inter = {};

	LockedAdd(ray_count, 1);

	// Too many ray bounces, no more light can be calculated.
	if (depth <= 0)
		return color3{0,0,0};

	if (scene.tree->Hit(r, constants::TOLERANCE, constants::INFIN, inter)) {
		color3 attenuation;
		ray3 next_ray;

		if(inter.mat->Scatter(r, inter, attenuation, next_ray))
			return Hadamard(attenuation, CastRays(scene, ray_count, next_ray, depth - 1));

		return color3{0, 0, 0};
	}

	// Color the sky
	vec3 unit_d = Norm(r.dir);
	auto t = 0.5 * (unit_d.y + 1.0);
	return Lerp({1.0, 1.0, 1.0}, {0.5, 0.7, 1.0}, t);
}

internal u32*
GetBitmapDataPointer(bitmap32 *bitmap, i32 y, i32 x)
{
	return bitmap->data + ((bitmap->height - 1 - y) * bitmap->width + x);
}

internal void
RenderTile(world* scene,
		bitmap32* bitmap,
		i32 xmin, i32 ymin, i32 xmax, i32 ymax, volatile u64 *ray_counter)
{
	// Number of samples per pixel
	auto samples_per_pixel = 100;

	// Maximum number of ray bounces
	auto max_depth = 50;

	for (i32 y = ymin; y < ymax; ++y) {
		u32* data = GetBitmapDataPointer(bitmap, y, xmin);

		for (i32 x = xmin; x < xmax; ++x) {
			color3 out = {};

			for (i32 s = 0; s < samples_per_pixel; ++s)
			{
				auto u = r64(x + RandomUnilateral()) / (bitmap->width-1);
				auto v = r64(y + RandomUnilateral()) / (bitmap->height-1);

				ray3 r = GetRay(scene->cam, u, v);

				out += CastRays(*scene, ray_counter, r, max_depth);
			}
			out *= 1.0/samples_per_pixel;

			// Gamma correct the picture (gamma 2 approximation)
			out = ApproximateGammaCorrection(out, 2);

			*data++ = Color3ToABGR(out);
		}
	}
}

internal b32x
TryRenderTile(task_queue *queue) {
	task order;

	if (!DequeueTask(queue, &order)) {
		return false;
	}

	u64 entropy = order.xmax*123913 + order.ymax*85939 + order.xmin*28282 + order.ymin*99274;

	RenderTile(order.scene,
			order.bmp,
			order.xmin,
			order.ymin,
			order.xmax,
			order.ymax,
			&queue->ray_count);

	RetireTask(queue);

	return true;
}

internal THREAD_PROC_RET
ThreadProc(void *arguments) {
	task_queue *queue = static_cast<task_queue*>(arguments);

	while (TryRenderTile(queue));

	return nullptr;
}

internal world
TwoSpheres()
{
	world scene = {};

	auto checker = std::make_shared<checker_texture>();

	auto odd = std::make_shared<solid_color>();
	odd->color_value = {0.2, 0.3, 0.1};

	checker->odd = odd;

	auto even = std::make_shared<solid_color>();
	even->color_value = {0.9, 0.9, 0.9};

	checker->even = even;

	auto l = AddLambertian(scene, checker);

	AddSphere(scene, point3{0, 10, 0}, 10, l);
	AddSphere(scene, point3{0, -10, 0}, 10, l);

	CalculateBVH(scene, 0.0, 1.0);

	return scene;
}

internal world
RandomScene()
{
	world scene = {};

	/* auto ground_material = AddLambertian(scene, color3{0.5, 0.5, 0.5}); */
	/* AddSphere(scene, point3{0, -1000, 0}, 1000, ground_material); */

	auto checker = std::make_shared<checker_texture>();

	auto odd = std::make_shared<solid_color>();
	odd->color_value = {0.2, 0.3, 0.1};

	checker->odd = odd;

	auto even = std::make_shared<solid_color>();
	even->color_value = {0.9, 0.9, 0.9};

	checker->even = even;

	auto l = AddLambertian(scene, checker);
	AddSphere(scene, point3{0, -1000, 0}, 1000, l);

	for (int a = -11; a < 11; ++a) {
		for (int b = -11; b < 11; ++b) {
			auto mat_prob = RandomUnilateral();

			point3 center{a + 0.9*RandomUnilateral(), 0.2, b + 0.9*RandomUnilateral()};

			if (Length(FromPointAToB(point3{4, 0.2, 0}, center)) > 0.9) {
				material *sphere_mat;

				if (mat_prob < 0.8) {
					// Lambertian/diffuse
					auto albedo = Hadamard(SampleRandomVector(), SampleRandomVector());
					sphere_mat = AddLambertian(scene, albedo);
					auto center2 = center + vec3{0, RandomUnilateral(0,0.5), 0};
					AddMovingSphere(scene, center, center2, 0.0, 1.0, 0.2, sphere_mat);

				} else if (mat_prob < 0.95) {
					// Metal
					auto albedo = SampleRandomVector(0.5, 1.0);
					auto fuzz = RandomUnilateral(0, 0.5);
					sphere_mat = AddMetal(scene, albedo, fuzz);
					AddSphere(scene, center, 0.2, sphere_mat);
				} else {
					// Glass
					sphere_mat = AddDielectric(scene, 1.5);
					AddSphere(scene, center, 0.2, sphere_mat);
				}
			}
		}
	}

	auto mat1 = AddDielectric(scene, 1.5);
	AddSphere(scene, point3{0, 1, 0}, 1.0, mat1);

	auto mat2 = AddLambertian(scene, color3{0.4, 0.2, 0.1});
	AddSphere(scene, point3{-4, 1, 0}, 1.0, mat2);

	auto mat3 = AddMetal(scene, color3{0.7, 0.6, 0.5}, 0.0);
	AddSphere(scene, point3{4, 1, 0}, 1.0, mat3);

	CalculateBVH(scene, 0, 1.0);

	return scene;
}

int
main(int argc, char **argv) {
	// Image
	const r64 aspect_ratio = 16.0 / 9;
	const u32 image_width = 400;
	const u32 image_height = static_cast<u32>(image_width / aspect_ratio);

	// Bitmap file allocation
	const char* filename = "result.bmp";
	bitmap32 bmp = {};

	if(!AllocateBitmap(&bmp, image_width, image_height))
	{
		fprintf(stderr, "[ERROR] Could not allocate bitmap!\n");
		return -1;
	}

	// Camera creation
	world scene;

	point3 lookfrom;
	point3 lookat;
	auto vfov = 40.0;
	auto aperture = 0.0;

	switch (0) {
		case 1:
			scene = RandomScene();
			lookfrom = {13, 2, 3};
			lookat = {0, 0, 0};
			vfov = 20.0;
			aperture = 0.1;
			break;

		default:
		case 2:
			scene = TwoSpheres();
			lookfrom = {13, 2, 3};
			lookat = {0, 0, 0};
			vfov = 20.0;
			break;
	}


	vec3 vup{0, 1, 0};
	auto focus_distance = 10.0;

	camera cam = CreateCamera(
			lookfrom,
			lookat,
			vup,
			vfov,
			aspect_ratio,
			aperture,
			focus_distance,
			0.0,
			1.0
	);

	scene.cam = cam;

	// Seed random number generator
	RandomUnilateral(true);

	// World creation

	printf("Finished initializing scene!\n");

	// TODO(kosi): Replace this with platform specific call.
	// Split image into tiles equal to number of extra cores.
	i32 core_count = 4;
	i32 extra_cores = core_count - 1;

	i32 tile_width = (bmp.width + extra_cores - 1) / extra_cores;
	i32 tile_height = tile_width = 64;

	i32 tile_count_x = (bmp.width + tile_width - 1) / tile_width;
	i32 tile_count_y = (bmp.height + tile_height - 1) / tile_height;
	i32 total_tile_count = tile_count_x * tile_count_y;

	printf("Configuration: %d cores: %dx%d tiles (total=%d)\n",
			extra_cores,
			tile_width,
			tile_height,
			total_tile_count);
	printf("Configuration: %dx%d bitmap\n",
			bmp.width,
			bmp.height);

	auto start = GetTimeMilliseconds();

	task_queue queue = {};

	for (i32 tiley = 0; tiley < tile_count_y; ++tiley)
	{
		i32 ymin = tiley * tile_height;
		i32 ymax = Clamp(ymin + tile_height, 0, image_height);

		for (i32 tilex = 0; tilex < tile_count_x; ++tilex)
		{
			task order = {};

			order.xmin  = tilex * tile_width;
			order.xmax  = Clamp(order.xmin + tile_width, 0, bmp.width);
			order.scene = &scene;
			order.bmp   = &bmp;
			order.ymin  = ymin;
			order.ymax  = ymax;

			EnqueueTask(&queue, order);
		}
	}

	std::vector<thread*> threads;

	for (i32 core = 1; core < core_count; ++core) {
		threads.push_back(CreateTaskThread(ThreadProc, &queue));
	}

	while (!AllTasksRetired(&queue)) {
		printf("\rRaycasting completion: %.2f%%",
				100.0 * r64(queue.retired_tiles)/r64(total_tile_count));

		fflush(stdout);
		TryRenderTile(&queue);
	}

	Assert(queue.retired_tiles == QueueSize(&queue));

	printf("\rRaycasting completion: %.1f%%",
			100.0 * r64(queue.retired_tiles)/r64(total_tile_count));

	fflush(stdout);

	auto end_time = GetTimeMilliseconds() - start;

	printf("\nRaycasting runtime: %lums\n", end_time);
	printf("Total computed rays: %lurays\n", queue.ray_count);
	printf("Performance: %fms/ray\n", r64(end_time)/r64(queue.ray_count));

	// Join all threads
	for (auto thread : threads) {
		JoinTaskThread(thread);
		free(thread);
	}

	// Destroy the world
	DestroyWorld(scene);

	// Bitmap file writing
	if (!WriteBitmapToFile(filename, &bmp))
	{
		fprintf(stderr, "[ERROR] Could not output bitmap to %s!\n", filename);
		return -1;
	}

	// Destroy the bitmap
	DestroyBitmap(&bmp);

	return 0;
}
