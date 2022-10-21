#include "raytracer_renderer.h"

#include "utils/resource_utils.h"

#include <iostream>
#include <random>

std::mt19937 mt;
std::uniform_real_distribution<float> dist(0.f, 1.f);

float3x3 angleAxis3x3(float angle, float3 axis)
{
	float c = cos(angle), s = sin(angle);

	float t = 1 - c;
	float x = axis.x;
	float y = axis.y;
	float z = axis.z;

	return float3x3{
			{t * x * x + c, t * x * y - s * z, t * x * z + s * y},
			{t * x * y + s * z, t * y * y + c, t * y * z - s * x},
			{t * x * z - s * y, t * y * z + s * x, t * z * z + c}};
}

float3 getConeSample(float3 direction, float coneAngle) {

	float cosAngle = cos(coneAngle);

	// Generate points on the spherical cap around the north pole [1].
	// [1] See https://math.stackexchange.com/a/205589/81266
	float z =  dist(mt) * (1.0f - cosAngle) + cosAngle;
	float phi =  dist(mt) * 2.0f * 3.1415926f;

	float x = sqrt(1.0f - z * z) * cos(phi);
	float y = sqrt(1.0f - z * z) * sin(phi);
	float3 north = float3(0.f, 0.f, 1.f);

	// Find the rotation axis `u` and rotation angle `rot` [1]
	float3 axis = normalize(cross(north, normalize(direction)));
	float angle = acos(dot(normalize(direction), north));

	// Convert rotation axis and angle to 3x3 rotation matrix [2]
	float3x3 R = angleAxis3x3(angle, axis);

	auto res = mul(R, float3(x, y, z));
	return {-res.x, -res.y, res.z};
}

void cg::renderer::ray_tracing_renderer::init()
{
	model = std::make_shared<cg::world::model>();
	model->load_obj(settings->model_path);

	camera = std::make_shared<cg::world::camera>();
	camera->set_height(static_cast<float>(settings->height));
	camera->set_width(static_cast<float>(settings->width));
	camera->set_position(float3{settings->camera_position[0], settings->camera_position[1], settings->camera_position[2]});
	camera->set_phi(settings->camera_phi);
	camera->set_theta(settings->camera_theta);
	camera->set_angle_of_view(settings->camera_angle_of_view);
	camera->set_z_near(settings->camera_z_near);
	camera->set_z_far(settings->camera_z_far);

	render_target = std::make_shared<cg::resource<unsigned_color>>(settings->width, settings->height);

	raytracer = std::make_shared<cg::renderer::raytracer<cg::vertex, cg::unsigned_color>>();
	raytracer->set_render_target(render_target);
	raytracer->set_viewport(settings->width, settings->height);
	raytracer->set_vertex_buffers(model->get_vertex_buffers());
	raytracer->set_index_buffers(model->get_index_buffers());

	lights.push_back({
			float3 {0.f, 1.58f, -0.03f},
			float3 {0.78f, 0.78f, 0.78f}
	});

	shadow_raytracer = std::make_shared<cg::renderer::raytracer<cg::vertex, cg::unsigned_color>>();
}

void cg::renderer::ray_tracing_renderer::destroy() {}

void cg::renderer::ray_tracing_renderer::update() {}

void cg::renderer::ray_tracing_renderer::render()
{

	raytracer->clear_render_target({0, 0, 0});
	raytracer->miss_shader = [](const ray& ray) {
		payload payload{};
		payload.color = {0.f, 0.f, (ray.direction.y + 1.f) * 0.5f};
		return payload;
	};
	shadow_raytracer->miss_shader = [](const ray& ray) {
	  return payload{.t = -1.f};
	};

	raytracer->closest_hit_shader = [&](const ray& ray, payload& payload, const triangle<cg::vertex>& triangle, size_t depth) {
		float3 normal = normalize(payload.bary.x * triangle.na + payload.bary.y * triangle.nb + payload.bary.z * triangle.nc);

		float3 result_color = triangle.emissive;
		for (auto & light : lights) {
			float3 position = ray.position + payload.t * ray.direction;
			cg::renderer::ray to_light(position, light.position - position);
			float3 perpL = cross(to_light.direction, float3(0.f, 1.0f, 0.f)); // get perpendicular to light direction
			if (perpL.x == 0.f && perpL.y == 0.f && perpL.z == 0.f) {
				perpL.x = 1.f;
			}
			float3 toLightEdge = normalize((light.position + perpL * 0.05f) - position);
			float coneAngle = acos(dot(to_light.direction, toLightEdge)) * 2.0f;

			cg::renderer::ray appr_to_light(position, getConeSample(light.position - position, coneAngle));

			auto shadow_payload = shadow_raytracer->trace_ray(appr_to_light, 1, length(light.position - position) - 0.05f);
			result_color += triangle.diffuse * light.color * std::max(dot(normal, to_light.direction), 0.125f) * (shadow_payload.t < 0.f ? 1.f : 0.125f);
		}
		payload.color = cg::color::from_float3(result_color);
		return payload;
	};


	shadow_raytracer->any_hit_shader = [](const ray& ray, payload& payload, const triangle<cg::vertex>& triangle) {
		return payload;
	};

	raytracer->build_acceleration_structure();
	shadow_raytracer->acceleration_structures = raytracer->acceleration_structures;

	auto start = std::chrono::high_resolution_clock::now();
	raytracer->ray_generation(camera->get_position(), camera->get_direction(), camera->get_right(), camera->get_up(), settings->raytracing_depth, settings->accumulation_num);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli> duration = end - start;
	std::cout << duration.count() << " ms" << std::endl;

	cg::utils::save_resource(*render_target, settings->result_path);


}