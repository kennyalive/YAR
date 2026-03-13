#pragma once

struct Descriptor_Heap;

constexpr uint32_t max_swapchain_images = 8;

struct Descriptors {
    uint32_t output_image = 0;
    uint32_t swapchain_images = 0;
    uint32_t images_2d = 0;
    uint32_t image_sampler = 0;
    uint32_t instance_infos = 0;
    uint32_t index_buffers = 0;
    uint32_t vertex_buffers = 0;

    uint32_t lambertian_materials = 0;
    uint32_t point_lights = 0;
    uint32_t directional_lights = 0;
    uint32_t diffuse_rectangular_lights = 0;

    uint32_t image_descriptor_size = 0;
    uint32_t buffer_descriptor_size = 0;
    uint32_t sampler_descriptor_size = 0;

    void initialize(Descriptor_Heap& descriptor_heap);
};
