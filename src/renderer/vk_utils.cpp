#include "std.h"
#include "lib/common.h"

#include "vk_utils.h"
#include "lib/math.h"
#include <cassert>

VkPipelineLayout create_pipeline_layout(std::initializer_list<VkDescriptorSetLayout> set_layouts,
    std::initializer_list<VkPushConstantRange> push_constant_ranges, const char* name)
{
    VkPipelineLayoutCreateInfo create_info { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    create_info.setLayoutCount = (uint32_t)set_layouts.size();
    create_info.pSetLayouts = set_layouts.begin();
    create_info.pushConstantRangeCount = (uint32_t)push_constant_ranges.size();
    create_info.pPushConstantRanges = push_constant_ranges.begin();

    VkPipelineLayout pipeline_layout;
    VK_CHECK(vkCreatePipelineLayout(vk.device, &create_info, nullptr, &pipeline_layout));
    vk_set_debug_name(pipeline_layout, name);
    return pipeline_layout;
}

VkPipeline create_compute_pipeline(const std::string& spirv_file, VkPipelineLayout pipeline_layout, const char* name)
{
    Shader_Module shader(spirv_file);

    VkPipelineShaderStageCreateInfo compute_stage { VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    compute_stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    compute_stage.module = shader.handle;
    compute_stage.pName = "main";

    VkComputePipelineCreateInfo create_info{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
    create_info.stage = compute_stage;
    create_info.layout = pipeline_layout;

    VkPipeline pipeline;
    VK_CHECK(vkCreateComputePipelines(vk.device, VK_NULL_HANDLE, 1, &create_info, nullptr, &pipeline));
    vk_set_debug_name(pipeline, name);
    return pipeline;
}

VkDescriptorSet allocate_descriptor_set(VkDescriptorSetLayout set_layout)
{
    VkDescriptorSetAllocateInfo alloc_info { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
    alloc_info.descriptorPool = vk.descriptor_pool;
    alloc_info.descriptorSetCount = 1;
    alloc_info.pSetLayouts = &set_layout;
    VkDescriptorSet descriptor_set;
    VK_CHECK(vkAllocateDescriptorSets(vk.device, &alloc_info, &descriptor_set));
    return descriptor_set;
}

//
// Descriptor_Writes
//
Descriptor_Writes& Descriptor_Writes::sampled_image(uint32_t binding, VkImageView image_view, VkImageLayout layout) {
    assert(write_count < max_writes);
    VkDescriptorImageInfo& image = resource_infos[write_count].image;
    image = VkDescriptorImageInfo{};
    image.imageView = image_view;
    image.imageLayout = layout;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet = descriptor_set;
    write.dstBinding = binding;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    write.pImageInfo = &image;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::sampled_image_array(uint32_t binding, uint32_t array_size, const VkDescriptorImageInfo* image_infos) {
    assert(write_count < max_writes);
    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet = descriptor_set;
    write.dstBinding = binding;
    write.descriptorCount = array_size;
    write.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    write.pImageInfo = image_infos;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::storage_image(uint32_t binding, VkImageView image_view) {
    assert(write_count < max_writes);
    VkDescriptorImageInfo& image = resource_infos[write_count].image;
    image = VkDescriptorImageInfo{};
    image.imageView = image_view;
    image.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet = descriptor_set;
    write.dstBinding = binding;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    write.pImageInfo = &image;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::sampler(uint32_t binding, VkSampler sampler) {
    assert(write_count < max_writes);
    VkDescriptorImageInfo& image = resource_infos[write_count].image;
    image = VkDescriptorImageInfo{};
    image.sampler = sampler;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet = descriptor_set;
    write.dstBinding = binding;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
    write.pImageInfo = &image;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::uniform_buffer(uint32_t binding, VkBuffer buffer_handle, VkDeviceSize offset, VkDeviceSize range) {
    assert(write_count < max_writes);
    VkDescriptorBufferInfo& buffer = resource_infos[write_count].buffer;
    buffer.buffer = buffer_handle;
    buffer.offset = offset;
    buffer.range = range;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet = descriptor_set;
    write.dstBinding = binding;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    write.pBufferInfo = &buffer;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::storage_buffer(uint32_t binding, VkBuffer buffer_handle, VkDeviceSize offset, VkDeviceSize range) {
    assert(write_count < max_writes);
    VkDescriptorBufferInfo& buffer = resource_infos[write_count].buffer;
    buffer.buffer = buffer_handle;
    buffer.offset = offset;
    buffer.range = range;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet = descriptor_set;
    write.dstBinding = binding;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    write.pBufferInfo = &buffer;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::storage_buffer_array(uint32_t binding, uint32_t array_size, const VkDescriptorBufferInfo* buffer_infos) {
    assert(write_count < max_writes);
    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet = descriptor_set;
    write.dstBinding = binding;
    write.descriptorCount = array_size;
    write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    write.pBufferInfo = buffer_infos;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::accelerator(uint32_t binding, VkAccelerationStructureNV acceleration_structure) {
    ASSERT(write_count < max_writes);
    Accel_Info_NV& accel_info = resource_infos[write_count].accel_info_nv;
    accel_info.handle = acceleration_structure;

    VkWriteDescriptorSetAccelerationStructureNV& accel = accel_info.accel;
    accel = VkWriteDescriptorSetAccelerationStructureNV { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_NV };
    accel.accelerationStructureCount = 1;
    accel.pAccelerationStructures = &accel_info.handle;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.pNext = &accel;
    write.dstSet = descriptor_set;
    write.dstBinding = binding;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_NV;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::accelerator(uint32_t binding, VkAccelerationStructureKHR acceleration_structure) {
    assert(write_count < max_writes);
    Accel_Info& accel_info = resource_infos[write_count].accel_info;
    accel_info.handle = acceleration_structure;

    VkWriteDescriptorSetAccelerationStructureKHR& accel = accel_info.accel;
    accel = VkWriteDescriptorSetAccelerationStructureKHR { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR };
    accel.accelerationStructureCount = 1;
    accel.pAccelerationStructures = &accel_info.handle;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.pNext = &accel;
    write.dstSet = descriptor_set;
    write.dstBinding = binding;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
    return *this;
}

void Descriptor_Writes::commit() {
    assert(descriptor_set != VK_NULL_HANDLE);
    if (write_count > 0) {
        vkUpdateDescriptorSets(vk.device, write_count, descriptor_writes, 0, nullptr);
        write_count = 0;
    }
}

//
// Descriptor_Set_Layout
//
static VkDescriptorSetLayoutBinding get_set_layout_binding(uint32_t binding, uint32_t count,
    VkDescriptorType descriptor_type, VkShaderStageFlags stage_flags)
{
    VkDescriptorSetLayoutBinding entry{};
    entry.binding = binding;
    entry.descriptorType = descriptor_type;
    entry.descriptorCount = count;
    entry.stageFlags = stage_flags;
    return entry;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::sampled_image(uint32_t binding, VkShaderStageFlags stage_flags) {
    assert(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::sampled_image_array(uint32_t binding, uint32_t array_size, VkShaderStageFlags stage_flags) {
    assert(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, array_size, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::storage_image(uint32_t binding, VkShaderStageFlags stage_flags) {
    assert(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::sampler(uint32_t binding, VkShaderStageFlags stage_flags) {
    assert(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_SAMPLER, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::uniform_buffer(uint32_t binding, VkShaderStageFlags stage_flags) {
    assert(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::storage_buffer(uint32_t binding, VkShaderStageFlags stage_flags) {
    assert(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::storage_buffer_array(uint32_t binding, uint32_t array_size, VkShaderStageFlags stage_flags) {
    ASSERT(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, array_size, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::accelerator(uint32_t binding, VkShaderStageFlags stage_flags) {
    assert(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::accelerator_nv(uint32_t binding, VkShaderStageFlags stage_flags) {
    assert(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_NV, stage_flags);
    return *this;
}

VkDescriptorSetLayout Descriptor_Set_Layout::create(const char* name) {
    VkDescriptorSetLayoutCreateInfo create_info { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    create_info.bindingCount    = binding_count;
    create_info.pBindings       = bindings;

    VkDescriptorSetLayout set_layout;
    VK_CHECK(vkCreateDescriptorSetLayout(vk.device, &create_info, nullptr, &set_layout));
    vk_set_debug_name(set_layout, name);
    return set_layout;
}

//
// GPU time queries.
//
void GPU_Time_Scope::begin()
{
    ASSERT(parent->frame_active_scope_count < GPU_Time_Keeper::max_scopes);
    parent->frame_active_scopes[parent->frame_active_scope_count++] = this;
    vkCmdWriteTimestamp(vk.command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pool, start_query[vk.frame_index]);
}

void GPU_Time_Scope::end()
{
    vkCmdWriteTimestamp(vk.command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pool, start_query[vk.frame_index] + 1);
}

GPU_Time_Scope* GPU_Time_Keeper::allocate_time_scope(const std::string& name)
{
    ASSERT(scope_count < max_scopes);
    GPU_Time_Scope* time_scope = &scopes[scope_count++];

    time_scope->name = name;
    time_scope->parent = this;
    time_scope->start_query[0] = time_scope->start_query[1] = vk_allocate_timestamp_queries(2);
    time_scope->length_ms = 0.f;
    return time_scope;
}

void GPU_Time_Keeper::initialize_time_scopes()
{
    vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
        vkCmdResetQueryPool(command_buffer, vk.timestamp_query_pools[0], 0, 2 * scope_count);
        vkCmdResetQueryPool(command_buffer, vk.timestamp_query_pools[1], 0, 2 * scope_count);
        for (uint32_t i = 0; i < scope_count; i++) {
            vkCmdWriteTimestamp(command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pools[0], scopes[i].start_query[0]);
            vkCmdWriteTimestamp(command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pools[0], scopes[i].start_query[0] + 1);
            vkCmdWriteTimestamp(command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pools[1], scopes[i].start_query[1]);
            vkCmdWriteTimestamp(command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pools[1], scopes[i].start_query[1] + 1);
            frame_active_scopes[frame_active_scope_count++] = &scopes[i];
        }
        });
}

void GPU_Time_Keeper::retrieve_query_results()
{
    const float influence = 0.25f;

    for (int i = 0; i < frame_active_scope_count; i++) {
        uint32_t start_query = frame_active_scopes[i]->start_query[vk.frame_index];

        uint64_t query_results[2 /*query result + availability*/ * 2 /*start+end timestamps*/];
        VkResult result = vkGetQueryPoolResults(vk.device, vk.timestamp_query_pool, start_query, 2,
            4*sizeof(uint64_t), query_results, 2*sizeof(uint64_t), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WITH_AVAILABILITY_BIT);
        VK_CHECK_RESULT(result);
        ASSERT(result == VK_SUCCESS);

        ASSERT(query_results[2] >= query_results[0]); // check that end time >= start time
        float measured_duration = float(double(query_results[2] - query_results[0]) * vk.timestamp_period_ms);
        scopes[start_query / 2].length_ms = lerp(scopes[start_query / 2].length_ms, measured_duration, 0.25f);
        vkCmdResetQueryPool(vk.command_buffer, vk.timestamp_query_pool, start_query, 2);
    }
    frame_active_scope_count = 0;
}

//
// GPU debug markers.
//
void begin_gpu_marker_scope(VkCommandBuffer command_buffer, const char* name) {
    VkDebugUtilsLabelEXT label { VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT };
    label.pLabelName = name;
    vkCmdBeginDebugUtilsLabelEXT(command_buffer, &label);
}

void end_gpu_marker_scope(VkCommandBuffer command_buffer) {
    vkCmdEndDebugUtilsLabelEXT(command_buffer);
}

void write_gpu_marker(VkCommandBuffer command_buffer, const char* name) {
    VkDebugUtilsLabelEXT label { VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT };
    label.pLabelName = name;
    vkCmdInsertDebugUtilsLabelEXT(command_buffer, &label);
}
