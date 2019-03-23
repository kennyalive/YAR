#include "std.h"
#include "utils.h"

//
// Descriptor_Writes
//
Descriptor_Writes& Descriptor_Writes::sampled_image(uint32_t binding, VkImageView image_view, VkImageLayout layout) {
    ASSERT(write_count < max_writes);
    VkDescriptorImageInfo& image = resource_infos[write_count].image;
    image               = VkDescriptorImageInfo{};
    image.imageView     = image_view;
    image.imageLayout   = layout;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet             = descriptor_set;
    write.dstBinding         = binding;
    write.descriptorCount    = 1;
    write.descriptorType     = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    write.pImageInfo         = &image;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::storage_image(uint32_t binding, VkImageView image_view) {
    ASSERT(write_count < max_writes);
    VkDescriptorImageInfo& image = resource_infos[write_count].image;
    image               = VkDescriptorImageInfo{};
    image.imageView     = image_view;
    image.imageLayout   = VK_IMAGE_LAYOUT_GENERAL;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet             = descriptor_set;
    write.dstBinding         = binding;
    write.descriptorCount    = 1;
    write.descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    write.pImageInfo         = &image;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::sampler(uint32_t binding, VkSampler sampler) {
    ASSERT(write_count < max_writes);
    VkDescriptorImageInfo& image = resource_infos[write_count].image;
    image           = VkDescriptorImageInfo{};
    image.sampler   = sampler;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet             = descriptor_set;
    write.dstBinding         = binding;
    write.descriptorCount    = 1;
    write.descriptorType     = VK_DESCRIPTOR_TYPE_SAMPLER;
    write.pImageInfo         = &image;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::uniform_buffer(uint32_t binding, VkBuffer buffer_handle, VkDeviceSize offset, VkDeviceSize range) {
    ASSERT(write_count < max_writes);
    VkDescriptorBufferInfo& buffer = resource_infos[write_count].buffer;
    buffer.buffer   = buffer_handle;
    buffer.offset   = offset;
    buffer.range    = range;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet             = descriptor_set;
    write.dstBinding         = binding;
    write.descriptorCount    = 1;
    write.descriptorType     = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    write.pBufferInfo        = &buffer;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::storage_buffer(uint32_t binding, VkBuffer buffer_handle, VkDeviceSize offset, VkDeviceSize range) {
    ASSERT(write_count < max_writes);
    VkDescriptorBufferInfo& buffer = resource_infos[write_count].buffer;
    buffer.buffer   = buffer_handle;
    buffer.offset   = offset;
    buffer.range    = range;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet             = descriptor_set;
    write.dstBinding         = binding;
    write.descriptorCount    = 1;
    write.descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    write.pBufferInfo        = &buffer;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::storage_buffer_array(uint32_t binding, uint32_t array_size, const VkDescriptorBufferInfo* buffer_infos) {
    ASSERT(write_count < max_writes);

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet             = descriptor_set;
    write.dstBinding         = binding;
    write.descriptorCount    = array_size;
    write.descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    write.pBufferInfo        = buffer_infos;
    return *this;
}

Descriptor_Writes& Descriptor_Writes::accelerator(uint32_t binding, VkAccelerationStructureNV acceleration_structure) {
    ASSERT(write_count < max_writes);
    Accel_Info& accel_info = resource_infos[write_count].accel_info;
    accel_info.handle = acceleration_structure;

    VkWriteDescriptorSetAccelerationStructureNV& accel = accel_info.accel;
    accel = VkWriteDescriptorSetAccelerationStructureNV { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_NV };
    accel.accelerationStructureCount = 1;
    accel.pAccelerationStructures = &accel_info.handle;

    VkWriteDescriptorSet& write = descriptor_writes[write_count++];
    write = VkWriteDescriptorSet { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.pNext              = &accel;
    write.dstSet             = descriptor_set;
    write.dstBinding         = binding;
    write.descriptorCount    = 1;
    write.descriptorType     = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_NV;
    return *this;
}

void Descriptor_Writes::commit() {
    ASSERT(descriptor_set != VK_NULL_HANDLE);
    if (write_count > 0) {
        vkUpdateDescriptorSets(vk.device, write_count, descriptor_writes, 0, nullptr);
        write_count = 0;
    }
}

//
// Descriptor_Set_Layout
//
static VkDescriptorSetLayoutBinding get_set_layout_binding(uint32_t binding, uint32_t count, VkDescriptorType descriptor_type, VkShaderStageFlags stage_flags) {
    VkDescriptorSetLayoutBinding entry{};
    entry.binding           = binding;
    entry.descriptorType    = descriptor_type;
    entry.descriptorCount   = count;
    entry.stageFlags        = stage_flags;
    return entry;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::sampled_image(uint32_t binding, VkShaderStageFlags stage_flags) {
    ASSERT(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::storage_image(uint32_t binding, VkShaderStageFlags stage_flags) {
    ASSERT(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::sampler(uint32_t binding, VkShaderStageFlags stage_flags) {
    ASSERT(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_SAMPLER, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::uniform_buffer(uint32_t binding, VkShaderStageFlags stage_flags) {
    ASSERT(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::storage_buffer(uint32_t binding, VkShaderStageFlags stage_flags) {
    ASSERT(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::storage_buffer_array (uint32_t binding, uint32_t array_size, VkShaderStageFlags stage_flags) {
    ASSERT(binding_count < max_bindings);
    bindings[binding_count++] = get_set_layout_binding(binding, array_size, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, stage_flags);
    return *this;
}

Descriptor_Set_Layout& Descriptor_Set_Layout::accelerator(uint32_t binding, VkShaderStageFlags stage_flags) {
    ASSERT(binding_count < max_bindings);
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

void GPU_Time_Interval::begin() {
    vkCmdWriteTimestamp(vk.command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pool, start_query);
}
void GPU_Time_Interval::end() {
    vkCmdWriteTimestamp(vk.command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pool, start_query + 1);
}

GPU_Time_Interval* GPU_Time_Keeper::allocate_time_interval() {
    ASSERT(time_interval_count < max_time_intervals);
    GPU_Time_Interval* time_interval = &time_intervals[time_interval_count++];

    time_interval->start_query = vk_allocate_timestamp_queries(2);
    time_interval->length_ms = 0.f;
    return time_interval;
}

void GPU_Time_Keeper::initialize_time_intervals() {
    vk_execute(vk.command_pool, vk.queue, [this](VkCommandBuffer command_buffer) {
        vkCmdResetQueryPool(command_buffer, vk.timestamp_query_pool, 0, 2 * time_interval_count);
        for (uint32_t i = 0; i < time_interval_count; i++) {
            vkCmdWriteTimestamp(command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pool, time_intervals[i].start_query);
            vkCmdWriteTimestamp(command_buffer, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vk.timestamp_query_pool, time_intervals[i].start_query + 1);
        }
    });
}

void GPU_Time_Keeper::next_frame() {
    uint64_t timestamps[2*max_time_intervals];
    const uint32_t query_count = 2 * time_interval_count;
    VkResult result = vkGetQueryPoolResults(vk.device, vk.timestamp_query_pool, 0, query_count, query_count * sizeof(uint64_t), timestamps, 0, VK_QUERY_RESULT_64_BIT);
    VK_CHECK_RESULT(result);
    ASSERT(result != VK_NOT_READY);

    const float influence = 0.25f;

    for (uint32_t i = 0; i < time_interval_count; i++) {
        ASSERT(timestamps[2*i + 1] >= timestamps[2*i]);
        time_intervals[i].length_ms = (1.f-influence) * time_intervals[i].length_ms + influence * float(double(timestamps[2*i + 1] - timestamps[2*i]) * vk.timestamp_period_ms);
    }

    vkCmdResetQueryPool(vk.command_buffer, vk.timestamp_query_pool, 0, query_count);
}
