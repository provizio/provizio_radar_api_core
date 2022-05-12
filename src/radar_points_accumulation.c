#include "provizio/radar_api/radar_points_accumulation.h"

#include <assert.h>
#include <linmath.h>
#include <string.h>

static void provizio_transform_radar_point(const provizio_radar_point *point, const provizio_enu_fix *fix_when_received,
                                           const provizio_enu_fix *current_fix,
                                           provizio_radar_point *out_transformed_point)
{
    assert(provizio_quaternion_is_valid_rotation(&fix_when_received->orientation));
    assert(provizio_quaternion_is_valid_rotation(&current_fix->orientation));

    // 1. Convert point from "fix_when_received" reference frame to ENU
    // 1.1. Rotate
    vec3 point_vec3 = {point->x_meters, point->y_meters, point->z_meters};
    quat point_to_enu_quat = {fix_when_received->orientation.x, fix_when_received->orientation.y,
                              fix_when_received->orientation.z, fix_when_received->orientation.w};
    vec3 point_enu;
    quat_mul_vec3(point_enu, point_to_enu_quat, point_vec3);
    // 1.2. Translate
    point_enu[0] += fix_when_received->position.east_meters;
    point_enu[1] += fix_when_received->position.north_meters;
    point_enu[2] += fix_when_received->position.up_meters;
    // 2. Convert point from ENU to "current_fix" reference frame
    // 2.1. Reversed translate
    point_enu[0] -= current_fix->position.east_meters;
    point_enu[1] -= current_fix->position.north_meters;
    point_enu[2] -= current_fix->position.up_meters;
    // 2.2. Reversed rotate
    quat enu_to_out_point_quat = {-current_fix->orientation.x, -current_fix->orientation.y, -current_fix->orientation.z,
                                  current_fix->orientation.w};
    vec3 out_point;
    quat_mul_vec3(out_point, enu_to_out_point_quat, point_enu);

    // 3. Set out_transformed_point fields
    out_transformed_point->x_meters = out_point[0];
    out_transformed_point->y_meters = out_point[1];
    out_transformed_point->z_meters = out_point[2];
    out_transformed_point->velocity_m_s = point->velocity_m_s; // TODO(APT-746): Consider updating velocity
    out_transformed_point->signal_to_noise_ratio = point->signal_to_noise_ratio;
}

static void provizio_transform_radar_point_cloud(const provizio_radar_point_cloud *point_cloud,
                                                 const provizio_enu_fix *fix_when_received,
                                                 const provizio_enu_fix *current_fix,
                                                 provizio_radar_point_cloud *out_transformed_point_cloud)
{
    // Copy the point cloud header
    memcpy(out_transformed_point_cloud, point_cloud, offsetof(provizio_radar_point_cloud, radar_points));

    assert(point_cloud->num_points_received <= point_cloud->num_points_expected);

    provizio_radar_point *dest_point = out_transformed_point_cloud->radar_points;
    for (const provizio_radar_point *source_point = point_cloud->radar_points,
                                    *source_end = point_cloud->radar_points + point_cloud->num_points_received;
         source_point != source_end; ++source_point, ++dest_point)
    {
        provizio_transform_radar_point(source_point, fix_when_received, current_fix, dest_point);
    }
}

void provizio_quaternion_set_identity(provizio_quaternion *out_quaternion)
{
    out_quaternion->w = 1.0f;
    out_quaternion->x = 0.0f;
    out_quaternion->y = 0.0f;
    out_quaternion->z = 0.0f;
}

void provizio_quaternion_set_euler_angles(float x_rad, float y_rad, float z_rad, provizio_quaternion *out_quaternion)
{
    const float half = 0.5F;
    const float cos_half_x = cos(x_rad * half);
    const float sin_half_x = sin(x_rad * half);
    const float cos_half_y = cos(y_rad * half);
    const float sin_half_y = sin(y_rad * half);
    const float cos_half_z = cos(z_rad * half);
    const float sin_half_z = sin(z_rad * half);

    out_quaternion->w = cos_half_x * cos_half_y * cos_half_z + sin_half_x * sin_half_y * sin_half_z;
    out_quaternion->x = sin_half_x * cos_half_y * cos_half_z - cos_half_x * sin_half_y * sin_half_z;
    out_quaternion->y = cos_half_x * sin_half_y * cos_half_z + sin_half_x * cos_half_y * sin_half_z;
    out_quaternion->z = cos_half_x * cos_half_y * sin_half_z - sin_half_x * sin_half_y * cos_half_z;
}

uint8_t provizio_quaternion_is_valid_rotation(const provizio_quaternion *quaternion)
{
    const float epsilon = 0.0001F;
    const float squared_length = quaternion->w * quaternion->w + quaternion->x * quaternion->x +
                                 quaternion->y * quaternion->y + quaternion->z * quaternion->z;
    return 1.0F - epsilon < squared_length && squared_length < 1.0F + epsilon;
}

void provizio_accumulated_radar_point_clouds_init(provizio_accumulated_radar_point_cloud *accumulated_point_clouds,
                                                  size_t num_accumulated_point_clouds)
{
    memset(accumulated_point_clouds, 0, sizeof(provizio_accumulated_radar_point_cloud) * num_accumulated_point_clouds);
}

provizio_accumulated_radar_point_cloud_iterator provizio_accumulate_radar_point_cloud(
    const provizio_radar_point_cloud *point_cloud, const provizio_enu_fix *fix_when_received,
    provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds)
{
    provizio_accumulated_radar_point_cloud_iterator iterator = {0, 0};

    if (num_accumulated_point_clouds == 0)
    {
        // Nowhere to accumulate
        provizio_error("provizio_accumulate_radar_point_cloud: num_accumulated_point_clouds can't be 0");
        return iterator;
    }

    // Find the latest accumulated cloud
    for (provizio_accumulated_radar_point_cloud *accumulated_cloud = accumulated_point_clouds,
                                                *end = accumulated_point_clouds + num_accumulated_point_clouds;
         accumulated_cloud != end && accumulated_cloud->valid; ++accumulated_cloud)
    {
        const uint32_t current_frame_index = accumulated_cloud->point_cloud.frame_index;
        const uint32_t iterators_frame_index =
            accumulated_point_clouds[iterator.point_cloud_index].point_cloud.frame_index;

        if (current_frame_index > iterators_frame_index)
        {
            ++iterator.point_cloud_index;
        }
        else if (current_frame_index != iterators_frame_index)
        {
            break;
        }
    }

    assert(point_cloud->num_points_received <= point_cloud->num_points_expected);
    if (point_cloud->num_points_received == 0)
    {
        // Nothing to accumulate. Just skip.
        return iterator;
    }

    const int8_t no_latest = provizio_accumulated_radar_point_cloud_iterator_is_end(&iterator, accumulated_point_clouds,
                                                                                    num_accumulated_point_clouds);
    if (!no_latest &&
        accumulated_point_clouds[iterator.point_cloud_index].point_cloud.frame_index >= point_cloud->frame_index)
    {
        const uint32_t small_frame_index_cap = 0x0000ffff;
        const uint32_t large_frame_index_threashold = 0xffff0000;

        if (point_cloud->frame_index >= small_frame_index_cap ||
            accumulated_point_clouds[iterator.point_cloud_index].point_cloud.frame_index <=
                large_frame_index_threashold)
        {
            provizio_error(
                "provizio_accumulate_radar_point_cloud: Can't accumulate an older point cloud after a newer one");

            iterator.point_cloud_index = num_accumulated_point_clouds;
            return iterator;
        }

        // A very special case: frame indices seem to have exceeded the 0xffffffff and have been reset. Let's reset
        // accumulation to avoid complicated state-related issues.
        provizio_warning(
            "provizio_accumulate_radar_point_cloud: frame indices overflow detected - resetting accumulation");
        provizio_accumulated_radar_point_clouds_init(accumulated_point_clouds, num_accumulated_point_clouds);
        iterator.point_cloud_index = 0;
    }

    assert(iterator.point_index == 0);
    if (!no_latest)
    {
        iterator.point_cloud_index = (iterator.point_cloud_index + 1) % num_accumulated_point_clouds;
    }
    else
    {
        assert(iterator.point_cloud_index == 0);
    }
    provizio_accumulated_radar_point_cloud *accumulated_cloud = &accumulated_point_clouds[iterator.point_cloud_index];

    memcpy(&accumulated_cloud->point_cloud, point_cloud, sizeof(provizio_radar_point_cloud));
    memcpy(&accumulated_cloud->fix_when_received, fix_when_received, sizeof(provizio_enu_fix));
    accumulated_cloud->valid = 1;

    return iterator;
}

size_t provizio_accumulated_radar_point_clouds_count(
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds)
{
    // We check back to front as most of the time all num_accumulated_point_clouds are filled: it's quicker
    size_t invalid_count = 0;
    for (; invalid_count < num_accumulated_point_clouds; ++invalid_count)
    {
        if (accumulated_point_clouds[num_accumulated_point_clouds - invalid_count - 1].valid)
        {
            break;
        }
    }

    return num_accumulated_point_clouds - invalid_count;
}

size_t provizio_accumulated_radar_points_count(const provizio_accumulated_radar_point_cloud *accumulated_point_clouds,
                                               size_t num_accumulated_point_clouds)
{
    size_t num_points = 0;
    for (const provizio_accumulated_radar_point_cloud *accumulated_cloud = accumulated_point_clouds,
                                                      *end = accumulated_point_clouds + num_accumulated_point_clouds;
         accumulated_cloud != end && accumulated_cloud->valid; ++accumulated_cloud)
    {
        assert(accumulated_cloud->point_cloud.num_points_received <=
               accumulated_cloud->point_cloud.num_points_expected);
        num_points += accumulated_cloud->point_cloud.num_points_received;
    }

    return num_points;
}

int8_t provizio_accumulated_radar_point_cloud_iterator_is_end(
    const provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds)
{
    const int8_t end = iterator->point_cloud_index >= num_accumulated_point_clouds || // Explicit end
                       !accumulated_point_clouds[iterator->point_cloud_index].valid;  // End of accumulated clouds range

    // Make sure the iterator isn't broken, i.e. doesn't point to a valid cloud but out of points range
    assert(end || iterator->point_index <
                      accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received);

    return end;
}

void provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(
    provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds)
{
    if (num_accumulated_point_clouds == 0)
    {
        provizio_error("provizio_accumulated_radar_point_cloud_iterator_next_point_cloud: num_accumulated_point_clouds "
                       "can't be 0");
        return;
    }

    if (provizio_accumulated_radar_point_cloud_iterator_is_end(iterator, accumulated_point_clouds,
                                                               num_accumulated_point_clouds))
    {
        provizio_error(
            "provizio_accumulated_radar_point_cloud_iterator_next_point_cloud: can't go next cloud on an end iterator");
        return;
    }

    assert(iterator->point_cloud_index < num_accumulated_point_clouds);
    const uint32_t current_frame_index = accumulated_point_clouds[iterator->point_cloud_index].point_cloud.frame_index;

    iterator->point_index = 0;
    // Adding num_accumulated_point_clouds makes sure '- 1' doesn't get negative, which can't be stored in size_t
    iterator->point_cloud_index =
        (num_accumulated_point_clouds + iterator->point_cloud_index - 1) % num_accumulated_point_clouds;

    const provizio_accumulated_radar_point_cloud *accumulated_cloud =
        &accumulated_point_clouds[iterator->point_cloud_index];
    if (!accumulated_cloud->valid || accumulated_cloud->point_cloud.frame_index >= current_frame_index)
    {
        // Complete loop over the circular buffer, i.e. we finished iterating
        iterator->point_cloud_index = num_accumulated_point_clouds;
    }
    else
    {
        assert(accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received > 0);
        assert(accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received <=
               accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_expected);
        assert(accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received >
               iterator->point_index);
    }
}

void provizio_accumulated_radar_point_cloud_iterator_next_point(
    provizio_accumulated_radar_point_cloud_iterator *iterator,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds)
{
    if (provizio_accumulated_radar_point_cloud_iterator_is_end(iterator, accumulated_point_clouds,
                                                               num_accumulated_point_clouds))
    {
        provizio_error(
            "provizio_accumulated_radar_point_cloud_iterator_next_point: can't go next point on an end iterator");
        return;
    }

    assert(iterator->point_cloud_index < num_accumulated_point_clouds);
    ++iterator->point_index;
    if (iterator->point_index >= accumulated_point_clouds[iterator->point_cloud_index].point_cloud.num_points_received)
    {
        iterator->point_index = 0;
        provizio_accumulated_radar_point_cloud_iterator_next_point_cloud(iterator, accumulated_point_clouds,
                                                                         num_accumulated_point_clouds);
    }
}

const provizio_accumulated_radar_point_cloud *provizio_accumulated_radar_point_cloud_iterator_get_point_cloud(
    provizio_accumulated_radar_point_cloud_iterator *iterator, const provizio_enu_fix *current_fix,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_point_cloud *optional_out_transformed_point_cloud)
{
    if (provizio_accumulated_radar_point_cloud_iterator_is_end(iterator, accumulated_point_clouds,
                                                               num_accumulated_point_clouds))
    {
        if (optional_out_transformed_point_cloud)
        {
            memset(optional_out_transformed_point_cloud, 0, sizeof(provizio_radar_point_cloud));
        }

        return NULL;
    }

    assert(iterator->point_cloud_index < num_accumulated_point_clouds);
    const provizio_accumulated_radar_point_cloud *accumulated_cloud =
        &accumulated_point_clouds[iterator->point_cloud_index];
    if (optional_out_transformed_point_cloud)
    {
        provizio_transform_radar_point_cloud(&accumulated_cloud->point_cloud, &accumulated_cloud->fix_when_received,
                                             current_fix, optional_out_transformed_point_cloud);
    }

    return accumulated_cloud;
}

const provizio_radar_point *provizio_accumulated_radar_point_cloud_iterator_get_point(
    provizio_accumulated_radar_point_cloud_iterator *iterator, const provizio_enu_fix *current_fix,
    const provizio_accumulated_radar_point_cloud *accumulated_point_clouds, size_t num_accumulated_point_clouds,
    provizio_radar_point *optional_out_transformed_point)
{
    if (provizio_accumulated_radar_point_cloud_iterator_is_end(iterator, accumulated_point_clouds,
                                                               num_accumulated_point_clouds))
    {
        if (optional_out_transformed_point)
        {
            memset(optional_out_transformed_point, 0, sizeof(provizio_radar_point));
        }

        return NULL;
    }

    assert(iterator->point_cloud_index < num_accumulated_point_clouds);
    const provizio_accumulated_radar_point_cloud *accumulated_cloud =
        &accumulated_point_clouds[iterator->point_cloud_index];
    assert(iterator->point_index < accumulated_cloud->point_cloud.num_points_received);
    assert(accumulated_cloud->point_cloud.num_points_received <= accumulated_cloud->point_cloud.num_points_expected);

    const provizio_radar_point *point = &accumulated_cloud->point_cloud.radar_points[iterator->point_index];

    if (optional_out_transformed_point)
    {
        provizio_transform_radar_point(point, &accumulated_cloud->fix_when_received, current_fix,
                                       optional_out_transformed_point);
    }

    return point;
}
