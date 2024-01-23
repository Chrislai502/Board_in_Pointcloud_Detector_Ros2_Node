#include "board_detector_pcd/utils.hpp"

template <typename PointT>
void toPCLPointCloud2(const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)
{
    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloud.width == 0 && cloud.height == 0)
    {
        msg.width  = cloud.size ();
        msg.height = 1;
    }
    else
    {
        assert (cloud.size () == cloud.width * cloud.height);
        msg.height = cloud.height;
        msg.width  = cloud.width;
    }

    // Fill point cloud binary data (padding and all)
    std::size_t data_size = sizeof (PointT) * cloud.size ();
    msg.data.resize (data_size);
    if (data_size)
    {
        memcpy(msg.data.data(), cloud.data(), data_size);
    }

    // Fill fields metadata
    msg.fields.clear ();
    for_each_type<typename traits::fieldList<PointT>::type> (detail::FieldAdder<PointT>(msg.fields));

    msg.header     = cloud.header;
    msg.point_step = sizeof (PointT);
    msg.row_step   = (sizeof (PointT) * msg.width);
    msg.is_dense   = cloud.is_dense;
    /// @todo msg.is_bigendian = ?;
}

/** \brief Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map.
* \param[in] msg the PCLPointCloud2 binary blob
* \param[out] cloud the resultant pcl::PointCloud<T>
* \param[in] field_map a MsgFieldMap object
*
* \note Use fromPCLPointCloud2 (PCLPointCloud2, PointCloud<T>) directly or create you
* own MsgFieldMap using:
*
* \code
* MsgFieldMap field_map;
* createMapping<PointT> (msg.fields, field_map);
* \endcode
*/
template <typename PointT>
void fromPCLPointCloud2(const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
                        const pcl::MsgFieldMap& field_map) 
{
    fromPCLPointCloud2 (msg, cloud, field_map, msg.data.data());
}

/** \brief Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map.
* \param[in] msg the PCLPointCloud2 binary blob (note that the binary point data in msg.data will not be used!)
* \param[out] cloud the resultant pcl::PointCloud<T>
* \param[in] field_map a MsgFieldMap object
* \param[in] msg_data pointer to binary blob data, used instead of msg.data
*
* \note Use fromPCLPointCloud2 (PCLPointCloud2, PointCloud<T>) instead, except if you have a binary blob of
* point data that you do not want to copy into a pcl::PCLPointCloud2 in order to use fromPCLPointCloud2.
*/
template <typename PointT>
void fromPCLPointCloud2(const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
                        const pcl::MsgFieldMap& field_map, const std::uint8_t* msg_data)
{
    // Copy info fields
    cloud.header   = msg.header;
    cloud.width    = msg.width;
    cloud.height   = msg.height;
    cloud.is_dense = msg.is_dense == 1;

    // Resize cloud
    cloud.resize (msg.width * msg.height);

    // check if there is data to copy
    if (msg.width * msg.height == 0)
    {
        PCL_WARN("[pcl::fromPCLPointCloud2] No data to copy.\n");
        return;
    }

    // Copy point data
    std::uint8_t* cloud_data = reinterpret_cast<std::uint8_t*>(cloud.data());

    // Check if we can copy adjacent points in a single memcpy.  We can do so if there
    // is exactly one field to copy and it is the same size as the source and destination
    // point types.
    if (field_map.size() == 1 &&
        field_map[0].serialized_offset == 0 &&
        field_map[0].struct_offset == 0 &&
        field_map[0].size == msg.point_step &&
        field_map[0].size == sizeof(PointT))
    {
        const auto cloud_row_step = (sizeof (PointT) * cloud.width);
        // Should usually be able to copy all rows at once
        if (msg.row_step == cloud_row_step)
        {
            memcpy (cloud_data, msg_data, msg.width * msg.height * sizeof(PointT));
        }
        else
        {
            for (uindex_t i = 0; i < msg.height; ++i, cloud_data += cloud_row_step, msg_data += msg.row_step)
                memcpy (cloud_data, msg_data, cloud_row_step);
        }

    }
    else
    {
        // If not, memcpy each group of contiguous fields separately
        for (std::size_t row = 0; row < msg.height; ++row)
        {
            const std::uint8_t* row_data = msg_data + row * msg.row_step;
            for (std::size_t col = 0; col < msg.width; ++col)
            {
                const std::uint8_t* msg_data = row_data + col * msg.point_step;
                for (const detail::FieldMapping& mapping : field_map)
                {
                std::copy(msg_data + mapping.serialized_offset, msg_data + mapping.serialized_offset + mapping.size,
                            cloud_data + mapping.struct_offset);
                }
                cloud_data += sizeof (PointT);
            }
        }
    }
}
