#include <geographic_msgs/msg/geo_point.hpp>
#include <geodesy/utm.h>

geometry_msgs::msg::Point gps_to_utm(const geographic_msgs::msg::GeoPoint &gps)
{
    geographic_msgs::msg::GeoPointStamped geo_stamped;
    geo_stamped.position = gps;

    auto utm_stamped = geodesy::UTMPoint(geo_stamped);

    geometry_msgs::msg::Point point;
    point.x = utm_stamped.easting;
    point.y = utm_stamped.northing;
    point.z = utm_stamped.altitude;

    return point;
}