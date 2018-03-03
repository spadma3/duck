package mcomp_msgs;
struct vgps_location_msg
{
    int64_t  timestamp;
    double   location[2];
    double   theta;
    string   vehicle;
    string   vehicle_id;
}
