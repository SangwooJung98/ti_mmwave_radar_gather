#include <radar_gather/Gather.h>
#include <tf/transform_datatypes.h>

#define DEBUG 1

RadarG::RadarG(void){}

RadarG::RadarG(int num_radar)
                : num_radar_(num_radar){
    if(num_radar == 0){
        // str_point_ = "/radar_0/ti_mmwave/module0/radar_scan";
        str_point_ = "/ti_mmwave/radar_scan_0";
		str_cld_ = "/radar_0/radarcloud_gather";
    } else if(num_radar == 1){
        // str_point_ = "/radar_1/ti_mmwave/module1/radar_scan";
        str_point_ = "/ti_mmwave/radar_scan_1";
		str_cld_ = "/radar_1/radarcloud_gather";
    }
}

RadarG::~RadarG(void){}

void RadarG::ros_init (ros::NodeHandle &n){
    sub_point_ = n.subscribe<radar_gather::RadarScan>(str_point_, 10000000, &RadarG::point_cb, this);
    pub_cld_ = n.advertise<PointCloud> (str_cld_, 100000);
}

void RadarG::point_cb(const radar_gather::RadarScan::ConstPtr& msg){
    PointScan pnt;
    pnt.x = msg->x;
    pnt.y = msg->y;
    pnt.z = msg->z;
    pnt.intensity = msg->intensity;
    pnt.range = msg->range;
    pnt.velocity = msg->velocity;
    pnt.point_id = msg->point_id;
    q.push(pnt);
    time_stamp.push(msg->header.stamp);
}

void RadarG::runOnce(void){
    mtx1.lock();
    PointScan pnt;
    pnt = q.front();
    mtx1.unlock();
    q.pop();
    
    // if(!num_radar_)
    //     cout << pnt.point_id << endl;

    if(!pnt.point_id){
        int i = 0;
        PointCloud::Ptr vel_cloud{new PointCloud};
        while(!p_data.empty()){
            PointRADAR temp;

            mtx2.lock();
            PointScan n_pnt = p_data.front();
            ros::Time first_time = time_stamp.front();
            mtx2.unlock();

            p_data.pop();
            time_stamp.pop();

            if(!i)
                pcl_conversions::toPCL(first_time, vel_cloud->header.stamp);

            temp.x = n_pnt.x;
            temp.y = n_pnt.y;
            temp.z = n_pnt.z;
            temp.az = n_pnt.range;
            temp.vr = n_pnt.velocity;
            vel_cloud->push_back(temp);
            i++;
        }

        // test for rviz
        vel_cloud->header.frame_id = "test_frame_id";

        vel_cloud->width = i;
        pub_cld_.publish(vel_cloud);


        // ros::Time h_t;
        // pcl_conversions::fromPCL(vel_cloud->header.stamp, h_t);

    }
    
    p_data.push(pnt);
}