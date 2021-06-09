#include "mcl.h"
#define _USE_MATH_DEFINES

//#include <nav_msgs/OccupancyGrid.h>
//#include "nav_msgs/GetMap.h"


MCL::MCL(ranges::OMap omap, float max_range): rmgpu_ (omap, max_range)
{
    ros::NodeHandle private_nh_("~");
    private_nh_.getParam("angle_step", p_angle_step_);
    private_nh_.getParam("max_particles", p_max_particles_);
    private_nh_.getParam("max_viz_particles", p_max_viz_particles_);
    float tmp;
    private_nh_.getParam("squash_factor", tmp);
    p_inv_squash_factor_ = 1.0f / tmp;
    p_max_range_meters_ = max_range;
    private_nh_.getParam("theta_discretization", p_theta_discretization_);
    private_nh_.param("which_rm", p_which_rm_, std::string("rmgpu"));
    private_nh_.param("publish_odom", p_publish_odom_, 1);
    private_nh_.getParam("viz", p_do_viz_);

    private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
    //private_nh_.param("max_range", p_max_range_meters_, 30.0f);

    private_nh_.param("z_short", p_z_short_, 0.01f);
    private_nh_.param("z_max", p_z_max_, 0.07f);
    private_nh_.param("z_rand", p_z_rand_, 0.12f);
    private_nh_.param("z_hit", p_z_hit_, 0.75f);
    private_nh_.param("sigma_hit", p_sigma_hit_, 8.0f);

    private_nh_.param("motion_dispersion_x", p_motion_dispersion_x_, 0.05f);
    private_nh_.param("motion_dispersion_y", p_motion_dispersion_y_, 0.025f);
    private_nh_.param("motion_dispersion_theta", p_motion_dispersion_theta_, 0.25f);

    p_max_range_px_ = static_cast<int> (max_range / omap.world_scale);

    // std::cout<< "range method: " << which_rm << std::endl;
    ROS_INFO("Parameters: ");
    ROS_INFO("    Angle step:            %d", p_angle_step_);
    ROS_INFO("    Max particles:         %d", p_max_particles_);
    ROS_INFO("    Max viz particles:     %d", p_max_viz_particles_);
    ROS_INFO("    Inv squash factor:     %f", p_inv_squash_factor_);
    ROS_INFO("    Max range meters:      %f", p_max_range_meters_);
    ROS_INFO("    Theta discretization:  %d", p_theta_discretization_);
    ROS_INFO("    Range method:          %s", p_which_rm_.c_str());
    ROS_INFO("    Publish odom:          %s", p_publish_odom_==1?"Yes":"No");
    ROS_INFO("    Do viz:                %s", p_do_viz_==1?"Yes":"No");

    ROS_INFO("    Scan topic:            %s", p_scan_topic_.c_str());
    ROS_INFO("    max_range_px:          %d", p_max_range_px_);

    lidar_initialized_ = 0;
    odom_initialized_ = 0;
    map_initialized_ = 0;

    scan_sub_ = node_.subscribe(p_scan_topic_, 1, &MCL::scanCallback, this);

    /* initialize the state */
    get_omap(omap);
    precompute_sensor_model();
    initialize_global();
}

MCL::~MCL()
{
    // std::cout<< "All done, bye you!!"<< std::endl;
    ROS_INFO("All done, bye yo!!");
}

void MCL::scanCallback(const sensor_msgs::LaserScan& scan)
{
    float angle_min = scan.angle_min;
    float angle_max = scan.angle_max;
    float num_rays = (angle_max - angle_min)/scan.angle_increment;
    int ahead_idx = static_cast<int>(num_rays/2.0);
    ROS_INFO("Received scan data, range ahead is %f",scan.ranges[ahead_idx]);
}

void MCL::get_omap(ranges::OMap omap)
{
    /*
     * The omap was obtained from the map_server in main.cpp.
     * The only job left is to set the permissible_region_ in the map.
     *
     * In the original particle_filter.py, a cell is included in the permissible
     * region if the occupancy value of the cell is 0. The occupancy value is
     * within the range [0, 100], which indicated the probability that the cell
     * is occupied.
     *
     * Here we include a cell if omap.isOccupied(x,y) return false.
     * The difference is that in OMap::OMap(omap), a cell is occupied when its
     * occupancy value is > 10 or == -1 (unknown)
     *
     * When the occupancy map only contains 0, 100, and -1, the above two methods
     * are the same
     */
    map_height_ = omap.height;
    map_width_ = omap.width;
    permissible_region_ = (char*) malloc(sizeof(char)*map_width_*map_height_);
    int count = 0;
    for (int row = 0; row < map_height_; row++)
    {
        for (int col = 0; col < map_width_; col++)
        {
            if (!omap.isOccupied(col, row))
            {
                permissible_region_[row*map_width_ + col] = 1;
                count ++;
            }
            else
                permissible_region_[row*map_width_ + col] = 0;
        }
    }
    ROS_INFO("Process omap and count is %d, which is %f", count, count*1.0 / (map_width_*map_height_));
    map_initialized_ = true;

    /*
     * The following code is used to encode the map as a png image for visualization
     */
    if (false){
        unsigned char *image = (unsigned char*)malloc(sizeof(char)*map_height_*map_width_);
        for (int r = 0; r < map_height_; r ++)
            for (int c = 0; c < map_width_; c++)
                image[r * map_width_ + c] = permissible_region_[r*map_width_+c]?0:255;
        if (lodepng_encode_file("/home/lixun/reconstructed_omap.png",image, map_width_, map_height_, LCT_GREY, 8))
        {
            ROS_ERROR("lodepng_encode");
        }
    }
}

void MCL::precompute_sensor_model()
{
    ROS_INFO("Precomputing sensor model ... ");
    int table_width = p_max_range_px_ + 1;
    size_t table_sz = sizeof(double) * table_width * table_width;
    sensor_model_table_ = (double *)malloc(table_sz);

    /* d is the computed range from RangeLibc */
    for (int d = 0; d < table_width; d++)
    {
        double norm = 0.0;
        /* r is the measured range from Lidar sensor */
        for (int r = 0; r < table_width; r ++)
        {
            double prob = 0.0;
            double z = r - d;
            /* Known obstacle detection: Gaussian function */
            prob += p_z_hit_ * exp(-(z*z)/(2.0*p_sigma_hit_*p_sigma_hit_)) / (p_sigma_hit_*sqrt(2.0*M_PI));
            /* Unknown obstacle, i.e. measured range (r) is shorter than computed range (d) */
            if (r < d)
                prob += 2.0 * p_z_short_ * (d-r) / d;
            /* missed measurement, i.e. r = max_range_px_ */
            if (r == p_max_range_px_)
                prob += p_z_max_;
            /* random measurement, i.e. there is a change that r could be any value */
            if ( r < p_max_range_px_)
                prob += p_z_rand_ * 1.0 / p_max_range_px_;

            norm += prob;
            sensor_model_table_[r * table_width + d] = prob;
        }
        /* normalize */
        for (int r = 0 ; r < table_width; r++)
            sensor_model_table_[r * table_width + d] /= norm;
    }
    /* Upload the sensor mode to RayMarchingGPU */
    rmgpu_.set_sensor_model(sensor_model_table_, table_width);
    if (0)
    {
        ROS_INFO("Sensor mode done. Preview: ");
        for (int row = 0 ; row < 10; row ++)
        {
            ROS_INFO("    %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf",
                     sensor_model_table_[row*table_width + 0],
                     sensor_model_table_[row*table_width + 1],
                     sensor_model_table_[row*table_width + 2],
                     sensor_model_table_[row*table_width + 3],
                     sensor_model_table_[row*table_width + 4],
                     sensor_model_table_[row*table_width + 5],
                     sensor_model_table_[row*table_width + 6],
                     sensor_model_table_[row*table_width + 7],
                     sensor_model_table_[row*table_width + 8],
                     sensor_model_table_[row*table_width + 9]
                );
        }
    }
}

void MCL::initialize_global()
{
    ROS_INFO("GLOBAL INITIALIZATION");
}
