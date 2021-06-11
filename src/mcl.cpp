#include "mcl.h"
#define _USE_MATH_DEFINES // for pi


MCL::MCL(ranges::OMap omap, float max_range): omap_(omap), rmgpu_ (omap, max_range)
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
    private_nh_.param("odometry_topic", p_odom_topic_, std::string("odom"));
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
    ROS_INFO("    Odom topic:            %s", p_odom_topic_.c_str());
    ROS_INFO("    max_range_px:          %d", p_max_range_px_);
    ROS_INFO("    last_pose size is %ld", last_pose_.size());

    lidar_initialized_ = 0;
    odom_initialized_ = 0;
    map_initialized_ = 0;

    /*
     * Initialize particles and weights
     * Each particle has three components: x, y, and angle
     */
    size_t particles_sz = p_max_particles_*3*sizeof(float);
    particles = (float*)malloc(particles_sz);
    weights = (double*)malloc(sizeof(double)*p_max_particles_);

    /* initialize the state */
    get_omap();
    precompute_sensor_model();
    initialize_global();

    /* these topics are for visualization */
    pose_pub_      = node_.advertise<geometry_msgs::PoseStamped>("/pf/viz/inferred_pose", 1);
    particle_pub_  = node_.advertise<geometry_msgs::PoseArray>("/pf/viz/particles", 1);
    fake_scan_pub_ = node_.advertise<sensor_msgs::LaserScan>("/pf/viz/fake_scan", 1);
    rect_pub_      = node_.advertise<geometry_msgs::PolygonStamped>("/pf/viz/poly1", 1);

    if (p_publish_odom_)
        odom_pub_ = node_.advertise<nav_msgs::Odometry>("/pf/pose/odom", 1);

    scan_sub_ = node_.subscribe(p_scan_topic_, 1, &MCL::lidarCB, this);
    odom_sub_ = node_.subscribe(p_odom_topic_, 1, &MCL::odomCB, this);
    pose_sub_ = node_.subscribe("/initialpose", 1, &MCL::pose_initCB, this);
    click_sub_ = node_.subscribe("/clicked_point", 1, &MCL::rand_initCB, this);

    ROS_INFO("Finished initializing, waiting on messages ...");
}

MCL::~MCL()
{
    // std::cout<< "All done, bye you!!"<< std::endl;
    ROS_INFO("All done, bye yo!!");
}

// void MCL::scanCallback(const sensor_msgs::LaserScan& scan)
// {
//     float angle_min = scan.angle_min;
//     float angle_max = scan.angle_max;
//     float num_rays = (angle_max - angle_min)/scan.angle_increment;
//     int ahead_idx = static_cast<int>(num_rays/2.0);
//     ROS_INFO("Received scan data, range ahead is %f",scan.ranges[ahead_idx]);
// }

void MCL::get_omap()
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
    map_height_ = omap_.height;
    map_width_ = omap_.width;
    permissible_region_ = (char*) malloc(sizeof(char)*map_width_*map_height_);
    int count = 0;
    for (int row = 0; row < map_height_; row++)
    {
        for (int col = 0; col < map_width_; col++)
        {
            if (!omap_.isOccupied(col, row))
            {
                std::array<int, 2> cell_id;
                cell_id[0] = col;
                cell_id[1] = row;
                free_cell_id_.push_back(cell_id);
                permissible_region_[row*map_width_ + col] = 1;
                count ++;
            }
            else
                permissible_region_[row*map_width_ + col] = 0;
        }
    }
    ROS_INFO("Process omap and count is %d, which is %f", count, count*1.0 / (map_width_*map_height_));
    ROS_INFO("Number of free cells in the map: %ld", free_cell_id_.size());
    map_initialized_ = true;

    /*
     * The following code is used to encode the map as a png image for visualization
     */
    if (true){
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

/*
 * The following implementation is very slow, compared to the numpy version
 * (7.9 s vs. 0.005 s)
 *
 * TODO: try to use Eigen library
 */
void MCL::initialize_global()
{
    ROS_INFO("GLOBAL INITIALIZATION");
    ros::WallTime start = ros::WallTime::now();
    /*
     * The following code generates random numbers between 0 and 1 uniformly
     * Check: https://www.delftstack.com/howto/cpp/cpp-random-number-between-0-and-1/
     */
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<float> distr(0,1);
    ros::WallTime spot1 = ros::WallTime::now();
    /* First we shuffle the free_cell_ids */
    std::random_shuffle(free_cell_id_.begin(), free_cell_id_.end());
    ros::WallTime spot2 = ros::WallTime::now();
    /* Then we use the first p_max_particles_ free_cell ids to initialize the particles */
    for (int p = 0 ; p < p_max_particles_; p++)
    {
        /* make sure p is not beyond the free_cell_id_'s capacity */
        if (p >= free_cell_id_.size())
            p = p % free_cell_id_.size();
        std::array<float, 3> p_in_map;
        /* x */
        p_in_map[0] = free_cell_id_.at(p).at(0);
        /* y */
        p_in_map[1] = free_cell_id_.at(p).at(1);
        /* angle */
        p_in_map[2] = distr(eng) * 2.0 * M_PI;
        /* convert map coordinates to world coordinates */
        std::array<float, 3> p_in_world = utils::map_to_world(p_in_map, omap_);
        /* now initialize the particles */
        particles[p                       ] = p_in_world[0];
        particles[p + p_max_particles_    ] = p_in_world[1];
        particles[p + p_max_particles_ * 2] = p_in_world[2];
        /* weights */
        weights[p] = 1.0 / p_max_particles_;
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration elapsedTime = end - start;
    ros::WallDuration rngTime = spot1 - start;
    ros::WallDuration shuffleTime = spot2 - spot1;
    ros::WallDuration loopTime = end - spot2;

    ROS_INFO("    Total %lf, rng %lf, shuffle %lf, forloop %lf",
             elapsedTime.toSec(), rngTime.toSec(), shuffleTime.toSec(), loopTime.toSec());
}

void MCL::lidarCB(const sensor_msgs::LaserScan& msg)
{
    float amin = msg.angle_min;
    float amax = msg.angle_max;
    float inc  = msg.angle_increment;
    int num_ranges = (amax - amin) / inc + 1;
    int num_ranges_downsampled = num_ranges / p_angle_step_;
    if (!lidar_initialized_)
    {
        /* When receiving the first lidar data, do the following */
        /* 1. allocate space for num_ranges_downsampled_ */
        downsampled_ranges_ = (float *) malloc(sizeof(float) * num_ranges_downsampled);
    }
    /* down sample the range */
    for (int i = 0; i < num_ranges; i += p_angle_step_)
        downsampled_ranges_[i/p_angle_step_] = msg.ranges[i];
    lidar_initialized_ = 1;
}

void MCL::odomCB(const nav_msgs::Odometry& msg)
{
    std::array<float, 3> pose;
    pose[0] = msg.pose.pose.position.x;
    pose[1] = msg.pose.pose.position.y;
    pose[2] = omap_.quaternion_to_angle(msg.pose.pose.orientation);
    if (odom_initialized_ == 1)
    {
        float c = cos(-last_pose_[2]);
        float s = sin(-last_pose_[2]);
        float dx = pose[0] - last_pose_[0];
        float dy = pose[1] - last_pose_[1];
        odometry_delta_[0] = c*dx-s*dy;
        odometry_delta_[1] = s*dx+c*dy;
        odometry_delta_[2] = pose[2] - last_pose_[2];
        odom_initialized_ = 2;
    }
    last_pose_ = pose;
    last_stamp_ = msg.header.stamp;
    odom_initialized_ = 1;

    /* this topic is slower than lidar, so update every time we receive a message */
    update();
}

void MCL::pose_initCB(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    ROS_INFO("initial pose set");
}
void MCL::rand_initCB(const geometry_msgs::PointStamped& msg)
{
    ROS_INFO("clicked point set");
    initialize_global();
}

void MCL::update()
{

}


std::array<float, 3> utils::map_to_world(std::array<float, 3> p_in_map,ranges::OMap omap)
{
    std::array<float, 3> p_in_world;

    float scale = omap.world_scale;
    float angle = omap.world_angle;
    float ca = omap.world_cos_angle;
    float sa = omap.world_sin_angle;
    float x = p_in_map[0];
    float y = p_in_map[1];
    p_in_world[0] = (ca*x-sa*y)*scale+omap.world_origin_x;
    p_in_world[1] = (sa*x+ca*y)*scale+omap.world_origin_y;
    p_in_world[2] = p_in_map[2] + angle;
    return p_in_world;
}
