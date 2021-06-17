#include "mcl.h"
#define _USE_MATH_DEFINES // for pi ==> M_PI
#include <numeric> // for transform_reduce


MCL::MCL(ranges::OMap omap, float max_range_px):
    omap_(omap),
    rmgpu_ (omap, max_range_px),
    rm_ (omap, max_range_px)
{
    ros::NodeHandle private_nh_("~");
    private_nh_.getParam("angle_step", p_angle_step_);
    private_nh_.getParam("max_particles", p_max_particles_);
    private_nh_.getParam("max_viz_particles", p_max_viz_particles_);
    float tmp;
    private_nh_.getParam("squash_factor", tmp);
    p_inv_squash_factor_ = 1.0f / tmp;
    p_max_range_meters_ = max_range_px * omap.world_scale;
    private_nh_.getParam("theta_discretization", p_theta_discretization_);
    private_nh_.param("which_rm", p_which_rm_, std::string("rmgpu"));
    private_nh_.param("which_impl", p_which_impl_, std::string("cpu"));
    private_nh_.param("publish_odom", p_publish_odom_, 1);
    private_nh_.getParam("viz", p_do_viz_);

    private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
    private_nh_.param("odometry_topic", p_odom_topic_, std::string("odom"));

    private_nh_.param("z_short", p_z_short_, 0.01f);
    private_nh_.param("z_max", p_z_max_, 0.07f);
    private_nh_.param("z_rand", p_z_rand_, 0.12f);
    private_nh_.param("z_hit", p_z_hit_, 0.75f);
    private_nh_.param("sigma_hit", p_sigma_hit_, 8.0f);

    private_nh_.param("motion_dispersion_x", p_motion_dispersion_x_, 0.05f);
    private_nh_.param("motion_dispersion_y", p_motion_dispersion_y_, 0.025f);
    private_nh_.param("motion_dispersion_theta", p_motion_dispersion_theta_, 0.25f);

    p_max_range_px_ = static_cast<int> (max_range_px);

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

    ROS_INFO("    which_impl:            %s", p_which_impl_.c_str());

    lidar_initialized_ = 0;
    odom_initialized_ = 0;
    map_initialized_ = 0;

    /*
     * Initialize particles and weights
     * Each particle has three components: x, y, and angle
     */
    size_t particles_sz = p_max_particles_*3*sizeof(float);
    particles_ = (float*)malloc(particles_sz);
    weights_.resize(p_max_particles_);
    particles_x_.resize(p_max_particles_);
    particles_y_.resize(p_max_particles_);
    particles_angle_.resize(p_max_particles_);

    /* these topics are for visualization */
    pose_pub_      = node_.advertise<geometry_msgs::PoseStamped>("/pf/viz/inferred_pose", 1, true);
    particle_pub_  = node_.advertise<geometry_msgs::PoseArray>("/pf/viz/particles", 1, true);
    fake_scan_pub_ = node_.advertise<sensor_msgs::LaserScan>("/pf/viz/fake_scan", 1);
    rect_pub_      = node_.advertise<geometry_msgs::PolygonStamped>("/pf/viz/poly1", 1);


    if (p_publish_odom_)
        odom_pub_ = node_.advertise<nav_msgs::Odometry>("/pf/pose/odom", 1);

    scan_sub_ = node_.subscribe(p_scan_topic_, 1, &MCL::lidarCB, this);
    odom_sub_ = node_.subscribe(p_odom_topic_, 1, &MCL::odomCB, this);
    pose_sub_ = node_.subscribe("/initialpose", 1, &MCL::pose_initCB, this);
    click_sub_ = node_.subscribe("/clicked_point", 1, &MCL::rand_initCB, this);

    /* initialize the state */
    get_omap();
    precompute_sensor_model();
    initialize_global();

    ROS_INFO("Finished initializing, waiting on messages ...");
}

MCL::~MCL()
{
    free(particles_);
    ROS_INFO("All done, bye yo!!");
}

void MCL::get_omap()
{
    /*
      @note
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
                count ++;
            }
        }
    }
    ROS_DEBUG("Process omap and count is %d, which is %f", count, count*1.0 / (map_width_*map_height_));
    ROS_DEBUG("Number of free cells in the map: %ld", free_cell_id_.size());
    map_initialized_ = true;

#if 0
    /*
     * The following code is used to encode the map as a png image for visualization
     */
    unsigned char *image = (unsigned char*)malloc(sizeof(char)*map_height_*map_width_);
    for (int r = 0; r < map_height_; r ++)
        for (int c = 0; c < map_width_; c++)
            image[r * map_width_ + c] = 255;
    for (std::array<int,2> cell : free_cell_id_)
        image[cell[1] * map_width_ + cell[0]] = 0;
    if (lodepng_encode_file("/home/lixun/reconstructed_omap.png",image, map_width_, map_height_, LCT_GREY, 8))
    {
        ROS_ERROR("lodepng_encode");
    }
#endif
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
    /* Upload the sensor mode to RayMarching Methods */
    if (!p_which_rm_.compare("rmgpu"))
        rmgpu_.set_sensor_model(sensor_model_table_, table_width);
    else if (!p_which_rm_.compare("rm"))
        rm_.set_sensor_model(sensor_model_table_, table_width);
    else
        ROS_ERROR("Unknown rm method!!");
#if 0
    /* preview the sensor table */
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
#endif
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
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine eng(seed);
    std::uniform_real_distribution<float> distr(0,1);
    ros::WallTime spot1 = ros::WallTime::now();
    /* First we shuffle the free_cell_ids */
    std::srand(std::time(0));
    std::random_shuffle(free_cell_id_.begin(), free_cell_id_.end());
    ros::WallTime spot2 = ros::WallTime::now();
    /* Then we use the first p_max_particles_ free_cell ids to initialize the particles */
    for (int p = 0 ; p < p_max_particles_; p++)
    {
        /* make sure p is not beyond the free_cell_id_'s capacity */
        if (p >= free_cell_id_.size())
            p = p % free_cell_id_.size();
        pose_t p_in_map;
        /* x */
        p_in_map[0] = free_cell_id_.at(p).at(0);
        /* y */
        p_in_map[1] = free_cell_id_.at(p).at(1);
        /* angle */
        p_in_map[2] = distr(eng) * 2.0 * M_PI;
        /* convert map coordinates to world coordinates */
        pose_t p_in_world = omap_.grid_to_world(p_in_map);
        /* now initialize the particles */
        particles_x_[p] = p_in_world[0];
        particles_y_[p] = p_in_world[1];
        particles_angle_[p] = p_in_world[2];
        /* weights */
        weights_[p] = 1.0 / p_max_particles_;
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration elapsedTime = end - start;
    ros::WallDuration rngTime = spot1 - start;
    ros::WallDuration shuffleTime = spot2 - spot1;
    ros::WallDuration loopTime = end - spot2;

    ROS_INFO("    Total %lf, rng %lf, shuffle %lf, forloop %lf",
             elapsedTime.toSec(), rngTime.toSec(), shuffleTime.toSec(), loopTime.toSec());

#ifdef TESTING
    ROS_DEBUG("TESTING THE MCL_cpu() FUNCTION ... ");
    MCL_cpu();
    publish_tf();
    visualize();
#endif
}

void MCL::lidarCB(const sensor_msgs::LaserScan& msg)
{
    float amin = msg.angle_min;
    float amax = msg.angle_max;
    float inc  = msg.angle_increment;
    int num_ranges = (amax - amin) / inc + 1;
    if (!lidar_initialized_)
    {
        int num_ranges_downsampled = num_ranges / p_angle_step_;
        downsampled_ranges_.resize(num_ranges_downsampled);
        downsampled_angles_.resize(num_ranges_downsampled);
        for (int i = 0; i < num_ranges_downsampled; i ++)
            downsampled_angles_[i] = amin + i * p_angle_step_ * inc;

    }
    /* down sample the range */
    for (int i = 0; i < num_ranges; i += p_angle_step_)
        downsampled_ranges_[i/p_angle_step_] = msg.ranges[i];
    lidar_initialized_ = 1;
}

void MCL::odomCB(const nav_msgs::Odometry& msg)
{
    pose_t pose;
    pose[0] = msg.pose.pose.position.x;
    pose[1] = msg.pose.pose.position.y;
    pose[2] = omap_.quaternion_to_angle(msg.pose.pose.orientation);
    if (odom_initialized_ >= 1)
    {
        /*
         * Compute the delta between two odometry in the car's frame,
         * in which x axis is forward and y axis is left
         */
        float c = cos(-last_pose_[2]);
        float s = sin(-last_pose_[2]);
        float dx = pose[0] - last_pose_[0];
        float dy = pose[1] - last_pose_[1];
        odometry_delta_[0] = c*dx-s*dy;
        odometry_delta_[1] = s*dx+c*dy;
        odometry_delta_[2] = pose[2] - last_pose_[2];
        odom_initialized_ = 2;
    }
    else
        odom_initialized_ = 1;
    last_pose_ = pose;
    last_stamp_ = msg.header.stamp;

    /* this topic is slower than lidar, so update every time we receive a message */
    update();
}

void MCL::pose_initCB(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    ROS_INFO("initial pose set");
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::generate_n(particles_x_.begin(), p_max_particles_,
                    [x=msg.pose.pose.position.x,
                     distribution = std::normal_distribution<float>(0.0, 0.5),
                     generator = std::default_random_engine(seed)] () mutable
                        {return x + distribution(generator);});
    std::generate_n(particles_y_.begin(), p_max_particles_,
                    [y=msg.pose.pose.position.y,
                     distribution = std::normal_distribution<float>(0.0, 0.5),
                     generator = std::default_random_engine(seed)] () mutable
                        {return y + distribution(generator);});
    std::generate_n(particles_angle_.begin(), p_max_particles_,
                    [angle=omap_.quaternion_to_angle(msg.pose.pose.orientation),
                     distribution = std::normal_distribution<float>(0.0, 0.4),
                     generator = std::default_random_engine(seed)] () mutable
                        {return angle + distribution(generator);});
    visualize();
}
void MCL::rand_initCB(const geometry_msgs::PointStamped& msg)
{
    ROS_INFO("clicked point set");
    initialize_global();

}

void MCL::update()
{
    if (lidar_initialized_ && odom_initialized_ == 2 && map_initialized_)
    {
        ros::Time t1 = ros::Time::now();
        /*
         * one step of MCL algorithm:
         *   1. resampling
         *   2. apply motion model
         *   3. apply sensor model
         *   4. normalize particle weights
         */
        if (!p_which_impl_.compare("cpu"))
            MCL_cpu();
        else if (!p_which_impl_.compare("gpu"))
            MCL_gpu();
        else if (!p_which_impl_.compare("adaptive"))
            MCL_adaptive();
        else
            ROS_ERROR("The chosen method is not implemented yet");

        /* update inferred_pose_ */
        expected_pose();

        ros::Time t2 = ros::Time::now();

        /* publish transformation frame based on inferred pose */
        publish_tf();

        /* seconds per iteration */
        ros::Duration spi = t2 - t1;
        /* iterations per second */
        double ips = 1.0 / spi.toSec();

        visualize();

    }
}

void MCL::expected_pose()
{
    /*
      @note
      * std::transform_reduce requires c++17
      */
    inferred_pose_[0] = std::transform_reduce(particles_x_.begin(), particles_x_.end(),
                                              weights_.begin(), 0.0);
    inferred_pose_[1] = std::transform_reduce(particles_y_.begin(), particles_y_.end(),
                                              weights_.begin(), 0.0);
    inferred_pose_[2] = std::transform_reduce(particles_angle_.begin(), particles_angle_.end(),
                                              weights_.begin(), 0.0);
}

void MCL::publish_tf()
{
    /*
      @note
     * Check the tutorial for how to write a tf broadcaster
     * https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
     * Note: sendTransform and StampedTransform have opposite ordering of parent and child.
     */
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(inferred_pose_[0], inferred_pose_[1], 0.0) );
    tf::Quaternion q;
    q.setRPY(0,0, inferred_pose_[2]);
    transform.setRotation(q);
    tf_pub_.sendTransform( tf::StampedTransform( transform, last_stamp_, "/map", "/laser" ) );

    if (p_publish_odom_)
    {
        /* Publish the odometry message over ROS */
        /*
         * Create a quaternion message
         * Note that this is different from the quaternion in tf
         */
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(inferred_pose_[2]);
        nav_msgs::Odometry odom;
        odom.header.stamp = last_stamp_;
        odom.header.frame_id = "/map";
        odom.pose.pose.position.x = inferred_pose_[0];
        odom.pose.pose.position.y = inferred_pose_[1];
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom_pub_.publish(odom);
    }
}

void MCL::visualize()
{
    if (!p_do_viz_)
        return;

    /* publish the inferred pose */
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(inferred_pose_[2]);
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "map";
    ps.pose.position.x = inferred_pose_[0];
    ps.pose.position.y = inferred_pose_[1];
    ps.pose.position.z = 0.0;
    ps.pose.orientation = quat;
    pose_pub_.publish(ps);

    /* publish a downsampled version of the particles distribution */
    if (p_max_particles_ > p_max_viz_particles_)
    {
        /* Choose p_max_viz_particles_ particle according to the weights */
        fvec_t px;
        fvec_t py;
        fvec_t pangle;
        select_particles(px, py, pangle, p_max_viz_particles_);
        publish_particles(px, py, pangle, p_max_viz_particles_);
    }
    else
        publish_particles(particles_x_, particles_y_, particles_angle_, p_max_particles_);
}

void MCL::publish_particles(fvec_t px, fvec_t py, fvec_t pangle, int num_particles)
{
    geometry_msgs::PoseArray pa;
    pa.header.stamp = ros::Time::now();
    pa.header.frame_id = "map";
    pa.poses.resize(num_particles);
    geometry_msgs::Quaternion quat;
    for (int i = 0; i < num_particles; i++)
    {
        pa.poses[i].position.x = px[i];
        pa.poses[i].position.y = py[i];
        pa.poses[i].position.z = 0.0;
        quat = tf::createQuaternionMsgFromYaw(pangle[i]);
        pa.poses[i].orientation = quat;
    }
    particle_pub_.publish(pa);
}

/*
 * Selects num_particles particles from the population according to their weights
 */
void MCL::select_particles(fvec_t &px, fvec_t &py, fvec_t &pangle, int num_particles)
{
    px.resize(num_particles);
    py.resize(num_particles);
    pangle.resize(num_particles);
    /*
     * https://stackoverflow.com/questions/42926209/equivalent-function-to-numpy-random-choice-in-c
     * Use weights_ to construct a distribution
     */
    std::discrete_distribution<int> distribution(weights_.begin(), weights_.end());
    /* vector used to hold indices of selected particles */
    std::vector<int> indices;
    indices.reserve(num_particles);
    /* construct a trivial random generator engine from a time-based seed: */
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    /* use a generator lambda to draw random indices based on distribution */
    std::generate_n(back_inserter(indices), num_particles,
                    [distribution = std::move(distribution),
                     generator = std::default_random_engine(seed)
                        ]() mutable {return distribution(generator);});
    /*
     * Use indices to select particles
     * Note that the lambda need to capture this to access member variables of MCL
     * check: https://riptutorial.com/cplusplus/example/2461/class-lambdas-and-capture-of-this
     */
    std::transform(indices.begin(), indices.end(), px.begin(),
                   [this](int index) {return particles_x_[index];});
    std::transform(indices.begin(), indices.end(), py.begin(),
                   [this](int index) {return particles_y_[index];});
    std::transform(indices.begin(), indices.end(), pangle.begin(),
                   [this](int index) {return particles_angle_[index];});
}

void MCL::MCL_cpu()
{
    ROS_INFO("Calling MCL_cpu()");
#ifdef TESTING
    /*
     * Test the sensor model
     * Save the first particle as the true state
     */
    float ins[3];
    ins[0] = particles_x_[0];
    ins[1] = particles_y_[0];
    ins[2] = particles_angle_[0];
#endif
    ROS_DEBUG("Printing particles before resampling");
    print_particles(10);
    /*
     * step 1: Resampling using discrete distribution
     */
    /* temp local vectors used by resampling */
    fvec_t particles_x;
    fvec_t particles_y;
    fvec_t particles_angle;
    select_particles(particles_x, particles_y, particles_angle, p_max_particles_);

    std::copy(particles_x.begin(), particles_x.end(), particles_x_.begin());
    std::copy(particles_y.begin(), particles_y.end(), particles_y_.begin());
    std::copy(particles_angle.begin(), particles_angle.end(), particles_angle_.begin());

    ROS_DEBUG("Printing particles after resampling");
    print_particles(10);

    ROS_DEBUG("Odometry delta: %f  %f  %f",
             odometry_delta_[0], odometry_delta_[1], odometry_delta_[2]);

    /* Step 2: apply the motion model to the particles */
    motion_model();
    ROS_DEBUG("Printing particles after motion model");
    print_particles(10);

#ifdef TESTING
    /*
     * make a fake downsampled_ranges_ and downsampled_angles_
     * Since lidarCB is not called, so downsampled_ranges_ and downsampled_angles_
     * are not initialized. For testing purpose, we need to manually do so.
     */
    float amin = -3.1415927410125732;
    float amax = 3.1415927410125732;
    float inc = 0.005823155865073204;
    int num_angles = (amax - amin) / inc + 1;
    num_angles /= p_angle_step_;
    ROS_DEBUG("num_angles: %d", num_angles);
    downsampled_angles_.resize(num_angles);
    downsampled_ranges_.resize(num_angles);
    for (int a = 0; a < num_angles; a++)
        downsampled_angles_[a] = amin + a * inc * p_angle_step_;

    rm_.numpy_calc_range_angles (ins, downsampled_angles_.data(),
                                 downsampled_ranges_.data(), 1, num_angles);

    ROS_DEBUG("The chosen particles is %f  %f  %f", ins[0], ins[1], ins[2]);
    for (int i=0; i<downsampled_ranges_.size(); i++){
        if (i != 0 && i %6 == 0)
            printf("\n");
        printf("[%4.2f <- %4.2f]  ", downsampled_ranges_[i], ins[2]-omap_.world_angle - downsampled_angles_[i]);
    }
    printf("\n");
#endif
    /* Step 3: apply the sensor model to compute the weights of particles */
#ifdef TESTING
    /*
     * make the first particle exactly match the ins
     */
    particles_x_[0] = ins[0];
    particles_y_[0] = ins[1];
    particles_angle_[0] = ins[2];
#endif
    sensor_model();
    ROS_DEBUG("Printing particles after sensor model");
    print_particles(10);

    /* Step 4: normalize the weights_  (but why?) */
    /*
      @note
     * std::reduce requires c++17
     */
    double sum_weight = std::reduce(weights_.begin(), weights_.end(), 0.0);
    std::transform(weights_.begin(), weights_.end(), weights_.begin(),
                  [s = sum_weight](double w) {return w / s;});
    ROS_DEBUG("Printing particles after normalizing");
    print_particles(10);

    /* Inferref pose */
    expected_pose();
    ROS_DEBUG("Inferred pose: %f  %f  %f",
             inferred_pose_[0], inferred_pose_[1], inferred_pose_[2]);
}

void MCL::MCL_gpu(){}

void MCL::MCL_adaptive(){}

void MCL::motion_model()
{
    /* rotate the action into the coordinate space of each particle */
    fvec_t cosines;
    fvec_t sines;
    cosines.resize(p_max_particles_);
    sines.resize(p_max_particles_);
    std::transform(particles_angle_.begin(), particles_angle_.end(), cosines.begin(),
                   [](float theta) {return cos(theta);});
    std::transform(particles_angle_.begin(), particles_angle_.end(), sines.begin(),
                   [](float theta) {return sin(theta);});

    fvec_t local_deltas_x;
    fvec_t local_deltas_y;
    local_deltas_x.resize(p_max_particles_);
    local_deltas_y.resize(p_max_particles_);
    std::transform(cosines.begin(), cosines.end(),
                   sines.begin(),
                   local_deltas_x.begin(),
                   [this](float c, float s)
                       {return c*odometry_delta_[0]-s*odometry_delta_[1];});
    std::transform(cosines.begin(), cosines.end(),
                   sines.begin(),
                   local_deltas_y.begin(),
                   [this](float c, float s)
                       {return s*odometry_delta_[0]+c*odometry_delta_[1];});
    /* Add the local delta to each particle */
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::transform(particles_x_.begin(), particles_x_.end(),
                   local_deltas_x.begin(),
                   particles_x_.begin(),
                   [distribution = std::normal_distribution<float>(0.0, p_motion_dispersion_x_),
                    generator = std::default_random_engine(seed)]
                   (float x, float delta) mutable
                       {return x + delta + distribution(generator);});
    std::transform(particles_y_.begin(), particles_y_.end(),
                   local_deltas_y.begin(),
                   particles_y_.begin(),
                   [distribution = std::normal_distribution<float>(0.0, p_motion_dispersion_y_),
                    generator = std::default_random_engine(seed)]
                   (float y, float delta) mutable
                       {return y + delta + distribution(generator);});
    std::transform(particles_angle_.begin(), particles_angle_.end(),
                   particles_angle_.begin(),
                   [distribution = std::normal_distribution<float>(0.0, p_motion_dispersion_theta_),
                    generator = std::default_random_engine(seed), this]
                   (float angle) mutable
                       {return angle + odometry_delta_[2] + distribution(generator);});
}

/*
 * For each particle p:
 *   1. compute the ranges in all directions given by downsampled_angles_
 *   2. Evaluate one range using the sensor_model_table
 *   3. Multiply scores from all ranges to make the weight of particle p
 */
void MCL::sensor_model()
{
    if (!p_which_rm_.compare("rmgpu"))
    {
        /*
         * one thread computes one particle
         *
         * Note that we split the particles into 3 vectors so that cuda kernels
         * can have coalesced memory access patterns when accessing particles.
         */
        rmgpu_.calc_range_eval_sensor_model_particle(
            particles_x_, particles_y_, particles_angle_,
            downsampled_ranges_, downsampled_angles_, weights_,
            p_max_particles_, (int)downsampled_ranges_.size());
#if 0
        /* one thread compute one range */
        rmgpu_.calc_range_eval_sensor_model_angle(
            particles_x_, particles_y_, particles_angle_,
            downsampled_ranges_, downsampled_angles_, weights_,
            p_max_particles_, (int)downsampled_ranges_.size());
#endif
    }
    else if (!p_which_rm_.compare("rm"))
    {
        rm_.calc_range_eval_sensor_model(
            particles_x_.data(), particles_y_.data(), particles_angle_.data(),
            downsampled_ranges_.data(), downsampled_angles_.data(),
            weights_.data(),
            p_max_particles_, (int)downsampled_angles_.size());
    }
}

void MCL::print_particles(int n)
{
    if (n > particles_x_.size())
        n = particles_x_.size();
    for (int i = 0; i < n; i ++)
        ROS_DEBUG("%3d:  %f  %f  %f\t(%e)", i, particles_x_[i], particles_y_[i], particles_angle_[i], weights_[i]);
}
