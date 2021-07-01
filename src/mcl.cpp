#include "mcl.h"
#define _USE_MATH_DEFINES // for pi ==> M_PI
#include <numeric> // for transform_reduce



MCL::MCL(ranges::OMap omap, float max_range_px):
    omap_(omap),
    rmgpu_ (omap, max_range_px),
    rm_ (omap, max_range_px),
    timer_ (Utils::Timer(10)),
    maxW_ (Utils::CircularArray<double>(10)),
    diffW_ (Utils::CircularArray<double>(10))
{
    ros::NodeHandle private_nh_("~");
    private_nh_.getParam("angle_step", p_angle_step_);
    private_nh_.getParam("max_particles", p_max_particles_);
    private_nh_.getParam("max_viz_particles", p_max_viz_particles_);
    double tmp;
    private_nh_.getParam("squash_factor", tmp);
    p_inv_squash_factor_ = 1.0 / tmp;
    p_max_range_meters_ = max_range_px * omap.world_scale;
    private_nh_.getParam("theta_discretization", p_theta_discretization_);
    private_nh_.param("which_rm", p_which_rm_, std::string("rmgpu"));
    private_nh_.param("which_impl", p_which_impl_, std::string("cpu"));
    private_nh_.param("which_viz", p_which_viz_, std::string("largest"));
    private_nh_.param("which_expect", p_which_expect_, std::string("largest"));
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
    ROS_INFO("    Inv squash factor:     %lf", p_inv_squash_factor_);
    ROS_INFO("    Max range meters:      %f", p_max_range_meters_);
    ROS_INFO("    Theta discretization:  %d", p_theta_discretization_);
    ROS_INFO("    Range method:          %s", p_which_rm_.c_str());
    ROS_INFO("    Publish odom:          %s", p_publish_odom_==1?"Yes":"No");
    ROS_INFO("    Do viz:                %s", p_do_viz_==1?"Yes":"No");

    ROS_INFO("    Scan topic:            %s", p_scan_topic_.c_str());
    ROS_INFO("    Odom topic:            %s", p_odom_topic_.c_str());
    ROS_INFO("    max_range_px:          %d", p_max_range_px_);

    ROS_INFO("    which_impl:            %s", p_which_impl_.c_str());
    ROS_INFO("    which_viz:             %s", p_which_viz_.c_str());

    lidar_initialized_ = 0;
    odom_initialized_ = 0;
    map_initialized_ = 0;
    init_pose_set_ = 0;

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
    pose_pub_      = node_.advertise<geometry_msgs::PoseStamped>("/pf/viz/inferred_pose", 1);
    particle_pub_  = node_.advertise<geometry_msgs::PoseArray>("/pf/viz/particles", 1);
    fake_scan_pub_ = node_.advertise<sensor_msgs::LaserScan>("/pf/viz/fake_scan", 1);
    rect_pub_      = node_.advertise<geometry_msgs::PolygonStamped>("/pf/viz/poly1", 1);


    if (p_publish_odom_)
        odom_pub_ = node_.advertise<nav_msgs::Odometry>("/pf/pose/odom", 1);

    scan_sub_ = node_.subscribe(p_scan_topic_, 1, &MCL::lidarCB, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = node_.subscribe(p_odom_topic_, 1, &MCL::odomCB, this, ros::TransportHints().tcpNoDelay());
    pose_sub_ = node_.subscribe("/initialpose", 1, &MCL::pose_initCB, this);
    click_sub_ = node_.subscribe("/clicked_point", 1, &MCL::rand_initCB, this);

    /* initialize the state */
    get_omap();
    precompute_sensor_model();
    // initialize_global();
    /*
     * Initialize the particles around the origin
     */
    init_pose_[0] = 0.0f;
    init_pose_[1] = 0.0f;
    init_pose_[2] = 0.0f;
    initialize_initpose();


#ifdef TESTING
    /*
     * For testing only, set fake downsampled angles
     */
    set_fake_angles_and_ranges(0);
    /* end of testing code */
#endif
    /*
     * Initialize the MCLGPU object
     */
    if (!p_which_impl_.compare("gpu"))
    {
        mclgpu_ = new MCLGPU(p_max_particles_);
        mclgpu_->init_constants(
            /* motion model dispersion constants */
            p_motion_dispersion_x_, p_motion_dispersion_y_, p_motion_dispersion_theta_,
            /* squash factor used for weight computation */
            p_inv_squash_factor_);
        mclgpu_->set_sensor_table(sensor_model_table_, p_max_range_px_ + 1);
        mclgpu_->set_map(omap_, max_range_px);
#ifdef TESTING
        /*
         * For testing only, set fake downsampled angles
         */
        mclgpu_->set_angles(downsampled_angles_.data());
        ROS_DEBUG("Testing MCL_gpu");
        MCL_gpu();
        /* end of testing code */
#endif
    }
#ifdef TESTING
    else if (!p_which_impl_.compare("cpu"))
    {
        ROS_DEBUG("Testing MCL_cpu");
        MCL_cpu();
    }
#endif

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
    ros::Time start = ros::Time::now();
    /*
     * The following code generates random numbers between 0 and 1 uniformly
     * Check: https://www.delftstack.com/howto/cpp/cpp-random-number-between-0-and-1/
     */
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine eng(seed);
    std::uniform_real_distribution<float> distr(0,1);
    ros::Time spot1 = ros::Time::now();
    /* First we shuffle the free_cell_ids */
    std::srand(std::time(0));
    std::random_shuffle(free_cell_id_.begin(), free_cell_id_.end());
    ros::Time spot2 = ros::Time::now();
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
    ros::Time end = ros::Time::now();
    ros::Duration elapsedTime = end - start;
    ros::Duration rngTime = spot1 - start;
    ros::Duration shuffleTime = spot2 - spot1;
    ros::Duration loopTime = end - spot2;

    ROS_INFO("    Total %lf, rng %lf, shuffle %lf, forloop %lf",
             elapsedTime.toSec(), rngTime.toSec(), shuffleTime.toSec(), loopTime.toSec());

    initialize_acc();

}

void MCL::initialize_acc()
{
    acc_error_x_ = 0.0f;
    acc_error_y_ = 0.0f;
    acc_error_angle_ = 0.0f;
    acc_time_ms_ = 0.0f;
    iter_ = 0;
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
        if (NUM_ANGLES != num_ranges_downsampled)
        {
            ROS_ERROR("Need to reset NUM_ANGLES to %d in CMakeLists.txt", num_ranges_downsampled);
            ros::shutdown();
        }
        downsampled_ranges_.resize(num_ranges_downsampled);
        downsampled_angles_.resize(num_ranges_downsampled);
        for (int i = 0; i < num_ranges_downsampled; i ++)
            downsampled_angles_[i] = amin + i * p_angle_step_ * inc;
        if (!p_which_impl_.compare("gpu"))
            mclgpu_->set_angles(downsampled_angles_.data());
    }
    /* down sample the range */
    for (int i = 0; i < num_ranges; i += p_angle_step_)
        downsampled_ranges_[i/p_angle_step_] = msg.ranges[i];
    std::replace_if(downsampled_ranges_.begin(), downsampled_ranges_.end(),
                    [](float range){return range > 10.0;}, 10.0);
    lidar_initialized_ = 1;
}

void print_odom_msg(const nav_msgs::Odometry& msg)
{
    printf("header:\n");
    printf("  seq: %d\n", msg.header.seq);
    printf("  stamp:\n");
    printf("    secs: %d\n", msg.header.stamp.sec);
    printf("    nsecs: %d\n", msg.header.stamp.nsec);
    printf("  frame_id: \"%s\"\n", msg.header.frame_id.c_str());
    printf("child_frame_id: \"%s\"\n", msg.child_frame_id.c_str());
    printf("pose:\n");
    printf("  pose:\n");
    printf("    position:\n");
    printf("      x: %.16lf\n", msg.pose.pose.position.x);
    printf("      y: %.16lf\n", msg.pose.pose.position.y);
    printf("      z: %.16lf\n", msg.pose.pose.position.z);
    printf("    orientation:\n");
    printf("      x: %.16lf\n", msg.pose.pose.orientation.x);
    printf("      y: %.16lf\n", msg.pose.pose.orientation.y);
    printf("      z: %.16lf\n", msg.pose.pose.orientation.z);
    printf("      w: %.16lf\n", msg.pose.pose.orientation.w);
    printf("  covariance: [");
    for (int i = 0; i < 36; i++) printf("0.0, ");
    printf("]\n");

    printf("twist:\n");
    printf("  twist:\n");
    printf("    linear:\n");
    printf("      x: %.16lf\n", msg.twist.twist.linear.x);
    printf("      y: %.16lf\n", msg.twist.twist.linear.y);
    printf("      z: %.16lf\n", msg.twist.twist.linear.z);
    printf("    angular:\n");
    printf("      x: %.16lf\n", msg.twist.twist.angular.x);
    printf("      y: %.16lf\n", msg.twist.twist.angular.y);
    printf("      z: %.16lf\n", msg.twist.twist.angular.z);
    printf("  covariance: [");
    for (int i = 0; i < 36; i++) printf("0.0, ");
    printf("]\n");
    printf("---\n");
}

void MCL::odomCB(const nav_msgs::Odometry& msg)
{
    pose_t pose;
    pose[0] = msg.pose.pose.position.x;
    pose[1] = msg.pose.pose.position.y;
    pose[2] = omap_.quaternion_to_angle(msg.pose.pose.orientation);
    odom_mtx_.lock();
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
    odom_mtx_.unlock();

    /* this topic is slower than lidar, so update every time we receive a message */
    //update();
}

void MCL::pose_initCB(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    init_pose_[0] = msg.pose.pose.position.x;
    init_pose_[1] = msg.pose.pose.position.y;
    init_pose_[2] = omap_.quaternion_to_angle(msg.pose.pose.orientation);
    init_pose_set_ = 1;
    ROS_INFO("initial pose set at %f %f %f", init_pose_[0], init_pose_[1], init_pose_[2]);
    initialize_initpose();
    expected_pose();
    visualize();
}

/* Initialize the particles around the init_pose_ */
void MCL::initialize_initpose()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::generate_n(particles_x_.begin(), p_max_particles_,
                    [x=init_pose_[0],
                     distribution = std::normal_distribution<float>(0.0, 0.5),
                     generator = std::default_random_engine(seed)] () mutable
                        {return x + distribution(generator);});
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::generate_n(particles_y_.begin(), p_max_particles_,
                    [y=init_pose_[1],
                     distribution = std::normal_distribution<float>(0.0, 0.5),
                     generator = std::default_random_engine(seed)] () mutable
                        {return y + distribution(generator);});
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::generate_n(particles_angle_.begin(), p_max_particles_,
                    [angle=init_pose_[2],
                     distribution = std::normal_distribution<float>(0.0, 0.4),
                     generator = std::default_random_engine(seed)] () mutable
                        {return angle + distribution(generator);});
    double w = 1.0 / p_max_particles_;
    std::fill(weights_.begin(), weights_.end(), w);

    initialize_acc();
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
        pose_t old_pose;
        odom_mtx_.lock();
        old_pose[0] = last_pose_[0];
        old_pose[1] = last_pose_[1];
        old_pose[2] = last_pose_[2];
        odom_mtx_.unlock();
        timer_.tick();
        ros::Time t1 = ros::Time::now();
        double maxW;
        /*
         * one step of MCL algorithm:
         *   1. resampling
         *   2. apply motion model
         *   3. apply sensor model
         *   4. normalize particle weights
         */
        if (!p_which_impl_.compare("cpu"))
            maxW = MCL_cpu();
        else if (!p_which_impl_.compare("gpu"))
            maxW = MCL_gpu();
        else if (!p_which_impl_.compare("adaptive"))
            MCL_adaptive();
        else
            ROS_ERROR("The chosen method is not implemented yet");

        /* update inferred_pose_ */
        expected_pose();

        ros::Time t2 = ros::Time::now();

        /* publish transformation frame based on inferred pose */
        //publish_tf();

        /* seconds per iteration */
        ros::Duration spi = t2 - t1;

        do_acc(spi.toSec()*1000.0);
        maxW_.append(maxW);
        pose_t new_pose;
        odom_mtx_.lock();
        new_pose[0] = last_pose_[0];
        new_pose[1] = last_pose_[1];
        new_pose[2] = last_pose_[2];
        odom_mtx_.unlock();
        double diffW = calc_diff(new_pose);
        diffW_.append(diffW);
        iter_ ++;

        if (iter_ != 0 && iter_ %10 == 0)
            printf("iter %4d: time %7.4f ms (event time %7.4f ms)  maxW: %e  diffW: %e\n",
                   iter_,
                   acc_time_ms_ / iter_,
                   timer_.fps(),
                   maxW_.mean(),
                   diffW_.mean()
                );

        visualize();


        // printf("%3d delta: (%e, %e, %e) --- w: %e  maxW: %e\n", iter_,
        //        old_pose[0] - new_pose[0],
        //        old_pose[1] - new_pose[1],
        //        old_pose[2] - new_pose[2],
        //        calc_diff(old_pose),
        //        maxW
        //     );
    }
}

double MCL::calc_diff(pose_t reference)
{
    return rm_.calc_diff_pose(reference.data(), inferred_pose_.data(),
                              downsampled_angles_.data(), downsampled_angles_.size(),
                              p_inv_squash_factor_);
}

void MCL::do_acc(float time_in_ms)
{
    acc_time_ms_ += time_in_ms;
    acc_error_x_ += fabs(last_pose_[0] - inferred_pose_[0]);
    acc_error_y_ += fabs(last_pose_[1] - inferred_pose_[1]);
    acc_error_angle_ += fabs(last_pose_[2] - inferred_pose_[2]);
}

void MCL::expected_pose()
{
    if (!p_which_expect_.compare("ave"))
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
    else if (!p_which_expect_.compare("largest"))
    {
        int maxPIdx = std::max_element(weights_.begin(), weights_.end()) - weights_.begin();
        inferred_pose_[0] = particles_x_[maxPIdx];
        inferred_pose_[1] = particles_y_[maxPIdx];
        inferred_pose_[2] = particles_angle_[maxPIdx];
    }
    else
        ROS_ERROR("Expect method %s not recognized!", p_which_expect_.c_str());

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
#if 0
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
#endif

    int n = std::min(p_max_particles_, p_max_viz_particles_);
    if (!p_which_viz_.compare("first"))
    {
        /*
         * In this method, we simply publish the first n particles.
         * We did not do a weight-based selection because we may only select a few particles given
         * the weights are not normalized.
         */
        publish_particles(particles_x_, particles_y_, particles_angle_, n);
    }
    else if (!p_which_viz_.compare("largest"))
    {
        /*
         * In this method, we publish the n particles with larger weights
         * check https://stackoverflow.com/a/12399290
         */
        std::vector<int> idx (p_max_particles_);
        std::iota(idx.begin(), idx.end(), 0);
        stable_sort(idx.begin(), idx.end(),
                    [this](int i1, int i2) { return weights_[i1] > weights_[i2];});
        fvec_t px(n);
        fvec_t py(n);
        fvec_t pangle(n);

        std::transform(idx.begin(), idx.begin()+n, px.begin(),
                       [this](int index) {return particles_x_[index];});
        std::transform(idx.begin(), idx.begin()+n, py.begin(),
                       [this](int index) {return particles_y_[index];});
        std::transform(idx.begin(), idx.begin()+n, pangle.begin(),
                       [this](int index) {return particles_angle_[index];});

        publish_particles(px, py, pangle, n);
    }
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

double MCL::MCL_cpu()
{
    /*************************************************
     * Step 1: Resampling using discrete_distribution
     *************************************************/
    resampling();

    /***************************************************
     * Step 2: apply the motion model to the particles
     ***************************************************/
    motion_model();

#ifdef TESTING
    if (!init_pose_set_)
    {
        init_pose_[0] = 0.0;
        init_pose_[1] = 0.0;
        init_pose_[2] = 0.0;
    }
    fvec_t fake_ranges(downsampled_ranges_.size());

    printf("-----\n");
    printf("ranges from init pose\n");
    calc_range_one_pose(init_pose_, fake_ranges, false);

    ROS_DEBUG("The chosen particles is %f  %f  %f", init_pose_[0], init_pose_[1], init_pose_[2]);
    printf("ranges from simulator:\n");
    calc_range_one_pose(init_pose_, downsampled_ranges_, true);

    /*
     * Put the init pose into the particles
     */
    particles_x_[99] = init_pose_[0];
    particles_y_[99] = init_pose_[1];
    particles_angle_[99] = init_pose_[2];
#endif

    /*********************************************************************
     * Step 3: apply the sensor model to compute the weights of particles
     *********************************************************************/
    sensor_model();

#ifdef TESTING
    int maxPIdx = std::max_element(weights_.begin(), weights_.end()) - weights_.begin();
    pose_t ins;
    ins[0] = particles_x_[maxPIdx];
    ins[1] = particles_y_[maxPIdx];
    ins[2] = particles_angle_[maxPIdx];
    printf("ranges from largest particle (%d)\n", maxPIdx);
    calc_range_one_pose(ins, fake_ranges, false);
#endif

    /*********************************************
     * Step 4: normalize the weights_  (but why?)
     *********************************************/
    /*
      @note
      * std::reduce requires c++17
      */
    double sum_weight = std::reduce(weights_.begin(), weights_.end(), 0.0);
    double maxW = *std::max_element(weights_.begin(), weights_.end());
    std::transform(weights_.begin(), weights_.end(), weights_.begin(),
                   [s = sum_weight](double w) {return w / s;});
    return maxW;
}

double MCL::MCL_gpu()
{
    /***************************************************
     * step 1: Resampling
     * using std::discrete_distribution
     ***************************************************/
    resampling();

    /********************************************************************************
     * step 2: apply motion model to advance the particles and compute their weights
     * All done on GPU
     ********************************************************************************/
    mclgpu_->update(
        /* inputs */
        /* particles */
        particles_x_.data(), particles_y_.data(), particles_angle_.data(),
        /* action, i.e. odometry_delta_ */
        odometry_delta_.data(), downsampled_ranges_.data(), (int)downsampled_ranges_.size(),
        /* output */
        weights_.data());

    /*********************************
     * Step 3: normalize the weights
     *********************************/
    double sum_weight = std::reduce(weights_.begin(), weights_.end(), 0.0);
    double maxW = *std::max_element(weights_.begin(), weights_.end());
    std::transform(weights_.begin(), weights_.end(), weights_.begin(),
                   [s = sum_weight](double w) {return w / s;});
    return maxW;
}

void MCL::MCL_adaptive(){}

void MCL::motion_model()
{
    pose_t odelta;
    odom_mtx_.lock();
    odelta[0] = odometry_delta_[0];
    odelta[1] = odometry_delta_[1];
    odelta[2] = odometry_delta_[2];
    odom_mtx_.unlock();
    /* rotate the action into the coordinate space of each particle */
    fvec_t cosines(p_max_particles_);
    fvec_t sines(p_max_particles_);
    std::transform(particles_angle_.begin(), particles_angle_.end(), cosines.begin(),
                   [](float theta) {return cos(theta);});
    std::transform(particles_angle_.begin(), particles_angle_.end(), sines.begin(),
                   [](float theta) {return sin(theta);});

    fvec_t local_deltas_x(p_max_particles_);
    fvec_t local_deltas_y(p_max_particles_);
    std::transform(cosines.begin(), cosines.end(),
                   sines.begin(),
                   local_deltas_x.begin(),
                   [odelta](float c, float s)
                       {return c*odelta[0]-s*odelta[1];});
    std::transform(cosines.begin(), cosines.end(),
                   sines.begin(),
                   local_deltas_y.begin(),
                   [odelta](float c, float s)
                       {return s*odelta[0]+c*odelta[1];});
    /* Add the local delta to each particle */
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::transform(particles_x_.begin(), particles_x_.end(),
                   local_deltas_x.begin(),
                   particles_x_.begin(),
                   [distribution = std::normal_distribution<float>(0.0, p_motion_dispersion_x_),
                    generator = std::default_random_engine(seed)]
                   (float x, float delta) mutable
                       {return x + delta + distribution(generator);});
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::transform(particles_y_.begin(), particles_y_.end(),
                   local_deltas_y.begin(),
                   particles_y_.begin(),
                   [distribution = std::normal_distribution<float>(0.0, p_motion_dispersion_y_),
                    generator = std::default_random_engine(seed)]
                   (float y, float delta) mutable
                       {return y + delta + distribution(generator);});
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::transform(particles_angle_.begin(), particles_angle_.end(),
                   particles_angle_.begin(),
                   [distribution = std::normal_distribution<float>(0.0, p_motion_dispersion_theta_),
                    generator = std::default_random_engine(seed), odelta]
                   (float angle) mutable
                       {return angle + odelta[2] + distribution(generator);});
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
            p_max_particles_, (int)downsampled_angles_.size(), p_inv_squash_factor_);
    }
    else
        ROS_ERROR("range method %s not recognized!", p_which_rm_.c_str());
}

void MCL::resampling()
{
    /* temp local vectors used by resampling */
    fvec_t particles_x;
    fvec_t particles_y;
    fvec_t particles_angle;
    select_particles(particles_x, particles_y, particles_angle, p_max_particles_);

    std::copy(particles_x.begin(), particles_x.end(), particles_x_.begin());
    std::copy(particles_y.begin(), particles_y.end(), particles_y_.begin());
}

void MCL::print_particles(int n)
{
    if (n > particles_x_.size())
        n = particles_x_.size();
    for (int i = 0; i < n; i ++)
        ROS_DEBUG("%3d:  %f  %f  %f\t(%e)", i, particles_x_[i], particles_y_[i], particles_angle_[i], weights_[i]);
}

void MCL::calc_range_one_pose(pose_t ins, fvec_t &ranges, bool printonly)
{
    if (!printonly)
        rm_.numpy_calc_range_angles (ins.data(), downsampled_angles_.data(),
                                     ranges.data(), 1, downsampled_angles_.size());
    for (int i=0; i<downsampled_ranges_.size(); i++){
        if (i != 0 && i %6 == 0)
            printf("\n");
        printf("[%5.2f <- %5.2f]  ", ranges[i], ins[2] + downsampled_angles_[i]);
    }
    printf("\n");
}

/* set fake angles and ranges according particle pidx */
void MCL::set_fake_angles_and_ranges(int pidx)
{
    float amin = -3.1415927410125732;
    float amax = 3.1415927410125732;
    float inc = 0.005823155865073204;
    int num_angles = (amax - amin) / inc + 1;
    num_angles /= p_angle_step_;
    if (num_angles != NUM_ANGLES){
        ROS_ERROR("NUM_ANGLES needs to be set to %d in CMakeLists.txt", num_angles);
        ros::shutdown();
    }
    ROS_DEBUG("num_angles: %d", num_angles);
    downsampled_angles_.resize(num_angles);
    downsampled_ranges_.resize(num_angles);
    for (int a = 0; a < num_angles; a++)
        downsampled_angles_[a] = amin + a * inc * p_angle_step_;
    if (!p_which_impl_.compare("gpu"))
        mclgpu_->set_angles(downsampled_angles_.data());

    /* generate fake downsampled ranges */
    pose_t ins;
    ins[0] = particles_x_[pidx];
    ins[1] = particles_y_[pidx];
    ins[2] = particles_angle_[pidx];
    rm_.numpy_calc_range_angles (ins.data(), downsampled_angles_.data(),
                                 downsampled_ranges_.data(), 1, downsampled_angles_.size());
}
