#include "mcl.h"
#define _USE_MATH_DEFINES // for pi ==> M_PI
#include <numeric> // for transform_reduce

void set_affinity(std::thread &t, int cpuid)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpuid, &cpuset);
    int rc = pthread_setaffinity_np(t.native_handle(), sizeof(cpu_set_t), &cpuset);
    if (rc != 0)
        std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
}

MCL::MCL(ranges::OMap omap, float max_range_px):
    omap_(omap),
    rmgpu_ (omap, max_range_px),
    rm_ (omap, max_range_px),
    timer_ (Utils::Timer(10)),
    maxW_ (Utils::CircularArray<double>(10)),
    diffW_ (Utils::CircularArray<double>(10)),
    dis_ (Utils::CircularArray<double>(10))
{
    ros::NodeHandle private_nh_("~");
    private_nh_.getParam("angle_step", p_angle_step_);
    private_nh_.getParam("max_particles", p_max_particles_);
    private_nh_.getParam("N_gpu", p_N_gpu_);
    private_nh_.getParam("max_viz_particles", p_max_viz_particles_);
    private_nh_.getParam("cpu_threads", p_cpu_threads_);
    double tmp;
    private_nh_.getParam("squash_factor", tmp);
    p_inv_squash_factor_ = 1.0 / tmp;
    p_max_range_meters_ = max_range_px * omap.world_scale;
    private_nh_.getParam("theta_discretization", p_theta_discretization_);
    private_nh_.param("which_rm", p_which_rm_, std::string("rmgpu"));
    private_nh_.param("which_impl", p_which_impl_, std::string("cpu"));
    private_nh_.param("which_viz", p_which_viz_, std::string("largest"));
    private_nh_.param("which_expect", p_which_expect_, std::string("largest"));
    private_nh_.param("which_res", p_which_res_, std::string("gpu"));
    private_nh_.param("publish_odom", p_publish_odom_, 1);
    private_nh_.getParam("viz", p_do_viz_);
    private_nh_.getParam("init_var", p_init_var_);

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
#ifdef PRINT_INFO
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
#endif

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

    px_.resize(p_max_particles_);
    py_.resize(p_max_particles_);
    pangle_.resize(p_max_particles_);

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
    initialize_initpose(1);

    visualize();

    /*
     * Initialize the MCLGPU object
     */
    if (!p_which_impl_.compare("gpu") || !p_which_impl_.compare("hybrid"))
    {
        mclgpu_ = new MCLGPU(p_max_particles_);
        mclgpu_->init_constants(
            /* motion model dispersion constants */
            p_motion_dispersion_x_, p_motion_dispersion_y_, p_motion_dispersion_theta_,
            /* squash factor used for weight computation */
            p_inv_squash_factor_);
        mclgpu_->set_sensor_table(sensor_model_table_, p_max_range_px_ + 1);
        mclgpu_->set_map(omap_, max_range_px);
    }

    /*
     * Initialize the GPU implementation of resampling
     */
    if (!p_which_res_.compare("gpu")){
        resgpu_ = new ResamplingGPU(p_max_particles_, 3);
    }

    /*
     * allocate space and initialize downsampled_ranges_ and downsampled_angles_
     * If NUM_ANGLES is not the correct number, ros will shutdown when the first
     * lidar message comes.
     */
    downsampled_ranges_.resize(NUM_ANGLES);
    downsampled_angles_.resize(NUM_ANGLES);

    /*
     * Initialize the multi-threading environment
     */
    initialize_mt();


    ROS_INFO("Finished initializing, waiting on messages ...");
}

/*
 * Create cpu worker threads
 *
 * 1. bound cpu_update() to each thread
 * 2. initialize start_map and n_map to 0
 * 3. Set CPU affinity of each worker
 *      We assume that the main thread is set to run on CPU0. Then each worker
 *      thread occupies another physical core. Since num_threads is no larger
 *      than PCORES, no two worker threads are running on the same core.
 *      Note that we set the affinity of the worker threads before they start doing
 *      meaningful work. They should be sleeping right now, waiting on the cv_.
 */
void MCL::initialize_workers(int num_threads)
{
    for (int i = 0; i < num_threads; i ++){
        cpu_workers_.emplace_back(&MCL::cpu_update, this);
        worker_start_map_[cpu_workers_[i].native_handle()] = 0;
        worker_n_map_[cpu_workers_[i].native_handle()] = 0;
        set_affinity(cpu_workers_[i], i+1);
        //ROS_DEBUG_STREAM("Thread "<<cpu_workers_[i].native_handle() << " running on CPU "<<i+1);
    }
}

void MCL::initialize_mt()
{
    /*
     * set the policy and priority of the main thread
     * so that the created threads will inherit the scheduling policy and priority
     */
    sched_param sch;
    ROS_DEBUG("max priority of FIFO: %d",  sched_get_priority_max(SCHED_FIFO));
    //sch.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    sch.sched_priority = 50;
    if ( pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) )
        std::cout << "setschedparam failed: " << std::strerror(errno) << '\n';
    /* cpu topology of the TITAN machine
    ┌────────────────────────────────────────────────────────────────────────┐
    │ Machine (7872MB)                                                       │
    │                                                                        │
    │ ┌────────────────────────────────────────────────────────────────────┐ │
    │ │ Package P#0                                                        │ │
    │ │                                                                    │ │
    │ │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │ │
    │ │ │ Core P#0    │  │ Core P#1    │  │ Core P#2    │  │ Core P#3    │ │ │
    │ │ │             │  │             │  │             │  │             │ │ │
    │ │ │ ┌─────────┐ │  │ ┌─────────┐ │  │ ┌─────────┐ │  │ ┌─────────┐ │ │ │
    │ │ │ │ PU P#0  │ │  │ │ PU P#1  │ │  │ │ PU P#2  │ │  │ │ PU P#3  │ │ │ │
    │ │ │ └─────────┘ │  │ └─────────┘ │  │ └─────────┘ │  │ └─────────┘ │ │ │
    │ │ │ ┌─────────┐ │  │ ┌─────────┐ │  │ ┌─────────┐ │  │ ┌─────────┐ │ │ │
    │ │ │ │ PU P#8  │ │  │ │ PU P#9  │ │  │ │ PU P#10 │ │  │ │ PU P#11 │ │ │ │
    │ │ │ └─────────┘ │  │ └─────────┘ │  │ └─────────┘ │  │ └─────────┘ │ │ │
    │ │ └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │ │
    │ │                                                                    │ │
    │ │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │ │
    │ │ │ Core P#4    │  │ Core P#5    │  │ Core P#6    │  │ Core P#7    │ │ │
    │ │ │             │  │             │  │             │  │             │ │ │
    │ │ │ ┌─────────┐ │  │ ┌─────────┐ │  │ ┌─────────┐ │  │ ┌─────────┐ │ │ │
    │ │ │ │ PU P#4  │ │  │ │ PU P#5  │ │  │ │ PU P#6  │ │  │ │ PU P#7  │ │ │ │
    │ │ │ └─────────┘ │  │ └─────────┘ │  │ └─────────┘ │  │ └─────────┘ │ │ │
    │ │ │ ┌─────────┐ │  │ ┌─────────┐ │  │ ┌─────────┐ │  │ ┌─────────┐ │ │ │
    │ │ │ │ PU P#12 │ │  │ │ PU P#13 │ │  │ │ PU P#14 │ │  │ │ PU P#15 │ │ │ │
    │ │ │ └─────────┘ │  │ └─────────┘ │  │ └─────────┘ │  │ └─────────┘ │ │ │
    │ │ └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │ │
    │ └────────────────────────────────────────────────────────────────────┘ │
    └────────────────────────────────────────────────────────────────────────┘
    There are 8 physical cores, each of which has two logical CPUs.
    */

    if (!p_which_impl_.compare("gpu") || (!p_which_impl_.compare("cpu") && p_cpu_threads_ == 1) ){
        /*
         * For GPU implementation and CPU implementation with single thread
         * no need to initialize cpu_workers_
         */
    }
    else if (!p_which_impl_.compare("cpu")){
        /*
         * For CPU implementation with more than 2 threads, create p_cpu_threads_ - 1 threads
         * ==> together with the main thread, there are totally p_cpu_threads_
         * threads working on the particles
         *
         * It is asserted that p_cpu_threads_ <= PCORES
         */
        initialize_workers(p_cpu_threads_ - 1);
    } else {
        /*
         * For hybrid implementation, create PCORES - 2 threads
         * ==> together with the main thread, there are toally PCORES-1
         * threads working on the particles with the CPU implementation.
         * The last core is used to hold another thread, which is used to launch
         * the GPU implementation.
         */
        initialize_workers(PCORES - 2);
    }
}


MCL::~MCL()
{
    free(particles_);
    free(sensor_model_table_);
    /* join cpu worker threads if there are any */
    for (int i = 0; i < cpu_workers_.size(); i++){
        auto t = cpu_workers_[i].native_handle();
        assert(cpu_workers_[i].joinable());
        std::cout<<"Joining thread " << t<<"\n";
        cpu_workers_[i].join();
    }
    std::cout<< "All done, bye yo!!" << "\n";
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
    //ROS_DEBUG("Process omap and count is %d, which is %f", count, count*1.0 / (map_width_*map_height_));
    //ROS_DEBUG("Number of free cells in the map: %ld", free_cell_id_.size());
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
    acc_focus_time_ms_ = 0.0f;
    iter_ = 0;
    focus_iter_ = 0;

    acc_res_ = 0.0f;
    acc_update_ = 0.0f;
    acc_total_ = 0.0f;
    acc_expect_ = 0.0f;
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
        // downsampled_ranges_.resize(num_ranges_downsampled);
        // downsampled_angles_.resize(num_ranges_downsampled);
        for (int i = 0; i < num_ranges_downsampled; i ++)
            downsampled_angles_[i] = amin + i * p_angle_step_ * inc;
        if (!p_which_impl_.compare("gpu") || !p_which_impl_.compare("hybrid"))
            mclgpu_->set_angles(downsampled_angles_.data());
    }
    {
        /* down sample the range */
        std::lock_guard<std::mutex> rangeLock(range_mtx_);
        for (int i = 0; i < num_ranges; i += p_angle_step_)
            downsampled_ranges_[i/p_angle_step_] = msg.ranges[i];
        /* limit the max range to be 10.0 */
        std::replace_if(downsampled_ranges_.begin(), downsampled_ranges_.end(),
                        [](float range){return range > 10.0;}, 10.0);
    }
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
    {
        std::lock_guard<std::mutex> odomLock(odom_mtx_);
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
    }
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
    initialize_initpose(1);
    expected_pose();
    visualize();
}

/* Initialize the particles around the init_pose_ */
void MCL::initialize_initpose(int startover)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::generate_n(particles_x_.begin(), p_max_particles_,
                    [x=init_pose_[0],
                     distribution = std::normal_distribution<float>(0.0, p_init_var_),
                     generator = std::default_random_engine(seed)] () mutable
                        {return x + distribution(generator);});
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::generate_n(particles_y_.begin(), p_max_particles_,
                    [y=init_pose_[1],
                     distribution = std::normal_distribution<float>(0.0, p_init_var_),
                     generator = std::default_random_engine(seed)] () mutable
                        {return y + distribution(generator);});
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::generate_n(particles_angle_.begin(), p_max_particles_,
                    [angle=init_pose_[2],
                     //distribution = std::normal_distribution<float>(0.0, 4),
                     distribution = std::uniform_real_distribution<float>(0, 1),
                     generator = std::default_random_engine(seed)] () mutable
                    //{return angle + distribution(generator);});
                        {return 2.0 * M_PI *  distribution(generator);});
    double w = 1.0 / p_max_particles_;
    std::fill(weights_.begin(), weights_.end(), w);

    if (startover)
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
        {
            std::lock_guard<std::mutex> odomLock(odom_mtx_);
            old_pose = last_pose_;
        }
        timer_.tick();
        auto t0 = ros::Time::now();
        //double maxW;
        std::tuple<float, float, float> timing_results;
        /*
         * one step of MCL algorithm:
         *   1. resampling
         *   2. apply motion model
         *   3. apply sensor model
         *   4. normalize particle weights
         */
        if (!p_which_impl_.compare("cpu"))
            timing_results  = MCL_cpu();
        else if (!p_which_impl_.compare("gpu"))
            timing_results = MCL_gpu();
        else if (!p_which_impl_.compare("adaptive"))
            MCL_adaptive();
        else if (!p_which_impl_.compare("hybrid"))
            timing_results = MCL_hybrid();
        else
            ROS_ERROR("The chosen method is not implemented yet");

        auto t1 = ros::Time::now();
        /* update inferred_pose_ */
        expected_pose();

        auto t2 = ros::Time::now();

        /* publish transformation frame based on inferred pose */
        //publish_tf();

        /* seconds per iteration */
        // ros::Duration spi = t2 - t1;
        auto MCL_t = (t2 - t0).toSec()*1000.0;
        auto expect_t = (t2 - t1).toSec()*1000.0;

        /* accumulate duration */
        acc_time_ms_ += MCL_t;
        acc_expect_ += expect_t;
        /* record max weight of this iteration */
        //maxW_.append(maxW);
        /*
         * obtain the current pose.
         * A mutex is used since the ros::spin() thread might be
         * calling odomCB to update last_pose_
         */
        pose_t new_pose;
        {
            std::lock_guard<std::mutex> odomLock(odom_mtx_);
            new_pose = last_pose_;
        }
        /*
         * calculate the weight of the inferred_pose_ according to
         * the new_pose using the sensor model.
         * Actually, the ranges generated by the two poses are compared.
         */
        double diffW = calc_diff(new_pose);
        /*
         * calculate the euler distance between the inferred_pose_ and
         * the new_pose. This metric is used to evaluate the quality of
         * the estimation.
         * If the distance is larger than 1.0, the particles are not
         * following the robot and it makes no sense to do resampling.
         *
         * TODO: explain why to choose 1.0 as the threshold.
         */
        float dis    = calc_dis(new_pose);
        diffW_.append(diffW);
        dis_.append(dis);
        iter_ ++;

        if (dis > 1.0)
            do_res_ = 0;
        else {
            do_res_ = 1;
            focus_iter_ ++;
            acc_focus_time_ms_ += MCL_t;
            acc_res_  += std::get<0>(timing_results);
        }
        float ave_res;
        if (focus_iter_)
            ave_res = acc_res_ / focus_iter_;
        else
            ave_res = 0.0f;

        acc_update_ += std::get<1>(timing_results);
        acc_total_  += std::get<2>(timing_results);
        /* print info per 10 iterations */
        if (iter_ != 0 && iter_ %10 == 0){
            // printf("iter %4d time %7.4f ms interval %7.4f ms  maxW: %e  diffW: %e  dis: %7.4f\n",
            //        iter_, elapsedTime, timer_.fps(),
            //        maxW_.mean(), diffW_.mean(), dis_.mean() );
            printf("iter %4d res: %7.4f update: %7.4f total: %7.4f "
                   "|| expect: %7.4f MCL: %7.4f interval %7.4f  diffW: %e  dis: %7.4f\n",
                   iter_, ave_res, acc_update_ / iter_, acc_total_ / iter_,
                   acc_expect_ / iter_, acc_time_ms_ / iter_ , timer_.fps(), diffW_.mean(), dis_.mean() );
        }
    }
    visualize();
}

double MCL::calc_diff(pose_t reference)
{
    return rm_.calc_diff_pose(reference.data(), inferred_pose_.data(),
                              downsampled_angles_.data(), downsampled_angles_.size(),
                              p_inv_squash_factor_);
}

float MCL::calc_dis(pose_t reference)
{
    float delta_x = reference[0] - inferred_pose_[0];
    float delta_y = reference[1] - inferred_pose_[1];
    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
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
    std::transform(indices.begin(), indices.end(), px_.begin(),
                   [this](int index) {return particles_x_[index];});
    std::transform(indices.begin(), indices.end(), py_.begin(),
                   [this](int index) {return particles_y_[index];});
    std::transform(indices.begin(), indices.end(), pangle_.begin(),
                   [this](int index) {return particles_angle_[index];});
}

std::tuple<float, float, float> MCL::MCL_cpu()
{
    /*************************************************
     * Step 1: Resampling using discrete_distribution
     *************************************************/
    auto t0 = ros::Time::now();
    if (do_res_){
        if (!p_which_res_.compare("cpu"))
            resampling();
        else if (!p_which_res_.compare("gpu"))
            resampling_gpu();
        else
            ROS_ERROR("Unrecognized resampling implementation %s", p_which_res_.c_str());
    }

    auto t1 = ros::Time::now();

    /***************************************************
     * Step 2-3: apply the motion model and sensor model
     ***************************************************/
    t_cpu_update(0, p_max_particles_, p_cpu_threads_);

    auto t2 = ros::Time::now();

    /*********************************************
     * Step 4: normalize the weights_  (but why?)
     *********************************************/
    // double maxW = normalize_weight();

    auto res_t = (t1 - t0).toSec()*1000.0;
    auto update_t = (t2 - t1).toSec()*1000.0;
    auto total_t = (t2 - t0).toSec()*1000.0;
    return std::make_tuple(res_t, update_t, total_t);
}

std::tuple<float, float, float> MCL::MCL_gpu()
{
    /***************************************************
     * step 1: Resampling
     ***************************************************/
    auto t0 = ros::Time::now();
    if (do_res_){
        if (!p_which_res_.compare("cpu"))
            resampling();
        else if (!p_which_res_.compare("gpu"))
            resampling_gpu();
        else
            ROS_ERROR("Unrecognized resampling implementation %s", p_which_res_.c_str());
    }

    auto t1 = ros::Time::now();
    /********************************************************************************
     * step 2: apply motion model to advance the particles and compute their weights
     * All done on GPU
     ********************************************************************************/
    gpu_update(p_max_particles_);

    auto t2 = ros::Time::now();
    /*********************************
     * Step 3: normalize the weights
     *********************************/
    // double maxW = normalize_weight();

    auto res_t = (t1 - t0).toSec()*1000.0;
    auto update_t = (t2 - t1).toSec()*1000.0;
    auto total_t = (t2 - t0).toSec()*1000.0;

    return std::make_tuple(res_t, update_t, total_t);
}

/*
 * Perform motion model and sensor model on GPU
 * This function updates and evaluates the first N_gpu particles in the population
 */
void MCL::gpu_update(int N_gpu)
{
    /*
     * Note the difference between mclgpu->update() called in MCL_gpu() is the number of
     * particles used. Here we only launch N_gpu particles on GPU
     */
    mclgpu_->update(
        /* inputs */
        /* particles */
        particles_x_.data(), particles_y_.data(), particles_angle_.data(),
        /* action, i.e. odometry_delta_ */
        odometry_delta_.data(), downsampled_ranges_.data(), (int)downsampled_ranges_.size(),
        /* output */
        weights_.data(),
        /* number of particles */
        N_gpu);
}

/*
 * Perform the motion model and sensor model on CPU
 * This is the function bound to each worker thread
 *
 * @note
 * How are the worker threads sync with each other as well as the main thread?
 *
 *    The worker thread waits for the main thread to prepare data and wakes up
 *    when the following condition satisfy:
 *
 *    1) ready_ == 1 && iter_old != iter_
 *    Or
 *    2) ros::ok() return false
 *
 *    ready_ == 1 means that the main thread has set up the start and num_particles
 *    parameters for each worker so that the worker should be able to start to work
 *    However, it is possible that worker A has finished the current loop and comes
 *    back again at the cv_mait(). Now we do not want worker A to do the work again
 *    (although it is not wrong to let it do it again, we do want to save some CPU
 *    time), so we use iter_old to keep track of the global iteration information,
 *    which is saved in iter_. The worker should not do the work for the same iter_
 *    value.
 *
 *    We also do not want the worker the wait if ros is not ok, i.e. ros::shutdown()
 *    is called. This happens when roslaunch sends SIGINT to mcl_gpu, and the default
 *    signal handler installed by ROS simply calls ros::shutdown().
 *    When ROS is shutdown, we want the worker thread to terminate. And the dtor
 *    of MCL does not need to cancel the worker threads before joining them.
 *
 *    item_ is used to figure out who is the last thread to finish. The last thread
 *    is responsible to notify the main thread and set ready_ back to 0. This is
 *    safe since all other worker threads should finished their work and they do
 *    expect ready_ to be 0. This is also necessary since we do not want the worker
 *    threads to start the work on the next iter_ value before the main thread set
 *    start, n, and item_. Without setting ready_ back to 0, the worker threads
 *    will start as soon as iter_ is updated (note that threads can have spurious
 *    wake up and we need to make sure the condition is false even when no one has
 *    notified them).
 */
void MCL::cpu_update()
{
    int iter_old = -1;
    /* The worker thread never terminates unless ros::ok() == false */
    while(1){
        {
            // std::cout<< "@@@iter "<< iter_ << ": thread "<< pthread_self();
            // std::cout<< "waiting for cv\n";
            std::unique_lock<std::mutex> workerLk (cv_mtx_);
            cv_.wait(workerLk, [this, iter_old]{return (ready_ == 1 && iter_ != iter_old) || !ros::ok();});
            // if (!ros::ok()){
            //     std::cout<< "@@@iter "<< iter_ << ": thread "<< pthread_self();
            //     std::cout<< " ros not OK!!! I am going to terminate!\n";
            // }
            // std::cout<< "@@@iter "<< iter_ << ": thread "<< pthread_self();
            // std::cout<< " has started ==> ready_ = " << ready_ <<" item_ = " << item_;
            // std::cout<< "on cpu " << sched_getcpu() <<"\n";
            // fflush(stdout);
        }
        if (!ros::ok()){
            break;
        }
        /* get my start and num_particles from the map using my pthread id */
        pthread_t myTid = pthread_self();
        int start = worker_start_map_[myTid];
        int num_particles = worker_n_map_[myTid];
        motion_model(start, num_particles);
        sensor_model(start, num_particles);

        /*
         * After finished, decrement ready_ and check if I am the last thread
         * to finish. If so, notify the main thread
         */
        int isLast = 0;
        {
            std::lock_guard<std::mutex> workerLk (cv_mtx_);
            item_ --;
            // std::cout<< "@@@iter "<< iter_ << ": thread "<< pthread_self();
            // std::cout<< " has finished ready_ = " << ready_ << " item = "<< item_;
            if (item_ == 0){
                isLast = 1;
                ready_ = 0;
            }
            // std::cout<< "\n";
            // fflush(stdout);
        }
        iter_old = iter_;
        if (isLast)
            cv_.notify_all();
    }
}

/*
 * Perform cpu_update in different threads concurrently
 */
void MCL::t_cpu_update(int start, int num_particles, int num_threads)
{
    /* single thread case, simply call motion_model() and sensor_model() */
    if (num_threads == 1){
        motion_model(start, num_particles);
        sensor_model(start, num_particles);
        return;
    }

    /* Now num_threads is at least 2 */
    /* ppt: particles per thread */
    int ppt = num_particles / num_threads;
    /* plt: particles last thread */
    int plt = num_particles - ppt * (num_threads - 1);
    /*
     * Set start and num_particles for each worker thread
     * and signal them to start working
     */
    for (int i = 0 ; i < cpu_workers_.size(); i ++){
        worker_start_map_[cpu_workers_[i].native_handle()] = start + i*ppt;
        worker_n_map_[cpu_workers_[i].native_handle()] = ppt;
    }
    {
        std::lock_guard<std::mutex> mainLk (cv_mtx_);
        item_ = cpu_workers_.size();
        ready_ = 1;
        // printf("@@@iter %d: Main thread set ready_ = %d  item_ = %d\n", iter_, ready_, item_);
        // fflush(stdout);
    }
    cv_.notify_all();

    /* main thread deals with the last group */
    motion_model(start + num_particles - plt, plt);
    sensor_model(start + num_particles - plt, plt);

    {
        std::unique_lock<std::mutex> mainLk (cv_mtx_);
        cv_.wait(mainLk, [this] { return item_ == 0;});
        // std::cout << "@@@iter " << iter_ << ": main thread synced with all workers\n";
        // fflush(stdout);
    }
}

int MCL::particle_partition()
{
    return p_N_gpu_;
}

std::tuple<float, float, float> MCL::MCL_hybrid()
{
    /***************************************************
     * step 1: Resampling
     ***************************************************/
    auto t0 = ros::Time::now();
    if (do_res_){
        if (!p_which_res_.compare("cpu"))
            resampling();
        else if (!p_which_res_.compare("gpu"))
            resampling_gpu();
        else
            ROS_ERROR("Unrecognized resampling implementation %s", p_which_res_.c_str());
    }

    /********************************************************************************
     * step 2: apply motion model to advance the particles and compute their weights
     * Distribute particles among CPU and GPU
     ********************************************************************************/
    int N_gpu = particle_partition();
    int N_cpu = p_max_particles_ - N_gpu;

    auto t1 = ros::Time::now();
    /*
     * This thread will propagate and evaluate the first N_gpu particles on GPU
     * And we put this thread on the last physical core
     */
    std::thread t_test(&MCL::gpu_update, this, N_gpu);

    set_affinity(t_test, PCORES-1);

    auto t2 = ros::Time::now();
    /*
     * The main thread will propagate (motion_model()) and evaluate (sensor_model())
     * the particles on CPU
     */
    t_cpu_update(N_gpu, N_cpu, PCORES-1);
    auto t3 = ros::Time::now();

    t_test.join();

    auto t4 = ros::Time::now();

    /*********************************
     * Step 3: normalize the weights
     *********************************/
    //double maxW = normalize_weight();

    auto res_t = (t1 - t0).toSec()*1000.0;
    auto gpu_thread_t = (t2 - t1).toSec()*1000.0;
    auto cpu_t = (t3 - t2).toSec()*1000.0;
    auto total_cpu_gpu_t = (t4 - t1).toSec()*1000.0;
    auto wait_t = (t4 - t3).toSec()*1000.0;
    auto total_t = res_t + total_cpu_gpu_t;

    return std::make_tuple(res_t, total_cpu_gpu_t, total_t);
}

double MCL::normalize_weight()
{
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


void MCL::MCL_adaptive(){}

void MCL::motion_model(int start, int num_particles)
{
    pose_t odelta;
    {
        std::lock_guard<std::mutex> odomLock(odom_mtx_);
        odelta = odometry_delta_;
    }
    /* rotate the action into the coordinate space of each particle */
    fvec_t cosines(num_particles);
    fvec_t sines(num_particles);
    int end = start + num_particles;
    std::transform(particles_angle_.begin()+start, particles_angle_.begin()+end, cosines.begin(),
                   [](float theta) {return cos(theta);});
    std::transform(particles_angle_.begin()+start, particles_angle_.begin()+end, sines.begin(),
                   [](float theta) {return sin(theta);});

    fvec_t local_deltas_x(num_particles);
    fvec_t local_deltas_y(num_particles);
    std::transform(cosines.begin(), cosines.end(), sines.begin(), local_deltas_x.begin(),
                   [odelta](float c, float s) {return c*odelta[0]-s*odelta[1];});
    std::transform(cosines.begin(), cosines.end(), sines.begin(), local_deltas_y.begin(),
                   [odelta](float c, float s) {return s*odelta[0]+c*odelta[1];});
    /* Add the local delta to each particle */
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::transform(particles_x_.begin()+start, particles_x_.begin()+end,
                   local_deltas_x.begin(),
                   particles_x_.begin()+start,
                   PlusWithNoise<float>(seed, p_motion_dispersion_x_));
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::transform(particles_y_.begin()+start, particles_y_.begin()+end,
                   local_deltas_y.begin(),
                   particles_y_.begin()+start,
                   PlusWithNoise<float>(seed, p_motion_dispersion_y_));
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::transform(particles_angle_.begin()+start, particles_angle_.begin()+end,
                   particles_angle_.begin()+start,
                   std::bind(PlusWithNoise<float>(seed, p_motion_dispersion_theta_),
                             std::placeholders::_1, odelta[2]) );
}

/*
 * For each particle p:
 *   1. compute the ranges in all directions given by downsampled_angles_
 *   2. Evaluate one range using the sensor_model_table
 *   3. Multiply scores from all ranges to make the weight of particle p
 */
void MCL::sensor_model(int start, int num_particles)
{
    fvec_t ranges(downsampled_ranges_.size());
    {
        std::lock_guard<std::mutex> rangeLock(range_mtx_);
        std::copy(downsampled_ranges_.begin(), downsampled_ranges_.end(), ranges.begin());
    }

    if (!p_which_rm_.compare("rmgpu"))
    {
        /*
         * one thread computes one particle
         *
         * Note that we split the particles into 3 vectors so that cuda kernels
         * can have coalesced memory access patterns when accessing particles.
         */
        rmgpu_.calc_range_eval_sensor_model_particle(
            particles_x_.data()+start, particles_y_.data()+start, particles_angle_.data()+start,
            ranges.data(), downsampled_angles_.data(), weights_.data()+start,
            num_particles, (int)downsampled_ranges_.size());
#if 0
        /* one thread compute one range */
        rmgpu_.calc_range_eval_sensor_model_angle(
            particles_x_, particles_y_, particles_angle_,
            ranges, downsampled_angles_, weights_,
            p_max_particles_, (int)downsampled_ranges_.size());
#endif
    }
    else if (!p_which_rm_.compare("rm"))
    {
        rm_.calc_range_eval_sensor_model(
            particles_x_.data()+start, particles_y_.data()+start, particles_angle_.data()+start,
            ranges.data(), downsampled_angles_.data(),
            weights_.data()+start,
            num_particles, (int)downsampled_angles_.size(), p_inv_squash_factor_);
    }
    else
        ROS_ERROR("range method %s not recognized!", p_which_rm_.c_str());
}

void MCL::resampling()
{
    /* temp local vectors used by resampling */
    select_particles(px_, py_, pangle_, p_max_particles_);

    particles_x_ = px_;
    particles_y_ = py_;
    particles_angle_ = pangle_;
}

void MCL::resampling_gpu()
{
    resgpu_->doSystematicRes(particles_x_.data(),particles_y_.data(),
                             particles_angle_.data(), weights_.data());
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
