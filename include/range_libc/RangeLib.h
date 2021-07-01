/*
* Copyright 2017 Corey H. Walsh (corey.walsh11@gmail.com)

* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

*     http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

/*
Useful Links: https://github.com/MRPT/mrpt/blob/4137046479222f3a71b5c00aee1d5fa8c04017f2/libs/slam/include/mrpt/slam/PF_implementations.h
  - collision avoidance http://users.isy.liu.se/en/rt/fredrik/reports/01SPpf4pos.pdf
  - σMCL http://www.roboticsproceedings.org/rss01/p49.pdf
  - nifty heuristic to figure out when localization is lost
    https://www.deutsche-digitale-bibliothek.de/binary/6WAGERZFR4H4PREZXILJRER6N7XDVX3H/full/1.pdf
  - http://www.cs.cmu.edu/~16831-f14/notes/F11/16831_lecture04_tianyul.pdf
  - https://april.eecs.umich.edu/pdfs/olson2009icra.pdf
*/

#ifndef RANGE_LIB_H
#define RANGE_LIB_H

#include "vendor/lodepng/lodepng.h"
#include "vendor/distance_transform.h"
//#include "range_libc/RangeUtils.h"

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
/*
 * The portion of the Bullet LinearMath library which was used by tf has been copied
 * into the tf namespace and renamed to avoid collisions.
 * Check this: https://wiki.ros.org/geometry/bullet_migration
 */
#include "tf/LinearMath/Matrix3x3.h"

#include <stdio.h>      /* printf */
#include <cstdlib>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>    // std::min
#include <time.h>
#include <chrono>
#include <set>
#include <iomanip>      // std::setw
#include <unistd.h>
#include <stdexcept>
#include <sstream>
// #define NDEBUG
#include <cassert>
#include <tuple>
#include <random>

#ifndef _MAKE_TRACE_MAP 
	#define _MAKE_TRACE_MAP 0
#endif

#define _TRACK_LUT_SIZE 0
#define _TRACK_COLLISION_INDEXES 0

#define _EPSILON 0.00001
#define M_2PI 6.28318530718
#define _BINARY_SEARCH_THRESHOLD 64 // if there are more than this number of elements in the lut bin, use binary search

// fast optimized version
#define _USE_CACHED_TRIG 0
#define _USE_ALTERNATE_MOD 1
#define _USE_CACHED_CONSTANTS 1
#define _USE_FAST_ROUND 0
#define _NO_INLINE 0
#define _USE_LRU_CACHE 0
#define _LRU_CACHE_SIZE 1000000

// not implemented yet -> use 16 bit integers to store zero points
#define _CDDT_SHORT_DATATYPE 1
#define _GIANT_LUT_SHORT_DATATYPE 1

// these flags determine whether to compile helper functions specially designed for 6.141 lab 5
#define ROS_WORLD_TO_GRID_CONVERSION 1
#define SENSOR_MODEL_HELPERS 1

// slow unoptimized version
// #define _USE_ALTERNATE_MOD 0
// #define _USE_CACHED_CONSTANTS 1
// #define _USE_FAST_ROUND 1
// #define _DO_MOD 0 // this might not be necessary (aka 1 & 0 might be equivalent), will evaluate later
// #define _NO_INLINE 0
#if _USE_LRU_CACHE
#include "includes/lru_cache.h"
#endif

// No inline
#if _NO_INLINE == 1
#define ANIL __attribute__ ((noinline))
#else
#define ANIL
#endif

// these defines are for yaml/JSON serialization
#define T1 "  "
#define T2 T1 T1
#define T3 T1 T1 T1
#define T4 T2 T2

#define J1 "  "
#define J2 J1 J1
#define J3 J1 J1 J1
#define J4 J2 J2

#if USE_CUDA == 1
  #ifndef CHUNK_SIZE
  #define CHUNK_SIZE 262144
  #define CHUNK_THREADS 256
  #endif
  #include "range_libc/CudaRangeLib.h"
#else
  #define USE_CUDA 0
#endif

typedef std::array<float, 3> pose_t;
typedef std::vector<float> fvec_t;

namespace ranges {
    struct OMap
    {
        bool has_error;
        unsigned width;  // x axis
        unsigned height; // y axis
        std::vector<std::vector<bool> > grid;
        std::vector<std::vector<float> > raw_grid;
        std::string fn; // filename
#if _MAKE_TRACE_MAP == 1
        std::vector<std::vector<bool> > trace_grid;
#endif

        // this stuff is for ROS integration, not necessary for raw usage
#if ROS_WORLD_TO_GRID_CONVERSION == 1
        float world_scale;
        float world_angle;
        float world_origin_x;
        float world_origin_y;
        float world_sin_angle;
        float world_cos_angle;
#endif

        OMap(int w, int h) : width(w), height(h), fn(""), has_error(false) {
            for (int i = 0; i < w; ++i) {
                std::vector<bool> y_axis;
                for (int q = 0; q < h; ++q) y_axis.push_back(false);
                grid.push_back(y_axis);
            }
#if _MAKE_TRACE_MAP == 1
            for (int i = 0; i < w; ++i) {
                std::vector<bool> y_axis;
                for (int q = 0; q < h; ++q) y_axis.push_back(false);
                trace_grid.push_back(y_axis);
            }
#endif
        }

        OMap(std::string filename) : OMap(filename, 128) {}
        OMap(std::string filename, float threshold) : fn(filename), has_error(false) {
            unsigned error;
            unsigned char* image;

            error = lodepng_decode32_file(&image, &width, &height, filename.c_str());
            if(error) {
                printf("ERROR %u: %s\n", error, lodepng_error_text(error));
                has_error = true;
                return;
            }

            for (int i = 0; i < width; ++i) {
                std::vector<bool> y_axis;
                for (int q = 0; q < height; ++q) y_axis.push_back(false);
                grid.push_back(y_axis);
            }

            for (int i = 0; i < width; ++i) {
                std::vector<float> y_axis;
                for (int q = 0; q < height; ++q) y_axis.push_back(0);
                raw_grid.push_back(y_axis);
            }

#if _MAKE_TRACE_MAP == 1
            for (int i = 0; i < width; ++i) {
                std::vector<bool> y_axis;
                for (int q = 0; q < height; ++q) y_axis.push_back(false);
                trace_grid.push_back(y_axis);
            }
#endif

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    unsigned idx = 4 * y * width + 4 * x;
                    int r = image[idx + 2];
                    int g = image[idx + 1];
                    int b = image[idx + 0];
                    int gray = (int) rgb2gray(r,g,b);
                    if (gray < threshold) grid[x][y] = true;
                    raw_grid[x][y] = gray;
                }
            }
        }

        OMap (const nav_msgs::OccupancyGrid &map) {
            width = map.info.width;
            height = map.info.height;

            for (int i = 0; i < width; ++i) {
                std::vector<bool> y_axis;
                for (int q = 0; q < height; ++q) y_axis.push_back(false);
                grid.push_back(y_axis);
            }

            for (int i = 0; i < width; ++i) {
                std::vector<float> y_axis;
                for (int q = 0; q < height; ++q) y_axis.push_back(0);
                raw_grid.push_back(y_axis);
            }

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int val = map.data[y*width + x];
                    /*
                     * 0 - 100 is the probability of being occupied
                     * -1: unknown
                     */
                    if (val > 10 || val == -1) grid[x][y] = true;
                    raw_grid[x][y] = val;
                }
            }

            world_scale = map.info.resolution;
            world_origin_x = map.info.origin.position.x;
            world_origin_y = map.info.origin.position.y;

            //world_angle = -1.0f*quaternion_to_angle(map.info.origin.orientation);
            /*
              @fix
             * This is different from the implementation given in RangeLibc.pyx line 160,
             * in which the angle is negated.
             * I haven't figured out why the python wrapper author did so.
             * It works in most maps since the info.origin.orientation is 0 in most maps
             */
            world_angle = quaternion_to_angle(map.info.origin.orientation);
            world_sin_angle = sin(world_angle);
            world_cos_angle = cos(world_angle);

            printf("OMap parameters:\n");
            printf("  width (x axis):  %d\n", width);
            printf("  height (y axis): %d\n", height);
            printf("  world_scale:     %f\n", world_scale);
            printf("  world_origin_x:  %f\n", world_origin_x);
            printf("  world_origin_y:  %f\n", world_origin_y);
            printf("  world_angle:     %f\n", world_angle);
            printf("  world_sin_angle: %f\n", world_sin_angle);
            printf("  world_cos_angle: %f\n", world_cos_angle);

        }

        float quaternion_to_angle(geometry_msgs::Quaternion orientation)
        {
            /*
              @note
             * Convert quaternion message to Euler angle
             * Check: https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106
             */
            tf::Quaternion quat;
            tf::quaternionMsgToTF(orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
            return yaw;
        }

        pose_t grid_to_world(pose_t p_in_grid)
        {
            pose_t p_in_world;

            float ca = world_cos_angle;
            float sa = world_sin_angle;
            float x = p_in_grid[0];
            float y = p_in_grid[1];
            p_in_world[0] = (ca*x - sa*y) * world_scale + world_origin_x;
            p_in_world[1] = (sa*x + ca*y) * world_scale + world_origin_y;
            p_in_world[2] = p_in_grid[2] + world_angle;
            return p_in_world;
        }

        /*
         * This is one of the helper functions defined in RangeUtils.h.
         * It is moved here as one of the member functions in OMap.
         * There will be an error of "multiple definitions of function xxx" since
         * multiple cpp files included RangeUtils.h
         */
        float rgb2gray(float r, float g, float b) {
            return 0.229 * r + 0.587 * g + 0.114 * b;
        }
        bool get(int x, int y) { return grid[x][y]; }
        bool isOccupied(int x, int y) {
            if (x < 0 || x >= width || y < 0 || y >= height) return false;
#if _MAKE_TRACE_MAP == 1
            trace_grid[x][y] = true;
#endif
            return grid[x][y];
        }

        // query the grid without a trace
        bool isOccupiedNT(int x, int y) { return grid[x][y]; }

#if _MAKE_TRACE_MAP == 1
        bool saveTrace(std::string filename) {
            std::vector<unsigned char> png;
            lodepng::State state; //optionally customize this one
            // char image = new char[width * height * 4] = 0;
            char image[width * height * 4];

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    unsigned idx = 4 * y * width + 4 * x;

                    // if (trace_grid[x][y]) {
                    // 	image[idx + 0] = 255;
                    // 	image[idx + 1] = 255;
                    // 	image[idx + 2] = 255;
                    // }

                    image[idx + 2] = 255;
                    image[idx + 1] = 255;
                    image[idx + 0] = 255;

                    if (trace_grid[x][y]) {
                        image[idx + 0] = 0;
                        image[idx + 1] = 0;
                        image[idx + 2] = 200;
                    }

                    if (grid[x][y]) {
                        image[idx + 0] = 255;
                        image[idx + 1] = 0;
                        image[idx + 2] = 0;
                    }

                    if (grid[x][y] && trace_grid[x][y]) {
                        image[idx + 0] = 0;
                        image[idx + 1] = 0;
                        image[idx + 2] = 0;
                    }
                    image[idx + 3] = 255;
                }
            }
            unsigned error = lodepng::encode(png, reinterpret_cast<const unsigned char*> (image), width, height, state);
            if(!error) lodepng::save_file(png, filename);
            //if there's an error, display it
            if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
            return error;
        }
#endif

        bool save(std::string filename) {
            std::vector<unsigned char> png;
            lodepng::State state; //optionally customize this one
            // char image = new char[width * height * 4] = 0;
            char image[width * height * 4];

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    unsigned idx = 4 * y * width + 4 * x;

                    image[idx + 2] = (char)255;
                    image[idx + 1] = (char)255;
                    image[idx + 0] = (char)255;
                    image[idx + 3] = (char)255;

                    if (grid[x][y]) {
                        image[idx + 0] = 0;
                        image[idx + 1] = 0;
                        image[idx + 2] = 0;
                    }
                }
            }
            unsigned error = lodepng::encode(png, reinterpret_cast<const unsigned char*> (image), width, height, state);
            if(!error) lodepng::save_file(png, filename);
            //if there's an error, display it
            if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
            return error;
        }

        /*
         * This is one of the helper functions defined in RangeUtils.h.
         * It is moved here as one of the member functions in OMap.
         * There will be an error of "multiple definitions of function xxx" since
         * multiple cpp files included RangeUtils.h
         */
        std::vector<std::pair<int,int> > outline_fn(int x, int y, bool use_corners) {
            std::vector<std::pair<int,int> > corners;

            corners.push_back(std::make_pair(x+1,y));
            corners.push_back(std::make_pair(x-1,y));
            corners.push_back(std::make_pair(x,y+1));
            corners.push_back(std::make_pair(x,y-1));

            if (use_corners) {
                corners.push_back(std::make_pair(x+1,y+1));
                corners.push_back(std::make_pair(x-1,y+1));
                corners.push_back(std::make_pair(x+1,y-1));
                corners.push_back(std::make_pair(x-1,y-1));
            }

            return corners;
        }

        OMap make_edge_map(bool count_corners) {
            OMap edge_map = OMap(width, height);
            for (int x = 0; x < width; ++x) {
                for (int y = 0; y < height; ++y) {
                    if (!isOccupiedNT(x,y)) continue;

                    std::vector<std::pair<int,int>> outline = outline_fn(x,y,count_corners);
                    for (int i = 0; i < outline.size(); ++i) {
                        int cx;
                        int cy;
                        std::tie(cx, cy) = outline[i];
                        if (0 <= cx && 0 <= cy && cx < width && cy < height && !isOccupiedNT(cx,cy)) {
                            edge_map.grid[x][y] = true;
                            break;
                        }
                    }
                }
            }
            return edge_map;
        }

        bool error() {
            return has_error;
        }

        // returns memory usage in bytes
        int memory() {
            return sizeof(bool) * width * height;
        }
    };

    struct DistanceTransform
    {
        unsigned width;
        unsigned height;
        std::vector<std::vector<float> > grid;
        // float *grid;

        float get(int x, int y) { return grid[x][y]; }

        DistanceTransform() : width(0), height(0) {}

        DistanceTransform(int w, int h) : width(w), height(h) {
            // allocate space in the vectors
            for (int i = 0; i < width; ++i) {
                std::vector<float> y_axis;
                for (int q = 0; q < height; ++q) y_axis.push_back(1.0);
                grid.push_back(y_axis);
            }
        }

        // computes the distance transform of a given OMap
        DistanceTransform(OMap *map) {
            width = map->width;
            height = map->height;

            std::vector<std::size_t> grid_size({width, height});
            dt::MMArray<float, 2> f(grid_size.data());
            dt::MMArray<std::size_t, 2> indices(grid_size.data());

            for (std::size_t i = 0; i < width; ++i)
                for (std::size_t j = 0; j < height; ++j)
                    if (map->isOccupied(i,j)) f[i][j] = 0.0f;
                    else f[i][j] = std::numeric_limits<float>::max();

            dt::DistanceTransform::distanceTransformL2(f, f, indices, false);

            // allocate space in the vectors
            for (int i = 0; i < width; ++i) {
                std::vector<float> y_axis;
                for (int q = 0; q < height; ++q) y_axis.push_back(1.0);
                grid.push_back(y_axis);
            }

            // store to array
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    grid[x][y] = f[x][y];
                }
            }
        }

        bool save(std::string filename) {
            std::vector<unsigned char> png;
            lodepng::State state;
            char image[width * height * 4];

            float scale = 0;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    scale = std::max(grid[x][y], scale);
                }
            }
            scale *= 1.0 / 255.0;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    unsigned idx = 4 * y * width + 4 * x;
                    // std::cout << (int)(grid[x][y] / scale) << " " << grid[x][y] / scale << std::endl;
                    // image[idx + 2] = std::min(255, (int)grid[x][y]);
                    // image[idx + 1] = std::min(255, (int)grid[x][y]);
                    // image[idx + 0] = std::min(255, (int)grid[x][y]);
                    image[idx + 2] = (int)(grid[x][y] / scale);
                    image[idx + 1] = (int)(grid[x][y] / scale);
                    image[idx + 0] = (int)(grid[x][y] / scale);
                    image[idx + 3] = (char)255;
                }
            }
            unsigned error = lodepng::encode(png, reinterpret_cast<const unsigned char*> (image), width, height, state);
            if(!error) lodepng::save_file(png, filename);
            //if there's an error, display it
            if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
            return error;
        }

        int memory() {
            return width*height*sizeof(float);
        }
    };

    class RangeMethod
    {
    public:
        RangeMethod(OMap m, float mr) : map(m), max_range(mr) {};
        virtual ~RangeMethod() {};

        virtual float calc_range(float x, float y, float heading) = 0;
        virtual std::pair<float,float> calc_range_pair(float x, float y, float heading) { return std::make_pair(-1,-1); }
        virtual OMap *getMap() {return &map; }
        virtual void report() {};
        float maxRange() { return max_range; }
        float memory() { return -1; }

        void saveTrace(std::string fn) {
#if _MAKE_TRACE_MAP == 1
            map.saveTrace(fn);
#else
            std::cout << "WARNING: trace map not generated, must compile with trace support enabled." << std::endl;
#endif
        }

        // wrapper function to call calc_range repeatedly with the given array of inputs
        // and store the result to the given outputs. Useful for avoiding cython function
        // call overhead by passing it a numpy array pointer. Indexing assumes a 3xn numpy array
        // for the inputs and a 1xn numpy array of the outputs
        void numpy_calc_range(float * ins, float * outs, int num_casts) {
            std::cout << "Do not call numpy_calc_range";
            std::cout << " until the algorithm is verified" << std::endl;
            return ;
#if ROS_WORLD_TO_GRID_CONVERSION == 1
            // cache these constants on the stack for efficiency
            float inv_world_scale = 1.0 / map.world_scale;
            float world_scale = map.world_scale;
            float world_angle = map.world_angle;
            float world_origin_x = map.world_origin_x;
            float world_origin_y = map.world_origin_y;
            float world_sin_angle = map.world_sin_angle;
            float world_cos_angle = map.world_cos_angle;

            float rotation_const = -1.0 * world_angle - 3.0*M_PI / 2.0;

            // avoid allocation on every loop iteration
            float x_world;
            float y_world;
            float theta_world;
            float x;
            float y;
            float temp;
            float theta;
#endif

            for (int i = 0; i < num_casts; ++i) {
#if ROS_WORLD_TO_GRID_CONVERSION == 1
                x_world = ins[i*3];
                y_world = ins[i*3+1];
                theta_world = ins[i*3+2];

                x = (x_world - world_origin_x) * inv_world_scale;
                y = (y_world - world_origin_y) * inv_world_scale;
                temp = x;
                x = world_cos_angle*x - world_sin_angle*y;
                y = world_sin_angle*temp + world_cos_angle*y;
                theta = -theta_world + rotation_const;

                outs[i] = calc_range(y, x, theta) * world_scale;
#else
                outs[i] = calc_range(ins[i*3], ins[i*3+1], ins[i*3+2]);
#endif
            }
        }

        void numpy_calc_range_angles(float * ins, float * angles, float * outs, int num_particles, int num_angles) {
#if ROS_WORLD_TO_GRID_CONVERSION == 1
            // cache these constants on the stack for efficiency
            float inv_world_scale = 1.0 / map.world_scale;
            float world_scale = map.world_scale;
            float world_angle = map.world_angle;
            float world_origin_x = map.world_origin_x;
            float world_origin_y = map.world_origin_y;
            float world_sin_angle = map.world_sin_angle;
            float world_cos_angle = map.world_cos_angle;
            // float rotation_const = -1.0 * world_angle - 3.0*M_PI / 2.0;

            // avoid allocation on every loop iteration
            float x_world;
            float y_world;
            float theta_world;
            float x;
            float y;
            float temp;
            float theta;
#endif

            for (int i = 0; i < num_particles; ++i)
            {
                x_world = ins[i*3];
                y_world = ins[i*3+1];
                theta_world = ins[i*3+2];
                /*
                  @fix
                 * conversion of theta from world to grid
                 */
                //theta = -theta_world + rotation_const;
                theta = theta_world - world_angle;

                x = (x_world - world_origin_x) * inv_world_scale;
                y = (y_world - world_origin_y) * inv_world_scale;
                temp = x;
                /*
                  @fix
                 * Rotation of -world_angle
                 */
                // x = world_cos_angle*x - world_sin_angle*y;
                // y = world_sin_angle*temp + world_cos_angle*y;
                x = world_cos_angle*x + world_sin_angle*y;
                y = - world_sin_angle*temp + world_cos_angle*y;

                for (int a = 0; a < num_angles; ++a){
                    /*
                      @fix
                     * No need to switch x and y
                     */
                    // outs[i*num_angles+a] = calc_range(y, x, theta - angles[a]) * world_scale;
                    outs[i*num_angles+a] = calc_range(x, y, theta + angles[a]) * world_scale;
                }
            }
        }

#if SENSOR_MODEL_HELPERS == 1
        void set_sensor_model(double *table, int table_width) {
            // convert the sensor model from a numpy array to a vector array
            for (int i = 0; i < table_width; ++i)
            {
                std::vector<double> table_row;
                for (int j = 0; j < table_width; ++j)
                    table_row.push_back(table[table_width*i + j]);
                sensor_model.push_back(table_row);
            }
        }
        void eval_sensor_model(float * obs, float * ranges, double * outs, int rays_per_particle, int particles) {
            float inv_world_scale = 1.0 / map.world_scale;
            // do no allocations in the main loop
            double weight;
            float r;
            float d;
            int i;
            int j;

            for (i = 0; i < particles; ++i)
            {
                weight = 1.0;
                for (j = 0; j < rays_per_particle; ++j)
                {
                    r = obs[j] * inv_world_scale;
                    r = std::min<float>(std::max<float>(r,0.0),(float)sensor_model.size()-1.0);
                    d = ranges[i*rays_per_particle+j] * inv_world_scale;
                    d = std::min<float>(std::max<float>(d,0.0),(float)sensor_model.size()-1.0);
                    weight *= sensor_model[(int)r][(int)d];
                }
                outs[i] = weight;
            }
        }

        // calc range for each pose, adding every angle, evaluating the sensor model
        void calc_range_repeat_angles_eval_sensor_model(float * ins, float * angles, float * obs, double * weights, int num_particles, int num_angles) {
            std::cout << "Do not call calc_range_repeat_angles_eval_sensor_model";
            std::cout << " until the algorithm is verified" << std::endl;
            return ;
#if ROS_WORLD_TO_GRID_CONVERSION == 1
            // cache these constants on the stack for efficiency
            float inv_world_scale = 1.0 / map.world_scale;
            float world_scale = map.world_scale;
            float world_angle = map.world_angle;
            float world_origin_x = map.world_origin_x;
            float world_origin_y = map.world_origin_y;
            float world_sin_angle = map.world_sin_angle;
            float world_cos_angle = map.world_cos_angle;
            float rotation_const = -1.0 * world_angle - 3.0*M_PI / 2.0;

            // avoid allocation on every loop iteration
            float x_world;
            float y_world;
            float theta_world;
            float x;
            float y;
            float temp;
            float theta;

            // do no allocations in the main loop
            double weight;
            float r;
            float d;
            int i;
            int a;

            for (i = 0; i < num_particles; ++i)
            {
                x_world = ins[i*3];
                y_world = ins[i*3+1];
                theta_world = ins[i*3+2];
                theta = -theta_world + rotation_const;

                x = (x_world - world_origin_x) * inv_world_scale;
                y = (y_world - world_origin_y) * inv_world_scale;
                temp = x;
                x = world_cos_angle*x - world_sin_angle*y;
                y = world_sin_angle*temp + world_cos_angle*y;

                weight = 1.0;
                for (a = 0; a < num_angles; ++a)
                {
                    d = calc_range(y, x, theta - angles[a]);
                    d = std::min<float>(std::max<float>(d,0.0),(float)sensor_model.size()-1.0);

                    r = obs[a] * inv_world_scale;
                    r = std::min<float>(std::max<float>(r,0.0),(float)sensor_model.size()-1.0);
                    weight *= sensor_model[(int)r][(int)d];
                }
                weights[i] = weight;
            }
#endif
        }

        // this is to compute a lidar sensor model using radial (calc_range_pair) optimizations
        // this is only exact for a certain set of downsample amounts
        void calc_range_many_radial_optimized(float * ins, float * outs, int num_particles, int num_rays, float min_angle, float max_angle) {
#if ROS_WORLD_TO_GRID_CONVERSION == 1
            // cache these constants on the stack for efficiency
            float inv_world_scale = 1.0 / map.world_scale;
            float world_scale = map.world_scale;
            float world_angle = map.world_angle;
            float world_origin_x = map.world_origin_x;
            float world_origin_y = map.world_origin_y;
            float world_sin_angle = map.world_sin_angle;
            float world_cos_angle = map.world_cos_angle;
            float rotation_const = -1.0 * world_angle - 3.0*M_PI / 2.0;

            // avoid allocation on every loop iteration
            float x_world, y_world, theta_world, x, y, temp, theta = 0.0;

            // do no allocations in the main loop
            int i, a = 0;


            float step = (max_angle - min_angle) / (num_rays - 1);
            float angle = min_angle;


            int max_pairwise_index = (float)num_rays / 3.0;
            float index_offset_float = (num_rays - 1.0) * M_PI / (max_angle - min_angle);

            // TODO: check if this index_offset_float is not very close to an integer
            // in which case throw a warning that this downsample factor is poorly compatible
            // with this radial optimization
            int index_offset = roundf(index_offset_float);
            float r, r_inv = 0.0;

            for (i = 0; i < num_particles; ++i)
            {
                x_world = ins[i*3];
                y_world = ins[i*3+1];
                theta_world = ins[i*3+2];
                theta = -theta_world + rotation_const;

                x = (x_world - world_origin_x) * inv_world_scale;
                y = (y_world - world_origin_y) * inv_world_scale;
                temp = x;
                x = world_cos_angle*x - world_sin_angle*y;
                y = world_sin_angle*temp + world_cos_angle*y;

                angle = min_angle;
                for (a = 0; a <= max_pairwise_index; ++a) {
                    std::tie(r, r_inv) = calc_range_pair(y,x,theta - angle);
                    outs[i*num_rays+a] = r * world_scale;
                    outs[i*num_rays+a+index_offset] = r_inv * world_scale;
                    angle += step;
                }

                for (a = max_pairwise_index + 1; a < index_offset; ++a) {
                    outs[i*num_rays+a] = calc_range(y,x,theta - angle) * world_scale;
                    angle += step;
                }
            }
#endif
            return;
        }
#endif

    protected:
        OMap map;
        float max_range;

#if SENSOR_MODEL_HELPERS == 1
        std::vector<std::vector<double> > sensor_model;
#endif
    };


    class RayMarchingGPU : public RangeMethod
    {
    public:
        RayMarchingGPU(OMap m, float mr) : RangeMethod(m, mr) {
            distImage = new DistanceTransform(&m);
#if USE_CUDA == 1
            rmc = new RayMarchingCUDA(distImage->grid, distImage->width, distImage->height, max_range);

#if ROS_WORLD_TO_GRID_CONVERSION == 1
            rmc->set_conversion_params(m.world_scale,m.world_angle,m.world_origin_x, m.world_origin_y,
                                       m.world_sin_angle, m.world_cos_angle);
#endif

#else
            throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
#endif
        }
        ~RayMarchingGPU() {
            delete distImage;
#if USE_CUDA == 1
            delete rmc;
#else
            throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
#endif
        };

        float calc_range(float x, float y, float heading) {
#if USE_CUDA == 1
            std::cout << "Do not call calc_range on RayMarchingGPU, requires batched queries" << std::endl;
            return -1.0;
#else
            throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
#endif
        }

        void maybe_warn(int num_casts) {
#if USE_CUDA == 1
            if (!(num_casts % CHUNK_SIZE == 0) && !already_warned) {
                std::cout << "\nFor better performance, call calc_range_many with some multiple of " << CHUNK_SIZE << " queries. ";
                std::cout << "You can change the chunk size with -DCHUNK_SIZE=[integer].\n" << std::endl;
                already_warned = true;
            }
#endif
        }

        void calc_range_many(float *ins, float *outs, int num_casts) {
#if USE_CUDA == 1
            maybe_warn(num_casts);
            int iters = std::ceil((float)num_casts / (float)CHUNK_SIZE);
            for (int i = 0; i < iters; ++i) {
                int num_in_chunk = CHUNK_SIZE;
                if (i == iters - 1) num_in_chunk = num_casts-i*CHUNK_SIZE;
                rmc->calc_range_many(&ins[i*CHUNK_SIZE*3],&outs[i*CHUNK_SIZE],num_in_chunk);
            }
#else
            throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
#endif
        }

        // wrapper function to call calc_range repeatedly with the given array of inputs
        // and store the result to the given outputs. Useful for avoiding cython function
        // call overhead by passing it a numpy array pointer. Indexing assumes a 3xn numpy array
        // for the inputs and a 1xn numpy array of the outputs
        void numpy_calc_range(float * ins, float * outs, int num_casts) {
#if USE_CUDA == 1
#if ROS_WORLD_TO_GRID_CONVERSION == 0
            std::cout << "Cannot use GPU numpy_calc_range without ROS_WORLD_TO_GRID_CONVERSION == 1" << std::endl;
            return;
#endif
            maybe_warn(num_casts);
            int iters = std::ceil((float)num_casts / (float)CHUNK_SIZE);
            for (int i = 0; i < iters; ++i) {
                int num_in_chunk = CHUNK_SIZE;
                if (i == iters - 1) num_in_chunk = num_casts-i*CHUNK_SIZE;
                rmc->numpy_calc_range(&ins[i*CHUNK_SIZE*3],&outs[i*CHUNK_SIZE],num_in_chunk);
            }
#else
            throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
#endif
        }

        void numpy_calc_range_angles(float * ins, float * angles, float * outs, int num_particles, int num_angles) {
#if USE_CUDA == 1
#if ROS_WORLD_TO_GRID_CONVERSION == 0
            std::cout << "Cannot use GPU numpy_calc_range without ROS_WORLD_TO_GRID_CONVERSION == 1" << std::endl;
            return;
#endif

            int particles_per_iter = std::ceil((float)CHUNK_SIZE / (float)num_angles);
            int iters = std::ceil((float)num_particles / (float) particles_per_iter);
            // must allways do the correct number of angles, can only split on the particles
            for (int i = 0; i < iters; ++i) {
                int num_in_chunk = particles_per_iter;
                if (i == iters - 1) num_in_chunk = num_particles-i*particles_per_iter;
                rmc->numpy_calc_range_angles(&ins[i*num_in_chunk*3], angles, &outs[i*num_in_chunk*num_angles],
                                             num_in_chunk, num_angles);
            }
#else
            throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
#endif
        }

#if SENSOR_MODEL_HELPERS == 1
#if USE_CUDA == 1
        void set_sensor_model(double *table, int table_width) {
            // convert the sensor model from a numpy array to a vector array
            for (int i = 0; i < table_width; ++i)
            {
                std::vector<double> table_row;
                for (int j = 0; j < table_width; ++j)
                    table_row.push_back(table[table_width*i + j]);
                sensor_model.push_back(table_row);
            }
            rmc->set_sensor_table(table, table_width);
        }
#endif

        // calc range for each pose, adding every angle, evaluating the sensor model
        void calc_range_repeat_angles_eval_sensor_model(float * ins, float * angles, float * obs, double * weights, int num_particles, int num_angles) {
#if USE_CUDA == 1
#if ROS_WORLD_TO_GRID_CONVERSION == 0
            std::cout << "Cannot use GPU numpy_calc_range without ROS_WORLD_TO_GRID_CONVERSION == 1" << std::endl;
            return;
#endif

            int particles_per_iter = std::ceil((float)CHUNK_SIZE / (float)num_angles);
            int iters = std::ceil((float)num_particles / (float) particles_per_iter);
            // must allways do the correct number of angles, can only split on the particles
            for (int i = 0; i < iters; ++i) {
                int num_in_chunk = particles_per_iter;
                if (i == iters - 1) num_in_chunk = num_particles-i*particles_per_iter;
                rmc->calc_range_repeat_angles_eval_sensor_model(&ins[i*num_in_chunk*3], angles, obs, &weights[i*num_in_chunk],num_in_chunk, num_angles);
            }
#else
            throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
#endif
        }

        /*
         * calc range for each pose, adding every angle, evaluating the sensor model
         *
         * This method uses one thread to do computation for one particle
         */
        void calc_range_eval_sensor_model_particle(
            fvec_t px, fvec_t py, fvec_t pangle, // particles
            fvec_t obs, // observation, i.e. downsampled_ranges_
            fvec_t angles, // downsampled_angles_
            std::vector<double> weights, // output, i.e. weight of each particle
            int num_particles,
            int num_angles // number of downsampled angles
            ){
#if USE_CUDA == 1
#if ROS_WORLD_TO_GRID_CONVERSION == 0
            std::cout << "Cannot use GPU numpy_calc_range without ROS_WORLD_TO_GRID_CONVERSION == 1" << std::endl;
            return;
#endif
            rmc->calc_range_eval_sensor_model_particle(
                px.data(), py.data(), pangle.data(),
                obs.data(), angles.data(), weights.data(),
                num_particles, num_angles
                );
#else
            throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
#endif
        }
#endif

        int memory() { return distImage->memory(); }
    protected:
        DistanceTransform *distImage = 0;
#if USE_CUDA == 1
        RayMarchingCUDA * rmc = 0;
#endif
        bool already_warned = false;
    };

    class BresenhamsLine : public RangeMethod
    {
    public:
        BresenhamsLine(OMap m, float mr) : RangeMethod(m, mr) {};

        float ANIL calc_range(float x, float y, float heading) {
            // first check if the cell underneath the query point is occupied, if so return
            if (map.isOccupied((int)x,(int)y)) {
                return 0.0;
            }

            /*
              this defines the coordinate system such that
              ------> +x
              |
              |
              \/
              +y
              0* heading lies along the x axis, positive heading rotates towards the positive y axis
            */
            float x0 = y;
            float y0 = x;
            float x1 = y + max_range*sinf(heading);
            float y1 = x + max_range*cosf(heading);

            bool steep = false;
            if (std::abs(y1-y0) > std::abs(x1-x0)) steep = true;

            if (steep) {
                float tmp = x0;
                x0 = y0;
                y0 = tmp;
                tmp = x1;
                x1 = y1;
                y1 = tmp;
            }

            float deltax = std::abs(x1-x0);
            float deltay = std::abs(y1-y0);

            float error = 0;
            float deltaerr = deltay;
            float _x = x0;
            float _y = y0;

            int xstep = -1;
            if (x0 < x1) xstep = 1;

            int ystep = -1;
            if (y0 < y1) ystep = 1;

            unsigned width = map.width;
            unsigned height = map.height;

            while ((int)_x != (int)(x1 + xstep)) {
                _x += xstep;
                error += deltaerr;

                if (error * 2.00 >= deltax) {
                    _y += ystep;
                    error -= deltax;
                }

                if (!steep) {
                    if (0 <= _y && _y < width && 0 <= _x && _x < height && map.isOccupied(_y, _x)) {
                        float xd = _x - x0;
                        float yd = _y - y0;
                        return sqrtf(xd*xd + yd*yd);
                    }
                } else {
                    if (0 <= _x && _x < width && 0 <= _y && _y < height && map.isOccupied(_x, _y)) {
                        float xd = _x - x0;
                        float yd = _y - y0;
                        return sqrtf(xd*xd + yd*yd);
                    }
                }
            }
            return max_range;
        }

        int memory() { return map.memory(); }
    };


    class RayMarching : public RangeMethod
    {
    public:
        RayMarching(OMap m, float mr) : RangeMethod(m, mr) { distImage = DistanceTransform(&m); }

        float ANIL calc_range(float x, float y, float heading) {
            float x0 = x;
            float y0 = y;

            float ray_direction_x = cosf(heading);
            float ray_direction_y = sinf(heading);

            int px = 0;
            int py = 0;

            float t = 0.0;
            while (t < max_range) {
                px = x0 + ray_direction_x * t;
                py = y0 + ray_direction_y * t;

                if (px >= map.width || px < 0 || py < 0 || py >= map.height) {
                    return max_range;
                }

                float d = distImage.get(px, py);
#if _MAKE_TRACE_MAP == 1
                map.isOccupied(px,py); // this makes a dot appear in the trace map
#endif

                if (d <= distThreshold) {
                    float xd = px - x0;
                    float yd = py - y0;
                    return sqrtf(xd*xd + yd*yd);
                }

                t += std::max<float>(d * step_coeff, 1.0);
            }

            return max_range;
        }

        double calc_diff_pose(
            float *ref_pose,
            float *inferred_pose,
            float *angles,
            int num_angles,
            double inv_squash_factor)
        {
            double w;
            float r,d, temp;
            /* cache these constants on the stack for efficiency */
            float inv_world_scale = 1.0 / map.world_scale;
            //float world_scale = map.world_scale;
            float world_angle = map.world_angle;
            float world_origin_x = map.world_origin_x;
            float world_origin_y = map.world_origin_y;
            float world_sin_angle = map.world_sin_angle;
            float world_cos_angle = map.world_cos_angle;

            float x_world_ref = ref_pose[0];
            float y_world_ref = ref_pose[1];
            float theta_world_ref = ref_pose[2];
            float theta_ref = theta_world_ref - world_angle;
            float x_ref = (x_world_ref - world_origin_x) * inv_world_scale;
            float y_ref = (y_world_ref - world_origin_y) * inv_world_scale;
            temp = x_ref;
            x_ref = world_cos_angle*x_ref + world_sin_angle*y_ref;
            y_ref = -world_sin_angle*temp + world_cos_angle*y_ref;

            float x_world_inferred = inferred_pose[0];
            float y_world_inferred = inferred_pose[1];
            float theta_world_inferred = inferred_pose[2];
            float theta_inferred = theta_world_inferred - world_angle;
            float x_inferred = (x_world_inferred - world_origin_x) * inv_world_scale;
            float y_inferred = (y_world_inferred - world_origin_y) * inv_world_scale;
            temp = x_inferred;
            x_inferred = world_cos_angle*x_inferred + world_sin_angle*y_inferred;
            y_inferred = -world_sin_angle*temp + world_cos_angle*y_inferred;

            w = 1.0;
            for (int a = 0; a < num_angles; a++)
            {
                d = calc_range(x_inferred, y_inferred, theta_inferred + angles[a]);
                d = std::min<float>(std::max<float>(d,0.0),(float)sensor_model.size()-1.0);
                r = calc_range(x_ref, y_ref, theta_ref + angles[a]);
                r = std::min<float>(std::max<float>(r,0.0),(float)sensor_model.size()-1.0);
                w *= sensor_model[(int)r][(int)d];
            }
            return pow(w, inv_squash_factor);
        }

        void calc_range_eval_sensor_model(
            float *px, float *py, float *pangle,
            float *obs, float *angles,
            double *weights,
            int num_particles, int num_angles,
            double inv_squash_factor
            )
        {
            /* cache these constants on the stack for efficiency */
            float inv_world_scale = 1.0 / map.world_scale;
            float world_scale = map.world_scale;
            float world_angle = map.world_angle;
            float world_origin_x = map.world_origin_x;
            float world_origin_y = map.world_origin_y;
            float world_sin_angle = map.world_sin_angle;
            float world_cos_angle = map.world_cos_angle;
            //float rotation_const = -1.0 * world_angle - 3.0*M_PI / 2.0;

            /* avoid allocation on every loop iteration */
            float x_world;
            float y_world;
            float theta_world;
            float x;
            float y;
            float temp;
            float theta;
            double w;
            float r,d;

            for (int i = 0; i < num_particles; i ++)
            {
                x_world = px[i];
                y_world = py[i];
                theta_world = pangle[i];
                theta = theta_world - world_angle;

                x = (x_world - world_origin_x) * inv_world_scale;
                y = (y_world - world_origin_y) * inv_world_scale;
                temp = x;
                x = world_cos_angle*x + world_sin_angle*y;
                y = -world_sin_angle*temp + world_cos_angle*y;


                w = 1.0;
                for (int a = 0; a < num_angles; a ++)
                {
                    d = calc_range(x, y, theta + angles[a]);
                    d = std::min<float>(std::max<float>(d,0.0),(float)sensor_model.size()-1.0);
                    /* Need to convert range from world to map */
                    r = obs[a] * inv_world_scale;
                    r = std::min<float>(std::max<float>(r,0.0),(float)sensor_model.size()-1.0);
                    w *= sensor_model[(int)r][(int)d];
                }
                weights[i] = pow(w, inv_squash_factor);
            }
        }


        int memory() { return distImage.memory(); }
    protected:
        DistanceTransform distImage;
        float distThreshold = 0.0;
        float step_coeff = 0.999;
    };
}

namespace benchmark {
    template <class range_T>
    class Benchmark
    {
    public:
        Benchmark(range_T rm) : range(rm) {
            map = range.getMap();
        };
        ~Benchmark() {};

        void set_log(std::stringstream* ss) { log = ss; }

        int memory() { return range.memory(); }

        void grid_sample(int step_size, int num_rays, int samples) {
            float coeff = (2.0 * M_PI) / num_rays;
            double t_accum = 0;
            float num_cast = 0;

            volatile clock_t t;
            t = clock();

            if (log) (*log) << "x,y,theta,time" << std::endl;
            if (log) (*log) << std::fixed;
            if (log) (*log) << std::setprecision(9);

            for (int i = 0; i < num_rays; ++i)
            {
                float angle = i * coeff;
                for (int x = 0; x < map->width; x += step_size)
                {
                    for (int y = 0; y < map->height; y += step_size)
                    {
                        auto start_time = std::chrono::high_resolution_clock::now();
                        for (int j = 0; j < samples; ++j)
                        {
                            volatile float r = range.calc_range(x,y,angle);
                        }

                        auto end_time = std::chrono::high_resolution_clock::now();

                        num_cast += samples;
                        // std::cout << (end_time - start_time).count() << std::endl;

                        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);

                        t_accum += time_span.count();

                        if (log) (*log) << x << "," << y << "," << angle << "," << time_span.count() << std::endl;
                    }
                }
            }

            std::cout << "finished grid sample after: " << (((float) (clock() - t)) / CLOCKS_PER_SEC) << " sec" << std::endl;
            std::cout << " -avg time per ray: " << ( t_accum / (float) num_cast) << " sec" << std::endl;
            std::cout << " -rays cast: " << num_cast << std::endl;
            std::cout << " -total time: " << t_accum << " sec" << std::endl;
        }

        void grid_sample2(int step_size, int num_rays, int samples) {
            float coeff = (2.0 * M_PI) / num_rays;
            double t_accum = 0;

            if (log) (*log) << "x,y,theta,time" << std::endl;
            if (log) (*log) << std::fixed;
            if (log) (*log) << std::setprecision(9);

            int num_samples = num_grid_samples(step_size, num_rays, samples, map->width, map->height);
            float *samps = new float[num_samples*3];
            float *outs = new float[num_samples];
            get_grid_samples(samps, step_size, num_rays, samples, map->width, map->height);

            auto start_time = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < num_samples; ++i)
                outs[i] = range.calc_range(samps[i*3],samps[i*3+1],samps[i*3+2]);
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
            t_accum += time_span.count();

            // print first few outputs for sanity checking
            for (int i = 0; i < 10; ++i)
                std::cout << outs[i] << std::endl;

            std::cout << "finished grid sample after: " << (float) t_accum << " sec" << std::endl;
            std::cout << " -avg time per ray: " << ( t_accum / (float) num_samples) << " sec" << std::endl;
            std::cout << " -rays cast: " << num_samples << std::endl;
            std::cout << " -total time: " << t_accum << " sec" << std::endl;
        }

        static int num_grid_samples(int step_size, int num_rays, int samples, int map_width, int map_height) {
            int num_samples = 0;
            for (int i = 0; i < num_rays; ++i)
                for (int x = 0; x < map_width; x += step_size)
                    for (int y = 0; y < map_height; y += step_size)
                        for (int j = 0; j < samples; ++j)
                            num_samples++;
            return num_samples;
        }

        static void get_grid_samples(float *queries, int step_size, int num_rays, int samples, int map_width, int map_height) {
            float coeff = (2.0 * M_PI) / num_rays;
            double t_accum = 0;
            int ind = 0;
            for (int i = 0; i < num_rays; ++i) {
                float angle = i * coeff;
                for (int x = 0; x < map_width; x += step_size) {
                    for (int y = 0; y < map_height; y += step_size) {
                        for (int j = 0; j < samples; ++j) {
                            queries[ind*3] =  (float)x;
                            queries[ind*3+1] = (float)y;
                            queries[ind*3+2] = angle;
                            ind++;
                        }
                    }
                }
            }
        }

        void random_sample(int num_samples) {
            std::default_random_engine generator;
            generator.seed(clock());
            std::uniform_real_distribution<float> randx = std::uniform_real_distribution<float>(1.0,map->width - 1.0);
            std::uniform_real_distribution<float> randy = std::uniform_real_distribution<float>(1.0,map->height - 1.0);
            std::uniform_real_distribution<float> randt = std::uniform_real_distribution<float>(0.0,M_2PI);

            double t_accum = 0;
            for (int i = 0; i < num_samples; ++i)
            {
                float x = randx(generator);
                float y = randy(generator);
                float angle = randt(generator);

                auto start_time = std::chrono::high_resolution_clock::now();
                volatile float r = range.calc_range(x,y,angle);
                auto end_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);

                t_accum += time_span.count();
                if (log) (*log) << x << "," << y << "," << angle << "," << time_span.count() << std::endl;
            }

            std::cout << "finished random sample after: " << t_accum << " sec" << std::endl;
            std::cout << " -avg time per ray: " << ( t_accum / (float) num_samples) << " sec" << std::endl;
            std::cout << " -rays cast: " << num_samples << std::endl;
        }

        static void get_random_samples(float *queries, int num_samples, int map_width, int map_height) {
            std::default_random_engine generator;
            generator.seed(std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::system_clock::now().time_since_epoch()).count());
            std::uniform_real_distribution<float> randx = std::uniform_real_distribution<float>(1.0,map_width - 1.0);
            std::uniform_real_distribution<float> randy = std::uniform_real_distribution<float>(1.0,map_height - 1.0);
            std::uniform_real_distribution<float> randt = std::uniform_real_distribution<float>(0.0,M_2PI);

            for (int i = 0; i < num_samples; ++i) {
                queries[3*i]   = randx(generator);
                queries[3*i+1] = randy(generator);
                queries[3*i+2] = randt(generator);
            }
        }

        ranges::OMap *getMap() {return range.getMap(); }

    protected:
        range_T range;
        ranges::OMap *map;
        std::stringstream* log = NULL;
    };
}

#endif
