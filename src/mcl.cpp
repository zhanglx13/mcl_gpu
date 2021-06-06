#include "mcl.h"

MCL::MCL()
{
    ros::NodeHandle private_nh_("~");
    private_nh_.param("which_rm", which_rm, std::string("rmgpu"));

    std::cout<< "range method: " << which_rm << std::endl;
}

MCL::~MCL()
{
    std::cout<< "All done, bye you!!"<< std::endl;
}
