#include <OdomToNavSatFixNode.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agribot_odom_to_navsatfix_node");
    ros::NodeHandle nh("~");
    OdomToNavSatFixNode _odomToNavSatFixNode(&nh);
    _odomToNavSatFixNode.run();
    return 0;
}
