#include <ros/ros.h>
#include <tum_ics_skin_descr/Patch/TfMarkerDataPatches.h>
#include <QApplication>
#include <QFileInfo>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace tum_ics_skin_descr;
using namespace Eigen;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "skin_control",
              ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate r(100);

    QApplication a(argc, argv);
    QString configFilePath = QString(argv[1]);
    QFileInfo fi(configFilePath);
    if(!fi.absoluteDir().exists())
    {
        qCritical("Invalid path '%s'",configFilePath.toLatin1().data());
        return -1;
    }

    Patch::TfMarkerDataPatches tfPatches;

    if(!tfPatches.loadFromParam(configFilePath,"~patch_list"))
    {
        return -1;
    }
    tfPatches.createDataConnections();
    tfPatches.enableDataConnctions();

    while(ros::ok())
    {
        /*Vector3d acc;
        acc << tfPatches.patch(0)->dataFromId(2).acc.at(0),
               tfPatches.patch(0)->dataFromId(2).acc.at(1),
               tfPatches.patch(0)->dataFromId(2).acc.at(2); // get grav vals

        ROS_WARN_STREAM(acc);
        */
        // std::cout << tfPatches.patch(0)->dataFromId(9).cellId<< "\n";
//         void chatterCallback(const tum_ics_skin_msgs::SkinCellDataArray msg)
// {
// //   ROS_INFO("I heard: [%i]", msg.data(0).cellId);

//     std::cout << msg.data(0).cellId<< "\n";
// }
        std::cout << tfPatches.patch(0)->dataFromId(10).prox.at(0)<< "\n";
        ros::spinOnce();
        r.sleep();
    }

    qDebug("exit");

    return 0;

}