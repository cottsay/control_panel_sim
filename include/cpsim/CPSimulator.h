#ifndef CPSIMULATOR_H
#define CPSIMULATOR_H

#include <QtOpenGL/QGLWidget>
#include <GL/glu.h>
#include <QContextMenuEvent>
#include <QThread>
#include <QMutex>
#include <QTimer>
#include <QTime>
#include <QDialog>
#include <QGridLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>
#include <QLabel>
#include <QImage>

#include <nodelet/nodelet.h>
#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/signals2/mutex.hpp>

namespace Ui {
class CPSimulatorPlugin;
}

namespace control_panel
{
class CPSimulatorPlugin;

class CPSimulatorNodelet : public nodelet::Nodelet
{
public:
    CPSimulatorNodelet(CPSimulatorPlugin *_parent);
    void activate(bool passive = false);
    void deactivate();
    bool isActive();
    void onInit();
    void setupLidar(double full_angle, double angle_step, double distance_min, double distance_max);
    void pub_bump_left(bool bump);
    void pub_bump_right(bool bump);
    void pub_sim_pose(double x, double y, double theta);
    void pub_map(const nav_msgs::OccupancyGrid &og);
    void pub_wall(const unsigned short int dist);
    void pub_laser(const std::vector<float> &scan);

private:
    ros::Subscriber joint_trajectory_sub;
    ros::Publisher sim_pose_pub;
    ros::Publisher bump_left_pub;
    ros::Publisher bump_right_pub;
    ros::Publisher wall_pub;
    ros::Publisher map_pub;
    ros::Publisher laser_pub;
    CPSimulatorPlugin *parent;
    double lidar_full_angle;
    double lidar_angle_step;
    double lidar_distance_min;
    double lidar_distance_max;
};

class CPSimulatorGL : public QGLWidget
{
    Q_OBJECT

public:
    CPSimulatorGL(QWidget *_parent = NULL);
    ~CPSimulatorGL();

public slots:
    void updatePose(double x, double y, double theta);
    void setMap(const QImage &img);
    void setMapScale(const double scale);
    void reloadTexture();

protected:
    // OpenGL Methods
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void paintEnvironment();
    void paintRobot();
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void updateZoom();

    // Queries
    float getZoom();

private:
    // Widget Properties
    QWidget *parent;

    // Internal Properties
    double zoom_level;
    int w;
    int h;
    int drag_origin_w;
    int drag_origin_h;
    double window_center_w;
    double window_center_h;
    double drag_window_center_start_w;
    double drag_window_center_start_h;

    // Robot Pose
    double x;
    double y;
    double theta;

    // Map
    QImage map;
    GLuint map_id;
    double map_scale;
};

// Robot State
class CPRobotSim : public QObject
{
    Q_OBJECT

public:
    CPRobotSim(QObject *_parent = 0);

    double x();
    double y();
    double theta();
    double x_d();
    double y_d();
    double theta_d();
    double x_dd();
    double y_dd();
    double theta_dd();
    double maxAccel();

public slots:
    void spinOnce();
    void updateTarget(double l_d_t, double r_d_t);
    void breakSim();
    void updateMaxAccel(double maxAccel);
    void setOG(const nav_msgs::OccupancyGrid &og);
    void setPose(double x, double y, double theta);
    void setupLidar(double full_angle, double angle_step, double max);

signals:
    void updatePose(double x, double y, double theta);
    void updateBump(bool bump, double theta);
    void updateWall(const unsigned int dist);
    void updateTwist(double x, double theta);
    void updateLaser(const std::vector<float> scan);

private:
    bool pixelOccupied(int x, int y) const;
    float pixelStep(int x0, int y0, int x1, int y1) const;

    // robot
    double _x;
    double _y;
    double _theta;
    double _x_d;
    double _y_d;
    double _theta_d;
    double _x_dd;
    double _y_dd;
    double _theta_dd;

    // wheels
    double _l;
    double _l_d;
    double _l_d_t;
    double _l_dd;
    double _r;
    double _r_d;
    double _r_d_t;
    double _r_dd;

    double _maxAccel;
    QObject parent;
    QMutex mutex;
    QTime last_time;

    // map
    nav_msgs::OccupancyGrid og;

    // wall
    static const unsigned short int wallLookup[120];

    // lidar
    double lidar_full_angle;
    double lidar_angle_step;
    double lidar_max;
};

class CPSimulatorPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPSimulatorPlugin();
    ~CPSimulatorPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();
    double torque2angaccel(double torque) const;
    double angaccel2torque(double angaccel) const;

public slots:
    void configDialog();
    void resetSimulation();
    void setPaused(bool paused);
    void publishBump(bool bump, double theta);
    void publishSimPose(double x, double y, double theta);
    void publishWall(const unsigned int dist);
    void publishLaser(const std::vector<float> scan);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    
private:
    void JointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);
    bool loadMap(QString mapfilename);

    Ui::CPSimulatorPlugin *ui;
    CPSimulatorNodelet *nodelet_priv;

    // Robot State
    CPRobotSim robotSim;
    QTimer robotSimTimer;
    QThread robotSimThread;

    // Map
    bool lastMapLoadSucceeded;
    nav_msgs::OccupancyGrid og;

signals:
    void mapUpdated(const QImage &img);

friend class CPSimulatorNodelet;
};

class CPSimConfig : public QDialog
{
    Q_OBJECT

public:
    CPSimConfig(double maxTorque, QString mapfile = QString(), double mapScale = 1.0, double x = 0.0, double y = 0.0, double theta = 0.0, double lidar_sweep = 1.01229097, double lidar_step = 1.01229097 / 480, double lidar_min = 0.8, double lidar_max = 3.8, double sim_rate = 66.6);

public slots:
    void fileSelected(QString newfilename);

protected:
    void keyPressEvent(QKeyEvent *event);

private:
    QGridLayout layout;
    QLabel maxtorquetxt;
    QLabel mapfiletxt;
    QPushButton mapfilebrowse;
    QFileDialog mapfiledialog;
    QLabel mapscaletxt;
    QLabel mapscaleunit;
    QLabel startposetxt;
    QLabel startorienttxt;
    QLabel lidarsweeptxt;
    QLabel lidarsweepunit;
    QLabel lidarsteptxt;
    QLabel lidarstepunit;
    QLabel lidarmintxt;
    QLabel lidarminunit;
    QLabel lidarmaxtxt;
    QLabel lidarmaxunit;
    QLabel simratetxt;
    QPushButton okbutton;

public:
    QLineEdit maxtorqueedit;
    QLineEdit mapfilename;
    QLineEdit mapscaleedit;
    QLineEdit startx;
    QLineEdit starty;
    QLineEdit starttheta;
    QLineEdit lidarsweep;
    QLineEdit lidarstep;
    QLineEdit lidarmin;
    QLineEdit lidarmax;
    QLineEdit simrate;
};
}

#endif // CPSIMULATOR_H
