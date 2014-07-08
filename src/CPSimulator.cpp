#include "cpsim/CPSimulator.h"
#include "ui_CPSimulator.h"
#include <pluginlib/class_list_macros.h>
#include <QMenu>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QFileDialog>

#include <GL/glu.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS(control_panel::CPSimulatorPlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{
void CPSimConfig::fileSelected(QString newfilename)
{
    QFile newfile(newfilename);

    mapfilename.setText(newfile.fileName());
}

CPSimConfig::CPSimConfig(double maxTorque, QString mapfile, double mapScale, double x, double y, double theta, double lidar_sweep, double lidar_step, double lidar_min, double lidar_max, double sim_rate) :
    maxtorquetxt(tr("Max Wheel Torque:")),
    mapfiletxt(tr("Map File:")),
    mapfiledialog(0, "Map File", QString(), "Image (*.bmp *.gif *.jpg *.jpeg *.png *.tif)"),
    mapfilebrowse("&Browse"),
    mapscaletxt(tr("Scale:")),
    mapscaleunit(tr("pix/m")),
    startposetxt(tr("Start Position:")),
    startorienttxt(tr("Start Orientation:")),
    lidarsweeptxt(tr("Lidar Sweep:")),
    lidarsweepunit(tr("rad")),
    lidarsteptxt(tr("Lidar Step:")),
    lidarstepunit(tr("rad")),
    lidarmintxt(tr("Lidar Minimum:")),
    lidarminunit(tr("m")),
    lidarmaxtxt(tr("Lidar Maximum:")),
    lidarmaxunit(tr("m")),
    simratetxt(tr("Simulation Rate:")),
    okbutton(tr("&OK")),
    maxtorqueedit(QString::number(maxTorque)),
    mapfilename(mapfile),
    mapscaleedit(QString::number(mapScale)),
    startx(QString::number(x)),
    starty(QString::number(y)),
    starttheta(QString::number(theta)),
    lidarsweep(QString::number(lidar_sweep)),
    lidarstep(QString::number(lidar_step)),
    lidarmin(QString::number(lidar_min)),
    lidarmax(QString::number(lidar_max)),
    simrate(QString::number(sim_rate))
{
    setWindowTitle("Plugin Configuration - Simulator");

    layout.addWidget(&maxtorquetxt, 0, 0);
    layout.addWidget(&maxtorqueedit, 0, 1, 1, 2);

    connect(&mapfilebrowse, SIGNAL(clicked()), &mapfiledialog, SLOT(exec()));
    connect(&mapfiledialog, SIGNAL(fileSelected(QString)), this, SLOT(fileSelected(QString)));
    if(!mapfile.isEmpty())
        mapfiledialog.selectFile(mapfile);
    layout.addWidget(&mapfiletxt, 1, 0);
    layout.addWidget(&mapfilename, 1, 1);
    layout.addWidget(&mapfilebrowse, 1, 2);

    layout.addWidget(&mapscaletxt, 2, 0);
    layout.addWidget(&mapscaleedit, 2, 1);
    layout.addWidget(&mapscaleunit, 2, 2);

    layout.addWidget(&startposetxt, 3, 0);
    layout.addWidget(&startx, 3, 1);
    layout.addWidget(&starty, 3, 2);

    layout.addWidget(&startorienttxt, 4, 0);
    layout.addWidget(&starttheta, 4, 1);

    layout.addWidget(&lidarsweeptxt, 5, 0);
    layout.addWidget(&lidarsweep, 5, 1);
    layout.addWidget(&lidarsweepunit, 5, 2);

    layout.addWidget(&lidarsteptxt, 6, 0);
    layout.addWidget(&lidarstep, 6, 1);
    layout.addWidget(&lidarstepunit, 6, 2);

    layout.addWidget(&lidarmintxt, 7, 0);
    layout.addWidget(&lidarmin, 7, 1);
    layout.addWidget(&lidarminunit, 7, 2);

    layout.addWidget(&lidarmaxtxt, 8, 0);
    layout.addWidget(&lidarmax, 8, 1);
    layout.addWidget(&lidarmaxunit, 8, 2);

    layout.addWidget(&simratetxt, 9, 0);
    layout.addWidget(&simrate, 9, 1, 1, 2);

    layout.addWidget(&okbutton, 10, 1, 1, 2);
    connect(&okbutton, SIGNAL(clicked()), this, SLOT(accept()));

    setLayout(&layout);
}

void CPSimConfig::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return)
    {
        event->accept();
        this->accept();
    }
}

const unsigned short int CPRobotSim::wallLookup[120] =
{
  0,   1,   2,   3,   4,   5,   6,   7,   7,   8,
  9,  12,  15,  19,  22,  25,  55,  85, 115, 145,
175, 191, 207, 223, 239, 255, 257, 260, 262, 265,
267, 272, 277, 283, 288, 293, 292, 290, 289, 287,
286, 278, 270, 262, 254, 246, 235, 224, 212, 201,
190, 178, 167, 155, 144, 132, 124, 117, 109, 102,
 94,  89,  84,  78,  73,  68,  63,  59,  54,  50,
 45,  42,  39,  37,  34,  31,  29,  27,  25,  23,
 21,  20,  19,  17,  16,  15,  14,  13,  13,  12,
 11,  10,   9,   8,   7,   6,   6,   5,   5,   4,
  4,   4,   4,   3,   3,   3,   3,   3,   2,   2,
  2,   2,   2,   1,   1,   1,   1,   1,   0,   0
};

CPRobotSim::CPRobotSim(QObject *_parent)
    : _x(0.0),
      _y(0.0),
      _theta(0.0),
      _x_d(0.0),
      _y_d(0.0),
      _theta_d(0.0),
      _x_dd(0.0),
      _y_dd(0.0),
      _theta_dd(0.0),
      _l(0.0),
      _l_d(0.0),
      _l_d_t(0.0),
      _l_dd(0.0),
      _r(0.0),
      _r_d(0.0),
      _r_d_t(0.0),
      _r_dd(0.0),
      _maxAccel(1.0),
      last_time(0, 0),
      parent(_parent),
      lidar_full_angle( 1.01229097 ),
      lidar_angle_step( 1.01229097 / 480 ),
      lidar_max( 3.8 )
{

}

void CPRobotSim::spinOnce()
{
    QMutexLocker locker(&mutex);

    const QTime c_t = QTime::currentTime();
    const double d_t = last_time.msecsTo(c_t) * 0.001;
    last_time = c_t;
    if(d_t > 1.0)
        return;

    // We first work with the wheels
    // left accel
    if(_l_d_t > _l_d)
    {
        if((_l_d_t - _l_d) / d_t >= _maxAccel)
            _l_dd = _maxAccel;
        else
            _l_dd = (_l_d_t - _l_d) / d_t;
    }
    else if(_l_d_t < _l_d)
    {
        if((_l_d - _l_d_t) / d_t >= _maxAccel)
            _l_dd = -_maxAccel;
        else
            _l_dd = -(_l_d - _l_d_t) / d_t;
    }
    else
        _l_dd = 0.0;

    // right accel
    if(_r_d_t > _r_d)
    {
        if((_r_d_t - _r_d) / d_t >= _maxAccel)
            _r_dd = _maxAccel;
        else
            _r_dd = (_r_d_t - _r_d) / d_t;
    }
    else if(_r_d_t < _r_d)
    {
        if((_r_d - _r_d_t) / d_t >= _maxAccel)
            _r_dd = -_maxAccel;
        else
            _r_dd = -(_r_d - _r_d_t) / d_t;
    }
    else
        _r_dd = 0.0;

    // wheel velcities
    _l_d += _l_dd * d_t;
    _r_d += _r_dd * d_t;

    // wheel positions
    _l += _l_d * d_t;
    _r += _r_d * d_t;

    // Slip calculations would go somewhere in here

    const static double wheel_diam = 0.07;     // meters
    const static double wheel_base = 0.26035 / 2.0;  // meters

    // New Position
    double _x_new = _x + d_t * ( ( wheel_diam / 2.0 ) / 2.0 ) * ( _l_d + _r_d ) * cos( _theta );
    double _y_new = _y + d_t * ( ( wheel_diam / 2.0 ) / 2.0 ) * ( _l_d + _r_d ) * sin( _theta );
    const double _theta_new = _theta + d_t * ( ( wheel_diam / 2.0 ) / ( 2.0 * wheel_base ) ) * ( _r_d - _l_d );

    // Bump/Collision Detection
    const float ogc_x = _x_new / og.info.resolution + (og.info.width / 2.0);
    const float ogc_y = _y_new / og.info.resolution + (og.info.height / 2.0);
    float bump = 2 * M_PI;
    if(ogc_y >= 0 && ogc_y < og.info.height && ogc_x >= 0 && ogc_x < og.info.width)
    {
        float this_bump;
        for(this_bump = 0; this_bump < 2 * M_PI; this_bump += M_PI / 8)
        {
            const int ogb_x = ogc_x + cos(this_bump) * (0.170 / og.info.resolution);
            const int ogb_y = ogc_y + sin(this_bump) * (0.170 / og.info.resolution);
            if(ogb_y >= 0 && ogb_y < og.info.height && ogb_x >= 0 && ogb_x < og.info.width && og.data[ogb_y * og.info.width + ogb_x] > 90)
                bump = this_bump;

            const int og_x = ogc_x + cos(this_bump) * (0.165 / og.info.resolution);
            const int og_y = ogc_y + sin(this_bump) * (0.165 / og.info.resolution);
            if(og_y >= 0 && og_y < og.info.height && og_x >= 0 && og_x < og.info.width && og.data[og_y * og.info.width + og_x] > 90)
            {
                _x_new = _x;
                _y_new = _y;
            }
        }
    }

    // Wall Sensor
    // find the output point
    const float ws_x = ogc_x + cos(_theta - 2 * M_PI / 6) * (0.170 / og.info.resolution);
    const float ws_y = ogc_y + sin(_theta - 2 * M_PI / 6) * (0.170 / og.info.resolution);
    const float ws_ct = cos(_theta - M_PI / 2);
    const float ws_st = sin(_theta - M_PI / 2);
    unsigned int ws = 101;
    // step out in 1mm steps up to 10cm
    for(ws = 1; ws < 101; ws++)
    {
        const int wss_x = ws_x + ws_ct * (ws / 1000.0 / og.info.resolution);
        const int wss_y = ws_y + ws_st * (ws / 1000.0 / og.info.resolution);
        if(wss_y >= 0 && wss_y < og.info.height && wss_x >= 0 && wss_x < og.info.width && og.data[wss_y * og.info.width + wss_x] > 90)
            break;
    }
    if(ws > 100)
        ws = 0;
    ws = wallLookup[ws];

    // Laser
    const float l_x = ogc_x + cos(_theta - M_PI) * (0.080 / og.info.resolution);
    const float l_y = ogc_y + sin(_theta - M_PI) * (0.080 / og.info.resolution);
    std::vector<float> laser;
    for(float i = -lidar_full_angle / 2.0; i < lidar_full_angle / 2.0 + lidar_angle_step / 2.0; i += lidar_angle_step )
    {
        laser.push_back(pixelStep(l_x, l_y, l_x + cos(_theta + i) * ((lidar_max + lidar_angle_step + lidar_angle_step) / og.info.resolution), l_y + sin(_theta + i) * ((lidar_max + lidar_angle_step + lidar_angle_step) / og.info.resolution)) * og.info.resolution);
    }

    // New Velocity
    const double _x_d_new = (_x_new - _x) / d_t;
    const double _y_d_new = (_y_new - _y) / d_t;
    const double _theta_d_new = (_theta_new - _theta) / d_t;

    // Acceleration
    _x_dd = (_x_d_new - _x_d) / d_t;
    _y_dd = (_y_d_new - _y_d) / d_t;
    _theta_dd = (_theta_d_new - _theta_d) / d_t;

    // Velocity
    _x_d = _x_d_new;
    _y_d = _y_d_new;
    _theta_d = _theta_d_new;

    // Position
    _x = _x_new;
    _y = _y_new;
    _theta = _theta_new;
    if(_theta > 2 * M_PI)
        _theta -= 2 * M_PI;
    else if(_theta < 0)
        _theta += 2 * M_PI;

    emit updatePose(_x, _y, _theta);
    const bool did_bump = bump < 2 * M_PI;
    bump -= _theta;
    if( bump < -M_PI )
        bump += 2 * M_PI;
    else if( bump > M_PI )
        bump -= 2 * M_PI;
    emit updateBump(did_bump, bump);
    emit updateWall(ws);
    emit updateTwist(sqrt(_x_d * _x_d + _y_d * _y_d), _theta_d);
    emit updateLaser(laser);
}

void CPRobotSim::updateTarget(double l_d_t, double r_d_t)
{
    QMutexLocker locker(&mutex);
    _l_d_t = l_d_t;
    _r_d_t = r_d_t;
}

void CPRobotSim::breakSim()
{
    QMutexLocker locker(&mutex);
    last_time = QTime(0, 0);
}

void CPRobotSim::updateMaxAccel(double maxAccel)
{
    QMutexLocker locker(&mutex);
    _maxAccel = maxAccel;
}

double CPRobotSim::x()
{
    QMutexLocker locker(&mutex);
    return _x;
}

double CPRobotSim::y()
{
    QMutexLocker locker(&mutex);
    return _y;
}

double CPRobotSim::theta()
{
    QMutexLocker locker(&mutex);
    return _theta;
}

double CPRobotSim::x_d()
{
    QMutexLocker locker(&mutex);
    return _x_d;
}

double CPRobotSim::y_d()
{
    QMutexLocker locker(&mutex);
    return _y_d;
}

double CPRobotSim::theta_d()
{
    QMutexLocker locker(&mutex);
    return _theta_d;
}

double CPRobotSim::x_dd()
{
    QMutexLocker locker(&mutex);
    return _x_dd;
}

double CPRobotSim::y_dd()
{
    QMutexLocker locker(&mutex);
    return _y_dd;
}

double CPRobotSim::theta_dd()
{
    QMutexLocker locker(&mutex);
    return _theta_dd;
}

double CPRobotSim::maxAccel()
{
    QMutexLocker locker(&mutex);
    return _maxAccel;
}

void CPRobotSim::setOG(const nav_msgs::OccupancyGrid &_og)
{
    QMutexLocker locker(&mutex);
    og = _og;
}

void CPRobotSim::setPose(double x, double y, double theta)
{
    QMutexLocker locker(&mutex);
    _x = x;
    _y = y;
    _theta = theta;
    emit updatePose(_x, _y, _theta);
}

void CPRobotSim::setupLidar(double full_angle, double angle_step, double max)
{
    lidar_full_angle = full_angle;
    lidar_angle_step = angle_step;
    lidar_max = max;
}

bool CPRobotSim::pixelOccupied(int x, int y) const
{
    return (y >= 0 && y < og.info.height && x >= 0 && x < og.info.width && og.data[y * og.info.width + x] > 90);
}

float CPRobotSim::pixelStep(int x0, int y0, int x1, int y1) const
{
    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);

    if(std::abs(dx) > std::abs(dy))
    {
        int D = 2 * dy - dx;
        int y = y0;

        const int y_step = (y1 > y0) ? 1 : -1;

        if(x1 > x0)
        {
            for(int x = x0 + 1; x <= x1; x++)
            {
                if(D > 0)
                {
                    y += y_step;
                    if(pixelOccupied(x, y))
                        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
                    D += (2 * dy - 2 * dx);
                }
                else
                {
                    if(pixelOccupied(x, y))
                        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
                    D += (2 * dy);
                }
            }
        }
        else
        {
            for(int x = x0 - 1; x >= x1; x--)
            {
                if(D > 0)
                {
                    y += y_step;
                    if(pixelOccupied(x, y))
                        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
                    D += (2 * dy - 2 * dx);
                }
                else
                {
                    if(pixelOccupied(x, y))
                        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
                    D += (2 * dy);
                }
            }
        }
    }
    else
    {
        int D = 2 * dx - dy;
        int x = x0;

        const int x_step = (x1 > x0) ? 1 : -1;

        if(y1 > y0)
        {
            for(int y = y0 + 1; y <= y1; y++)
            {
                if(D > 0)
                {
                    x += x_step;
                    if(pixelOccupied(x, y))
                        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
                    D += (2 * dx - 2 * dy);
                }
                else
                {
                    if(pixelOccupied(x, y))
                        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
                    D += (2 * dx);
                }
            }
        }
        else
        {
            for(int y = y0 - 1; y >= y1; y--)
            {
                if(D > 0)
                {
                    x += x_step;
                    if(pixelOccupied(x, y))
                        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
                    D += (2 * dx - 2 * dy);
                }
                else
                {
                    if(pixelOccupied(x, y))
                        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
                    D += (2 * dx);
                }
            }
        }
    }

    return sqrt(dx * dx + dy * dy);
}

CPSimulatorPlugin::CPSimulatorPlugin() :
    ui(new Ui::CPSimulatorPlugin),
    nodelet_priv(new CPSimulatorNodelet(this)),
    robotSim(0),
    robotSimTimer(0),
    lastMapLoadSucceeded(false)
{
    qRegisterMetaType<std::vector<float> >("std::vector<float>");

    ui->setupUi(this);

    robotSimTimer.setInterval(15);
    robotSimTimer.moveToThread(&robotSimThread);
    robotSim.moveToThread(&robotSimThread);

    og.info.origin.orientation.w = 1;

    connect(&robotSim, SIGNAL(updatePose(double, double, double)), ui->widget, SLOT(updatePose(double, double, double)));
    connect(&robotSimTimer, SIGNAL(timeout()), &robotSim, SLOT(spinOnce()));
    connect(&robotSimThread, SIGNAL(started()), &robotSimTimer, SLOT(start()));
    connect(this, SIGNAL(mapUpdated(const QImage &)), ui->widget, SLOT(setMap(const QImage &)));

    connect(&robotSim, SIGNAL(updatePose(double, double, double)), this, SLOT(publishSimPose(double, double, double)));
    connect(&robotSim, SIGNAL(updateBump(bool, double)), this, SLOT(publishBump(bool, double)));
    connect(&robotSim, SIGNAL(updateWall(const unsigned int)), this, SLOT(publishWall(const unsigned int)));
    connect(&robotSim, SIGNAL(updateLaser(const std::vector<float>)), this, SLOT(publishLaser(const std::vector<float>)));
}

CPSimulatorPlugin::~CPSimulatorPlugin()
{
    robotSimThread.quit();
    robotSimThread.wait();
    delete ui;
}

void CPSimulatorPlugin::start()
{
    robotSim.breakSim();
    robotSimThread.start();
    nodelet_priv->activate();
    settings->setValue(uuid.toString() + "/Paused", false);

    // Since ROS just came up, we should publish our map so it will latch
    nodelet_priv->pub_map(og);
}

void CPSimulatorPlugin::stop()
{
    robotSimThread.quit();
    nodelet_priv->deactivate();
    settings->setValue(uuid.toString() + "/Paused", true);
}

void CPSimulatorPlugin::setup()
{
    // TODO: Update this with wheel radius and mass
    robotSim.updateMaxAccel(torque2angaccel(settings->value(uuid.toString() + "/MaxTorque", (double)1.0).toDouble()));

    og.info.resolution = 1.0 / settings->value(uuid.toString() + "/MapScale", 100).toDouble();
    ui->widget->setMapScale(settings->value(uuid.toString() + "/MapScale", 100).toDouble());
    robotSim.setOG(og);

    if(!settings->value(uuid.toString() + "/MapFile", QString()).toString().isEmpty())
        if(!loadMap(settings->value(uuid.toString() + "/MapFile", QString()).toString()))
            std::cerr << "Failed to load map" << std::endl;

    robotSim.setPose(
        settings->value(uuid.toString() + "/StartX", (double)0.0).toDouble(),
        settings->value(uuid.toString() + "/StartY", (double)0.0).toDouble(),
        settings->value(uuid.toString() + "/StartT", (double)0.0).toDouble()
    );

    robotSim.setupLidar(
        settings->value(uuid.toString() + "/LidarFullAngle", (double)1.01229097).toDouble(),
        settings->value(uuid.toString() + "/LidarAngleStep", (double)(1.01229097 / 480)).toDouble(),
        settings->value(uuid.toString() + "/LidarDistanceMax", (double)3.8).toDouble()
    );

    nodelet_priv->setupLidar(
        settings->value(uuid.toString() + "/LidarFullAngle", (double)1.01229097).toDouble(),
        settings->value(uuid.toString() + "/LidarAngleStep", (double)(1.01229097 / 480)).toDouble(),
        settings->value(uuid.toString() + "/LidarDistanceMin", (double)0.8).toDouble(),
        settings->value(uuid.toString() + "/LidarDistanceMax", (double)3.8).toDouble()
    );

    // Set simulation rate
    robotSimTimer.setInterval(1000.0 / settings->value(uuid.toString() + "/SimRate", (double)66.6).toDouble());

    // Start the simulation (if we should)
    setPaused(settings->value(uuid.toString() + "/Paused", false).toBool());
}

boost::shared_ptr<nodelet::Nodelet> CPSimulatorPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

double CPSimulatorPlugin::torque2angaccel(double torque) const
{
    // start with N*m

    // Divide by the radius
    torque /= 0.035; // N or kg*m/s/s

    // Divide by the mass / 2
    torque /= 1.5; // m/s/s

    // We now have the linear acceleration at the wheel, convert to angular

    // Divide by the meters per revolution (circumference)
    torque /= 0.07 * M_PI; // rev/s/s

    // Multiply by the radians per revolution
    torque *= 2 * M_PI; // rad/s/s

    return torque;
}

double CPSimulatorPlugin::angaccel2torque(double angaccel) const
{
    // start with rad/s/s

    // Divide by the radians per revolution
    angaccel /= 2 * M_PI; // rev/s/s

    // Multiply by the meters per revolution (circumference)
    angaccel *= 0.07 * M_PI; // m/s/s

    // We now have the linear acceleration at the wheel, convert to torque

    // Multiply by mass / 2
    angaccel *= 1.5; // N or kg*m/s/s

    // Multiply by the radius
    angaccel *= 0.035; // N*m

    return angaccel;
}

void CPSimulatorPlugin::JointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
    int l_index = -1;
    int r_index = -1;
    for(size_t i = 0; i < msg->joint_names.size(); i++)
    {
        if(msg->joint_names[i] == "left_wheel_joint")
            l_index = i;
        else if(msg->joint_names[i] == "right_wheel_joint")
            r_index = i;
    }
    if(l_index < 0 || r_index < 0 || msg->points.size() < 1 || msg->points[0].velocities.size() <= l_index || msg->points[0].velocities.size() <= r_index)
    {
        ROS_WARN("Invalid joint trajectory message!");
        return;
    }
    robotSim.updateTarget(msg->points[0].velocities[l_index], msg->points[0].velocities[r_index]);
}

void CPSimulatorPlugin::setPaused(bool paused)
{
    if(paused)
        stop();
    else
        start();
}

void CPSimulatorPlugin::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu;
    menu.addAction("Paused");
    menu.actions()[0]->setCheckable(true);
    menu.actions()[0]->setChecked(settings->value(uuid.toString() + "/Paused", false).toBool());
    connect(menu.actions()[0], SIGNAL(toggled(bool)), this, SLOT(setPaused(bool)));
    menu.addAction("Configure", this, SLOT(configDialog()));
    menu.addAction("Reset", this, SLOT(resetSimulation()));
    menu.addAction("Delete", this, SLOT(delete_self()));
    menu.exec(event->globalPos());
}

void CPSimulatorPlugin::configDialog()
{
    bool publishMap = false;

    CPSimConfig dialog(
        angaccel2torque(robotSim.maxAccel()),
        settings->value(uuid.toString() + "/MapFile", QString()).toString(),
        settings->value(uuid.toString() + "/MapScale", 100).toDouble(),
        settings->value(uuid.toString() + "/StartX", 0.0).toDouble(),
        settings->value(uuid.toString() + "/StartY", 0.0).toDouble(),
        settings->value(uuid.toString() + "/StartT", 0.0).toDouble(),
        settings->value(uuid.toString() + "/LidarFullAngle", (double)1.01229097).toDouble(),
        settings->value(uuid.toString() + "/LidarAngleStep", (double)(1.01229097 / 480)).toDouble(),
        settings->value(uuid.toString() + "/LidarDistanceMin", (double)0.8).toDouble(),
        settings->value(uuid.toString() + "/LidarDistanceMax", (double)3.8).toDouble(),
        settings->value(uuid.toString() + "/SimRate", (double)66.6).toDouble()
    );

    if(!dialog.exec())
        return;

    robotSim.updateMaxAccel(torque2angaccel(dialog.maxtorqueedit.text().toDouble()));
    settings->setValue(uuid.toString() + "/MaxTorque", dialog.maxtorqueedit.text().toDouble());

    if(!lastMapLoadSucceeded || settings->value(uuid.toString() + "/MapFile", QString()).toString() != dialog.mapfilename.text())
    {
        settings->setValue(uuid.toString() + "/MapFile", dialog.mapfilename.text());
        if(!loadMap(dialog.mapfilename.text()))
            std::cerr << "Failed to load map" << std::endl;
        else
            publishMap = true;
    }
    if(settings->value(uuid.toString() + "/MapScale", 100).toDouble() != dialog.mapscaleedit.text().toDouble())
    {
        settings->setValue(uuid.toString() + "/MapScale", dialog.mapscaleedit.text().toDouble());
        ui->widget->setMapScale(dialog.mapscaleedit.text().toDouble());
        og.info.resolution = 1.0 / dialog.mapscaleedit.text().toDouble();
        og.info.origin.position.x = og.info.width * og.info.resolution * -0.5;
        og.info.origin.position.y = og.info.height * og.info.resolution * -0.5;
        robotSim.setOG(og);
        publishMap = true;
    }

    settings->setValue(uuid.toString() + "/StartX", dialog.startx.text().toDouble());
    settings->setValue(uuid.toString() + "/StartY", dialog.starty.text().toDouble());
    settings->setValue(uuid.toString() + "/StartT", dialog.starttheta.text().toDouble());

    if(dialog.lidarsweep.text().toDouble() > 0.0 && dialog.lidarsweep.text().toDouble() <= 2 * M_PI)
        settings->setValue(uuid.toString() + "/LidarFullAngle", dialog.lidarsweep.text().toDouble());
    if(dialog.lidarstep.text().toDouble() > .0001 && dialog.lidarstep.text().toDouble() < settings->value(uuid.toString() + "/LidarFullAngle", (double)1.01229097).toDouble())
        settings->setValue(uuid.toString() + "/LidarAngleStep", dialog.lidarstep.text().toDouble());
    if(dialog.lidarmin.text().toDouble() < dialog.lidarmax.text().toDouble())
    {
        settings->setValue(uuid.toString() + "/LidarDistanceMin", dialog.lidarmin.text().toDouble());
        settings->setValue(uuid.toString() + "/LidarDistanceMax", dialog.lidarmax.text().toDouble());
    }

    robotSim.setupLidar(
        settings->value(uuid.toString() + "/LidarFullAngle", (double)1.01229097).toDouble(),
        settings->value(uuid.toString() + "/LidarAngleStep", (double)(1.01229097 / 480)).toDouble(),
        settings->value(uuid.toString() + "/LidarDistanceMax", (double)3.8).toDouble()
    );

    nodelet_priv->setupLidar(
        settings->value(uuid.toString() + "/LidarFullAngle", (double)1.01229097).toDouble(),
        settings->value(uuid.toString() + "/LidarAngleStep", (double)(1.01229097 / 480)).toDouble(),
        settings->value(uuid.toString() + "/LidarDistanceMin", (double)0.8).toDouble(),
        settings->value(uuid.toString() + "/LidarDistanceMax", (double)3.8).toDouble()
    );

    if(publishMap)
        nodelet_priv->pub_map(og);

    settings->setValue(uuid.toString() + "/SimRate", dialog.simrate.text().toDouble());
    robotSimTimer.setInterval(1000.0 / settings->value(uuid.toString() + "/SimRate", (double)66.6).toDouble());
}

void CPSimulatorPlugin::resetSimulation()
{
    robotSim.setPose(
        settings->value(uuid.toString() + "/StartX", (double)0.0).toDouble(),
        settings->value(uuid.toString() + "/StartY", (double)0.0).toDouble(),
        settings->value(uuid.toString() + "/StartT", (double)0.0).toDouble()
    );
}

bool CPSimulatorPlugin::loadMap(QString mapfilename)
{
    QImage mapimg;

    if(!mapfilename.isEmpty())
    {
        if(!mapimg.load(mapfilename))
        {
            lastMapLoadSucceeded = false;
            return false;
        }
    }

    emit mapUpdated(mapimg);

    og.info.map_load_time = ros::Time::now();
    og.info.width = mapimg.width();
    og.info.height = mapimg.height();
    og.info.origin.position.x = og.info.width * og.info.resolution * -0.5;
    og.info.origin.position.y = og.info.height * og.info.resolution * -0.5;
    og.data.resize(og.info.width * og.info.height);
    for(unsigned int i = 0; i < og.info.height; i++)
    {
        for(unsigned int j = 0; j < og.info.width; j++)
            og.data[i * og.info.width + j] = 100 - (qGray(mapimg.pixel(j, mapimg.height() - i - 1)) * 100 / 255);
    }
    robotSim.setOG(og);

    lastMapLoadSucceeded = true;

    return true;
}

void CPSimulatorPlugin::publishBump(bool bump, double theta)
{
    bool left = false;
    bool right = false;
    if(bump)
    {
        if(theta < -M_PI)
            theta += 2 * M_PI;
        if(theta >= -3 * M_PI / 8 && theta <= M_PI / 8)
            right = true;
        if(theta >= -M_PI / 8 && theta <= 3 * M_PI / 8)
            left = true;
    }
    nodelet_priv->pub_bump_left(left);
    nodelet_priv->pub_bump_right(right);
}

void CPSimulatorPlugin::publishWall(const unsigned int dist)
{
    nodelet_priv->pub_wall((unsigned short int)dist);
}

void CPSimulatorPlugin::publishSimPose(double x, double y, double theta)
{
    nodelet_priv->pub_sim_pose(x, y, theta);
}

void CPSimulatorPlugin::publishLaser(const std::vector<float> scan)
{
    nodelet_priv->pub_laser(scan);
}

CPSimulatorGL::CPSimulatorGL(QWidget *_parent)
    : parent(_parent),
      zoom_level(0.05),
      w(100),
      h(100),
      drag_origin_w(-1),
      drag_origin_h(-1),
      window_center_w(0),
      window_center_h(0),
      drag_window_center_start_w(-1),
      drag_window_center_start_h(-1),
      robot_dl(0),
      map_id(0),
      map_scale(100),
      map_dl(0)
{
}

CPSimulatorGL::~CPSimulatorGL()
{
    if(robot_dl)
      glDeleteLists(robot_dl, 1);
    if(map_dl)
      glDeleteLists(map_dl, 1);
}

void CPSimulatorGL::setMap(const QImage &img)
{
    map = img;

    if(!isValid())
        return;

    reloadTexture();

    paintEnvironment();

    updateGL();
}

void CPSimulatorGL::setMapScale(const double scale)
{
    map_scale = scale;

    paintEnvironment();

    updateGL();
}

void CPSimulatorGL::reloadTexture()
{
    deleteTexture(map_id);
    map_id = bindTexture(map);
}

void CPSimulatorGL::updatePose(double _x, double _y, double _theta)
{
    x = _x;
    y = _y;
    theta = _theta;
    updateGL();
}

void CPSimulatorGL::initializeGL()
{
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(1, 1, 1, 1);

    // Create robot
    robot_dl = glGenLists(1);

    // Create map
    map_dl = glGenLists(1);

    // Paint the robot
    paintRobot();

    reloadTexture();
}

void CPSimulatorGL::resizeGL(int _w, int _h)
{
    w = _w;
    h = _h;
    updateZoom();
}

void CPSimulatorGL::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glCallList(map_dl);
    glTranslated(x, y, 0.0);
    glRotated(theta * 180.0 / M_PI, 0.0, 0.0, 1.0);
    glCallList(robot_dl);
}

void CPSimulatorGL::paintEnvironment()
{
    glNewList(map_dl, GL_COMPILE);
    glColor3f(1.0, 1.0, 1.0);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_POLYGON);
    glTexCoord2f(0, 0);
    glVertex2f(-map.width() / map_scale / 2.0, -map.height() / map_scale / 2.0);
    glTexCoord2f(1, 0);
    glVertex2f(map.width() / map_scale / 2.0, -map.height() / map_scale / 2.0);
    glTexCoord2f(1, 1);
    glVertex2f(map.width() / map_scale / 2.0, map.height() / map_scale / 2.0);
    glTexCoord2f(0, 1);
    glVertex2f(-map.width() / map_scale / 2.0, map.height() / map_scale / 2.0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glEndList();
}

void CPSimulatorGL::paintRobot()
{
    glNewList(robot_dl, GL_COMPILE);
    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
    glVertex2d(0, 0);
    glVertex2d(0.165, 0);
    glEnd();
    glColor3f(0, 0, 0);
    glBegin(GL_LINE_LOOP);
    for(float i = 0; i < 2 * M_PI; i+= M_PI / 50.0)
        glVertex2d(cos(i) * 0.165, sin(i) * 0.165);
    glEnd();
    glEndList();
}

void CPSimulatorGL::mousePressEvent(QMouseEvent *event)
{
    if(event->button() & Qt::LeftButton)
    {
        drag_origin_w = event->x();
        drag_origin_h = event->y();
        drag_window_center_start_w = window_center_w;
        drag_window_center_start_h = window_center_h;
        event->accept();
    }
}

void CPSimulatorGL::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button() & Qt::LeftButton)
    {
        drag_origin_w = drag_origin_h = -1;
        event->accept();
    }
}

void CPSimulatorGL::mouseMoveEvent(QMouseEvent *event)
{
    if(drag_origin_w >= 0 && drag_origin_h >= 0)
    {
        window_center_w = drag_window_center_start_w + zoom_level * (drag_origin_w - event->x());
        window_center_h = drag_window_center_start_h - zoom_level * (drag_origin_h - event->y());
        updateZoom();
        updateGL();
        event->accept();
    }
}

void CPSimulatorGL::wheelEvent(QWheelEvent *event)
{
    if(event->orientation() == Qt::Vertical)
    {
        zoom_level -= event->delta() / 100000.0;
        if(zoom_level < .0005)
            zoom_level = .0005;
        updateZoom();
        updateGL();
        event->accept();
    }
}

void CPSimulatorGL::keyPressEvent(QKeyEvent* event)
{
    switch(event->key()) {
    case Qt::Key_Escape:
        close();
        break;
    default:
        event->ignore();
        break;
    }
}

void CPSimulatorGL::updateZoom()
{
    const double w_2 = w / 2.0 * zoom_level;
    const double h_2 = h / 2.0 * zoom_level;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(window_center_w - w_2, window_center_w + w_2, window_center_h - h_2, window_center_h + h_2);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

float CPSimulatorGL::getZoom()
{
    return zoom_level;
}

CPSimulatorNodelet::CPSimulatorNodelet(CPSimulatorPlugin *_parent)
    : parent(_parent),
      lidar_full_angle( 1.01229097 ),
      lidar_angle_step( 1.01229097 / 480 ),
      lidar_distance_min( 0.8 ),
      lidar_distance_max( 3.8 )
{
}

void CPSimulatorNodelet::onInit()
{
}

void CPSimulatorNodelet::setupLidar(double full_angle, double angle_step, double distance_min, double distance_max)
{
    lidar_full_angle = full_angle;
    lidar_angle_step = angle_step;
    lidar_distance_min = distance_min;
    lidar_distance_max = distance_max;
}

void CPSimulatorNodelet::activate(bool passive)
{
    ros::NodeHandle nh = getNodeHandle();

    if(!joint_trajectory_sub && !passive)
        joint_trajectory_sub = nh.subscribe("joint_trajectory", 1, &CPSimulatorPlugin::JointTrajectoryCB, parent);
    if(!sim_pose_pub && !passive)
        sim_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("sim_pose", 1, false);
    if(!bump_left_pub && !passive)
        bump_left_pub = nh.advertise<std_msgs::Bool>("bump/left", 1, false);
    if(!bump_right_pub && !passive)
        bump_right_pub = nh.advertise<std_msgs::Bool>("bump/right", 1, false);
    if(!wall_pub && !passive)
        wall_pub = nh.advertise<std_msgs::UInt16>("wall", 1, false);
    if(!map_pub && !passive)
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 5, true);
    if(!laser_pub && !passive)
        laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1, false);
}

void CPSimulatorNodelet::deactivate()
{
    if(joint_trajectory_sub)
        joint_trajectory_sub.shutdown();

    if(sim_pose_pub)
        sim_pose_pub.shutdown();

    if(bump_left_pub)
        bump_left_pub.shutdown();

    if(bump_right_pub)
        bump_right_pub.shutdown();

    if(wall_pub)
        wall_pub.shutdown();

    if(map_pub)
        map_pub.shutdown();

    if(laser_pub)
        laser_pub.shutdown();
}

bool CPSimulatorNodelet::isActive()
{
    return (joint_trajectory_sub && sim_pose_pub && bump_left_pub && bump_right_pub);
}

void CPSimulatorNodelet::pub_bump_left(bool bump)
{
    if(!bump_left_pub)
        return;

    std_msgs::BoolPtr msg(new std_msgs::Bool);
    msg->data = bump;
    bump_left_pub.publish(msg);
}

void CPSimulatorNodelet::pub_bump_right(bool bump)
{
    if(!bump_right_pub)
        return;

    std_msgs::BoolPtr msg(new std_msgs::Bool);
    msg->data = bump;
    bump_right_pub.publish(msg);
}

void CPSimulatorNodelet::pub_sim_pose(double x, double y, double theta)
{
    if(!sim_pose_pub)
        return;

    geometry_msgs::PoseStampedPtr msg(new geometry_msgs::PoseStamped);

    msg->header.stamp = ros::Time::now( );
    msg->header.frame_id = "/base_link";
    msg->pose.position.x = x;
    msg->pose.position.y = y;
    msg->pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    sim_pose_pub.publish(msg);
}

void CPSimulatorNodelet::pub_map(const nav_msgs::OccupancyGrid &og)
{
    if(!map_pub)
        return;

    nav_msgs::OccupancyGridPtr msg(new nav_msgs::OccupancyGrid(og));
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "/map";

    map_pub.publish(msg);
}

void CPSimulatorNodelet::pub_wall(const unsigned short int dist)
{
    if(!wall_pub)
        return;

    std_msgs::UInt16Ptr msg(new std_msgs::UInt16);
    msg->data = dist;

    wall_pub.publish(msg);
}

void CPSimulatorNodelet::pub_laser(const std::vector<float> &scan)
{
    if(!laser_pub)
        return;

    sensor_msgs::LaserScanPtr msg(new sensor_msgs::LaserScan);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "/laser";

    msg->angle_min = -lidar_full_angle / 2.0;
    msg->angle_max = lidar_full_angle / 2.0;
    msg->angle_increment = lidar_angle_step;
    msg->time_increment = 0.0;
    msg->scan_time = 0.015;

    msg->range_min = lidar_distance_min;
    msg->range_max = lidar_distance_max;

    msg->ranges.assign(&scan[0], &scan[scan.size() - 1]);

    laser_pub.publish(msg);
}
}
