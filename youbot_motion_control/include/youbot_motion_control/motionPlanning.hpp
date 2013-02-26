/**************************************************************************************
 *NEUERUNGEN:
 *Geschwindigkeitssetter können ausgelassen werden: dazu velpoints eingeführt, die die
 *Anzahl an Bahnpunkten, mit der Anzahl an Geschwindigkeitspunkten vor jedem Setzen eines
 *neuen Bahnpunkts vergleichen. Wenn ungleich, wird Standardgeschwindigkeit gesetzt(velfactor=1).
 *
 *moveToPose kann auch nach einem Pathpoint gesetzt werden. Wird fälschicherweise die Methode aufgerufen,
 *wird geprüft, ob ein Endpunkt bereits gesetzt wurde, indem eine boolsche Variable(endPose_set) geprüft wird.
 *Die Variable wird nur dann true gesetzt, wenn ein Endpunkt aufgerufen wurde. *
 **************************************************************************************/

#ifndef MOTION_PLANNING_HPP
#define MOTION_PLANNING_HPP

//ROS includes
#include "ros/ros.h"
#include "rosbag/recorder.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>

#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>

//YouBot includes
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointAccelerations.h>
#include <sensor_msgs/JointState.h>
#include <youbot_kinematics/youbot_arm_kinematics.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <youbot_motion_control/motionControl.h>

#include <boost/units/io.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/units/systems/si.hpp>

//Jacobimatrix
#include "Jacobi/getJ.mc"
#include "Jacobi/invMAT.mc"
#include "Jacobi/getqDot_MAT.mc"
#include "Jacobi/getTE0.mc"

/**
  *@brief Diese Klasse dient zur Plannung einer Trajektorie mithilfe eines Polynoms 5.Ordnung im Jointspace und eingeschränkt auch im Taskspace.
  *       Dadurch sind flüssige und ruckfreie Bewegungen möglich. Vor der Plannung findet eine Synchronisation die langsamste Achse statt, sodass
  *       alle Gelenkwinkel gleichzeitig beginnen und stoppen. Mit dieser Klasse ist eine PTP- als auch eine kontinuirliche Bewegung ohne Anhalten
  *       an Bahnpunkten möglich, was im Folgenden als "überschleifen" bezeichnet wird. Die Geschwindigkeit lässt sich ebenfalls für jedes Segment
  *       variabel regulieren.
  */

/// Struktur mit maximaler Geschwindigeit max_vel und maximaler Beschleunigung max_acc
struct JointProperties {
    double max_acc;
    double max_vel;
    double minAngle;
    double maxAngle;
}Joint;

/// Struktur die eine Bahnpunkt beschreibt durch Position, Geschwindigkeit und Beschleunigung
struct parameter {
    double position;
    double velocity;
    double acceleration;
};

struct axis {
    std::vector <parameter> arm_axis;
};

/// Struktur die 6 Koeffizienten beinhaltet wodurch ein Polynom 5.Ordnung ausgedrückt wird
struct coefficients {
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;
} ;

typedef struct parameter PARAM;
typedef struct coefficients COEFFS;
typedef struct JointProperties PROPERTIES;
typedef struct axis AXIS;

class motionPlanning {

public:
    /// Instanz der youbot_arm_kinematics Klasse -> Hey Eddi, es ist für mich erheblich einfacher, wenn ich die Kinematics aus deiner Klasse nutzen kann!!! Andi
    youbot_arm_kinematics* youBotArm;

    /**
      *@brief constructor der motionPlanning Klasse
      */
    motionPlanning();

    ~motionPlanning();

    /**
      *@brief Initialisiere alle Variablen und Vektoren, die für neue Berechnung nötig sind
      */    
    void init(size_t workspace);

    void newTrajectory();

    /**
     * @brief ToDo ...
     */
    void newArmMove();

    /**
      *@brief setzt Softwareentschalter für jeden Joint und prüft, ob diese Überschritten sind.
      *       Bei Überschreitung werden die betroffenen Joints über ein Geschwindigkeitsprpofil zu stehen gebracht.
      */    
    void checkJointLimit();

    /**
      *@brief Für die Berechnung des Bahnprofils wird ein Polynom 5.Ordnung verwendet. In dieser Methode werden zunächst die Trajektorienzeiten für jeden
      *       Gelenkwinkel berechnet und die weitere Berechnung auf den langsamsten Arm synchronisiert, indem die maximalen Geschindigkeiten/bzw. Beschleunigungen
      *       der einzelnen Gelenkwinkel angepasst werden.
      *       Bei einer PTP-Bewegung sind Ziel- und Start Beschleunigungen, sowie Geschwindigkeiten 0. Bei einem Überschliffen Punkt ist das Positionsbedingt:
      *       Für P(i-1) > P(i) > P(i+1): v = -vmax , a = 0;
      *       Für P(i-1) > P(i) < P(i+1): v = 0     , a = amax;
      *       Für P(i-1) < P(i) > P(i+1): v = 0     , a = -amax;
      *       Für P(i-1) < P(i) < P(i+1): v = vmax  , a = 0;
      *       Im letzten Schritt erfolgt mithilfe der Randbedingungen die Berechnung der Polynomkoeffizienten.
      */    
    void calculatePathProperties();

    /**
      *@brief setze absolute Geschwindigkeiten der Gelenkwinkel für das Bahnsegment. Geschwindigkeit kann nicht größer als max. Geschwindigkeit sein.
      *@param JointVel1 absolute Geschwindigkeit für den 1.Gelenkwinkel
      *@param JointVel2 absolute Geschwindigkeit für den 2.Gelenkwinkel
      *@param JointVel3 absolute Geschwindigkeit für den 3.Gelenkwinkel
      *@param JointVel4 absolute Geschwindigkeit für den 4.Gelenkwinkel
      *@param JointVel5 absolute Geschwindigkeit für den 5.Gelenkwinkel
      */    
    void setTargetJointVelocity(double JointVel1, double JointVel2, double JointVel3, double JointVel4, double JointVel5);

    /**
      *@brief setze absolute Geschwindigkeiten der Gelenkwinkel für das Bahnsegment. Geschwindigkeit kann nicht größer als max. Geschwindigkeit sein.
      *@param JointVel Vektor mit 5 Elementen für die Gelenkwinkel
      */    
    void setTargetJointVelocity(std::vector <double> JointVel);

    /**
      *@brief setze Geschwindigkeit relativ zur max. Geschwindigkeit.
      *@param velfactor Faktor, der mit den Maximalgeschwindigkeiten multipliziert wird. Faktor muss größer als 0 sein.
      */    
    void setTargetJointVelocity(double velfactor);

    /**
      *@brief setze einen Bahnpunkt mit überschleifen
      *@param JointPos1 Zielgelenkwinkel der 1.Achse
      *@param JointPos1 Zielgelenkwinkel der 2.Achse
      *@param JointPos1 Zielgelenkwinkel der 3.Achse
      *@param JointPos1 Zielgelenkwinkel der 4.Achse
      *@param JointPos1 Zielgelenkwinkel der 5.Achse
      *@param gripper Gripperposition publishen, wenn true
      */    
    void setPathPoint(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5);

    void setPathPoint(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5, double gripperPos);

    /**
      *@brief setze einen Bahnpunkt mit überschleifen
      *@param JointPos Vektor mit 5 Elementen für Zielgelenkwinkelpositionen
      *@param gripper Gripperposition publishen, wenn true
      */    
    void setPathPoint(double JointPos[5]);

    /**
      *@brief
      *@param
      */

    void moveArm(double _x, double _y, double _z, double _roll, double _pitch);

    void setManipulatorVelocity(double manipulatorVel);

    void moveGripper(double dist);

    void getJointVelocity(double JointVel[5]);

    /**
      *@brief
      *@param
      */
    void getJointVelocity(int Joint, double &JointVel);

    /**
      *@brief
      *@param
      */
    void getJointPosition(double JointPos[5]);

    /**
      *@brief
      *@param
      */
    void getJointPosition(int Joint, double &JointVel);

    /**
      *@brief gibt true wieder, wenn der noch in Bewegung ist, sond false
      *@return true bedeutet, dass der Arm busy ist.
      */
    void armIsBusy();

    /**
      *@brief Nach dem setzen der Geschwindigkeiten und Bahnpunkte wird mit Aufruf dieser Funktion die Berechnung ausgeführt
      */
    void moveToPose();

    /**
      *@brief Generierung von Bahnpunkten entlang einer Strecke zwischen Ziel und Endpunkt im Kartesischen Raum.
      *       Die Bahnpunkte werden über die IK in Gelenkwinkelkoordinaten umgerechnet und in einer neuen Bahnplannung im Gelenkwinkelraum überschliffen.
      */
    void offlineMotionPlanning();

    /**
      *@brief für Experimente
      */
    void test_run(double _x, double _y, double _yaw);

    /**
      *@brief Methode zur Berechnung Parameter aus der aktuellen Pose
      */
    void moveRobot();

    void setTargetBasePosition(double _x, double _y, double _phi);

    void offlineBasePlanning();


    void BaseProperties();


    /// Wenn Arm in Bewegung, ist die Variable true
    bool busy;

private:
    void setPolynomCoefficients(PARAM _paramInit, PARAM _paramGoal, double _trajecTime, COEFFS &_coefficient);
    void setPolynomFunction(COEFFS _coeff, double _time, PARAM &_param);

    void interpOrient(double _t, double _x, double _z);

    void slerp(tf::Quaternion _q0, tf::Quaternion _q1, double _trel, tf::Quaternion &_q);

    void getOmega(tf::Quaternion _quatStart, tf::Quaternion _quatEnd, double _t, Eigen::Vector3d &_omega);

     /**
      *@brief Methode um Gripperposition zu publishen
      */
    void moveGripper();

    /**
      *@brief Callback zum Subsriben der Ist-Positionen und Ist-Geschwindigkeiten
      */
    void JointPositionCallback(const sensor_msgs::JointState& jointPositionCommand);
    void baseOdometryCallback(const nav_msgs::OdometryConstPtr& odomertry);

    /**
      *@brief Callback zur Berechnung der Geschwindigkeiten
      */
    void TimerCallback(const ros::TimerEvent& e);
    void TimerBaseCallback(const ros::TimerEvent& e);

    double getTurnedAngle(geometry_msgs::Pose _curr_pose);
    geometry_msgs::Pose getTraveledDist(geometry_msgs::Pose _curr_pose);

    /**
      *@brief Setze die Ist-Position als ersten Bahn/-Starpunkt
      */
    void setcurrentJointPosition();

    void setcurrentTaskPosition();

    void checkPositionRange(double JointPos1, double JointPos2, double JointPos3, double JointPos4, double JointPos5);

    bool getOverhead(double _x, double _y);

    double *cartesianToJointInit, *cartesianToJointGoal;
    double q2End;
    double q1;

    /// Subscriber für Positionen und Geschwindigkeiten aus dem JointStates Topic
    ros::Subscriber jointPosVelSubscriber;
    ros::Subscriber baseSubscriber;
    /// Publisher für die Geschwindigkeit an das cmd_velocity Topic
    ros::Publisher jointVelocityPublisher;
    ros::Publisher velocityPublisher;
    ros::Publisher testPublisher;
    ros::Publisher basePublisher;

    ros::Publisher controllerPublisher;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    /// RosTimer
    ros::Timer timer;
    ros::Timer timerBase;
    ros::Timer BaseTimer;

    ros::NodeHandle n;
    std::string configFile;

    size_t DOF;
    size_t orientation;

    geometry_msgs::Pose	lastBasePose;
    geometry_msgs::Pose	startBasePose;
    geometry_msgs::Pose firstBasePose;
    geometry_msgs::Pose secondBasePose;

    double periode;
    double jnt[5];
    tf::Quaternion quatRot;

    std::vector <double> gripperPosition;
    std::vector <size_t> gripperPoint;

    /// Messagetyp der Joinstates
    brics_actuator::JointVelocities currentVelocities;
    brics_actuator::JointVelocities armJointVelocities;
    geometry_msgs::Twist baseVelocity;
    brics_actuator::JointPositions armJointPositions;
    brics_actuator::JointPositions regler;

    youbot_motion_control::motionControl motionController;

    geometry_msgs::Twist cmd_vel;

    /// Ziel und Startwerte im Taskspace (für CartBahnplanung benötigt)
    double *xEndeffector;

    COEFFS pitchCoeff, rollCoeff, jntTwoCoeff, timeRel;
    PARAM jntTwoParamInit, jntTwoParamGoal;
    AXIS singleAxis, singleWheel;

    /// Vektor mit absoluten maximalen Trajektorienzeiten aller Bahnsegmente
    std::vector<double> t_f_ppoint;
    vector <double> t_f_base;

    size_t timeIt;
    size_t timeBaseIt;

    /// Dieser Vektor beinhaltet die Gelenkwinkeldifferenz zweier aufeinanderfolgender Bahnpunkte aller Gelenkwinkel.
    std::vector <double> Diff;

    /// Vektor in Vektor mit den Gelenkwinkeldifferenzen aller Bahnpunkte einer Bahn.
    std::vector <std::vector<double> > DiffPath;

    /// Vektor beinhaltet Position, Geschwindigkeit und Beschleunigung zu jedem inkrementellen Zeitpunkt
    std::vector <struct axis> timeIncPoint;
    std::vector <struct axis> timeIncBase;
    std::vector <struct axis> timeIncBaseRot;
    std::vector <struct axis> timeIncBaseTrans;
    /// Vektor beinhaltet Geschwindigkeits- und Beschleunigungsinformation aller Joints eines Bahnpunkts
    std::vector<struct JointProperties> maxJointParamValue;

    /// Vektor beinhaltet Geschwindigkeits- und Beschleunigungsinformation aller Bahnpunkte
    std::vector <std::vector <PROPERTIES> > maxParamValue;

    /// Vektor mit maximalen Beschleunigungs- und Geschwindigkeitswerten für jeden Joint
    std::vector<PROPERTIES> initParamValues;

    /// Vektor mit Beschreibung eines Bahnpunkts für jeden Joint
    std::vector< PARAM > pointParam;

    std::vector<PARAM> degreeOfFreedom;

    /// Vektor in Vektor mit Beschreibung aller Bahnpunkte
    std::vector <std::vector <PARAM> > pathParam;
    PARAM baseX, baseY, basePhi;
    std::vector <PARAM> baseParam;

    /// Vektor, der die Polynomkoeffizienten a0...a5 eines Bahnsegments beinhaltet
    std::vector<COEFFS> coefficientsSegment;

    /// Vektor in Vektor, der die Polynomkoeffizienten a0...a5 aller Bahnsegmente beinhaltet
    std::vector <std::vector<COEFFS> > coefficients_ppoint;
    std::vector <COEFFS> baseCoeffs;

    /// At the beginning true. If Joint is out of range set false, so starting time is set only once
    bool ramp_started;
    bool positionIsReady;
    bool orientationIsReady;

    /// true if velocity is bigger than 0.1
    bool positiv;

    /// true if velocity ist lower than -0.1
    bool negativ;

    bool velocity_set;
    bool calculation_done;

    bool positionRange;
    bool onlyOrientation;

    size_t points;
    size_t pPoints;

    /// Positionsabweichung
    double *ePos, e;
    double *ePosSum, eSum;
    double *ePosAlt, eAlt;

    /// Time when the Jointboundaries are reached
    double ramp_start_time;
    double start_time;
    struct timeval detail_time;
    struct timeval new_time;
    struct timeval detail_ramp;

    /// Macros
    static const unsigned int JOINT1  = 0;
    static const unsigned int JOINT2  = 1;
    static const unsigned int JOINT3  = 2;
    static const unsigned int JOINT4  = 3;
    static const unsigned int JOINT5  = 4;
    static const unsigned int ARMJOINTS = 5;
    static const unsigned int TESTJOINT = 0;

    static const double MINANGLE1 = 0.576 * M_PI/180;
    static const double MINANGLE2 = 0.576 * M_PI/180;
    static const double MINANGLE3 = -288.0 * M_PI/180;
    static const double MINANGLE4 = 1.266 * M_PI/180;
    static const double MINANGLE5 = 6.337 * M_PI/180;
    static const double MINANGLEGRIPPER = 0.000 * M_PI/180;

    static const double MAXANGLE1 = 334.616 * M_PI/180;
    static const double MAXANGLE2 = 150.0 * M_PI/180;
    static const double MAXANGLE3 = -0.9 * M_PI/180;
    static const double MAXANGLE4 = 196.5 * M_PI/180;
    static const double MAXANGLE5 = 323.241 * M_PI/180;
    static const double MAXANGLEGRIPPER = 0.060 * M_PI/180;

    static const double VMAX1 = 1.3;
    static const double VMAX2 = 1.3;
    static const double VMAX3 = 1.3;
    static const double VMAX4 = 1.5;
    static const double VMAX5 = 1.5;

    static const double AMAX1 = 1.0;//1.343;
    static const double AMAX2 = 1.25;//1.343;
    static const double AMAX3 = 1.25;//2.094;
    static const double AMAX4 = 2.0;
    static const double AMAX5 = 2.0;

    static const double BASE_VMAX_X = 0.5;
    static const double BASE_VMAX_Y = 0.5;
    static const double BASE_VMAX_PHI = 0.5;

    static const double LOOPRATE = 0.01;

    static const size_t TASKSPACE = 6;
    static const size_t JOINTSPACE = 5;
    static const size_t TRANSLATION = 3;
    static const size_t ORIENTATION = 3;

    //Regler
    double GAIN;
    vector <double> windUP;
    vector <double> stellwert;
    vector <double> KP;
    vector <double> KI;
    vector <double> KD;
    double fehlerQ, emax, emin;

    /**
     * @brief für PnP <<< WAS ZUM GEIER IST DAS ???
     */
    /// Publisher für die Geschwindigkeit an das cmd_velocity Topic Base
    struct velocity{
        double velX;
        double velY;
    };

    struct params{
         double distX;
         double distX_FB;
         double distY;
         double distAbs;
         double distAbs_Front;
         double eeX;
         double eeX_Front;
         double eeX_Back;
         double eeY;
         double eeY_Front;
         double eeY_Back;
         double eeZ;
         double eeZ_Front;
         double eeZ_Back;
         double eeRoll;
         double eeRoll_Front;
         double eeRoll_Back;
         double eePitch;
         double eePitch_Front;
         double eePitch_Back;
         double eeYaw;
         double eeYaw_Front;
         double eeYaw_Back;
     };

    std::vector<struct velocity> timeIncPointBase;
    struct params poseParams;

    std::vector<double> OmegaQ0;
    std::vector<double> VelBaseY;
    std::vector<double> PositionJoint4;

    bool BaseCalculation_done;
    size_t tInk;
    double JointPoses[5];
    double vBaseY;
    double tArmMovement;
    double PoseJoint0;
    double PoseJoint4;
    double alpha;
    double DiffPosQ4;
    double vq0;
    double Joint5;
    double theta_soll;
    double beta;



};

#endif // MOTION_PLANNING_HPP
