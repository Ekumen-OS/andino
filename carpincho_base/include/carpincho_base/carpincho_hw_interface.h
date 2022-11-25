#ifndef CARPINCHO_HW_INTERFACE_H
#define CARPINCHO_HW_INTERFACE_H

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace carpincho_base
{
    const unsigned int NUM_JOINTS = 2;

    struct JointState
    {
        float angular_position_;
        float angular_velocity_;
    };

    /// \brief Hardware interface for a robot
    class CarpinchoHWInterface : public hardware_interface::RobotHW
    {
    public:
        /**
         * \brief Constructor
         * \param nh - Node handle for topics.
         * \param urdf_model - optional pointer to a parsed robot model
         */
        CarpinchoHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

        /** \brief Destructor */
        virtual ~CarpinchoHWInterface() {}

        /** \brief The init function is called to initialize the RobotHW from a
         *         non-realtime thread.
         *
         * Initialising a custom robot is done by registering joint handles
         * (\ref hardware_interface::ResourceManager::registerHandle) to hardware
         * interfaces that group similar joints and registering those individual
         * hardware interfaces with the class that represents the custom robot
         * (derived from this \ref hardware_interface::RobotHW)
         *
         * \note Registering of joint handles and interfaces can either be done in the
         * constructor or this \ref init method.
         *
         * \param root_nh A NodeHandle in the root of the caller namespace.
         *
         * \param robot_hw_nh A NodeHandle in the namespace from which the RobotHW
         * should read its configuration.
         *
         * \returns True if initialization was successful
         */
        virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);

        /** \brief Read data from the robot hardware.
         *
         * The read method is part of the control loop cycle (\ref read, update, \ref write)
         * and is used to populate the robot state from the robot's hardware resources
         * (joints, sensors, actuators). This method should be called before
         * controller_manager::ControllerManager::update() and \ref write.
         *
         * \note The name \ref read refers to reading state from the hardware.
         * This complements \ref write, which refers to writing commands to the hardware.
         *
         * Querying WallTime inside \ref read is not realtime safe. The parameters
         * \p time and \p period make it possible to inject time from a realtime source.
         *
         * \param time The current time
         * \param period The time passed since the last call to \ref read
         */
        virtual void read(const ros::Time& time, const ros::Duration& period) override;

        /** \brief Write commands to the robot hardware.
         *
         * The write method is part of the control loop cycle (\ref read, update, \ref write)
         * and is used to send out commands to the robot's hardware
         * resources (joints, actuators). This method should be called after
         * \ref read and controller_manager::ControllerManager::update.
         *
         * \note The name \ref write refers to writing commands to the hardware.
         * This complements \ref read, which refers to reading state from the hardware.
         *
         * Querying WallTime inside \ref write is not realtime safe. The parameters
         * \p time and \p period make it possible to inject time from a realtime source.
         *
         * \param time The current time
         * \param period The time passed since the last call to \ref write
         */
        virtual void write(const ros::Time& time, const ros::Duration& period);

        /** \brief Check if measured joint states (position and velocity) are received.
         *
         * This function blocks until the sub_measured_joint_states_ subscriber receives measured joint states
         * (angular position (rad) and angular velocity (rad/s)) for each joint on the "measured_joint_states" topic
         * of type sensor_msgs::JointState.
         *
         * \param timeout Minimum time to wait for receiving encoder ticks
         */
        bool isReceivingMeasuredJointStates(const ros::Duration &timeout=ros::Duration(1));

    protected:

        /** \brief Get the URDF XML from the parameter server */
        virtual void loadURDF(const ros::NodeHandle& nh, std::string param_name);

        /** \brief Callback to receive the measured joint states from Teensy MCU */
        void measuredJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg_joint_states);

        // The following functions are currently unused
        // Carpincho directly calculates normalized angles from encoder ticks
        // The joint_commands from ros_control are mapped to percentage values for the motor driver
        // The following comments are incorrect
        /** \brief Carpincho reports travel distance in metres, need radians for ros_control RobotHW */
        // double linearToAngular(const double &distance) const;

        // Short name of this class
        std::string name_;

        // Startup and shutdown of the internal node inside a roscpp program
        ros::NodeHandle nh_;

        // Hardware interfaces
        // hardware_interface::JointStateInterface gives read access to all joint values
        // without conflicting with other controllers.
        hardware_interface::JointStateInterface joint_state_interface_;
        // hardware_interface::VelocityJointInterface inherits from
        // hardware_interface::JointCommandInterface and is used for reading and writing
        // joint velocities. Because this interface reserves the joints for write access,
        // conflicts with other controllers writing to the same joints might occure.
        // To only read joint velocities, avoid conflicts using
        // hardware_interface::JointStateInterface.
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        // Configuration
        std::vector<std::string> joint_names_;
        std::size_t num_joints_;
        urdf::Model *urdf_model_;

        // Data member array to store the controller commands which are sent to the
        // robot's resources (joints, actuators)
        // The diff_drive_controller uses the hardware_interface::VelocityJointInterface
        // It provides semantic meaning to the wheel joints describing that
        // they require velocity commands.
        double joint_velocity_commands_[NUM_JOINTS];

        // Data member arrays to store the state of the robot's resources (joints, sensors)
        // These values are filled in the read() method and are registered to the 
        // joint_state_interface_ of type hardware_interface::JointStateInterface.
        double joint_positions_[NUM_JOINTS];
        double joint_velocities_[NUM_JOINTS];
        double joint_efforts_[NUM_JOINTS];

        ros::ServiceServer srv_start_;
        ros::ServiceServer srv_stop_;

        // Declare publishers for the motor driver
        ros::Publisher pub_left_motor_value_;
        ros::Publisher pub_right_motor_value_;

        // Declare subscriber for the measured joint states (absolute angular position (rad) and angular velocity (rad/s) of the wheel)
        ros::Subscriber sub_measured_joint_states_;

        JointState measured_joint_states_[NUM_JOINTS];

    };  // class CarpinchoHWInterface

}  // namespace

#endif // CARPINCHO_HW_INTERFACE_H
