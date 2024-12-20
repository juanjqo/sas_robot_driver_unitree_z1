#include "DriverUnitreeZ1.h"
#include "unitree_arm_sdk/control/unitreeArm.h"

class DriverUnitreeZ1::Impl
{
public:
    std::shared_ptr<UNITREE_ARM::unitreeArm> arm_;
    std::shared_ptr<UNITREE_ARM::Timer> unitree_timer_;
    std::shared_ptr<UNITREE_ARM::CtrlComponents> ctrlComp_;
    Impl()
    {

    };
};



DriverUnitreeZ1::DriverUnitreeZ1(std::atomic_bool *st_break_loops, const MODE &mode,
                                           const bool &gripper_attached,
                                           const bool& verbosity)
    :st_break_loops_{st_break_loops}, mode_{mode}, verbosity_{verbosity},
    gripper_attached_{gripper_attached}, finish_echo_robot_state_{false}
{

    impl_       = std::make_shared<DriverUnitreeZ1::Impl>();


    /*
    impl_->ctrlComp_    = std::make_shared<UNITREE_ARM::CtrlComponents>(0.004, gripper_attached);
    impl_->ctrlComp_->dt = 0.004;//500HZ                               //8071      //8072                                                //500000
    impl_->ctrlComp_->udp = new UNITREE_ARM::UDPPort("192.168.123.220", 8071, 8072);
    impl_->ctrlComp_->armModel = new UNITREE_ARM::Z1Model();// no UnitreeGripper
    impl_->ctrlComp_->armModel->addLoad(0.03);// add 0.03kg payload to the end joint
    impl_->arm_ = std::make_shared<UNITREE_ARM::unitreeArm>(impl_->ctrlComp_.get());// new unitreeArm(ctrlComp1);
    */

    impl_->arm_ = std::make_shared<UNITREE_ARM::unitreeArm>(gripper_attached);


    current_status_ = STATUS::IDLE;
    status_msg_ = std::string("Idle.");

    H_  = MatrixXd::Identity(6, 6);
    I_  = MatrixXd::Identity(6, 6);
    cm_ = std::make_unique<Capybara::ConstraintsManager>(6);
    solver_ = std::make_unique<DQ_PROXQPSolver>();
    // solver_     = std::make_unique<DQ_QPOASESSolver>();

    _set_driver_mode(mode);
}

std::tuple<VectorXd, VectorXd> DriverUnitreeZ1::_compute_control_inputs(VectorXd &q,
                                                                             const VectorXd &qtarget,
                                                                             const double &gain)
{
    double nq = 0.3;
    VectorXd f = 2*gain*(q-qtarget);
    MatrixXd Aeq;
    VectorXd beq;
    VectorXd q_dot_min = -q_dot_max_;
    cm_->add_inequality_constraint(-I_, -nq*(-(q-q_min_)));
    cm_->add_inequality_constraint( I_, -nq*( (q-q_max_)));
    cm_->add_inequality_constraint(-I_,  -q_dot_min);
    cm_->add_inequality_constraint( I_,   q_dot_max_);
    auto [A, b] = cm_->get_inequality_constraints();
    VectorXd u = solver_->solve_quadratic_program(H_, f, A, b, Aeq, beq);
    q = q + T_*u;
    return {q, u};
}

void DriverUnitreeZ1::_show_status()
{
    if (verbosity_)
        std::cerr<<status_msg_<<std::endl;
}

void DriverUnitreeZ1::_update_q_for_numerical_integration()
{
    for (int i=0;i<100;i++) // Updated the robot configuration
    {
        _update_robot_state();
        q_ni_ = q_measured_;
        q_gripper_ni_ = gripper_position_measured_;
        impl_->unitree_timer_->sleep();
    }
}

void DriverUnitreeZ1::_set_driver_mode(const MODE &mode)
{
    switch (mode){

    case MODE::None:
        mode_ = mode;
        break;
    case MODE::PositionControl:
        mode_ = mode;
        break;
    case MODE::VelocityControl:
        throw std::runtime_error("DriverUnitreeZ1::_set_driver_mode: VelocityControl is unsupported");
        break;
    case MODE::ForceControl:
        throw std::runtime_error("DriverUnitreeZ1::_set_driver_mode: ForceControl is unsupported");
        break;
    }
}

void DriverUnitreeZ1::_update_robot_state()
{
    q_measured_     = impl_->arm_->lowstate->getQ();
    q_dot_measured_ = impl_->arm_->lowstate->getQd();
    q_dot_dot_measured_ = impl_->arm_->lowstate->getQdd();
    tau_measured_  = impl_->arm_->lowstate->getTau();

    std::vector<int> temp = impl_->arm_->lowstate->temperature;
    motor_temperatures_ = Eigen::Map<VectorXi>(temp.data(), temp.size());

    gripper_position_measured_ = impl_->arm_->lowstate->getGripperQ();
    /*
     * 0x01 : phase current is too large
     * 0x02 : phase leakage
     * 0x04 : motor winding overheat or temperature is too large
     * 0x20 : parameters jump
     * 0x40 : Ignore
     */
    errorstate_ = impl_->arm_->lowstate->errorstate;
}

void DriverUnitreeZ1::_start_echo_robot_state_mode()
{
    while(!finish_echo_robot_state_ or !st_break_loops_)
    {
        _update_robot_state();
        impl_->unitree_timer_->sleep();
    }
    status_msg_ = "Echo robot state finished.";
    _show_status();
}


/**
 * @brief DriverUnitreeZ1::_start_echo_robot_state_mode_thread
 */
void DriverUnitreeZ1::_start_echo_robot_state_mode_thread()
{
    finish_echo_robot_state_ = false;
    if (echo_robot_state_mode_thread_.joinable())
    {
        echo_robot_state_mode_thread_.join();
    }
    echo_robot_state_mode_thread_ = std::thread(&DriverUnitreeZ1::_start_echo_robot_state_mode, this);
}

void DriverUnitreeZ1::_finish_echo_robot_state()
{
    status_msg_ = "Finishing echo robot state.";
    finish_echo_robot_state_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    _show_status();
}

void DriverUnitreeZ1::_start_joint_position_control_mode()
{
    _update_q_for_numerical_integration();
    target_joint_positions_ = q_ni_;
    target_gripper_position_ = q_gripper_ni_;
    VectorXd q_dot;
    double q_gripper_position_dot;
    double gain_gripper = 1.0;
    while(!finish_motion_ or !st_break_loops_)
    {
        _update_robot_state();
        auto results = _compute_control_inputs(q_ni_, target_joint_positions_, 5.0);
        q_ni_    = std::get<0>(results);
        q_dot = std::get<1>(results);

        impl_->arm_->q = q_ni_;
        impl_->arm_->qd = q_dot;

        impl_->arm_->setArmCmd(impl_->arm_->q, impl_->arm_->qd);

        if (gripper_attached_)
        {
            q_gripper_position_dot = -gain_gripper*(q_gripper_ni_-target_gripper_position_);
            q_gripper_ni_ = q_gripper_ni_ + T_*q_gripper_position_dot;
            impl_->arm_->setGripperCmd(q_gripper_ni_ , q_gripper_position_dot);
        }

        impl_->unitree_timer_->sleep();
    }
    status_msg_ = "Joint position control finished.";
    _show_status();

}

void DriverUnitreeZ1::_start_joint_position_control_thread()
{
    finish_motion_ = false;
    if (joint_position_control_mode_thread_.joinable())
    {
        joint_position_control_mode_thread_.join();
    }
    joint_position_control_mode_thread_ = std::thread(&DriverUnitreeZ1::_start_joint_position_control_mode, this);
    status_msg_ = "Starting joint position control.";
    _show_status();
}


/**
 * @brief DriverUnitreeZ1::connect
 */
void DriverUnitreeZ1::connect()
{
    if (current_status_ == STATUS::IDLE)
    {
        //std::cout<<impl_->z1_controller_->_get_config_path()<<std::endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));

        impl_->arm_->sendRecvThread->start();
        T_ = impl_->arm_->_ctrlComp->dt;

        if (!impl_->unitree_timer_)
            impl_->unitree_timer_ = std::make_shared<UNITREE_ARM::Timer>(T_);

        _start_echo_robot_state_mode_thread();
        current_status_ = STATUS::CONNECTED;
        status_msg_ = "connected!";
        _show_status();
    }
}


/**
 * @brief DriverUnitreeZ1::initialize This method starts the corresponding threads for kinematic control.
 *                  WARNING: The robot will move to the forward position!!!!!!
 */
void DriverUnitreeZ1::initialize()
{
    if (current_status_ == STATUS::CONNECTED)
    {
        impl_->arm_->backToStart();
        impl_->arm_->startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL);

        std::cout<<"Reading robot configuration..."<<std::endl;
        _update_q_for_numerical_integration();


        initial_robot_configuration_ = q_ni_;

        if (move_robot_to_forward_position_when_initialized_)
        {
            std::cout<<"Setting forward robot configuration..."<<std::endl;
            _move_robot_to_target_joint_positions(Forward_, 0.15, st_break_loops_);
        }


        switch (mode_) {
        case MODE::None:
            break;
        case MODE::PositionControl:
            _finish_echo_robot_state();
            _start_joint_position_control_thread();
            break;
        case MODE::VelocityControl:
            break;
        case MODE::ForceControl:
            break;
        }
        current_status_ = STATUS::INITIALIZED;
        status_msg_ = "initialized!";
        _show_status();
    }
}

void DriverUnitreeZ1::deinitialize()
{

    _finish_motion();

    while(!finish_motion_){
        std::cout<<"Waiting..."<<std::endl;
    }; //wait to break the joint control thread

    // Force this loop to keep enabled to move the robot to its initial configuration
    std::atomic_bool flag = false;
    std::cout<<"Setting home robot configuration..."<<std::endl;
    _move_robot_to_target_joint_positions(initial_robot_configuration_, 0.15, &flag);

    impl_->arm_->backToStart();
    impl_->arm_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    impl_->arm_->sendRecvThread->shutdown();
    status_msg_ = "Deinitialized.";
    current_status_ = STATUS::DEINITIALIZED;
    _show_status();
}

void DriverUnitreeZ1::disconnect()
{
    status_msg_ = "Disconnecting...";
    _finish_echo_robot_state();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  if (joint_position_control_mode_thread_.joinable())
         joint_position_control_mode_thread_.join();

    if (echo_robot_state_mode_thread_.joinable())
        echo_robot_state_mode_thread_.join();

    status_msg_ = "Disconnected.";
    current_status_ = STATUS::DISCONNECTED;
    _show_status();
    //impl_->z1_controller_->stop();
}

void DriverUnitreeZ1::set_target_gripper_position(const double &target_gripper_position)
{
    target_gripper_position_ = target_gripper_position;
}

void DriverUnitreeZ1::_finish_motion()
{
    std::cout<<"Ending control loop..."<<std::endl;
    for (int i=0;i<10;i++)
    {
        set_target_joint_positions(q_ni_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    finish_motion_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void DriverUnitreeZ1::_move_robot_to_target_joint_positions(const VectorXd &q_target, const double &gain, std::atomic_bool *break_loop)
{
    _update_q_for_numerical_integration();
    VectorXd q_dot;
    while ( ((q_ni_-q_target).norm() > 0.01) and !(*break_loop))
    {
        _update_robot_state();
        auto results = _compute_control_inputs(q_ni_, q_target, gain);
        q_ni_     = std::get<0>(results);
        q_dot = std::get<1>(results);

        impl_->arm_->q = q_ni_;
        impl_->arm_->qd = q_dot;

        impl_->arm_->setArmCmd(impl_->arm_->q, impl_->arm_->qd);

        impl_->unitree_timer_->sleep();
    }
}

VectorXd DriverUnitreeZ1::get_joint_positions()
{
    return q_measured_;
}

VectorXd DriverUnitreeZ1::get_joint_velocities()
{
    return q_dot_measured_;
}

VectorXd DriverUnitreeZ1::get_joint_forces()
{
    return tau_measured_;
}

double DriverUnitreeZ1::get_gripper_position()
{
    return gripper_position_measured_;
}

void DriverUnitreeZ1::move_robot_to_forward_position_when_initialized(const bool &flag)
{
    move_robot_to_forward_position_when_initialized_ = flag;
}

void DriverUnitreeZ1::move_robot_to_target_joint_positions(const VectorXd &q_target)
{
    if (current_status_ == STATUS::INITIALIZED && mode_ == MODE::None)
    {
        _move_robot_to_target_joint_positions(q_target, 0.3, st_break_loops_);
    }else if( mode_ == MODE::PositionControl && current_status_ == STATUS::CONNECTED)
    {
        _move_robot_to_target_joint_positions(q_target, 0.3, st_break_loops_);
    }
    else
    {
        std::cerr<<"This method is enabled for MODE::None and STATUS::INITIALIZED"<<std::endl;
        std::cerr<<"Or MODE::PositionControl and STATUS::CONNECTED."<<std::endl;
    }
}

void DriverUnitreeZ1::set_target_joint_positions(const VectorXd &target_joint_positions_rad)
{
    target_joint_positions_ = target_joint_positions_rad;
}

void DriverUnitreeZ1::set_gripper_position(const double &gripper_position)
{
    target_gripper_position_ = gripper_position;
}

void DriverUnitreeZ1::set_target_joint_positions_with_gripper(const VectorXd &target_joint_positions_with_gripper_rad)
{
    set_target_joint_positions(target_joint_positions_with_gripper_rad.head(6));
    set_gripper_position(target_joint_positions_with_gripper_rad(6));
}

VectorXd DriverUnitreeZ1::get_joint_positions_with_gripper()
{
    VectorXd joint_positions_with_gripper = VectorXd::Zero(7);
    joint_positions_with_gripper << q_measured_(0), q_measured_(1), q_measured_(2), q_measured_(3), q_measured_(4), q_measured_(5), gripper_position_measured_;
    return joint_positions_with_gripper;
}


