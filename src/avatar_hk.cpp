#include "avatar.h"
#include <fstream>
using namespace TOCABI;

AvatarController::AvatarController(RobotData &rd) : rd_(rd)
{
    nh_avatar_.setCallbackQueue(&queue_avatar_);

    walking_slider_command = nh_avatar_.subscribe("/tocabi/dg/walkingslidercommand", 100, &AvatarController::WalkingSliderCommandCallback, this);
    upperbodymode_sub = nh_avatar_.subscribe("/tocabi/avatar/upperbodymodecommand", 100, &AvatarController::UpperbodyModeCallback, this);
    nextswingleg_sub = nh_avatar_.subscribe("/tocabi/dg/nextswinglegcommand", 100, &AvatarController::NextSwinglegCallback, this);
    com_walking_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/compospdgain", 100, &AvatarController::ComPosGainCallback, this);
    pelv_ori_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/pelvoripdgain", 100, &AvatarController::PelvOriGainCallback, this);
    support_foot_damping_gain_sub = nh_avatar_.subscribe("/tocabi/dg/supportfootdampinggain", 100, &AvatarController::SupportFootDampingGainCallback, this);
    dg_leg_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/legpdgain", 100, &AvatarController::LegJointGainCallback, this);
    alpha_x_sub = nh_avatar_.subscribe("/tocabi/dg/alpha_x", 100, &AvatarController::AlphaXCallback, this);
    alpha_y_sub = nh_avatar_.subscribe("/tocabi/dg/alpha_y", 100, &AvatarController::AlphaYCallback, this);
    step_width_sub = nh_avatar_.subscribe("/tocabi/dg/stepwidthcommand", 100, &AvatarController::StepWidthCommandCallback, this);
    test1_sub = nh_avatar_.subscribe("/tocabi/dg/test1command", 100, &AvatarController::Test1CommandCallback, this);
    test2_sub = nh_avatar_.subscribe("/tocabi/dg/test2command", 100, &AvatarController::Test2CommandCallback, this);
    arm_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/armpdgain", 100, &AvatarController::ArmJointGainCallback, this);
    waist_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/waistpdgain", 100, &AvatarController::WaistJointGainCallback, this);
    mujoco_ext_force_apply_pub = nh_avatar_.advertise<std_msgs::Float32MultiArray>("/tocabi_avatar/applied_ext_force", 10);
    mujoco_applied_ext_force_.data.resize(7);

    //opto_ftsensor_sub = nh_avatar_.subscribe("/atiforce/ftsensor", 100, &AvatarController::OptoforceFTCallback, this); // real robot experiment

    setGains();
    first_loop_larm_ = true;
    first_loop_rarm_ = true;
    first_loop_upperbody_ = true;
    first_loop_hqpik_ = true;
    first_loop_hqpik2_ = true;
    first_loop_qp_retargeting_ = true;
    first_loop_camhqp_ = true;

}

void AvatarController::Joint_gain_set_MJ()
{
    //simulation gains
    Kp(0) = 1800.0;
    Kd(0) = 70.0; // Left Hip yaw
    Kp(1) = 2100.0;
    Kd(1) = 90.0; // Left Hip roll
    Kp(2) = 2100.0;
    Kd(2) = 90.0; // Left Hip pitch
    Kp(3) = 2100.0;
    Kd(3) = 90.0; // Left Knee pitch
    Kp(4) = 2100.0;
    Kd(4) = 90.0; // Left Ankle pitch
    Kp(5) = 2100.0;
    Kd(5) = 90.0; // Left Ankle roll

    Kp(6) = 1800.0;
    Kd(6) = 70.0; // Right Hip yaw
    Kp(7) = 2100.0;
    Kd(7) = 90.0; // Right Hip roll
    Kp(8) = 2100.0;
    Kd(8) = 90.0; // Right Hip pitch
    Kp(9) = 2100.0;
    Kd(9) = 90.0; // Right Knee pitch
    Kp(10) = 2100.0;
    Kd(10) = 90.0; // Right Ankle pitch
    Kp(11) = 2100.0;
    Kd(11) = 90.0; // Right Ankle roll

    Kp(12) = 2200.0;
    Kd(12) = 90.0; // Waist yaw
    Kp(13) = 2200.0;
    Kd(13) = 90.0; // Waist pitch
    Kp(14) = 2200.0;
    Kd(14) = 90.0; // Waist roll

    Kp(15) = 400.0;
    Kd(15) = 10.0;
    Kp(16) = 800.0;
    Kd(16) = 10.0;
    Kp(17) = 400.0;
    Kd(17) = 10.0;
    Kp(18) = 400.0;
    Kd(18) = 10.0;
    Kp(19) = 250.0;
    Kd(19) = 2.5;
    Kp(20) = 250.0;
    Kd(20) = 2.0;
    Kp(21) = 50.0;
    Kd(21) = 2.0; // Left Wrist
    Kp(22) = 50.0;
    Kd(22) = 2.0; // Left Wrist

    Kp(23) = 50.0;
    Kd(23) = 2.0; // Neck
    Kp(24) = 50.0;
    Kd(24) = 2.0; // Neck

    Kp(25) = 400.0;
    Kd(25) = 10.0;
    Kp(26) = 800.0;
    Kd(26) = 10.0;
    Kp(27) = 400.0;
    Kd(27) = 10.0;
    Kp(28) = 400.0;
    Kd(28) = 10.0;
    Kp(29) = 250.0;
    Kd(29) = 2.5;
    Kp(30) = 250.0;
    Kd(30) = 2.0;
    Kp(31) = 50.0;
    Kd(31) = 2.0; // Right Wrist
    Kp(32) = 50.0;
    Kd(32) = 2.0; // Right Wrist
    //real robot experiment
    // Kp(0) = 2000.0;
    // Kd(0) = 20.0; // Left Hip yaw
    // Kp(1) = 5000.0;
    // Kd(1) = 55.0; // Left Hip roll //55
    // Kp(2) = 4000.0;
    // Kd(2) = 45.0; // Left Hip pitch
    // Kp(3) = 3700.0;
    // Kd(3) = 40.0; // Left Knee pitch
    // Kp(4) = 4000.0; // 5000
    // Kd(4) = 65.0; // Left Ankle pitch /5000 / 30  //55
    // Kp(5) = 4000.0; // 5000
    // Kd(5) = 65.0; // Left Ankle roll /5000 / 30 //55

    // Kp(6) = 2000.0;
    // Kd(6) = 20.0; // Right Hip yaw
    // Kp(7) = 5000.0;
    // Kd(7) = 55.0; // Right Hip roll  //55
    // Kp(8) = 4000.0;
    // Kd(8) = 45.0; // Right Hip pitch
    // Kp(9) = 3700.0;
    // Kd(9) = 40.0; // Right Knee pitch
    // Kp(10) = 4000.0; // 5000
    // Kd(10) = 65.0; // Right Ankle pitch //55
    // Kp(11) = 4000.0; // 5000
    // Kd(11) = 65.0; // Right Ankle roll //55

    // Kp(12) = 6000.0;
    // Kd(12) = 200.0; // Waist yaw
    // Kp(13) = 10000.0;
    // Kd(13) = 100.0; // Waist pitch
    // Kp(14) = 10000.0;
    // Kd(14) = 100.0; // Waist roll

    // Kp(15) = 400.0;
    // Kd(15) = 10.0;
    // Kp(16) = 800.0;
    // Kd(16) = 10.0;
    // Kp(17) = 400.0;
    // Kd(17) = 10.0;
    // Kp(18) = 400.0;
    // Kd(18) = 10.0;
    // Kp(19) = 250.0;
    // Kd(19) = 2.5;
    // Kp(20) = 250.0;
    // Kd(20) = 2.0;
    // Kp(21) = 50.0;
    // Kd(21) = 2.0; // Left Wrist
    // Kp(22) = 50.0;
    // Kd(22) = 2.0; // Left Wrist

    // Kp(23) = 50.0;
    // Kd(23) = 2.0; // Neck
    // Kp(24) = 50.0;
    // Kd(24) = 2.0; // Neck

    // Kp(25) = 400.0;
    // Kd(25) = 10.0;
    // Kp(26) = 800.0;
    // Kd(26) = 10.0;
    // Kp(27) = 400.0;
    // Kd(27) = 10.0;
    // Kp(28) = 400.0;
    // Kd(28) = 10.0;
    // Kp(29) = 250.0;
    // Kd(29) = 2.5;
    // Kp(30) = 250.0;
    // Kd(30) = 2.0;
    // Kp(31) = 50.0;
    // Kd(31) = 2.0; // Right Wrist
    // Kp(32) = 50.0;
    // Kd(32) = 2.0; // Right Wrist
}
void AvatarController::parameterSetting()
{       
    target_x_ = 0.0;
    target_y_ = 0.0;
    target_z_ = 0.0;
    com_height_ = 0.71;
    target_theta_ = 0.0;
    step_length_x_ = 0.10;
    step_length_y_ = 0.0;
    is_right_foot_swing_ = 1;

    // t_rest_init_ = 0.27*hz_;
    // t_rest_last_ = 0.27*hz_;
    // t_double1_ = 0.03*hz_;
    // t_double2_ = 0.03*hz_;
    // t_total_= 1.3*hz_;

    t_rest_init_ = 0.12 * hz_; // Slack, 0.9 step time
    t_rest_last_ = 0.12 * hz_;
    t_double1_ = 0.03 * hz_;
    t_double2_ = 0.03 * hz_;
    t_total_ = 0.9 * hz_;
    t_total_const_ = 0.9 * hz_; 

    // t_rest_init_ = 0.02 * hz_; // Slack, 0.9 step time
    // t_rest_last_ = 0.02 * hz_;
    // t_double1_ = 0.03 * hz_;
    // t_double2_ = 0.03 * hz_;
    // t_total_ = 0.5 * hz_;

    // t_rest_init_ = 0.08 * hz_; // slack
    // t_rest_last_ = 0.08 * hz_;
    // t_double1_ = 0.03 * hz_;
    // t_double2_ = 0.03 * hz_;
    // t_total_ = 0.7 * hz_;

    // t_rest_init_ = 0.2 * hz_; // slack
    // t_rest_last_ = 0.2 * hz_;
    // t_double1_ = 0.03 * hz_;
    // t_double2_ = 0.03 * hz_;
    // t_total_ = 1.1 * hz_;

    t_temp_ = 2.0 * hz_;
    t_last_ = t_total_ + t_temp_;
    t_start_ = t_temp_ + 1;
    t_start_real_ = t_start_ + t_rest_init_;

    current_step_num_ = 0;
    foot_height_ = 0.055;      // 0.9 sec 0.05
    pelv_height_offset_ = 0.0; // change pelvis height for manipulation when the robot stop walking
}
void AvatarController::updateInitialState()
{
    if (walking_tick_mj == 0)
    {
        //calculateFootStepTotal();
        calculateFootStepTotal_MJ();

        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);
        //pelv_float_init_.translation()(0) += 0.11;

        pelv_float_init_.translation()(0) = 0;
        pelv_float_init_.translation()(1) = 0;

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        lfoot_float_init_.translation()(0) = 0;
        lfoot_float_init_.translation()(1) = 0.1225;

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        rfoot_float_init_.translation()(0) = 0;
        rfoot_float_init_.translation()(1) = -0.1225;

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        com_float_init_(0) = 0;
        com_float_init_(1) = 0;

        if (aa == 0)
        {
            lfoot_float_init_.translation()(1) = 0.1025;
            rfoot_float_init_.translation()(1) = -0.1025;
            aa = 1;
        }
        cout << "First " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_mj_(2) * 180 / 3.141592 << endl;

        Eigen::Isometry3d ref_frame;

        if (foot_step_(0, 6) == 0) //right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(0, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), rfoot_float_init_);
        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);

        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

        supportfoot_float_init_.setZero();
        swingfoot_float_init_.setZero();

        if (foot_step_(0, 6) == 1) //left suppport foot
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }
        else
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }

        pelv_support_start_ = pelv_support_init_;
        // cout<<"pelv_support_start_.translation()(2): "<<pelv_support_start_.translation()(2);
        total_step_num_ = foot_step_.col(1).size();

        xi_mj_ = com_support_init_(0); // preview parameter
        yi_mj_ = com_support_init_(1);
        zc_mj_ = com_support_init_(2);
    }
    else if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {
        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        rfoot_rpy_current_.setZero();
        lfoot_rpy_current_.setZero();
        rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
        lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

        rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
        lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
        rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
        lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
        // lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
        // rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        Eigen::Isometry3d ref_frame;

        if (foot_step_(current_step_num_, 6) == 0) //right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        //////dg edit
        Eigen::Isometry3d ref_frame_yaw_only;
        ref_frame_yaw_only.translation() = ref_frame.translation();
        Eigen::Vector3d ref_frame_rpy;
        ref_frame_rpy = DyrosMath::rot2Euler(ref_frame.linear());
        ref_frame_yaw_only.linear() = DyrosMath::rotateWithZ(ref_frame_rpy(2));

        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame_yaw_only) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), com_float_init_);
        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), rfoot_float_init_);
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
        ///////////////

        // pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
        // com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);
        // pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

        // lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), lfoot_float_init_);
        // rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), rfoot_float_init_);
        // rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        // lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
    }
}
void AvatarController::getRobotState()
{
    pelv_rpy_current_mj_.setZero();
    pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

    R_angle = pelv_rpy_current_mj_(0);
    P_angle = pelv_rpy_current_mj_(1);
    pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

    rfoot_rpy_current_.setZero();
    lfoot_rpy_current_.setZero();
    rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
    lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

    rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
    lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
    rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
    lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

    pelv_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

    pelv_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);

    lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
    // lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm;
    lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

    rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
    // rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm;
    rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

    com_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치
    com_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].v);

    if (walking_tick_mj == 0)
    {
        com_float_current_dot_LPF = com_float_current_dot;
        com_float_current_dot_prev = com_float_current_dot;
    }

    com_float_current_dot_prev = com_float_current_dot;
    com_float_current_dot_LPF = 1 / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot_LPF + (2 * M_PI * 3.0 * del_t) / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot;
    // modified cut off freq. of CP error for joe's MPC 
    if (walking_tick_mj == 0)
    {
        com_float_current_LPF = com_float_current_;
    }

    com_float_current_LPF = 1 / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_LPF + (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_;

    double support_foot_flag = foot_step_(current_step_num_, 6);
    if (support_foot_flag == 0)
    {
        supportfoot_float_current_ = rfoot_float_current_;
    }
    else if (support_foot_flag == 1)
    {
        supportfoot_float_current_ = lfoot_float_current_;
    }

    ///////////dg edit
    Eigen::Isometry3d supportfoot_float_current_yaw_only;
    supportfoot_float_current_yaw_only.translation() = supportfoot_float_current_.translation();
    Eigen::Vector3d support_foot_current_rpy;
    support_foot_current_rpy = DyrosMath::rot2Euler(supportfoot_float_current_.linear());
    supportfoot_float_current_yaw_only.linear() = DyrosMath::rotateWithZ(support_foot_current_rpy(2));

    pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * pelv_float_current_;
    lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * lfoot_float_current_;
    rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * rfoot_float_current_;
    ////////////////////

    // pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * pelv_float_current_;
    // lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * lfoot_float_current_;
    // rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * rfoot_float_current_;

    //cout << "L : " << lfoot_float_current_.linear() << "\n" <<  "R : " << rfoot_float_current_.linear() << endl;
    com_support_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_);
    com_support_current_dot_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot);
    com_support_current_dot_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot_LPF);

    SC_err_compen(com_support_current_(0), com_support_current_(1));
    
    cp_measured_(0) = com_support_cp_(0) + com_float_current_dot_LPF(0) / wn;
    cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1) / wn;
 
    // l_ft : generated force by robot
    l_ft_ = rd_.LF_FT;
    r_ft_ = rd_.RF_FT;

    if (walking_tick_mj == 0)
    {
        l_ft_LPF = l_ft_;
        r_ft_LPF = r_ft_;
    }

    l_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_;
    r_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_;
 
    Eigen::Vector2d left_zmp, right_zmp;

    left_zmp(0) = l_ft_LPF(4) / l_ft_LPF(2) + lfoot_support_current_.translation()(0);
    left_zmp(1) = l_ft_LPF(3) / l_ft_LPF(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = r_ft_LPF(4) / r_ft_LPF(2) + rfoot_support_current_.translation()(0);
    right_zmp(1) = r_ft_LPF(3) / r_ft_LPF(2) + rfoot_support_current_.translation()(1);

    zmp_measured_mj_(0) = (left_zmp(0) * l_ft_LPF(2) + right_zmp(0) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP X
    zmp_measured_mj_(1) = (left_zmp(1) * l_ft_LPF(2) + right_zmp(1) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP Y
    // MJ_graph << cp_measured_(0) << "," << cp_measured_(1) << endl;
    wn = sqrt(GRAVITY / zc_mj_);

    // real robot experiment
    // opto_ft_ = opto_ft_raw_; 
    // //cout << opto_ft_(0) << "," << opto_ft_(1) << "," << opto_ft_(2) << endl; 
    // MJ_opto <<  opto_ft_(0) << "," << opto_ft_(1) << "," << opto_ft_(2) << "," << opto_ft_(3) << "," << opto_ft_(4) << "," << opto_ft_(5) << endl; 
    
    if (walking_tick_mj == 0)
    {
        zmp_measured_LPF_.setZero();
    } 
    zmp_measured_LPF_ = (2 * M_PI * 2.0 * del_t) / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_mj_ + 1 / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_LPF_;
    // MJ_graph2 << ZMP_X_REF_ << "," << ZMP_Y_REF_ << "," << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << "," << zmp_measured_LPF_(0) << "," << zmp_measured_LPF_(1) << endl;
}
void AvatarController::floatToSupportFootstep()
{
    Eigen::Isometry3d reference;

    if (current_step_num_ == 0)
    {
        if (foot_step_(0, 6) == 0)
        {
            reference.translation() = rfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
        else
        {
            reference.translation() = lfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
    }
    else
    {
        reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_ - 1, 5));
        for (int i = 0; i < 3; i++)
        {
            reference.translation()(i) = foot_step_(current_step_num_ - 1, i);
        }
    }

    Eigen::Vector3d temp_local_position;
    Eigen::Vector3d temp_global_position;

    for (int i = 0; i < total_step_num_; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            temp_global_position(j) = foot_step_(i, j);
        }

        temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

        for (int j = 0; j < 3; j++)
        {
            foot_step_support_frame_(i, j) = temp_local_position(j);
        }

        foot_step_support_frame_(i, 3) = foot_step_(i, 3);
        foot_step_support_frame_(i, 4) = foot_step_(i, 4);
        if (current_step_num_ == 0)
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - supportfoot_float_init_(5);
        }
        else
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - foot_step_(current_step_num_ - 1, 5);
        }
    }

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = swingfoot_float_init_(j); // swingfoot_float_init_은 Pelvis에서 본 Swing 발의 Position, orientation.

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        swingfoot_support_init_(j) = temp_local_position(j);

    swingfoot_support_init_(3) = swingfoot_float_init_(3);
    swingfoot_support_init_(4) = swingfoot_float_init_(4);

    if (current_step_num_ == 0)
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - supportfoot_float_init_(5);
    else
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = supportfoot_float_init_(j);

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        supportfoot_support_init_(j) = temp_local_position(j);

    supportfoot_support_init_(3) = supportfoot_float_init_(3);
    supportfoot_support_init_(4) = supportfoot_float_init_(4);

    if (current_step_num_ == 0)
        supportfoot_support_init_(5) = 0;
    else
        supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);
}
void AvatarController::getZmpTrajectory()
{
    unsigned int planning_step_number = 5;
    unsigned int norm_size = 0;

    if (current_step_num_ >= total_step_num_ - planning_step_number)
    {
        norm_size = t_total_const_ * (total_step_num_ - current_step_num_) + 4.0 * hz_;        
    }
    else
    {
        norm_size = t_total_const_ * planning_step_number + 1.0 * hz_;
        // norm_size = (t_last_ - t_start_ + 1) * (planning_step_number) + 1.0 * hz_;
    }         
    // t_total_ * planning step_num인데 t_total_이 현재스텝에서 0.7이면 norm size가 3.5s가 되어서 0.7 + 0.9 *(planning_step_num -1) size보다 작아서 터졌었음.
//    cout << total_step_num_ << "," << planning_step_number << "," << current_step_num_ << endl;
    if (current_step_num_ == 0)
    {
        norm_size = norm_size + t_temp_ + 1;
    }        
    
    addZmpOffset(); 
    zmpGenerator(norm_size, planning_step_number);

    // static int aa = 0;
    // if(walking_tick_mj >= 4.3*2000 && aa == 0)
    // {
    //     aa = 1;

    //     for(int i = 0; i < 5000; i ++)
    //     {        
    //         MJ_graph << ref_zmp_mj_wo_offset_(i, 1) << endl;            
    //     }            
    // }

    ref_zmp_wo_offset_mpc_.resize(norm_size, 2);
    ref_zmp_mpc_.resize(norm_size, 2);
}

//중요한 함수들..
void AvatarController::getComTrajectory_mpc()
{
    if (walking_tick_mj == 0)
    {       
        U_x_mpc_.setZero(135);
        U_y_mpc_.setZero(135);

        x_hat_.setZero();
        x_hat_(0) = xi_mj_;
        y_hat_.setZero();
        y_hat_(0) = yi_mj_;

        x_hat_thread_.setZero();
        x_hat_thread_(0) = xi_mj_;
        y_hat_thread_.setZero();
        y_hat_thread_(0) = yi_mj_;
        x_hat_p_thread_.setZero();
        x_hat_p_thread_(0) = xi_mj_;
        y_hat_p_thread_.setZero();
        y_hat_p_thread_(0) = yi_mj_;

        x_hat_thread2_.setZero();
        x_hat_thread2_(0) = xi_mj_;
        y_hat_thread2_.setZero();
        y_hat_thread2_(0) = yi_mj_;
        x_hat_p_thread2_.setZero();
        x_hat_p_thread2_(0) = xi_mj_;
        y_hat_p_thread2_.setZero();
        y_hat_p_thread2_(0) = yi_mj_;

        x_hat_r_.setZero();
        x_hat_r_p_.setZero();
        x_hat_r_(0) = xi_mj_;
        x_hat_r_p_(0) = xi_mj_;
        x_mpc_i_(0) = xi_mj_;

        y_hat_r_.setZero();
        y_hat_r_p_.setZero();
        y_hat_r_(0) = yi_mj_;
        y_hat_r_p_(0) = yi_mj_;
        y_mpc_i_(0) = yi_mj_;    

        cp_des_zmp_x_prev_ = xi_mj_;    
        cp_des_zmp_x_ = xi_mj_;
        
        cpmpc_des_zmp_x_thread_ = xi_mj_;
        cpmpc_des_zmp_x_thread2_ = xi_mj_;

        cp_des_zmp_y_prev_ = yi_mj_;    
        cp_des_zmp_y_ = yi_mj_;
        
        cpmpc_des_zmp_y_thread_ = yi_mj_;
        cpmpc_des_zmp_y_thread2_ = yi_mj_;

        cp_measured_thread_(0) = xi_mj_;
        cp_measured_thread_(1) = yi_mj_;

        des_zmp_interpol_.setZero();
        cpmpc_diff_.setZero();
        cpStepping_diff_.setZero();        
    }

    if (current_step_num_ == 0)
    {
        zmp_start_time_mj_ = 0.0;
    }
    else
    {
        zmp_start_time_mj_ = t_start_;
    }    

    ZMP_X_REF_ = ref_zmp_mj_(walking_tick_mj - zmp_start_time_mj_,0);
    ZMP_Y_REF_ = ref_zmp_mj_(walking_tick_mj - zmp_start_time_mj_,1); 
    
    // State variables x_hat_ and Control input U_mpc are updated with every MPC frequency.
        
    int alpha_step = 0;

    if (foot_step_(current_step_num_, 6) == 1)
    {
        alpha_step = 1;
    }
    else
    {
        alpha_step = -1;
    }        

    // support foot change           
    if(walking_tick_mj == t_start_ && current_step_num_ > 0)
    {
        x_hat_r_ = x_hat_r_sc_;
        x_hat_r_p_ = x_hat_r_p_sc_;

        y_hat_r_ = y_hat_r_sc_;
        y_hat_r_p_ = y_hat_r_p_sc_; 
        
        cp_des_zmp_x_ = des_zmp_x_stepchange_;
        cp_des_zmp_x_prev_ = des_zmp_x_prev_stepchange_;

        cp_des_zmp_y_ = des_zmp_y_stepchange_;
        cp_des_zmp_y_prev_ = des_zmp_y_prev_stepchange_;
    }

    // send vaiables to the mpc in the thread3
    if(atb_mpc_update_ == false)  
    {
        atb_mpc_update_ = true;
        walking_tick_mj_thread_ = walking_tick_mj;
        t_total_thread_ = t_total_;
        t_rest_init_thread_ = t_rest_init_;
        t_rest_last_thread_ = t_rest_last_;
        current_step_num_thread_ = current_step_num_;
        total_step_num_thread_ = total_step_num_;
        zmp_start_time_mj_thread_ = zmp_start_time_mj_;
        ref_zmp_thread_ = ref_zmp_mj_; 
        
        ref_zmp_wo_offset_thread_ = ref_zmp_mj_wo_offset_;  
        alpha_step_mpc_thread_ = alpha_step;        
        cp_measured_thread_ = cp_measured_;

        cam_thread_(1) = del_ang_momentum_(1); // CAM_real_(1);// 
        cam_thread_(0) = del_ang_momentum_(0); // CAM_real_(0);//  

        lfoot_support_current_thread_= lfoot_support_current_;
        rfoot_support_current_thread_= rfoot_support_current_;

        x_hat_p_thread2_ = x_hat_r_p_;
        y_hat_p_thread2_ = y_hat_r_p_; 
        x_hat_thread2_ = x_hat_r_;
        y_hat_thread2_ = y_hat_r_; // 이것만 쓰면 mpc안에서 덮어 씌워져버려서 따로 Step change 변수 만들어야함.

        if(walking_tick_mj == t_start_ && current_step_num_ > 0)
        {   
            cpmpc_des_zmp_x_thread2_ = des_zmp_x_stepchange_;
            cpmpc_des_zmp_y_thread2_ = des_zmp_y_stepchange_;
            cpmpc_interpol_cnt_x_ = 1;
            cpmpc_interpol_cnt_y_ = 1;
        }
  
        atb_mpc_update_ = false;
    } 

     // get the mpc result from thread3
    if(mpc_x_update_ == true) // 0.011 ~ 0.012 주기로 업데이트
    {   
        if(atb_mpc_x_update_ == false)
        {
            atb_mpc_x_update_ = true;
            if(current_step_num_thread2_ == current_step_num_)
            {    
                x_hat_r_ = x_hat_thread_;
                x_hat_r_p_ = x_hat_p_thread_;                  
                wieber_interpol_cnt_x_ = 1;
            }
            else
            {
                cout<<"MPC output X is ignored"<<endl;
            }
            atb_mpc_x_update_ = false;
        }
        x_diff_ = x_hat_r_ - x_hat_r_p_;
        mpc_x_update_ = false;
    } 

    if(mpc_y_update_ == true) // 0.011 ~ 0.012 주기로 업데이트
    {   
        if(atb_mpc_y_update_ == false)
        {
            atb_mpc_y_update_ = true; 
            if(current_step_num_thread2_ == current_step_num_)
            {        
                // cout<<"walking_tick_mj: "<<walking_tick_mj<<endl;
                y_hat_r_ = y_hat_thread_;
                y_hat_r_p_ = y_hat_p_thread_;
                wieber_interpol_cnt_y_ = 1;
            }
            else
            {
                cout<<"MPC output Y is ignored"<<endl;
            }
            atb_mpc_y_update_ = false;
        }
        
        y_diff_ = y_hat_r_ - y_hat_r_p_;
        mpc_y_update_ = false;
    }

    if(cpmpc_x_update_ == true)
    {
        if(atb_cpmpc_x_update_ == false) // 여기서 cp_des_zmp_y를 step change 시키고 cpmpc_des_zmp_y_thread_에 담아서 전달.
        {
            atb_cpmpc_x_update_ = true;

            if(current_step_num_thread2_ == current_step_num_)        
            {
                cp_des_zmp_x_prev_ = cp_des_zmp_x_; 
                cp_des_zmp_x_ = cpmpc_des_zmp_x_thread_;
                
                del_F_x_ = del_F_x_thread_;
                del_F_x_next_ = del_F_x_thread_;
                
                des_tau_y_ = des_tau_y_thread_;
                
                cpmpc_interpol_cnt_x_ = 1;
            }
            else
            {
                cout<<"CP output X is ignored"<<endl;
            }
            atb_cpmpc_x_update_ = false;
        }
        cpmpc_diff_(0) = cp_des_zmp_x_ - cp_des_zmp_x_prev_;
        cpmpc_x_update_ = false;
    }

    if(cpmpc_y_update_ == true)
    {
        if(atb_cpmpc_y_update_ == false) // 여기서 cp_des_zmp_y를 step change 시키고 cpmpc_des_zmp_y_thread_에 담아서 전달.
        {
            atb_cpmpc_y_update_ = true;

            if(current_step_num_thread2_ == current_step_num_)        
            {
                cp_des_zmp_y_prev_ = cp_des_zmp_y_; //cpmpc_des_zmp_y_prev_thread_; 
                cp_des_zmp_y_ = cpmpc_des_zmp_y_thread_;
                
                del_F_y_ = del_F_y_thread_;
                del_F_y_next_ = del_F_y_thread_;

                des_tau_x_ = des_tau_x_thread_;
                
                cpmpc_interpol_cnt_y_ = 1;
            }
            else
            {
                cout<<"CP output Y is ignored"<<endl;
            }
            atb_cpmpc_y_update_ = false;
        }
        cpmpc_diff_(1) = cp_des_zmp_y_ - cp_des_zmp_y_prev_;
        cpmpc_y_update_ = false;
    }
    
    double thread_freq = 50.0;

    double x_com_lin_spline = (thread_freq/hz_)*wieber_interpol_cnt_x_;
    double y_com_lin_spline = (thread_freq/hz_)*wieber_interpol_cnt_y_;

    x_com_lin_spline = DyrosMath::minmax_cut(x_com_lin_spline, 0.0, 1.0);
    y_com_lin_spline = DyrosMath::minmax_cut(y_com_lin_spline, 0.0, 1.0);

    x_mpc_i_ = x_com_lin_spline*x_diff_ + x_hat_r_p_; // 50.0 = MPC freq.
    y_mpc_i_ = y_com_lin_spline*y_diff_ + y_hat_r_p_;

    wieber_interpol_cnt_x_ ++;
    wieber_interpol_cnt_y_ ++;
    
    double x_cpmpc_lin_spline = (thread_freq/hz_)*cpmpc_interpol_cnt_x_;
    double y_cpmpc_lin_spline = (thread_freq/hz_)*cpmpc_interpol_cnt_y_;

    x_cpmpc_lin_spline = DyrosMath::minmax_cut(x_cpmpc_lin_spline, 0.0, 1.0);
    y_cpmpc_lin_spline = DyrosMath::minmax_cut(y_cpmpc_lin_spline, 0.0, 1.0);
    
    des_zmp_interpol_(0) = x_cpmpc_lin_spline*cpmpc_diff_(0) + cp_des_zmp_x_prev_;
    des_zmp_interpol_(1) = y_cpmpc_lin_spline*cpmpc_diff_(1) + cp_des_zmp_y_prev_;

    // del_F_(0) = del_F_x_;
    // del_F_(1) = del_F_y_;
    
    // MJ_graph << cp_des_zmp_y_prev_ << "," << cp_des_zmp_y_ << "," << del_F_x_prev_ << "," << del_F_x_ << "," << del_F_(0) << "," << des_zmp_interpol_(1) << endl;
    
    cpmpc_interpol_cnt_x_ ++;
    cpmpc_interpol_cnt_y_ ++;
    
    // Reference COM, CP position // CPMPC로 대체하면 필요 X
    cp_desired_(0) = x_mpc_i_(0) + x_mpc_i_(1) / wn;
    cp_desired_(1) = y_mpc_i_(0) + y_mpc_i_(1) / wn;

    com_desired_(0) = x_mpc_i_(0);
    com_desired_(1) = y_mpc_i_(0);
    com_desired_(2) = 0.77172;     
              
    if (walking_tick_mj == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1)
    {        
        Eigen::Vector3d com_pos_prev;
        Eigen::Vector3d com_pos;
        Eigen::Vector3d com_vel_prev;
        Eigen::Vector3d com_vel;
        Eigen::Vector3d com_acc_prev;
        Eigen::Vector3d com_acc;
        Eigen::Matrix3d temp_rot;
        Eigen::Vector3d temp_pos; 

        x_hat_r_p_sc_ = x_hat_r_p_;
        x_hat_r_sc_ = x_hat_r_;
        y_hat_r_p_sc_ = y_hat_r_p_;
        y_hat_r_sc_ = y_hat_r_; 

        des_zmp_x_prev_stepchange_ = cp_des_zmp_x_prev_;
        des_zmp_x_stepchange_ = cp_des_zmp_x_;
        des_zmp_y_prev_stepchange_ = cp_des_zmp_y_prev_;
        des_zmp_y_stepchange_ = cp_des_zmp_y_;

        temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_, 5));
        for (int i = 0; i < 3; i++)
            temp_pos(i) = foot_step_support_frame_(current_step_num_, i);        
       
        temp_pos(0) = temp_pos(0) + modified_del_zmp_(current_step_num_,0);
        temp_pos(1) = temp_pos(1) + modified_del_zmp_(current_step_num_,1);
         
        com_pos_prev(0) = x_hat_r_sc_(0);
        com_pos_prev(1) = y_hat_r_sc_(0);

        com_pos = temp_rot * (com_pos_prev - temp_pos); 

        com_vel_prev(0) = x_hat_r_sc_(1);
        com_vel_prev(1) = y_hat_r_sc_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = x_hat_r_sc_(2);
        com_acc_prev(1) = y_hat_r_sc_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        x_hat_r_sc_(0) = com_pos(0);
        y_hat_r_sc_(0) = com_pos(1);
        x_hat_r_sc_(1) = com_vel(0);
        y_hat_r_sc_(1) = com_vel(1);
        x_hat_r_sc_(2) = com_acc(0);
        y_hat_r_sc_(2) = com_acc(1);        

        com_pos_prev(0) = x_hat_r_p_sc_(0);
        com_pos_prev(1) = y_hat_r_p_sc_(0);
        com_pos = temp_rot * (com_pos_prev - temp_pos);

        com_vel_prev(0) = x_hat_r_p_sc_(1);
        com_vel_prev(1) = y_hat_r_p_sc_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = x_hat_r_p_sc_(2);
        com_acc_prev(1) = y_hat_r_p_sc_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        x_hat_r_p_sc_(0) = com_pos(0);
        y_hat_r_p_sc_(0) = com_pos(1);
        x_hat_r_p_sc_(1) = com_vel(0);
        y_hat_r_p_sc_(1) = com_vel(1); 
        x_hat_r_p_sc_(2) = com_acc(0);
        y_hat_r_p_sc_(2) = com_acc(1);

        //com_pos_prev(0) = x_hat_r_p_sc_(0);
        com_pos_prev(0) = des_zmp_x_stepchange_;
        com_pos_prev(1) = des_zmp_y_stepchange_;
        com_pos = temp_rot * (com_pos_prev - temp_pos);        

        des_zmp_x_stepchange_ = com_pos(0);
        des_zmp_y_stepchange_ = com_pos(1); // step change 1 tick 전 desired ZMP (MPC output) step change    

        com_pos_prev(0) = des_zmp_x_prev_stepchange_;
        com_pos_prev(1) = des_zmp_y_prev_stepchange_;
        com_pos = temp_rot * (com_pos_prev - temp_pos);        

        des_zmp_x_prev_stepchange_ = com_pos(0);
        des_zmp_y_prev_stepchange_ = com_pos(1); // step change 1 tick 전 desired ZMP (MPC output) step change  
                
    }    
    //MJ_graph1 << ZMP_X_REF_ << "," << ZMP_Y_REF_ << "," << com_desired_(0) << "," << com_desired_(1) << "," << cp_desired_(0) << ","  << cp_desired_(1) << endl;  
}
void AvatarController::CPMPC_bolt_Controller_MJ()
{   
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //Setting up nominal values w.r.t current footstep
 
    // L : del_F_x , W : del_F_y 
    
    double L_nom = 0.0;
    double L_min = 0.0; 
    double L_max = 0.0; 

    double W_nom = 0.0;
    double W_min = 0.0;
    double W_max = 0.0;

    double T_nom = 0.0;
    double T_min = 0.0; 
    double T_max = 0.0;
    double tau_nom = 0.0;
        
    double l_p = 0.0;
    l_p = foot_step_support_frame_(current_step_num_, 1);
    double w1_step = w_ux_temp_, w2_step = w_uy_temp_, w3_step = w_time_temp_, w4_step = w_bx_temp_, w5_step = w_by_temp_;

    //double w1_step = 1.0, w2_step = 0.02, w3_step = 3.0; // real robot experiment
    double u0_x = 0, u0_y = 0;   
    double b_nom_x_cpmpc = 0, b_nom_y_cpmpc = 0;
    // support foot // 어짜피 MPC 제어입력을 쓰는거기 때문에 아래의 minmax_cut이 의미가 없긴함. 혹시나 입력이 튈 경우
    // u0_x = DyrosMath::minmax_cut(des_cmp_ssp_mpc_x_, -0.09 - 0.016, 0.12 + 0.016); 
    // u0_y = DyrosMath::minmax_cut(des_cmp_ssp_mpc_y_, -0.06 - 0.016, 0.06 + 0.016);         

    u0_x = DyrosMath::minmax_cut(P_ssp_x_, -0.09 - 0.016, 0.12 + 0.016); 
    u0_y = DyrosMath::minmax_cut(P_ssp_y_, -0.06 - 0.016, 0.06 + 0.016);
 
    u0_x_data_ = u0_x;
    u0_y_data_ = u0_y;
    // MJ_graph << des_cmp_ssp_mpc_x_ << "," << P_ssp_x_ << "," << des_cmp_ssp_mpc_y_ << "," << P_ssp_y_ << endl;
    if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        del_F_x_ = 0;
        del_F_y_ = 0;
        u0_x = 0;
        u0_y = 0;        
    }
    else if(walking_tick_mj >= t_start_ + t_total_ - (t_rest_last_ + t_double2_) && walking_tick_mj < t_start_ + t_total_ )
    {
        del_F_x_ = 0;
        del_F_y_ = 0;
        u0_x = 0;
        u0_y = 0;
    }

    L_nom = foot_step_support_frame_(current_step_num_, 0) + del_F_x_;     
    W_nom = foot_step_support_frame_(current_step_num_, 1) + del_F_y_;   
    // W_nom = 0*foot_step_support_frame_(current_step_num_, 1) + del_F_y_;  
    L_min = L_nom - 0.1; // 0.05
    L_max = L_nom + 0.1;
    W_min = W_nom - 0.1;
    W_max = W_nom + 0.1; 
    
    T_nom = (t_total_const_ - (t_rest_init_ + t_rest_last_ + t_double1_ + t_double2_))/hz_; // 0.6하면 370 못버팀.
    T_min = T_nom - 0.2;  
    T_max = T_nom + 0.2;
    tau_nom = exp(wn*T_nom); 

    // Bolt
    // b_nom_x = L_nom/(exp(wn*T_nom)-1); 
    // b_nom_y = l_p/(1 + exp(wn*T_nom)) - W_nom/(1 - exp(wn*T_nom));
    // Wieber
    // b_nom_x = cp_eos_x_mpc_ - L_nom; 
    // b_nom_y = cp_eos_y_mpc_ - W_nom;

    double cp_eos_x_cpmpc_temp = 0, cp_eos_y_cpmpc_temp = 0;

    cp_eos_x_cpmpc_temp = DyrosMath::minmax_cut(cp_eos_x_cpmpc_, -0.25, 0.25);
    cp_eos_y_cpmpc_temp = DyrosMath::minmax_cut(cp_eos_y_cpmpc_, -0.25, 0.25);  

    // b_nom_x_cpmpc = L_nom/(exp(wn*T_nom)-1);
    // b_nom_y_cpmpc = l_p/(1 + exp(wn*T_nom)) - W_nom/(1 - exp(wn*T_nom)); 
    b_nom_x_cpmpc = cp_eos_x_cpmpc_temp - L_nom;
    b_nom_y_cpmpc = cp_eos_y_cpmpc_temp - W_nom;
    // b_nom_x_cpmpc = cp_eos_x_cpmpc_temp - foot_step_support_frame_(current_step_num_, 0);
    // b_nom_y_cpmpc = cp_eos_y_cpmpc_temp - foot_step_support_frame_(current_step_num_, 1);
   
    Eigen::MatrixXd H_step;
    Eigen::VectorXd g_step; 
    
    H_step.setZero(5,5);
    H_step(0,0) = w1_step; // U_T,x (step position in x-direction)
    H_step(1,1) = w2_step; // U_T,y (step position in y-direction) // 200
    H_step(2,2) = w3_step; // tau (step timing)
    H_step(3,3) = w4_step; // DCM offset in x
    H_step(4,4) = w5_step; // DCM offset in y // 0.01
    
    g_step.setZero(5);
    g_step(0) = -w1_step * L_nom; //-w1_step * (u0_x + L_nom);
    g_step(1) = -w2_step * W_nom; //-w2_step * (u0_y + W_nom); 
    g_step(2) = -w3_step * tau_nom;
    g_step(3) = -w4_step * b_nom_x_cpmpc;  
    g_step(4) = -w5_step * b_nom_y_cpmpc;  

    Eigen::VectorXd lb_step;
    Eigen::VectorXd ub_step;
    Eigen::MatrixXd A_step;         

    double stepping_start_time = 0;
    
    if (current_step_num_ == 0)
    {
        stepping_start_time = t_start_ + t_double1_ + t_rest_init_;
    }
    else
    {
        stepping_start_time = t_start_ + t_double1_ + t_rest_init_;
    }    
    
    A_step.setZero(7,5);

    A_step(0,0) = 1; // U_T,x
    A_step(0,1) = 0; // U_T,y
    A_step(0,2) = -(cp_measured_(0)-u0_x)*exp(-wn*(walking_tick_mj - stepping_start_time)/hz_); // tau
    A_step(0,3) = 1; // b_x
    A_step(0,4) = 0; // b_y

    A_step(1,0) = 0; // U_T,x
    A_step(1,1) = 1; // U_T,y 
    A_step(1,2) = -(cp_measured_(1)-u0_y)*exp(-wn*(walking_tick_mj - stepping_start_time)/hz_); // tau
    A_step(1,3) = 0; // b_x
    A_step(1,4) = 1; // b_y

    A_step(2,0) = 1; // U_T,x

    A_step(3,1) = 1; // U_T,y

    A_step(4,2) = 1; // tau

    A_step(5,3) = 1; // b_x
    A_step(6,4) = 1; // b_x

    lb_step.setZero(7);
    ub_step.setZero(7);

    lb_step(0) = u0_x; // equality 
    lb_step(1) = u0_y; // equality 
    lb_step(2) = L_min; // u0_x + L_min;
    lb_step(3) = W_min; // u0_y + W_min;
    lb_step(4) = exp(wn*T_min);
    lb_step(5) = b_nom_x_cpmpc - 0.1; 
    lb_step(6) = b_nom_y_cpmpc - 0.1;
    
    ub_step(0) = u0_x;
    ub_step(1) = u0_y;
    ub_step(2) = L_max;// u0_x + L_max;
    ub_step(3) = W_max;// u0_y + W_max;
    ub_step(4) = exp(wn*T_max);
    ub_step(5) = b_nom_x_cpmpc + 0.1; 
    ub_step(6) = b_nom_y_cpmpc + 0.1;    
    
    if(walking_tick_mj == 0)
    {
        stepping_input_.setZero(5);
        stepping_input_(0) = foot_step_support_frame_(current_step_num_, 0);
        stepping_input_(1) = foot_step_support_frame_(current_step_num_, 1);
        stepping_input_(2) = T_nom;
    }         
    // DSP scaler
    Eigen::Vector2d stepping_err;
    stepping_err.setZero();

    if(walking_tick_mj > t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {        
        stepping_err(1) = target_swing_foot(1) + del_F_(1) - fixed_swing_foot(1);

        if(target_swing_foot(0) + del_F_(0) < 0)
        {
            stepping_err(0) = target_swing_foot(0) + del_F_(0) - fixed_swing_foot(0);
            
            if(stepping_err(0) > 0)
            {
                stepping_err(0) = 0;
            }
        }
        else if(target_swing_foot(0) + del_F_(0) >= 0)
        {
            stepping_err(0) = target_swing_foot(0) + del_F_(0) - fixed_swing_foot(0);

            if(stepping_err(0) < 0)
            {
                stepping_err(0) = 0;
            }
        }     
    }
    else
    {
        stepping_err.setZero();
    }    

    // simulation
    dsp_scaler_dot_(0) = 350.0 * stepping_err(0) - 20.0 * dsp_scaler_(0); // 300, -10
    dsp_scaler_dot_(1) = 100.0 * stepping_err(1) - 20.0 * dsp_scaler_(1); // 300, -10

    //real robot experiment
    // dsp_scaler_dot_(0) = 200.0 * stepping_err(0) - 50.0 * dsp_scaler_(0); // 300, -10
    // dsp_scaler_dot_(1) = 100.0 * stepping_err(1) - 50.0 * dsp_scaler_(1); // 300, -10

    dsp_scaler_(0) = dsp_scaler_(0) + dsp_scaler_dot_(0)*del_t;
    dsp_scaler_(1) = dsp_scaler_(1) + dsp_scaler_dot_(1)*del_t;
    
  
    if (walking_tick_mj == t_start_ + t_total_ - t_rest_last_ - t_double2_  && current_step_num_ != total_step_num_ - 1) // SSP 끝날때 미리 계산된 DSP time 저장.
    {
        dsp_time_reducer_fixed_ = dsp_time_reducer_;
        
        if(dsp_time_reducer_fixed_ < 0.01 && dsp_time_reducer_fixed_ > -0.01)
        {
            dsp_time_reducer_fixed_ = 0;
        } 
    }
    else 
    {                  
        dsp_time_reducer_ = dsp_scaler_.norm();  
        dsp_time_reducer_ = DyrosMath::minmax_cut(round(dsp_time_reducer_*1000)/1000.0, 0.0, 0.08);
        // dsp_reducer_2_ = 1 / (1 + 2 * M_PI * 10.0 * del_t) * dsp_reducer_2_ + (2 * M_PI * 10.0 * del_t) / (1 + 2 * M_PI * 10.0 * del_t) * dsp_reducer_1_;
    } 

    // if(walking_tick_mj >= t_start_ + t_total_ - (t_rest_last_ + t_double2_) && walking_tick_mj < t_start_ + t_total_) // -x
    // {   
    //     t_rest_last_ = (0.12 - dsp_time_reducer_fixed_)* hz_; //0.0 * hz_;
    // } 
    // else if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ ) // -x
    // {
    //     t_rest_init_ = (0.12 - dsp_time_reducer_fixed_)* hz_; //0.0 * hz_;
    //     t_rest_last_ = 0.12*hz_;
    // }  
    if(current_step_num_ > 0 && (current_step_num_ != total_step_num_-1))
    {   // Solving the QP during only SSP
        if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
        {
            QP_stepping_.InitializeProblemSize(5, 7);
            QP_stepping_.EnableEqualityCondition(equality_condition_eps_);
            QP_stepping_.UpdateMinProblem(H_step, g_step);
            QP_stepping_.DeleteSubjectToAx();      
            QP_stepping_.UpdateSubjectToAx(A_step, lb_step, ub_step);
        
            if(QP_stepping_.SolveQPoases(200, stepping_input))
            {   
                stepping_input_ = stepping_input.segment(0, 5);
            }
            else
            {
                // cout << "bolt is not solved." << endl;
            }
        }

        if(stepping_input_(2) != 0)
        {
            if(walking_tick_mj - stepping_start_time < round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ - zmp_modif_time_margin_ - 1 )
            {           
                t_total_ = round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ + t_rest_init_ + t_double1_ + t_rest_last_ + t_double2_;
                t_total_ = DyrosMath::minmax_cut(t_total_, t_total_const_ - 0.2*hz_, t_total_const_ + 0.2*hz_);
                // t_total_ = 0.9*hz_;
                t_last_ = t_start_ + t_total_ - 1;
            }
        }
    }  
    
    // t_total_ = 0.9*hz_;
    // t_last_ = t_start_ + t_total_ - 1;
    // if(current_step_num_ == 4) // -y (t_total_이 많이 줄어들면 못버팀)
    // {
    //     t_rest_init_ = (0.12 )* hz_;// 얘만 넣어도 효과 있음. //0.02 * hz_;
    //     t_rest_last_ = (0.00 )* hz_;
    //     // t_total_ = 0.75 * hz_;
    //     // t_last_ = t_start_ + t_total_ - 1;
    // } 
    // else if(current_step_num_ == 5) // -y (t_total_이 많이 줄어들면 못버팀)
    // {
    //     t_rest_init_ = (0.02 )* hz_;// 얘만 넣어도 효과 있음. //0.02 * hz_;
    //     t_rest_last_ = (0.17 )* hz_;
    //     t_total_ = 0.85 * hz_;
    //     t_last_ = t_start_ + t_total_ - 1;
    // } 
    // else if(current_step_num_ > 5)
    // {
    //     t_rest_init_ = 0.17 * hz_;
    //     t_rest_last_ = 0.17 * hz_;
    //     t_total_ = 1.0 * hz_;
    //     // t_total_ = 1.1 * hz_;
    //     t_last_ = t_start_ + t_total_ - 1;
    // }  

    // if(current_step_num_ == 5) // -x (t_total_이 많이 줄어들면 못버팀)
    // {
    //     t_rest_init_ = 0*(0.12 + dsp_time_reducer_)* hz_;// 얘만 넣어도 효과 있음. //0.02 * hz_;
    //     t_rest_last_ = 0*(0.12 + dsp_time_reducer_)* hz_;
    //     t_total_ = 0.7 * hz_;
    //     t_last_ = t_start_ + t_total_ - 1;
    // } 
    // else if(current_step_num_ >= 6)
    // {
    //     t_rest_init_ = 0.12 * hz_;
    //     t_rest_last_ = 0.12 * hz_;
    //     // t_total_ = 1.1 * hz_;
    //     // t_last_ = t_start_ + t_total_ - 1;
    // } 
    if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        // stepping_input_(0) = 0*L_nom;
        // stepping_input_(1) = 0*W_nom;
        stepping_input_(0) = L_nom;
        stepping_input_(1) = W_nom;
        stepping_input_(2) = T_nom; 
    }
    else if(walking_tick_mj >= t_start_ + t_total_ - (t_rest_last_ + t_double2_) && walking_tick_mj < t_start_ + t_total_ )
    {
        // stepping_input_(0) = 0*del_F_(0);
        // stepping_input_(1) = 0*del_F_(1);
        stepping_input_(0) = del_F_(0);
        stepping_input_(1) = del_F_(1);
    } 
    // MJ_graph2 << t_total_ << "," << t_rest_init_ << "," << t_rest_last_ << "," << dsp_scaler_(0) << "," << dsp_scaler_(1) << "," << dsp_time_reducer_ << "," << dsp_time_reducer_fixed_ << endl;
    del_F_(0) = stepping_input_(0);
    del_F_(1) = stepping_input_(1);
    // MJ_graph << stepping_input_(0) << "," << stepping_input_(1) << "," << t_total_ << endl;
    // if(walking_tick_mj > t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - (t_rest_last_ + t_double2_) )
    // {   
    //     MJ_graph << del_F_(0) << "," << L_nom << "," << stepping_input_(3) << "," << b_nom_x_cpmpc << "," << t_total_/hz_<< endl;
    //     MJ_graph1 << del_F_(1) << "," << W_nom << "," << stepping_input_(4) << "," << b_nom_y_cpmpc << endl;
    // } 
    double b_mea_x = 0, b_mea_y = 0;
        
    // if(walking_tick_mj > t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - (t_rest_last_ + t_double2_) ) 
    // {
    //     if(foot_step_(current_step_num_, 6) == 1) // left foot support
    //     { 
    //         b_mea_x = (cp_measured_(0) - rfoot_support_current_.translation()(0));
    //         b_mea_y = (cp_measured_(1) - rfoot_support_current_.translation()(1));
    //     }
    //     else if(foot_step_(current_step_num_, 6) == 0) // right foot support
    //     {
    //         b_mea_x = (cp_measured_(0) - lfoot_support_current_.translation()(0));
    //         b_mea_y = (cp_measured_(1) - lfoot_support_current_.translation()(1));
    //     } 
    //      MJ_graph << b_nom_x_cpmpc << "," << b_nom_y_cpmpc << "," << b_mea_x << "," << b_mea_y << endl;
    // }       

    // MJ_graph1 << cp_desired_(0) << "," << cp_desired_(1) << "," << cp_measured_(0) << "," << cp_measured_(1) << endl;
    // MJ_graph << cp_eos_x_cpmpc_temp << "," << cp_eos_y_cpmpc_temp << endl;
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // MJ_graph1 << del_zmp(0) << "," << del_zmp(1) << "," << des_zmp_interpol_(0) << "," << des_zmp_interpol_(1) << "," << ZMP_X_REF_ << "," << ZMP_Y_REF_ << endl;
    
    // CP-MPC Journal Stepping controller data
    // MJ_graph_stepping << b_nom_x_cpmpc << "," << b_nom_y_cpmpc << "," << del_F_(0) << "," << del_F_(1) << "," << t_total_/hz_ << endl;
    // CPMPC Journal foot trajecotry data
    // MJ_graph_foottra_x << lfoot_trajectory_support_.translation()(0) << "," << rfoot_trajectory_support_.translation()(0) << "," << del_F_(0) << "," << desired_swing_foot(0) << "," << ssp_flag << endl;
    // MJ_graph_foottra_y << lfoot_trajectory_support_.translation()(1) << "," << rfoot_trajectory_support_.translation()(1) << "," << del_F_(1) << "," << desired_swing_foot(1) << endl;
    
}
void AvatarController::CentroidalMomentCalculator_new()
{
    
    if (walking_tick_mj == 0)
    {
        del_tau_.setZero();
        del_ang_momentum_.setZero();
        del_ang_momentum_prev_.setZero();
    }

    del_ang_momentum_prev_ = del_ang_momentum_;   
    // del_ang_momentum_prev_(1) = CAM_real_(1);
    // del_ang_momentum_prev_(0) = CAM_real_(0); 

    // double recovery_damping = 2.0; //damping 20 is equivalent to 0,99 exp gain // 2정도 하면 반대방향으로 치는게 15Nm, 20하면 150Nm
    // 나중에 반대방향 토크 limit 걸어야됨.
    // X direction CP control  
    del_tau_(1) =  des_tau_y_ ;//- recovery_damping*del_ang_momentum_(1);
 
    // Y direction CP control        
    del_tau_(0) = -des_tau_x_ ;//- recovery_damping*del_ang_momentum_(0); 
    
    //// Integrate Centroidal Moment
    del_ang_momentum_(1) = del_ang_momentum_prev_(1) + del_t * del_tau_(1);
    del_ang_momentum_(0) = del_ang_momentum_prev_(0) + del_t * del_tau_(0);
    
    // del CAM output limitation (220118/ DLR's CAM output is an approximately 4 Nms and TORO has a weight of 79.2 kg)    
    double A_limit = 15.0;
       
    if(del_ang_momentum_(0) > A_limit)
    { 
        del_ang_momentum_(0) = A_limit; 
    }
    else if(del_ang_momentum_(0) < -A_limit)
    { 
        del_ang_momentum_(0) = -A_limit; 
    }
    if(del_ang_momentum_(1) > A_limit)
    { 
        del_ang_momentum_(1) = A_limit; 
    }
    else if(del_ang_momentum_(1) < -A_limit)
    { 
        del_ang_momentum_(1) = -A_limit; 
    }
     
}

void AvatarController::getFootTrajectory_stepping()
{   
    // Eigen::Vector6d target_swing_foot;
    // Eigen::Vector6d desired_swing_foot;
    // Eigen::Vector6d fixed_swing_foot;
    double ssp_flag = 0;
    if(walking_tick_mj == 0)
    {
        desired_swing_foot.setZero();
        fixed_swing_foot.setZero();
        fixed_swing_foot_del_F_.setZero();
        target_swing_foot.setZero();    
        del_F_.setZero();
        opt_F_.setZero();
    }
          
    for (int i = 0; i < 6; i++)
    {
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_, i);        
    }
    zmp_modif_time_margin_ = 0.1*hz_;

    if(walking_tick_mj == t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_) // 조현민 처럼 Step으로 zmp를 변경하는게 아니라 부드럽게 바꿔줘도 좋을듯 / SSP 끝나기 0.1초 전 스윙 발 X,Y 고정
    {
        fixed_swing_foot(0) = desired_swing_foot(0); 
        fixed_swing_foot(1) = desired_swing_foot(1);
        
        modified_del_zmp_(current_step_num_,0) = del_F_(0) - target_swing_foot(0);
        modified_del_zmp_(current_step_num_,1) = del_F_(1) - target_swing_foot(1); 
        // modified_del_zmp_(current_step_num_,1) = del_F_(1);// - target_swing_foot(1);                         
    }

    if(current_step_num_ > 0)
    {
        if(foot_step_(current_step_num_, 6) == 1) // left support foot
        {
            m_del_zmp_x(current_step_num_,0) = modified_del_zmp_(current_step_num_,0); // 왼발 지지때 추가로 생성된 오른쪽 스윙 발 변위만큼 오른발의 참조 ZMP를 수정해주기 위해 저장한 val
            m_del_zmp_y(current_step_num_,0) = modified_del_zmp_(current_step_num_,1);               
        }
        else // right support foot
        {  
            m_del_zmp_x(current_step_num_,1) = modified_del_zmp_(current_step_num_,0); // 오른발 지지때 추가로 생성된 왼쪽 스윙 발 변위만큼 왼발의 참조 ZMP를 수정해주기 위해 저장한 val
            m_del_zmp_y(current_step_num_,1) = modified_del_zmp_(current_step_num_,1);                         
        }
    } 
    
    Eigen::Vector2d stepping_foot_init_pos;
    stepping_foot_init_pos.setZero();    

    if(foot_step_(current_step_num_,6) == 1) // left foot support
    {
        stepping_foot_init_pos(0) = rfoot_support_init_.translation()(0); //rfoot_trajectory_support_.translation()(0);
        stepping_foot_init_pos(1) = rfoot_support_init_.translation()(1); //rfoot_trajectory_support_.translation()(1);
    }
    else // right foot support
    {
        stepping_foot_init_pos(0) = lfoot_support_init_.translation()(0); //lfoot_trajectory_support_.translation()(0);
        stepping_foot_init_pos(1) = lfoot_support_init_.translation()(1); //lfoot_trajectory_support_.translation()(1);
    }
    
    if (walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_)
    {
        desired_swing_foot(0) = del_F_(0); // del_F_ is optimized by target_swing_foot_(0) + del_F_x
        // desired_swing_foot(1) = target_swing_foot(1) + del_F_(1);   
        desired_swing_foot(1) = del_F_(1);
    }
    else
    {
        desired_swing_foot(0) = fixed_swing_foot(0);
        desired_swing_foot(1) = fixed_swing_foot(1);
    }    
 
    // real robot experiment 
    // desired_swing_foot_LPF_(0) = 1 / (1 + 2 * M_PI * 1.0 * del_t) * desired_swing_foot_LPF_(0) + (2 * M_PI * 1.0 * del_t) / (1 + 2 * M_PI * 1.0 * del_t) * desired_swing_foot(0);
    // del_F_LPF_(1) = 1 / (1 + 2 * M_PI * 1.0 * del_t) * del_F_LPF_(1) + (2 * M_PI * 1.0 * del_t) / (1 + 2 * M_PI * 1.0 * del_t) * del_F_(1);
    
    if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            lfoot_trajectory_support_.translation().setZero();
            lfoot_trajectory_euler_support_.setZero();

            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_support_.translation()(2) = 0;
            rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            rfoot_trajectory_support_.translation().setZero();
            rfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_support_.translation()(2) = 0;
            lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
        }

        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {   
        ssp_flag = 0.1;        

        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

            if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0)
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, foot_height_, 0.0, 0.0);
            }
            else
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            // for (int i = 0; i < 2; i++)
            // {
            //     rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            // }

            // 220422
            rfoot_trajectory_support_.translation()(0) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(0), desired_swing_foot(0), 0.0, 0.0);
            rfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(1), desired_swing_foot(1), 0.0, 0.0);    
            
            rfoot_trajectory_euler_support_(0) = 0;
            rfoot_trajectory_euler_support_(1) = 0;
            rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_euler_support_.setZero();

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

            if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0)
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, foot_height_, 0.0, 0.0);
            }
            else
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            // for (int i = 0; i < 2; i++)
            // {
            //     lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            // }
            // 220422
            lfoot_trajectory_support_.translation()(0) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(0), desired_swing_foot(0), 0.0, 0.0);
            lfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(1), desired_swing_foot(1), 0.0, 0.0);    
            
            lfoot_trajectory_euler_support_(0) = 0;
            lfoot_trajectory_euler_support_(1) = 0;
            lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        }
    }
    else
    {
        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_euler_support_.setZero();
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            for (int i = 0; i < 3; i++)
            {
                rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                rfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }
            // 220422
            rfoot_trajectory_support_.translation()(0) =  desired_swing_foot(0);
            rfoot_trajectory_support_.translation()(1) =  desired_swing_foot(1);

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
            //rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

            //rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

            for (int i = 0; i < 3; i++)
            {
                lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                lfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }
            // 220422
            lfoot_trajectory_support_.translation()(0) =  desired_swing_foot(0); 
            lfoot_trajectory_support_.translation()(1) =  desired_swing_foot(1);    
            
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        }
    }

    // CPMPC Journal foot trajecotry data
    // MJ_graph_foottra_x << lfoot_trajectory_support_.translation()(0) << "," << rfoot_trajectory_support_.translation()(0) << "," << del_F_(0) << "," << desired_swing_foot(0) << "," << ssp_flag << endl;
    // MJ_graph_foottra_y << lfoot_trajectory_support_.translation()(1) << "," << rfoot_trajectory_support_.translation()(1) << "," << del_F_(1) << "," << desired_swing_foot(1) << endl;
     
}
void AvatarController::getPelvTrajectory()
{
    // double pelv_offset = -0.00; // DG
    // double pelv_transition_time = 1.0;
    // if (walking_enable_ == true)
    // {
    //     pelv_height_offset_ = DyrosMath::cubic(walking_tick_mj, 0, pelv_transition_time * hz_, pelv_support_init_.translation()(2) - com_desired_(2), 0.0, 0.0, 0.0);
    // }
    // else
    // {
    //     pelv_height_offset_ = DyrosMath::cubic(rd_.control_time_, init_leg_time_, init_leg_time_ + 5.0, pelv_support_init_.translation()(2) - com_desired_(2), pelv_offset, 0.0, 0.0);
    // }

    double pelv_transition_time = 2.0;
    double pelv_height_offset_ = 0.05;
    if (walking_enable_ == true)
    {
        pelv_height_offset_ = DyrosMath::cubic(walking_tick_mj, 0, pelv_transition_time * hz_, 0.0, 0.05, 0.0, 0.0);
    }

    double z_rot = foot_step_support_frame_(current_step_num_, 5);

    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7 * (com_desired_(0) - 0*0.15 * damping_x - com_support_current_(0)); 
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7 * (com_desired_(1) - 0*0.6 * damping_y - com_support_current_(1));  
    // pelv_trajectory_support_.translation()(2) = com_desired_(2) + pelv_height_offset_; //DG
    pelv_trajectory_support_.translation()(2) = com_desired_(2) - 0*pelv_height_offset_;

    Eigen::Vector3d Trunk_trajectory_euler;
    Trunk_trajectory_euler.setZero();

    if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_double2_ - t_rest_last_, pelv_support_euler_init_(2), z_rot / 2.0, 0.0, 0.0);
    }
    else
    {
        Trunk_trajectory_euler(2) = z_rot / 2.0;
    }

    // P_angle_i = P_angle_i + (0 - P_angle)*del_t;
    // Trunk_trajectory_euler(1) = 0.05*(0.0 - P_angle) + 1.5*P_angle_i;
    if (aa == 0 && walking_tick_mj == 0 && (walking_enable_ == true))
    {
        P_angle_input = 0;
        R_angle_input = 0;
    }

    P_angle_input_dot = 1.5 * (0.0 - P_angle) ;
    R_angle_input_dot = 2.0 * (0.0 - R_angle) ;

    P_angle_input = P_angle_input + P_angle_input_dot * del_t;
    R_angle_input = R_angle_input + R_angle_input_dot * del_t;

    if (R_angle_input > 3 * DEG2RAD) //1.5 degree
    {
        R_angle_input = 3 * DEG2RAD;
    }
    else if (R_angle_input < -3 * DEG2RAD)
    {
        R_angle_input = -3 * DEG2RAD;
    }

    if (P_angle_input > 5 * DEG2RAD) //5 degree
    {
        P_angle_input = 5 * DEG2RAD;
        // cout << "a" << endl;
    }
    else if (P_angle_input < -5 * DEG2RAD)
    {
        P_angle_input = -5 * DEG2RAD;
        // cout << "b" << endl;
    }
    //Trunk_trajectory_euler(0) = R_angle_input;
    Trunk_trajectory_euler(1) = P_angle_input;    
    
    pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2)) * DyrosMath::rotateWithY(Trunk_trajectory_euler(1)) * DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}
void AvatarController::supportToFloatPattern()
{
    //lfoot_trajectory_support_.linear() = lfoot_support_init_.linear();
    //rfoot_trajectory_support_.linear() = rfoot_support_init_.linear();
    pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * pelv_trajectory_support_;
    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * lfoot_trajectory_support_;
    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * rfoot_trajectory_support_;

    rfoot_trajectory_float_.translation()(2) = rfoot_trajectory_float_.translation()(2) + F_F_input * 0.5;
    lfoot_trajectory_float_.translation()(2) = lfoot_trajectory_float_.translation()(2) - F_F_input * 0.5;
}
void AvatarController::computeIkControl_MJ(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d &q_des)
{
    Eigen::Vector3d R_r, R_D, L_r, L_D;

    L_D << 0.11, +0.1025, -0.1025;
    R_D << 0.11, -0.1025, -0.1025;

    L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * L_D - float_lleg_transform.translation());
    R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * R_D - float_rleg_transform.translation());

    double R_C = 0, L_C = 0, L_upper = 0.351, L_lower = 0.351, R_alpha = 0, L_alpha = 0;

    L_C = sqrt(pow(L_r(0), 2) + pow(L_r(1), 2) + pow(L_r(2), 2));
    R_C = sqrt(pow(R_r(0), 2) + pow(R_r(1), 2) + pow(R_r(2), 2));
     
    double knee_acos_var_L = 0;
    double knee_acos_var_R = 0;

    knee_acos_var_L = (pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2))/ (2 * L_upper * L_lower);
    knee_acos_var_R = (pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2))/ (2 * L_upper * L_lower);

    knee_acos_var_L = DyrosMath::minmax_cut(knee_acos_var_L, -0.99, + 0.99);
    knee_acos_var_R = DyrosMath::minmax_cut(knee_acos_var_R, -0.99, + 0.99);

    q_des(3) = (-acos(knee_acos_var_L) + M_PI);  
    q_des(9) = (-acos(knee_acos_var_R) + M_PI);
    
    L_alpha = asin(L_upper / L_C * sin(M_PI - q_des(3)));
    R_alpha = asin(L_upper / R_C * sin(M_PI - q_des(9)));
    
    q_des(4) = -atan2(L_r(0), sqrt(pow(L_r(1), 2) + pow(L_r(2), 2))) - L_alpha;
    q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1), 2) + pow(R_r(2), 2))) - R_alpha;

    Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
    Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
    Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

    L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3) - q_des(4));
    L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
    R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9) - q_des(10));
    R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11));

    L_Hip_rot_mat.setZero();
    R_Hip_rot_mat.setZero();

    L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat;
    R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

    q_des(0) = atan2(-L_Hip_rot_mat(0, 1), L_Hip_rot_mat(1, 1));                                                       // Hip yaw
    q_des(1) = atan2(L_Hip_rot_mat(2, 1), -L_Hip_rot_mat(0, 1) * sin(q_des(0)) + L_Hip_rot_mat(1, 1) * cos(q_des(0))); // Hip roll
    q_des(2) = atan2(-L_Hip_rot_mat(2, 0), L_Hip_rot_mat(2, 2));                                                       // Hip pitch
    q_des(2) = DyrosMath::minmax_cut(q_des(2), -90*DEG2RAD, - 5*DEG2RAD);
    q_des(3) = q_des(3);                                                                                               // Knee pitch
    q_des(4) = q_des(4);                                                                                               // Ankle pitch
    q_des(5) = atan2(L_r(1), L_r(2));                                                                                  // Ankle roll

    q_des(6) = atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1));
    q_des(7) = atan2(R_Hip_rot_mat(2, 1), -R_Hip_rot_mat(0, 1) * sin(q_des(6)) + R_Hip_rot_mat(1, 1) * cos(q_des(6)));
    q_des(8) = atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2));
    q_des(8) = DyrosMath::minmax_cut(q_des(8), -90*DEG2RAD, - 5*DEG2RAD);
    q_des(9) = q_des(9);
    q_des(10) = q_des(10);
    q_des(11) = atan2(R_r(1), R_r(2));
    // MJ_graph << walking_tick_mj << "," <<  q_des(0) << "," << q_des(1) << "," << q_des(2) << "," << q_des(3) << "," << q_des(4) << "," << q_des(5) << endl; 
    // MJ_graph1 << q_des(6) << "," << q_des(7) << "," << q_des(8) << "," << q_des(9) << "," << q_des(10) << "," << q_des(11) << endl; 
    if (walking_tick_mj == 0)
    {
        sc_joint_err.setZero();
    }

    if (walking_tick_mj == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1) // step change 1 tick 이전
    {                                                                                           //5.3, 0
        sc_joint_before.setZero();
        sc_joint_before = q_des;
    }
    if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {                                                          //5.3005, 1
        sc_joint_after.setZero();
        sc_joint_after = q_des;

        sc_joint_err = sc_joint_after - sc_joint_before;
    }
    if (current_step_num_ != 0)
    {
        for (int i = 0; i < 12; i++)
        {
            SC_joint(i) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.005 * hz_, sc_joint_err(i), 0.0, 0.0, 0.0);
        }

        if (walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + 0.005 * hz_)
        {
            q_des = q_des - SC_joint;
        }
    }    
}
void AvatarController::Compliant_control(Eigen::Vector12d desired_leg_q)
{
    Eigen::Vector12d current_u;
    double del_t = 0.0, Kp = 0.0;
    del_t = 1 / hz_;
    Kp = 100.0; // 실험
                //   Kp = 20.0; // 시뮬

    if (walking_tick_mj == 0)
    {
        for (int i = 0; i < 12; i++)
        {
            DOB_IK_output_b_(i) = rd_.q_(i);
            DOB_IK_output_(i) = rd_.q_(i);
            current_u(i) = rd_.q_(i);
        }
    }

    if (walking_tick_mj > 0)
    {
        for (int i = 0; i < 12; i++)
        {
            current_u(i) = (rd_.q_(i) - (1 - Kp * del_t) * q_prev_MJ_(i)) / (Kp * del_t);
        }
    }

    Eigen::Vector12d d_hat;
    d_hat = current_u - DOB_IK_output_b_;

    if (walking_tick_mj == 0)
        d_hat_b = d_hat;

    d_hat = (2 * M_PI * 5.0 * del_t) / (1 + 2 * M_PI * 5.0 * del_t) * d_hat + 1 / (1 + 2 * M_PI * 5.0 * del_t) * d_hat_b;

    double default_gain = 0.0;
    double compliant_gain = 0.0;
    double compliant_tick = 0.0 * hz_;
    double gain_temp = 0.0;
    for (int i = 0; i < 12; i++)
    {
        if (i < 6)
        {
            gain_temp = default_gain;

            if (foot_step_(current_step_num_, 6) == 0)
            {
                if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick)
                {
                    gain_temp = default_gain;
                }
                else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick)
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, default_gain, compliant_gain, 0.0, 0.0);
                }
                else
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
                }
            }
            else
            {
                gain_temp = default_gain;
            }

            DOB_IK_output_(i) = desired_leg_q(i) + gain_temp * d_hat(i);
        }
        else
        {
            gain_temp = default_gain;

            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지 상태
            {
                if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick)
                {
                    gain_temp = default_gain;
                }
                else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick)
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, default_gain, compliant_gain, 0.0, 0.0);
                }
                else
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
                }
            }
            else // 오른발 지지 상태
            {
                gain_temp = default_gain;
            }

            DOB_IK_output_(i) = desired_leg_q(i) + gain_temp * d_hat(i);
        }
    }

    d_hat_b = d_hat;
    DOB_IK_output_b_ = DOB_IK_output_;
   
}

//두개가 무슨 차이 일까? 같이쓰던데?
void AvatarController::CP_compen_MJ()
{
    double alpha = 0;
    double F_R = 0, F_L = 0;

    // Tau_R.setZero(); Tau_L.setZero();

    Tau_CP.setZero();

    alpha = (com_float_current_(1) - rfoot_float_current_.translation()(1)) / (lfoot_float_current_.translation()(1) - rfoot_float_current_.translation()(1));

    if (alpha > 1)
    {
        alpha = 1;
    }
    else if (alpha < 0)
    {
        alpha = 0;
    }
    
    F_R = (1 - alpha) * rd_.link_[COM_id].mass * GRAVITY;
    F_L = alpha * rd_.link_[COM_id].mass * GRAVITY;

    Tau_CP(4) = 0 * F_L * del_zmp(0);  // L pitch
    Tau_CP(10) = 0 * F_R * del_zmp(0); // R pitch

    Tau_CP(5) = -0 * F_L * del_zmp(1);  // L roll
    Tau_CP(11) = -0 * F_R * del_zmp(1); // R roll
}
void AvatarController::CP_compen_MJ_FT()
{ 
    // 기존 알고리즘에서 바꾼거 : 0. previewcontroller에서 ZMP_Y_REF_ 변수 추가 1. zmp offset 2. getrobotstate에서 LPF 3. supportToFloatPattern 함수 4. Tau_CP -> 0  5. getfoottrajectory에서 발의 Euler angle
    double alpha = 0;
    double F_R = 0, F_L = 0;
    double Tau_all_y = 0, Tau_R_y = 0, Tau_L_y = 0;
    double Tau_all_x = 0, Tau_R_x = 0, Tau_L_x = 0;
    double zmp_offset = 0;
    double alpha_new = 0;
    zmp_offset = 0.01; // zmp_offset 함수 참고
            
    // Preview를 이용한 COM 생성시 ZMP offset을 x cm 안쪽으로 넣었지만, alpha 계산은 x cm 넣으면 안되기 때문에 조정해주는 코드
    // 어떻게 보면 COM, CP 궤적은 ZMP offset이 반영되었고, CP 제어기는 반영안시킨게 안맞는거 같기도함
    if (walking_tick_mj > t_temp_)
    {
        if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
            }
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset;
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset;
            }
        }
        else if (walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset - zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset + zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
            }
        }
        else
        {
            ZMP_Y_REF_alpha_ = ZMP_Y_REF_;
        }
    }
    else
    {
        ZMP_Y_REF_alpha_ = ZMP_Y_REF_;
    }
     
    del_zmp(0) = des_zmp_interpol_(0) - ZMP_X_REF_;
    del_zmp(1) = des_zmp_interpol_(1) - ZMP_Y_REF_alpha_;
     
    // del_zmp(0) = DyrosMath::minmax_cut(del_zmp(0), -0.1, 0.1);
    // del_zmp(1) = DyrosMath::minmax_cut(del_zmp(1), -0.07, 0.07); 
    ////////////////////////
    // double A = 0, B = 0, d = 0, X1 = 0, Y1 = 0, e_2 = 0, L = 0, l = 0;
    // A = (lfoot_support_current_.translation()(0) - rfoot_support_current_.translation()(0));
    // B = -(lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
    // X1 = ZMP_Y_REF_alpha_ + 0 * del_zmp(1) - rfoot_support_current_.translation()(1);
    // Y1 = ZMP_X_REF_ + 0 * del_zmp(0) - rfoot_support_current_.translation()(0);
    // L = sqrt(A * A + B * B);
    // d = abs(A * X1 + B * Y1) / L;
    // e_2 = X1 * X1 + Y1 * Y1;
    // l = sqrt(e_2 - d * d);
    // alpha_new = l / L;
    alpha = (ZMP_Y_REF_alpha_ + del_zmp(1) - rfoot_support_current_.translation()(1)) / (lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
    
    if(walking_tick_mj == 0)
    {
        alpha_lpf_ = alpha;
    }

    alpha_lpf_ = 1 / (1 + 2 * M_PI * 6.0 * del_t) * alpha_lpf_ + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * alpha;
    
    // 로봇에서 구현할때 alpha가 0~1로 나오는지 확인, ZMP offset 0으로 해야됨.
    if (alpha > 1)
    {
        alpha = 1;
    } // 왼발 지지때 alpha = 1
    else if (alpha < 0)
    {
        alpha = 0;
    }
    if (alpha_new > 1)
    {
        alpha_new = 1;
    } // 왼발 지지때 alpha = 1
    else if (alpha_new < 0)
    {
        alpha_new = 0;
    }
    if (alpha_lpf_ > 1)
    {
        alpha_lpf_ = 1;
    } // 왼발 지지때 alpha = 1
    else if (alpha_lpf_ < 0)
    {
        alpha_lpf_ = 0;
    }
    F_R = -(1 - alpha_lpf_) * rd_.link_[COM_id].mass * GRAVITY;
    F_L = -alpha_lpf_ * rd_.link_[COM_id].mass * GRAVITY; // alpha가 0~1이 아니면 desired force가 로봇 무게보다 계속 작게나와서 지면 반발력을 줄이기위해 다리길이를 줄임.
    if (walking_tick_mj == 0)
    {
        F_F_input = 0.0;
        F_T_L_x_input = 0.0;
        F_T_R_x_input = 0.0;
        F_T_L_y_input = 0.0;
        F_T_R_y_input = 0.0;
    }
    //////////// Force
    F_F_input_dot = 0.0005 * ((l_ft_(2) - r_ft_(2)) - (F_L - F_R)) - 3.0 * F_F_input; // F_F_input이 크면 다리를 원래대로 빨리줄인다. 이정도 게인 적당한듯0.001/0.00001 // SSP, DSP 게인값 바꿔야?
    F_F_input = F_F_input + F_F_input_dot * del_t;
    if (F_F_input >= 0.02)
    {
        F_F_input = 0.02;
    }
    else if (F_F_input <= -0.02)
    {
        F_F_input = -0.02;
    }
    //////////// Torque
    // X,Y 축을 X,Y 방향으로 헷갈렸었고, 위치 명령을 발목 IK각도에 바로 넣었었음.
    Tau_all_x = -((rfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha_ + del_zmp(1))) * F_R + (lfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha_ + del_zmp(1))) * F_L);
    Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_L);
    if (Tau_all_x > 100)
    {
        Tau_all_x = 100;
    }
    else if (Tau_all_x < -100)
    {
        Tau_all_x = -100;
    }
    if (Tau_all_y > 100)
    {
        Tau_all_y = 100;
    }
    else if (Tau_all_y < -100)
    {
        Tau_all_y = -100;
    }
    Tau_R_x = (1 - alpha) * Tau_all_x;
    Tau_L_x = (alpha)*Tau_all_x;
    Tau_L_y = -alpha * Tau_all_y;
    Tau_R_y = -(1 - alpha) * Tau_all_y;

    //Roll 방향 -0.3,50 -> High performance , -0.1, 50 평지 보행 적당
    F_T_L_x_input_dot = -0.1 * (Tau_L_x - l_ft_LPF(3)) - 50.0 * F_T_L_x_input;
    F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;
    //F_T_L_x_input = 0;
    F_T_R_x_input_dot = -0.1 * (Tau_R_x - r_ft_LPF(3)) - 50.0 * F_T_R_x_input;
    F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
    //F_T_R_x_input = 0;
    //Pitch 방향
    F_T_L_y_input_dot = 0.1 * (Tau_L_y - l_ft_LPF(4)) - 50.0 * F_T_L_y_input;
    F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;
    //F_T_L_y_input = 0;
    F_T_R_y_input_dot = 0.1 * (Tau_R_y - r_ft_LPF(4)) - 50.0 * F_T_R_y_input;
    F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t; 
    //F_T_R_y_input = 0;
    if (F_T_L_x_input >= 0.15) // 8.5 deg limit
    {
        F_T_L_x_input = 0.15;
    }
    else if (F_T_L_x_input < -0.15)
    {
        F_T_L_x_input = -0.15;
    }
    if (F_T_R_x_input >= 0.15) // 8.5 deg limit
    {
        F_T_R_x_input = 0.15;
    }
    else if (F_T_R_x_input < -0.15)
    {
        F_T_R_x_input = -0.15;
    }
    if (F_T_L_y_input >= 0.15) // 8.5 deg limit
    {
        F_T_L_y_input = 0.15;
    }
    else if (F_T_L_y_input < -0.15)
    {
        F_T_L_y_input = -0.15;
    }
    if (F_T_R_y_input >= 0.15) // 8.5 deg limit
    {
        F_T_R_y_input = 0.15;
    }
    else if (F_T_R_y_input < -0.15)
    {
        F_T_R_y_input = -0.15;
    }    
  
    // MJ_graph << stepping_input_(0) << "," << stepping_input_(1) << "," << t_total_ / hz_ << "," << ZMP_Y_REF_alpha_ + del_zmp(1) << "," << ZMP_Y_REF_alpha_ << endl;
}

void AvatarController::updateNextStepTime()
{       
    if (walking_tick_mj == t_last_)
    {   
        if (current_step_num_ != total_step_num_ - 1)
        {   
            // t_total_ = t_total_ + 0.05*hz_;
            t_start_ = t_last_ + 1;
            t_start_real_ = t_start_ + t_rest_init_;
            t_last_ = t_start_ + t_total_ - 1;
            current_step_num_++;            
        }
        
    }
    if (current_step_num_ == total_step_num_ - 1 && walking_tick_mj >= t_last_ + t_total_)
    {
        // walking_enable_ = false;
        // cout << "Last " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_mj_(2) * 180 / 3.141592 << endl;
    }
    else
    {
        walking_tick_mj++;
    }
}

void AvatarController::computeCAMcontrol_HQP()
{
    // const int hierarchy_num_camhqp_ = 2;
    // const int variable_size_camhqp_ = 8; 
    // const int constraint_size1_camhqp_ = 8; //[lb <=	x	<= 	ub] form constraints
    // const int constraint_size2_camhqp_[hierarchy_num_camhqp_] = {0, 3};	//[lb <=	Ax 	<=	ub] or [Ax = b]
    // const int control_size_camhqp_[hierarchy_num_camhqp_] = {3, 8}; //1: CAM control, 2: init pose
    if (first_loop_camhqp_)
    {
        for (int i = 0; i < hierarchy_num_camhqp_; i++)
        {   
            QP_cam_hqp_.resize(hierarchy_num_camhqp_);
            QP_cam_hqp_[i].InitializeProblemSize(variable_size_camhqp_, constraint_size2_camhqp_[i]);
            J_camhqp_[i].setZero(control_size_camhqp_[i], variable_size_camhqp_);
            u_dot_camhqp_[i].setZero(control_size_camhqp_[i]);

            ubA_camhqp_[i].setZero(constraint_size2_camhqp_[i]);
            lbA_camhqp_[i].setZero(constraint_size2_camhqp_[i]);

            H_camhqp_[i].setZero(variable_size_camhqp_, variable_size_camhqp_);
            g_camhqp_[i].setZero(variable_size_camhqp_);

            ub_camhqp_[i].setZero(constraint_size1_camhqp_);
            lb_camhqp_[i].setZero(constraint_size1_camhqp_);

            q_dot_camhqp_[i].setZero(variable_size_camhqp_);

            w1_camhqp_[0] = 2500; // |A*qdot - h|
            w2_camhqp_[0] = 50.0; // |q_dot| 
            
            w1_camhqp_[1] = 2500; // |q_dot - q_dot_zero|
            w2_camhqp_[1] = 0.0; // |q_dot| 
        }
         
        control_joint_idx_camhqp_[0] = 13; // waist pitch
        control_joint_idx_camhqp_[1] = 14; // waist roll

        control_joint_idx_camhqp_[2] = 15; // left shoulder yaw 
        control_joint_idx_camhqp_[3] = 16; // left shoulder pitch
        control_joint_idx_camhqp_[4] = 17; // left shoulder roll
        control_joint_idx_camhqp_[5] = 18; // left elbow yaw

        control_joint_idx_camhqp_[6] = 25; // right shoulder yaw
        control_joint_idx_camhqp_[7] = 26; // right shoulder pitch
        control_joint_idx_camhqp_[8] = 27; // right shoulder roll      
        control_joint_idx_camhqp_[9] = 28; // right elbow yaw       

        last_solved_hierarchy_num_camhqp_ = -1;
        first_loop_camhqp_ = false;
    }
    
    Eigen::VectorXd q_test, q_dot_test;
    q_test = rd_.q_virtual_;
    // modify the virtual joint value from the value expressed in global frame to be the value expressed in the base frame
    // Eigen::Vector6d base_velocity;
    // base_velocity = rd_.q_dot_virtual_.segment(0, 6);
    // base_velocity.segment(0, 3) = rd_.link_[Pelvis].rotm.transpose() * base_velocity.segment(0, 3); // virtual joint velocity of the robot w.r.t pelvis frame
    // base_velocity.segment(3, 3) = rd_.link_[Pelvis].rotm.transpose() * base_velocity.segment(3, 3);

    q_test.segment(0, 6).setZero();
    q_test.segment(18, 21) = motion_q_pre_.segment(12,21);
    q_test(39) = 1;
    // q_dot_test = rd_.q_dot_virtual_;
    // q_dot_test.segment(0, 6) = base_velocity;

    Eigen::MatrixXd mass_matrix_temp;
    Eigen::MatrixXd cmm;
    Eigen::MatrixXd cmm_support;
    Eigen::MatrixXd cmm_support1;
    Eigen::MatrixXd sel_matrix;
    Eigen::MatrixXd cmm_selected;
    Eigen::Vector2d eps;
    mass_matrix_temp.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    cmm.setZero(3, MODEL_DOF);
    cmm_support1.setZero(3, MODEL_DOF);
    cmm_support.setZero(2, MODEL_DOF);
    // sel_matrix.setZero(MODEL_DOF, MODEL_DOF);
    sel_matrix.setZero(MODEL_DOF, variable_size_camhqp_);
    cmm_selected.setZero(2, variable_size_camhqp_);
    // Defined the selection matrix //
    for(int i=0; i < variable_size_camhqp_; i++) // eight joints in upper body for CMP control
    { 
        sel_matrix(control_joint_idx_camhqp_[i], i) = 1.0;  
    } 
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_MJ_, q_test, mass_matrix_temp, true);

    getCentroidalMomentumMatrix(mass_matrix_temp, cmm);
    cmm_support1 = pelv_support_current_.linear() * cmm; // considering the gravity direction
    cmm_support = cmm_support1.block<2, MODEL_DOF>(0,0);
    
    cmm_selected = cmm_support * sel_matrix;
    
    Eigen::Vector2d del_ang_momentum_slow_2;
    del_ang_momentum_slow_2 = del_ang_momentum_slow_.segment(0, 2); 
    J_camhqp_[0] = cmm_selected;
    
    u_dot_camhqp_[0] = del_ang_momentum_slow_2;
    J_camhqp_[1].setIdentity(variable_size_camhqp_, variable_size_camhqp_);
    u_dot_camhqp_[1].setZero(control_size_camhqp_[1]); 
     
    for(int i =0; i < control_size_camhqp_[1]; i++) 
    {
        // recovery strategy
        u_dot_camhqp_[1](i) = 20*(CAM_upper_init_q_(control_joint_idx_camhqp_[i]) - motion_q_pre_(control_joint_idx_camhqp_[i]));        
    }
     
    for (int i = 0; i < hierarchy_num_camhqp_; i++)
    {
        if (i > last_solved_hierarchy_num_camhqp_)
        {
            QP_cam_hqp_[i].InitializeProblemSize(variable_size_camhqp_, constraint_size2_camhqp_[i]);
        }
    }

    last_solved_hierarchy_num_camhqp_ = -1;
    for (int i = 0; i < hierarchy_num_camhqp_; i++)
    {
        MatrixXd H1, H2;
        VectorXd g1, g2;

        H1 = J_camhqp_[i].transpose() * J_camhqp_[i];
        H2 = Eigen::MatrixXd::Identity(variable_size_camhqp_, variable_size_camhqp_);
        
        g1 = -J_camhqp_[i].transpose() * u_dot_camhqp_[i]; // (variable_size_camhqp_ x 1 (i.e. 6x1))
        g2.setZero(variable_size_camhqp_);  
          
          
        H_camhqp_[i] = w1_camhqp_[i] * H1 + w2_camhqp_[i] * H2 ; 
        g_camhqp_[i] = w1_camhqp_[i] * g1 + w2_camhqp_[i] * g2 ; 

        double speed_reduce_rate = 40; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.
            
        // MJ's joint limit 
        lb_camhqp_[i](0) = min(max(speed_reduce_rate * (-15.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[0])), joint_vel_limit_l_(control_joint_idx_camhqp_[0])), joint_vel_limit_h_(control_joint_idx_camhqp_[0]));
        ub_camhqp_[i](0) = max(min(speed_reduce_rate * (15.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[0])), joint_vel_limit_h_(control_joint_idx_camhqp_[0])), joint_vel_limit_l_(control_joint_idx_camhqp_[0]));
        lb_camhqp_[i](1) = min(max(speed_reduce_rate * (-15.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[1])), joint_vel_limit_l_(control_joint_idx_camhqp_[1])), joint_vel_limit_h_(control_joint_idx_camhqp_[1]));
        ub_camhqp_[i](1) = max(min(speed_reduce_rate * (15.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[1])), joint_vel_limit_h_(control_joint_idx_camhqp_[1])), joint_vel_limit_l_(control_joint_idx_camhqp_[1]));
        // Left Shoulder yaw  
        lb_camhqp_[i](2) = min(max(speed_reduce_rate * (-30*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[2])), joint_vel_limit_l_(control_joint_idx_camhqp_[2])), joint_vel_limit_h_(control_joint_idx_camhqp_[2]));
        ub_camhqp_[i](2) = max(min(speed_reduce_rate * (+20*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[2])), joint_vel_limit_h_(control_joint_idx_camhqp_[2])), joint_vel_limit_l_(control_joint_idx_camhqp_[2]));
        // Left Shoulder pitch // Init 17 deg, CAM 10 deg // Roll 방향 test -20~20?
        lb_camhqp_[i](3) = min(max(speed_reduce_rate * (-50*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[3])), joint_vel_limit_l_(control_joint_idx_camhqp_[3])), joint_vel_limit_h_(control_joint_idx_camhqp_[3]));
        ub_camhqp_[i](3) = max(min(speed_reduce_rate * (50*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[3])), joint_vel_limit_h_(control_joint_idx_camhqp_[3])), joint_vel_limit_l_(control_joint_idx_camhqp_[3]));
        // Left Shoulder roll // Init 86 deg, CAM 75 deg // 80 deg 보다 크면 몸통 부딪힘.  
        lb_camhqp_[i](4) = min(max(speed_reduce_rate * (45*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[4])), joint_vel_limit_l_(control_joint_idx_camhqp_[4])), joint_vel_limit_h_(control_joint_idx_camhqp_[4]));
        ub_camhqp_[i](4) = max(min(speed_reduce_rate * (65*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[4])), joint_vel_limit_h_(control_joint_idx_camhqp_[4])), joint_vel_limit_l_(control_joint_idx_camhqp_[4]));
        // Left elbow yaw // Init -72 deg, CAM -70 deg +40 deg 보다 크면 왼쪽 옆구리에 부딪힘. 
        lb_camhqp_[i](5) = min(max(speed_reduce_rate * (-90*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[5])), joint_vel_limit_l_(control_joint_idx_camhqp_[5])), joint_vel_limit_h_(control_joint_idx_camhqp_[5]));
        ub_camhqp_[i](5) = max(min(speed_reduce_rate * (-65*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[5])), joint_vel_limit_h_(control_joint_idx_camhqp_[5])), joint_vel_limit_l_(control_joint_idx_camhqp_[5]));

        // Right Shoulder yaw      // Y dir disturbance -> yaw joint, pitch joint singular 느낌.  Y dir일때 pitch joint 거의없애야할듯..
        lb_camhqp_[i](6) = min(max(speed_reduce_rate * (-20*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[6])), joint_vel_limit_l_(control_joint_idx_camhqp_[6])), joint_vel_limit_h_(control_joint_idx_camhqp_[6]));
        ub_camhqp_[i](6) = max(min(speed_reduce_rate * (+30*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[6])), joint_vel_limit_h_(control_joint_idx_camhqp_[6])), joint_vel_limit_l_(control_joint_idx_camhqp_[6]));
        // Right Shoulder pitch   
        lb_camhqp_[i](7) = min(max(speed_reduce_rate * (-50*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[7])), joint_vel_limit_l_(control_joint_idx_camhqp_[7])), joint_vel_limit_h_(control_joint_idx_camhqp_[7]));
        ub_camhqp_[i](7) = max(min(speed_reduce_rate * (+50*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[7])), joint_vel_limit_h_(control_joint_idx_camhqp_[7])), joint_vel_limit_l_(control_joint_idx_camhqp_[7]));
        // Right Shoulder roll
        lb_camhqp_[i](8) = min(max(speed_reduce_rate * (-65*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[8])), joint_vel_limit_l_(control_joint_idx_camhqp_[8])), joint_vel_limit_h_(control_joint_idx_camhqp_[8]));
        ub_camhqp_[i](8) = max(min(speed_reduce_rate * (-45*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[8])), joint_vel_limit_h_(control_joint_idx_camhqp_[8])), joint_vel_limit_l_(control_joint_idx_camhqp_[8]));
        // Right elbow yaw 
        lb_camhqp_[i](9) = min(max(speed_reduce_rate * (+65*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[9])), joint_vel_limit_l_(control_joint_idx_camhqp_[9])), joint_vel_limit_h_(control_joint_idx_camhqp_[9]));
        ub_camhqp_[i](9) = max(min(speed_reduce_rate * (+90*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[9])), joint_vel_limit_h_(control_joint_idx_camhqp_[9])), joint_vel_limit_l_(control_joint_idx_camhqp_[9]));
                      
        int higher_task_equality_num = 0;        
     
        for(int k = 0; k < constraint_size2_camhqp_[1]; k++)
        {
            eps(k) = DyrosMath::cubic(del_ang_momentum_slow_2.norm(), 1, 3, 1.0, 0.0, 0.0, 0.0); // 빠르게 돌리고 싶을때 늘리기..
        }

        for (int h = 0; h < i; h++)
        {
            if (constraint_size2_camhqp_[i] == 0)
            {
                break;
            }

            A_camhqp_[i].setZero(constraint_size2_camhqp_[i], variable_size_camhqp_);
            A_camhqp_[i].block(higher_task_equality_num, 0, control_size_camhqp_[h], variable_size_camhqp_) = J_camhqp_[h];
            ubA_camhqp_[i].segment(higher_task_equality_num, control_size_camhqp_[h]) = J_camhqp_[h] * q_dot_camhqp_[i-1] + eps;
            lbA_camhqp_[i].segment(higher_task_equality_num, control_size_camhqp_[h]) = J_camhqp_[h] * q_dot_camhqp_[i-1] - eps;
            higher_task_equality_num += control_size_camhqp_[h]; 
        }
 
        QP_cam_hqp_[i].EnableEqualityCondition(equality_condition_eps_);
        QP_cam_hqp_[i].UpdateMinProblem(H_camhqp_[i], g_camhqp_[i]);
        if(constraint_size2_camhqp_[i] == 0)
        {             
            QP_cam_hqp_[i].DeleteSubjectToAx();
        }
        else
        {
            QP_cam_hqp_[i].UpdateSubjectToAx(A_camhqp_[i], lbA_camhqp_[i], ubA_camhqp_[i]);
        }
        
        if(constraint_size1_camhqp_ == 0)
        {
            QP_cam_hqp_[i].DeleteSubjectToX();
        }
        else
        {
            QP_cam_hqp_[i].UpdateSubjectToX(lb_camhqp_[i], ub_camhqp_[i]);
        }        

        if (QP_cam_hqp_[i].SolveQPoases(200, qpres_camhqp_))
        {   
            q_dot_camhqp_[i] = qpres_camhqp_.segment(0, variable_size_camhqp_);

            last_solved_hierarchy_num_camhqp_ = i;
        }
        else
        {
            q_dot_camhqp_[i].setZero();
            
            std::cout << "Error hierarchy: " << i << std::endl;
            std::cout << "last solved q_dot: " << q_dot_camhqp_[last_solved_hierarchy_num_camhqp_].transpose() << std::endl;
             
            break;
        }
    }
        
    for (int i = 0; i < variable_size_camhqp_; i++)
    {
        //motion_q_dot_(control_joint_idx_camhqp_[i]) = q_dot_camhqp_[0](i); // first hierarchy solution
        motion_q_dot_(control_joint_idx_camhqp_[i]) = q_dot_camhqp_[last_solved_hierarchy_num_camhqp_](i);
        motion_q_(control_joint_idx_camhqp_[i]) = motion_q_pre_(control_joint_idx_camhqp_[i]) + motion_q_dot_(control_joint_idx_camhqp_[i]) * 0.0005;
        pd_control_mask_(control_joint_idx_camhqp_[i]) = 1;
    }
    // MJ_graph2 << motion_q_(25) << "," << motion_q_(26) << "," << motion_q_(27) << "," << motion_q_(28) << "," << motion_q_(29) << endl;

    // CAM calculation based on actual joint angular velocity    
    Eigen::VectorXd cam_a;
    Eigen::VectorXd real_q_dot_hqp_; 
    cam_a.setZero(3);
    real_q_dot_hqp_.setZero(10);
    
    for(int i = 0; i < variable_size_camhqp_; i++)
    {
      real_q_dot_hqp_(i) = rd_.q_dot_(control_joint_idx_camhqp_[i]);  
    }
    cam_a = -J_camhqp_[0]*real_q_dot_hqp_;
    CAM_real_.setZero();
    CAM_real_(0) = cam_a(0);
    CAM_real_(1) = cam_a(1);
    // CAM_real_ = cam_a;
    // CAM calculation based on commanded joint angular velocity    
    Eigen::VectorXd cam_c;
    Eigen::VectorXd cmd_q_dot_hqp;
    cam_c.setZero(3);
    cmd_q_dot_hqp.setZero(10);

    for(int i = 0; i < variable_size_camhqp_; i++)
    {
      cmd_q_dot_hqp(i) = motion_q_dot_(control_joint_idx_camhqp_[i]);  
    }
    cam_c = -J_camhqp_[0]*cmd_q_dot_hqp;
    // CAM_cmd_.setZero();
    // CAM_cmd_ = cam_c;
    // whole-body real _ CAM
    cam_wholebody_.setZero();

    cam_wholebody_ = cmm_support * rd_.q_dot_;

    // MJ_graph << CAM_real_(0) << "," << CAM_real_(1) << endl;    
}   

void AvatarController::computeSlow()
{
    queue_avatar_.callAvailable(ros::WallDuration());

    if (rd_.tc_.mode == 10)
    {
        if (initial_flag == 0)
        {
            Joint_gain_set_MJ();
            walking_enable_ = true;
            // Initial pose
            ref_q_ = rd_.q_;
            for (int i = 0; i < 12; i++)
            {
                Initial_ref_q_(i) = ref_q_(i);
            }
            
            // Saving for initial upper body pose
            // edited by MJ (Initial upper body trajectory generation for CAM control /220110)
            CAM_upper_init_q_.setZero();
            Initial_ref_upper_q_.setZero();
            for (int i = 12; i < MODEL_DOF; i++)
            {
                Initial_ref_upper_q_(i) = ref_q_(i);
            }
            
            CAM_upper_init_q_(15) = + 15.0 * DEG2RAD; // Left Shoulder Yaw joint // 17 deg
            CAM_upper_init_q_(16) = + 10.0 * DEG2RAD; // Left Shoulder Pitch joint // 17 deg
            CAM_upper_init_q_(17) = + 65.0 * DEG2RAD; // Left Shoulder Roll joint // 86 deg
            CAM_upper_init_q_(18) = - 70.0 * DEG2RAD; // Left Elbow Yaw joint // -72 deg
            // CAM_upper_init_q_(19) = - 65.0 * DEG2RAD; // Left Elbow Pitch joint // -57 deg

            CAM_upper_init_q_(25) = - 15.0 * DEG2RAD; // Right Shoulder Yaw joint // -17 deg
            CAM_upper_init_q_(26) = - 10.0 * DEG2RAD; // Right Shoulder Pitch joint           
            CAM_upper_init_q_(27) = - 65.0 * DEG2RAD; // Right Shoulder Roll joint 
            CAM_upper_init_q_(28) = + 70.0 * DEG2RAD; // Right Elbow Yaw joint
            // CAM_upper_init_q_(29) = + 65.0 * DEG2RAD; // Right Elbow Pich joint                       
            
            q_prev_MJ_ = rd_.q_;
            walking_tick_mj = 0;
            walking_end_flag = 0;
            parameterSetting();
            cout << "computeslow mode = 10 is initialized" << endl;
            cout << "time: "<<rd_.control_time_ << endl; //dg add

            WBC::SetContact(rd_, 1, 1);
            Gravity_MJ_ = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);
            //Gravity_MJ_.setZero();
            atb_grav_update_ = false;
            initial_flag = 1;
        }

        if (atb_grav_update_ == false)
        {
            atb_grav_update_ = true;
            Gravity_MJ_fast_ = Gravity_MJ_;
            atb_grav_update_ = false;
        }  

        // edited by MJ (Initial upper body trajectory generation for CAM control /220110)
        if(initial_tick_mj <= 2.0 * hz_)
        {
            ref_q_(15) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(15), CAM_upper_init_q_(15), 0.0, 0.0); // Left Shoulder Yaw joint
            ref_q_(16) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(16), CAM_upper_init_q_(16), 0.0, 0.0); // Left Shoulder Pitch joint
            ref_q_(17) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(17), CAM_upper_init_q_(17), 0.0, 0.0); // Left Shoulder Roll joint
            ref_q_(18) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(18), CAM_upper_init_q_(18), 0.0, 0.0); // Left Elbow Yaw joint
            // ref_q_(19) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(19), CAM_upper_init_q_(19), 0.0, 0.0);  // Left Elbow Pitch joint
            
            ref_q_(25) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(25), CAM_upper_init_q_(25), 0.0, 0.0); // Right Shoulder Yaw joint
            ref_q_(26) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(26), CAM_upper_init_q_(26), 0.0, 0.0); // Right Shoulder Pitch joint  
            ref_q_(27) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(27), CAM_upper_init_q_(27), 0.0, 0.0); // Right Shoulder Roll joint 
            ref_q_(28) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(28), CAM_upper_init_q_(28), 0.0, 0.0); // Right Elbow Yaw joint
            // ref_q_(29) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(29), CAM_upper_init_q_(29), 0.0, 0.0); // Right Elbow Pich joint  
                        
            initial_tick_mj ++;         
        }
        
        for (int i = 0; i < MODEL_DOF; i++)
        {
            rd_.torque_desired(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
        }        

    }
    else if (rd_.tc_.mode == 11)
    {
        ////////////////////////////////////////////////////////////////////////////
        /////////////////// Biped Walking Controller made by MJ ////////////////////
        ////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////
        if (walking_enable_ == true)
        {
            if (walking_tick_mj == 0)
            {
                parameterSetting();
                initial_flag = 0;

                atb_grav_update_ = false;
                atb_desired_q_update_ = false;
                atb_walking_traj_update_ = false;
                torque_upper_fast_.setZero();
                torque_upper_fast_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);
                torque_upper_.setZero();
                torque_upper_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);

                cout << "parameter setting OK" << endl;
                cout << "mode = 11" << endl;
            }
               
            updateInitialState();
            getRobotState();
            floatToSupportFootstep();
            
            if (current_step_num_ < total_step_num_)
            {                 
                getZmpTrajectory();
                // getComTrajectory(); // 조현민꺼에서 프리뷰에서 CP 궤적을 생성하기 때문에 필요                   
                getComTrajectory_mpc(); // working with thread3 (MPC thread)                    
                CPMPC_bolt_Controller_MJ();                 
                // CPMPC_bolt_Controller_MJ_ICRA();
                // BoltController_MJ(); // Stepping Controller for DCM eos                
                
                CentroidalMomentCalculator_new(); // working with computefast() (CAM controller)

                // getFootTrajectory(); 
                getFootTrajectory_stepping(); // working with CPMPC_bolt_Controller_MJ()  
                getPelvTrajectory();
                supportToFloatPattern();
                
                // STEP1: send desired AM to the slow thread
                if (atb_walking_traj_update_ == false)
                {
                    atb_walking_traj_update_ = true;
                    del_ang_momentum_fast_ = del_ang_momentum_;
                    atb_walking_traj_update_ = false;
                }
                
                computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_);
                Compliant_control(q_des_);
                for (int i = 0; i < 12; i++)
                {
                    // ref_q_(i) = q_des_(i);
                    ref_q_pre_(i) = ref_q_(i);
                    ref_q_(i) = DOB_IK_output_(i);
                }
                //hip_compensator();
                //GravityCalculate_MJ();

                if (atb_grav_update_ == false)
                {
                    atb_grav_update_ = true;
                    Gravity_MJ_fast_ = Gravity_MJ_;
                    atb_grav_update_ = false;
                }

                if (walking_tick_mj < 1.0 * hz_)
                {
                    //for leg
                    for (int i = 0; i < 12; i++)
                    {                           
                        ref_q_(i) = DyrosMath::cubic(walking_tick_mj, 0, 1.0 * hz_, Initial_ref_q_(i), q_des_(i), 0.0, 0.0);
                    }
                    //for waist
                    // ref_q_(13) = Initial_ref_q_(13);
                    // ref_q_(14) = Initial_ref_q_(14);
                    // //for arm
                    // ref_q_(16) = Initial_ref_q_(16);
                    // ref_q_(26) = Initial_ref_q_(26);
                    // ref_q_(17) = DyrosMath::cubic(walking_tick_mj, 0, 1.0 * hz_, Initial_ref_q_(17), 50.0 * DEG2RAD, 0.0, 0.0);  // + direction angle makes the left arm down.
                    // ref_q_(27) = DyrosMath::cubic(walking_tick_mj, 0, 1.0 * hz_, Initial_ref_q_(27), -50.0 * DEG2RAD, 0.0, 0.0); // - direction angle makes the right arm down.
                }
                del_zmp(0) = 1.4 * (cp_measured_(0) - cp_desired_(0));
                del_zmp(1) = 1.3 * (cp_measured_(1) - cp_desired_(1));
                CP_compen_MJ();
                CP_compen_MJ_FT();
                torque_lower_.setZero();
                for (int i = 0; i < 12; i++)
                {
                    torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + Tau_CP(i) + Gravity_MJ_fast_(i);
                    // 4 (Ankle_pitch_L), 5 (Ankle_roll_L), 10 (Ankle_pitch_R),11 (Ankle_roll_R)
                }


                desired_q_not_compensated_ = ref_q_;
                updateNextStepTime();
                q_prev_MJ_ = rd_.q_;
                                
                if(current_step_num_ == 4 && (walking_tick_mj >= t_start_ + 0.15*hz_ + 0.6*0.3*hz_)  && (walking_tick_mj < t_start_ + 0.15*hz_ + 0.6*0.3*hz_ + 0.2*hz_))
                {    
                    mujoco_applied_ext_force_.data[0] = force_temp_*sin(theta_temp_*DEG2RAD); //x-axis linear force
                    mujoco_applied_ext_force_.data[1] = -force_temp_*cos(theta_temp_*DEG2RAD); //y-axis linear force  
                    mujoco_applied_ext_force_.data[2] = 0.0; //z-axis linear force
                    mujoco_applied_ext_force_.data[3] = 0.0; //x-axis angular moment
                    mujoco_applied_ext_force_.data[4] = 0.0; //y-axis angular moment
                    mujoco_applied_ext_force_.data[5] = 0.0; //z-axis angular moment

                    mujoco_applied_ext_force_.data[6] = 1; //link idx; 1:pelvis

                    mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force_);  
                     
                } 
                else
                {
                    mujoco_applied_ext_force_.data[0] = 0; //x-axis linear force
                    mujoco_applied_ext_force_.data[1] = 0; //y-axis linear force
                    mujoco_applied_ext_force_.data[2] = 0; //z-axis linear force
                    mujoco_applied_ext_force_.data[3] = 0; //x-axis angular moment
                    mujoco_applied_ext_force_.data[4] = 0; //y-axis angular moment
                    mujoco_applied_ext_force_.data[5] = 0; //z-axis angular moment

                    mujoco_applied_ext_force_.data[6] = 1; //link idx; 1:pelvis

                    mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force_);
                }
            }
        }
        else
        {
            if (walking_end_flag == 0)
            {
                cout << "walking finish" << endl;
                walking_end_flag = 1;
                initial_flag = 0;
            }

            if (atb_grav_update_ == false)
            {
                atb_grav_update_ = true;
                Gravity_MJ_fast_ = Gravity_MJ_;
                atb_grav_update_ = false;
            }

            torque_lower_.setZero();
            for (int i = 0; i < 12; i++)
            {
                torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + Gravity_MJ_fast_(i);
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////
        if (atb_desired_q_update_ == false)
        {
            atb_desired_q_update_ = true;
            desired_q_fast_ = desired_q_slow_;
            desired_q_dot_fast_ = desired_q_dot_slow_;
            atb_desired_q_update_ = false;
        }

        torque_upper_.setZero();
        for (int i = 12; i < MODEL_DOF; i++)
        {
            torque_upper_(i) = (kp_joint_(i) * (desired_q_fast_(i) - rd_.q_(i)) + kv_joint_(i) * (desired_q_dot_fast_(i) - rd_.q_dot_(i)) + Gravity_MJ_fast_(i));
            //Is there any problem if i write the code as below? - myeongju-
            // torque_upper_(i) = (Kp(i) * (ref_q_(i) - del_cmm_q_(i) - rd_.q_(i)) + Kd(i) * (0.0 - rd_.q_dot_(i)) + 1.0 * Gravity_MJ_fast_(i));
            // torque_upper_(i) = torque_upper_(i) * pd_control_mask_(i); // masking for joint pd control
        }

        ///////////////////////////////FINAL TORQUE COMMAND/////////////////////////////
        rd_.torque_desired = torque_lower_ + torque_upper_;
        ///////////////////////////////////////////////////////////////////////////////
    }
}

void AvatarController::computeFast()
{
    if (rd_.tc_.mode == 10)
    {
        if (initial_flag == 1)
        {
            WBC::SetContact(rd_, 1, 1);

            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
                //cout<<"comutefast tc.mode =10 is initialized"<<endl;
            }
            //initial_flag = 2;
        }
    }
    else if (rd_.tc_.mode == 11)
    {
        ////////////////////////////////////////////////////////////////////////////
        /////////////////// Biped Walking Controller made by MJ ////////////////////
        ////////////////////////////////////////////////////////////////////////////
        if (walking_enable_ == true)
        {
            if (current_step_num_ < total_step_num_)
            {
                GravityCalculate_MJ();

                //STEP2: recieve desired AM
                if(walking_tick_mj >= 1)
                {
                    if (atb_walking_traj_update_ == false)
                    {
                        atb_walking_traj_update_ = true;
                        del_ang_momentum_slow_ = del_ang_momentum_fast_;
                        atb_walking_traj_update_ = false;
                    }
                }
            }
        }
        else
        {
            WBC::SetContact(rd_, 1, 1);
            int support_foot;
            if (foot_step_(current_step_num_, 6) == 1)
            {
                support_foot = 1;
            }
            else
            {
                support_foot = 0;
            }
             
            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, support_foot);

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////

        if (rd_.tc_init == true)
        {
            initWalkingParameter();
            rd_.tc_init = false;
        }

        //data process//
        getRobotData();
        walkingStateManager(); //avatar
        getProcessedRobotData();

        //motion planing and control//
        motionGenerator();
        //STEP3: Compute q_dot for CAM control
         
        computeCAMcontrol_HQP();
         
        for (int i = 12; i < MODEL_DOF; i++)
        {
            desired_q_(i) = motion_q_(i);
            desired_q_dot_(i) = motion_q_dot_(i); 
        }

        //STEP4: send desired q to the fast thread
        if (atb_desired_q_update_ == false)
        {
            atb_desired_q_update_ = true;
            desired_q_slow_ = desired_q_;
            desired_q_dot_slow_ = desired_q_dot_;
            atb_desired_q_update_ = false;
        }
        // if (atb_desired_q_update_ == false)
        // {
        //     atb_desired_q_update_ = true;
        //     torque_upper_.setZero();
        //     for (int i = 12; i < MODEL_DOF; i++)
        //     {
        //         torque_upper_(i) = (kp_joint_(i) * (desired_q_(i) - current_q_(i)) + kv_joint_(i) * (desired_q_dot_(i) - current_q_dot_(i)) + 1.0 * Gravity_MJ_(i));
        //         torque_upper_(i) = torque_upper_(i) * pd_control_mask_(i); // masking for joint pd control
        //     }
        //     atb_desired_q_update_ = false;
        // }

        savePreData();

        // printOutTextFile();
    }
}

