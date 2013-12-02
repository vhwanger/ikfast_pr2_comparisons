#include <ikfast_pr2_comparisons/comparison.h>
#include <time.h>

using namespace std;

Tester::Tester():counter(0),kdl_c(0),
                 ikfast_c(0),ikfast_time(0), kdl_time(0), kdl_fk(0), 
                 ikfast_fk(0){ 


    FILE* fp_arm= fopen("pr2_right_arm.cfg", "r");
    if (!fp_arm){ ROS_ERROR("Couldn't open right arm model file"); }
    m_arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_arm);
    m_arm_model->setResolution(.02);
    ROS_INFO("getting kdl chain from paramserver");
    m_arm_model->initKDLChainFromParamServer();
}

bool Tester::run_ikfast_test(const KDL::Frame& wrist_frame,
                             double free_angle){
    double wroll, wpitch, wyaw;
    wrist_frame.M.GetRPY(wroll, wpitch, wyaw);

    vector<double> ik_angles;
    struct timeval tv_b;
    struct timeval tv_a;
    gettimeofday(&tv_b, NULL);
    long unsigned int before = tv_b.tv_usec + (tv_b.tv_sec * 1000000);
    bool fastik_success = ik_solver.ikRightArm(wrist_frame, 
                                               free_angle, &ik_angles);
    gettimeofday(&tv_a, NULL);
    long unsigned int after = tv_a.tv_usec + (tv_a.tv_sec * 1000000);
    ikfast_time += after - before;
    printf("ikfast_time: %lu", after-before);

    if (fastik_success && fabs(ik_angles[2]-free_angle) < .0001){
    printf("ik angles %f %f %f %f %f %f %f\n",
            ik_angles[0],
            ik_angles[1],
            ik_angles[2],
            ik_angles[3],
            ik_angles[4],
            ik_angles[5],
            ik_angles[6]);
        gettimeofday(&tv_b, NULL);
        before = tv_b.tv_usec + (tv_b.tv_sec * 1000000);
        KDL::Frame obj_frame;
        m_arm_model->computeArmFK(ik_angles, 10, &obj_frame);
        gettimeofday(&tv_a, NULL);
        after = tv_a.tv_usec + (tv_a.tv_sec * 1000000);
        ikfast_fk += after - before;

        double roll, pitch, yaw;
        obj_frame.M.GetRPY(roll, pitch, yaw);
        assert(fabs(wrist_frame.p.x()-obj_frame.p.x()) < .001);
        assert(fabs(wrist_frame.p.y()-obj_frame.p.y()) < .001);
        assert(fabs(wrist_frame.p.z()-obj_frame.p.z()) < .001);
        assert(fabs(wroll-roll) < .0001);
        assert(fabs(wpitch-pitch) < .0001);
        assert(fabs(wyaw-yaw) < .0001);
        ikfast_c++;
    } else {
        ROS_ERROR("fast ik failed");
    }
    return fastik_success;
}

bool Tester::run_kdl_test(const KDL::Frame& wrist_frame, 
                          const vector<double>& seed_angles){
    double wroll, wpitch, wyaw;
    wrist_frame.M.GetRPY(wroll, wpitch, wyaw);

    geometry_msgs::Pose pose;
    pose.position.x = wrist_frame.p.x();
    pose.position.y = wrist_frame.p.y();
    pose.position.z = wrist_frame.p.z();
    wrist_frame.M.GetQuaternion(pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w);
    struct timeval tv_b;
    struct timeval tv_a;
    long unsigned before, after;
    vector<double> kdl_angles(7,0);
    gettimeofday(&tv_b, NULL);
    before = tv_b.tv_usec + (tv_b.tv_sec * 1000000);
    bool ik_success = m_arm_model->computeFastIK(wrist_frame, seed_angles, kdl_angles);
    if (!ik_success){
        ik_success = m_arm_model->computeIK(wrist_frame, seed_angles, kdl_angles);
    }
    gettimeofday(&tv_a, NULL);
    after = tv_a.tv_usec + (tv_a.tv_sec * 1000000);
    kdl_time += after - before;

    if (ik_success && fabs(kdl_angles[2]-seed_angles[2]) < .0001){
        printf("kdl time %lu\n", after-before);

        printf("kdl seed angle %f", seed_angles[2]);
        printf("kdl angles %f %f %f %f %f %f %f\n",
                kdl_angles[0],
                kdl_angles[1],
                kdl_angles[2],
                kdl_angles[3],
                kdl_angles[4],
                kdl_angles[5],
                kdl_angles[6]);

        KDL::Frame obj_frame;
        m_arm_model->computeArmFK(kdl_angles, 10, &obj_frame);
        double roll, pitch, yaw;
        obj_frame.M.GetRPY(roll, pitch, yaw);
        assert(fabs(wrist_frame.p.x()-obj_frame.p.x()) < .001);
        assert(fabs(wrist_frame.p.y()-obj_frame.p.y()) < .001);
        assert(fabs(wrist_frame.p.z()-obj_frame.p.z()) < .001);
        assert(fabs(wroll-roll) < .0001);
        assert(fabs(wpitch-pitch) < .0001);
        assert(fabs(wyaw-yaw) < .0001);
        kdl_c++;
    }

    vector<double> temp_pose(6,0);
    KDL::Frame towrist;
    gettimeofday(&tv_b, NULL);
    before = tv_b.tv_usec + (tv_b.tv_sec * 1000000);
    m_arm_model->computeArmFK(kdl_angles, 10, &towrist);
    gettimeofday(&tv_a, NULL);
    after = tv_a.tv_usec + (tv_a.tv_sec * 1000000);
    kdl_fk += after - before;
    return ik_success;
}


void Tester::run_ik(const sensor_msgs::JointState& msg){
    std::vector<double> angles;
    angles.push_back(msg.position[18]);
    angles.push_back(msg.position[19]);
    angles.push_back(msg.position[17]);
    angles.push_back(msg.position[21]);
    angles.push_back(msg.position[20]);
    angles.push_back(msg.position[22]);
    angles.push_back(msg.position[23]);

    tf::StampedTransform fk_transform;
    KDL::Frame wrist_frame;
    listener.waitForTransform("/torso_lift_link", "/r_wrist_roll_link", 
                              ros::Time(0), ros::Duration(10));
    listener.lookupTransform("/torso_lift_link", "/r_wrist_roll_link", 
                             ros::Time(0), fk_transform);
    tf::transformTFToKDL(fk_transform, wrist_frame);

    bool fastik_success = run_ikfast_test(wrist_frame, msg.position[17]);
    std::vector<double> bs_angles(7,0);
    bs_angles[2] = msg.position[17];
    bool kdl_success = run_kdl_test(wrist_frame, bs_angles);

    ROS_INFO("%d: kdl %d fast_ik %d", counter, kdl_success, fastik_success);
    counter++;
    if (counter == 1000){
        ROS_INFO("kdl: %d, fastik %d", kdl_c, ikfast_c);
        ROS_INFO("kdl: %lu, fastik %lu", kdl_time, ikfast_time);
        ROS_INFO("kdl fk: %lu, fastik fk %lu", kdl_fk, ikfast_fk);
        exit(0);
    }
}
int main(int argc, char** argv){
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle nh;
    Tester tester;
    ros::Subscriber sub = nh.subscribe("/joint_states", 1
                                       , &Tester::run_ik, &tester);
    sleep(1);
    ros::spin();
    return 0;
}
