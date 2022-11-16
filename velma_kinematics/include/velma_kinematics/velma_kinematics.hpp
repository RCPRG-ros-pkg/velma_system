#ifndef __VELMA_KINEMATICS__H__
#define __VELMA_KINEMATICS__H__

#include <array>
#include <vector>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <ros/node_handle.h>

class KinematicsSolverLWR4 {
public:
    typedef std::array<double, 7> JntArray;

    class Solution {
    public:
        JntArray q;
        double elbow_circle_angle;
        bool flip_shoulder;
        bool flip_elbow;
        bool flip_ee;
        bool isSimilar(const Solution& other, double angle_dist_max) const;
        double distanceSqr(const Solution& other) const;
        static double distanceSqr(const JntArray& q1, const JntArray& q2);
    };

    typedef std::vector<Solution > Solutions;

protected:
    const double m_A;
    const double m_B;
    const double m_C;
    const double m_D;
    const double m_arm_len_a;
    const double m_arm_len_b;
    const KDL::Frame m_T_A6_A7;
    const KDL::Frame m_T_A7_A6;
    const KDL::Vector m_pt_shoulder;
    const JntArray m_lim_lo;
    const JntArray m_lim_up;

    // Parameter
    const double m_elbow_ang_incr;

    std::vector<double > m_elbow_angle_samples;

    bool _calcQ3(const double& dist, double& out_q3) const;

    double _heronArea(const double& a, const double& b, const double& c) const;

    bool _calculateIkPart0(const KDL::Frame& T_A0_A7d, double& out_height,
                        double& out_dist_a, KDL::Frame& out_shoulder_frame, double& out_q3,
                        KDL::Vector& out_dir_vec) const;

    bool _calculateIkPart1(const double& elbow_circle_angle, bool flip_shoulder,
            bool flip_elbow, double height, double dist_a, const KDL::Frame& shoulder_frame,
            double tmp_q3, const KDL::Vector& dir_vec, double& out_q0, double& out_q1,
            double& out_q2, double& out_q3) const;

    bool _calculateIkPart2(const KDL::Frame& T_A0_A7d, const double& q0,
        const double& q1, const double& q2, const double& q3, bool flip_ee,
        double& out_q4, double& out_q5, double& out_q6) const;

public:
    KinematicsSolverLWR4(double elbow_ang_incr=0.261791667);

    int getMaximumSolutionsCount() const;

    bool calculateIk(const KDL::Frame& T_A0_A7d, const double& elbow_circle_angle,
                                                        int flip_case, Solution& out_sol) const;

    bool calculateIk(const KDL::Frame& T_A0_A7d, const double& elbow_circle_angle,
        bool flip_shoulder, bool flip_elbow, bool flip_ee, Solution& out_sol) const;

    bool calculateIkSet(const KDL::Frame& T_A0_A7d,
                                std::vector<Solution>& solutions, int& solutions_count) const;

    void calculateFk(const double& q0, const double& q1, const double& q2,
                        const double& q3, const double& q4, const double& q5, const double& q6,
                                                                        KDL::Frame& out_T) const;

    static double calculateJntDist(const JntArray& a, const JntArray& b);
};

class KinematicsSolverVelma;
typedef std::shared_ptr<KinematicsSolverVelma > KinematicsSolverVelmaPtr;

class KinematicsSolverVelmaLoader {
protected:
    KDL::Tree m_tree;

public:
    KinematicsSolverVelmaLoader(ros::NodeHandle& nh);

    KinematicsSolverVelmaPtr create(double elbow_ang_incr=0.26179166);
};

class KinematicsSolverVelma {
public:
    friend class KinematicsSolverVelmaLoader;
    typedef KinematicsSolverLWR4::JntArray ArmJntArray;
    enum Side {LEFT, RIGHT};

protected:

    KinematicsSolverLWR4 m_ik_solver_lwr;

    const KDL::Frame m_T_Er_Gr;
    const KDL::Frame m_T_El_Gl;
    const KDL::Frame m_T_Er_Pr;
    const KDL::Frame m_T_El_Pl;

    const KDL::Frame m_T_Gr_Er;
    const KDL::Frame m_T_Gl_El;
    const KDL::Frame m_T_Pr_Er;
    const KDL::Frame m_T_Pl_El;

    KDL::Tree m_tree;
    KDL::Chain m_chain_left;
    KDL::Chain m_chain_right;
    KDL::Chain m_chain_torso;
    KDL::Chain m_chain_elbow_right;
    KDL::Chain m_chain_elbow_left;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive > m_pfk_left;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive > m_pfk_right;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive > m_pfk_torso;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive > m_pfk_elbow_left;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive > m_pfk_elbow_right;


    KinematicsSolverVelma(KDL::Tree& tree, double elbow_ang_incr=0.26179166);

public:

    int getMaximumSolutionsCount() const;

    KDL::Frame getTorsoFk(double torso_angle) const;

    KDL::Frame getArmElbowFk(Side side, double torso_angle, double q0,
                                                        double q1, double q2, double q3) const;

    KDL::Frame getArmBaseFk(Side side, double torso_angle) const;
    KDL::Frame getArmBaseFkInv(Side side, double torso_angle) const;

    void getArmFk(Side side, double torso_angle, double q0, double q1, double q2, double q3,
                                    double q4, double q5, double q6, KDL::Frame& out_T_B_E) const;

    void getArmFk(Side side, double torso_angle, const ArmJntArray& q, KDL::Frame& out_T_B_E) const;

    const KDL::Frame& getT_E_G(Side side) const;
    const KDL::Frame& getT_E_P(Side side) const;

    const KDL::Frame& getT_G_E(Side side) const;
    const KDL::Frame& getT_P_E(Side side) const;

    bool calculateIkSetArm(Side side, double torso_angle,
                            const KDL::Frame& T_B_E, KinematicsSolverLWR4::Solutions& solutions,
                            int& solutions_count) const;

    bool calculateIkArm(Side side, double torso_angle, const KDL::Frame& T_B_E,
                double elbow_circle_angle, bool flip_shoulder, bool flip_elbow, bool flip_ee,
                KinematicsSolverLWR4::Solution& out_sol) const;

    bool calculateIkArm(Side side, double torso_angle, const KDL::Frame& T_B_E,
                double elbow_circle_angle, int flip_case,
                KinematicsSolverLWR4::Solution& out_sol) const;
};

#endif  // __VELMA_KINEMATICS__H__
