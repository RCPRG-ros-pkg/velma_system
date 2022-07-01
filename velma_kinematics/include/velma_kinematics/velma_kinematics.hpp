#ifndef __VELMA_KINEMATICS__H__
#define __VELMA_KINEMATICS__H__

#include <kdl/frames.hpp>
#include <array>
#include <vector>

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

    KDL::Frame m_T_A0_A6d;
    KDL::Frame m_T_A0_A7d;
    KDL::Vector m_dir_vec;
    double m_dist_a;
    KDL::Frame m_shoulder_frame;
    double m_height;
    double m_q3;
    bool m_ik_success;
    bool m_T_A0_A7d_dirty;

    std::vector<Solution > m_solutions;
    std::vector<double > m_elbow_angle_samples;
    int m_solutions_count;

    bool _calcQ3(const double& dist, double& out_q3) const;

    double _heronArea(const double& a, const double& b, const double& c) const;

    bool _calculateIkPart0();
    bool _calculateIkPart1(const double& elbow_circle_angle, bool flip_shoulder, bool flip_elbow,
                                double& out_q0, double& out_q1, double& out_q2, double& out_q3);

    bool _calculateIkPart2(const KDL::Frame& T_A0_A7d, const double& q0,
        const double& q1, const double& q2, const double& q3, bool flip_ee,
        double& out_q4, double& out_q5, double& out_q6);

public:
    KinematicsSolverLWR4(double elbow_ang_incr=0.261791667);

    bool calculateIk(const KDL::Frame& T_A0_A7d, const double& elbow_circle_angle,
        bool flip_shoulder, bool flip_elbow, bool flip_ee, Solution& out_sol);

    bool calculateIk(const double& elbow_circle_angle,
        bool flip_shoulder, bool flip_elbow, bool flip_ee, Solution& out_sol);

    void setEePose(const KDL::Frame& T_A0_A7d);

    bool calculateIkSet();

    const Solutions& getSolutions() const;
    int getSolutionsCount() const;

    void calculateFk(const double& q0, const double& q1, const double& q2,
                        const double& q3, const double& q4, const double& q5, const double& q6,
                                                                        KDL::Frame& out_T) const;

    static double calculateJntDist(const JntArray& a, const JntArray& b);
};

class KinematicsSolverVelma {
public:
    typedef KinematicsSolverLWR4::JntArray ArmJntArray;
    enum Side {LEFT, RIGHT};

protected:

    double m_torso_angle;

    KDL::Frame m_left_arm_base_fk;
    KDL::Frame m_left_arm_base_fk_inv;
    bool m_left_arm_base_fk_dirty;

    KDL::Frame m_right_arm_base_fk;
    KDL::Frame m_right_arm_base_fk_inv;
    bool m_right_arm_base_fk_dirty;

    KinematicsSolverLWR4 m_ik_solver_lwr;

    const KDL::Frame m_T_Er_Gr;
    const KDL::Frame m_T_El_Gl;
    const KDL::Frame m_T_Er_Pr;
    const KDL::Frame m_T_El_Pl;

    const KDL::Frame m_T_Gr_Er;
    const KDL::Frame m_T_Gl_El;
    const KDL::Frame m_T_Pr_Er;
    const KDL::Frame m_T_Pl_El;

public:
    KinematicsSolverVelma(double elbow_ang_incr=0.261791667);

    const KDL::Frame& getArmBaseFk(Side side);
    const KDL::Frame& getArmBaseFkInv(Side side);

    void getArmFk(Side side, double q0, double q1, double q2, double q3, double q4,
                                                double q5, double q6, KDL::Frame& out_T_B_E);

    void getArmFk(Side side, const ArmJntArray& q, KDL::Frame& out_T_B_E);

    const KDL::Frame& getT_E_G(Side side) const;
    const KDL::Frame& getT_E_P(Side side) const;

    const KDL::Frame& getT_G_E(Side side) const;
    const KDL::Frame& getT_P_E(Side side) const;

    void setTorsoAngle(double torso_angle);

    bool calculateIkSetArm(Side side, const KDL::Frame& T_B_Wl);

    bool calculateIkArm(Side side, const KDL::Frame& T_B_Wl, double elbow_circle_angle,
        bool flip_shoulder, bool flip_elbow, bool flip_ee, KinematicsSolverLWR4::Solution& out_sol);

    const KinematicsSolverLWR4::Solutions& getSolutions() const;
    int getSolutionsCount() const;

protected:
};

#endif  // __VELMA_KINEMATICS__H__
