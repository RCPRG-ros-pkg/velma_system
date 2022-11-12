#define _USE_MATH_DEFINES
#include <math.h>
#include <stdexcept>
#include <iostream>

#include <kdl/frames.hpp>
#include "velma_kinematics/velma_kinematics.hpp"

double wrapAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0*M_PI;
    }

    while (angle < -M_PI) {
        angle += 2.0*M_PI;
    }
    return angle;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// KinematicsSolverLWR4 ///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

bool KinematicsSolverLWR4::Solution::isSimilar(const Solution& other, double angle_dist_max) const {
    return flip_shoulder == other.flip_shoulder && flip_elbow == other.flip_elbow &&
            flip_ee == other.flip_ee &&
            (fabs(elbow_circle_angle-other.elbow_circle_angle) < angle_dist_max ||
                fabs( fabs(elbow_circle_angle-other.elbow_circle_angle) - 2*M_PI ) <
                                                                        angle_dist_max);
}

double KinematicsSolverLWR4::Solution::distanceSqr(const Solution& other) const {
    return KinematicsSolverLWR4::Solution::distanceSqr(q, other.q);
}

double KinematicsSolverLWR4::Solution::distanceSqr(const JntArray& q1, const JntArray& q2) {
    double result = 0.0;
    for (int i = 0 ; i < q1.size(); ++i) {
        result += (q1[i]-q2[i]) * (q1[i]-q2[i]);
    }
    return result;
}

KinematicsSolverLWR4::KinematicsSolverLWR4(double elbow_ang_incr)
    : m_A(0.078)
    , m_B(0.39)
    , m_C(0.2)
    , m_D(0.3105)
    , m_arm_len_a(2*m_C)
    , m_arm_len_b(m_B)
    , m_T_A6_A7( KDL::Rotation(), KDL::Vector(0, 0, m_A) )
    , m_T_A7_A6( m_T_A6_A7.Inverse() )
    , m_pt_shoulder(0, 0, m_D)
    , m_lim_lo({-2.96, -2.09, -2.96, -2.095, -2.96, -2.09, -2.96})
    , m_lim_up({2.96, 2.09, 2.96, 2.095, 2.96, 2.09, 2.96})
    , m_elbow_ang_incr( elbow_ang_incr )
{
    for (double elbow_ang = -M_PI; elbow_ang < M_PI; elbow_ang += m_elbow_ang_incr) {
        m_elbow_angle_samples.push_back(elbow_ang);
    }
}

int KinematicsSolverLWR4::getMaximumSolutionsCount() const {
    return m_elbow_angle_samples.size()*8;
}

bool KinematicsSolverLWR4::_calcQ3(const double& dist, double& out_q3) const {
    const double& a = m_arm_len_a;
    const double& b = m_arm_len_b;
    double acos_val = (a*a + b*b - dist*dist) / (2*a*b);
    if (acos_val >= -1.0 && acos_val <= 1.0) {
        out_q3 = M_PI - acos( acos_val );
        return true;
    }
    //print('ERROR: Could not compute arccos of {} = ({}**2 + {}**2 - {}**2) / (2*{}*{})'.format(acos_val, a, b, dist, a, b))
    return false;
}

double KinematicsSolverLWR4::_heronArea(const double& a, const double& b, const double& c) const {
    double s = (a+b+c)/2.0;
    double val = s*(s-a)*(s-b)*(s-c);
    if (val < 0) {
        throw std::logic_error("Square root of negative number");
    }
    return sqrt(val);
}


bool KinematicsSolverLWR4::_calculateIkPart0(const KDL::Frame& T_A0_A7d, double& out_height,
                        double& out_dist_a, KDL::Frame& out_shoulder_frame, double& out_q3,
                        KDL::Vector& out_dir_vec) const {
    KDL::Frame T_A0_A6d = T_A0_A7d * m_T_A7_A6;

    out_dir_vec = T_A0_A6d.p - m_pt_shoulder;
    double dist = out_dir_vec.Norm();
    if (_calcQ3(dist, out_q3)) {
        double area = _heronArea(dist, m_arm_len_a, m_arm_len_b);
        out_height = area / (0.5*dist);
        out_dist_a = sqrt( m_arm_len_a*m_arm_len_a - out_height*out_height );

        // The elbow circle angle is calculated wrt the axis of the first joint of the arm.
        // This is the parameter for IK that deals with redundancy problem.

        const KDL::Vector& shoulder_vec = m_pt_shoulder;
        KDL::Vector nz = out_dir_vec;
        KDL::Vector nx = shoulder_vec;
        KDL::Vector ny = nz*nx;
        nx = ny*nz;
        nx.Normalize();
        ny.Normalize();
        nz.Normalize();
        out_shoulder_frame = KDL::Frame( KDL::Rotation(nx, ny, nz), m_pt_shoulder );
        return true;
    }
    // else
    // No IK solution is possible
    return false;
}

bool KinematicsSolverLWR4::_calculateIkPart1(const double& elbow_circle_angle, bool flip_shoulder,
            bool flip_elbow, double height, double dist_a, const KDL::Frame& shoulder_frame,
            double tmp_q3, const KDL::Vector& dir_vec, double& out_q0, double& out_q1,
            double& out_q2, double& out_q3) const {

    double q3 = tmp_q3;

    KDL::Frame shoulder_frame_rot = shoulder_frame * KDL::Frame( KDL::Rotation::RotZ(elbow_circle_angle), KDL::Vector() );
    KDL::Vector elbow_pt = shoulder_frame_rot * KDL::Vector(height, 0, dist_a);
    KDL::Vector elbow_vec = elbow_pt - m_pt_shoulder;
    double q0 = atan2(-elbow_pt.y(), -elbow_pt.x());

    double q1 = -atan2( sqrt(elbow_vec.x()*elbow_vec.x() + elbow_vec.y()*elbow_vec.y()), elbow_vec.z() );

    // Calculate q2:
    // Project dir_vec to frame of right_arm_2_link, and compute atan2(-dir_vec.y(), -dir_vec.x())
    const KDL::Vector& shoulder_vec = m_pt_shoulder;

    KDL::Vector nx = shoulder_vec;
    KDL::Vector nz = elbow_vec;
    KDL::Vector ny = nz*nx;
    nx = ny*nz;
    nx.Normalize();
    ny.Normalize();
    nz.Normalize();
    KDL::Frame T_A0_A2( KDL::Rotation(nx, ny, nz), m_pt_shoulder );
    const KDL::Vector& dir_vec_A0 = dir_vec;
    KDL::Vector dir_vec_A2 = T_A0_A2.Inverse().M * dir_vec_A0;
    double q2 = atan2(-dir_vec_A2.y(), -dir_vec_A2.x());

    // There are two alternative angles for q3
    if (flip_elbow) {
        q0 = M_PI + q0;
        q1 = -q1;
        q3 = -q3;
    }

    if (flip_shoulder) {
        q0 = M_PI + q0;
        q1 = -q1;
        q2 = M_PI + q2;
    }

    q0 = wrapAngle(q0);
    q1 = wrapAngle(q1);
    q2 = wrapAngle(q2);
    q3 = wrapAngle(q3);
    if (q0 < m_lim_lo[0] || q0 > m_lim_up[0]
            || q1 < m_lim_lo[1] || q1 > m_lim_up[1]
            || q2 < m_lim_lo[2] || q2 > m_lim_up[2]
            || q3 < m_lim_lo[3] || q3 > m_lim_up[3]) {

        return false;
    }
    // else
    out_q0 = q0;
    out_q1 = q1;
    out_q2 = q2;
    out_q3 = q3;
    return true;
}

bool KinematicsSolverLWR4::_calculateIkPart2(const KDL::Frame& T_A0_A7d, const double& q0,
        const double& q1, const double& q2, const double& q3, bool flip_ee,
        double& out_q4, double& out_q5, double& out_q6) const {

    const double s1 = sin(q0);
    const double c1 = cos(q0);
    const double s2 = sin(q1);
    const double c2 = cos(q1);
    const double s3 = sin(q2);
    const double c3 = cos(q2);
    const double s4 = sin(q3);
    const double c4 = cos(q3);
    const double r11 = T_A0_A7d.M.UnitX().x();
    const double r21 = T_A0_A7d.M.UnitX().y();
    const double r31 = T_A0_A7d.M.UnitX().z();
    const double r12 = T_A0_A7d.M.UnitY().x();
    const double r22 = T_A0_A7d.M.UnitY().y();
    const double r32 = T_A0_A7d.M.UnitY().z();
    const double r13 = T_A0_A7d.M.UnitZ().x();
    const double r23 = T_A0_A7d.M.UnitZ().y();
    const double r33 = T_A0_A7d.M.UnitZ().z();
    const double px = T_A0_A7d.p.x();
    const double py = T_A0_A7d.p.y();
    const double pz = T_A0_A7d.p.z();

    //T_5_1*Td
    //m00 = r11*((-s1*s3 + c1*c2*c3)*c4 + s2*s4*c1) + r21*((s1*c2*c3 + s3*c1)*c4 + s1*s2*s4) + r31*(-s2*c3*c4 + s4*c2)
    //m01 = r12*((-s1*s3 + c1*c2*c3)*c4 + s2*s4*c1) + r22*((s1*c2*c3 + s3*c1)*c4 + s1*s2*s4) + r32*(-s2*c3*c4 + s4*c2)
    const double m02 = r13*((-s1*s3 + c1*c2*c3)*c4 + s2*s4*c1) + r23*((s1*c2*c3 + s3*c1)*c4 + s1*s2*s4) + r33*(-s2*c3*c4 + s4*c2);
    //m03 = px*((-s1*s3 + c1*c2*c3)*c4 + s2*s4*c1) + py*((s1*c2*c3 + s3*c1)*c4 + s1*s2*s4) + pz*(-s2*c3*c4 + s4*c2) + (-0.3105*c2 - 0.2)*s4 + 0.3105*s2*c3*c4 - 0.2*s4
    //m10 = r11*(-s1*c3 - s3*c1*c2) + r21*(-s1*s3*c2 + c1*c3) + r31*s2*s3
    //m11 = r12*(-s1*c3 - s3*c1*c2) + r22*(-s1*s3*c2 + c1*c3) + r32*s2*s3
    const double m12 = r13*(-s1*c3 - s3*c1*c2) + r23*(-s1*s3*c2 + c1*c3) + r33*s2*s3;
    //m13 = px*(-s1*c3 - s3*c1*c2) + py*(-s1*s3*c2 + c1*c3) + pz*s2*s3 - 0.3105*s2*s3
    const double m20 = r11*(-(-s1*s3 + c1*c2*c3)*s4 + s2*c1*c4) + r21*(-(s1*c2*c3 + s3*c1)*s4 + s1*s2*c4) + r31*(s2*s4*c3 + c2*c4);
    const double m21 = r12*(-(-s1*s3 + c1*c2*c3)*s4 + s2*c1*c4) + r22*(-(s1*c2*c3 + s3*c1)*s4 + s1*s2*c4) + r32*(s2*s4*c3 + c2*c4);
    const double m22 = r13*(-(-s1*s3 + c1*c2*c3)*s4 + s2*c1*c4) + r23*(-(s1*c2*c3 + s3*c1)*s4 + s1*s2*c4) + r33*(s2*s4*c3 + c2*c4);
    //m23 = px*(-(-s1*s3 + c1*c2*c3)*s4 + s2*c1*c4) + py*(-(s1*c2*c3 + s3*c1)*s4 + s1*s2*c4) + pz*(s2*s4*c3 + c2*c4) + (-0.3105*c2 - 0.2)*c4 - 0.3105*s2*s4*c3 - 0.2*c4

    //T_5_8
    //m[0,0] = -s5*s7 + c5*c6*c7
    //m[0,1] = -s5*c7 - s7*c5*c6
    //m[0,2] = s6*c5
    //m[0,3] = 0.078*s6*c5
    //m[1,0] = s5*c6*c7 + s7*c5
    //m[1,1] = -s5*s7*c6 + c5*c7
    //m[1,2] = s5*s6
    //m[1,3] = 0.078*s5*s6
    //m[2,0] = -s6*c7
    //m[2,1] = s6*s7
    //m[2,2] = c6
    //m[2,3] = 0.078*c6 + 0.39

    //tg7 = s7/c7 = -m[2,1]/m[2,0]
    double q6 = atan2(m21, -m20);

    //tg5 = s5/c5 = m[1,2]/m[0,2]
    double q4 = atan2(m12, m02);

    //tg6 = s6/c6 = m[2,1]/(s7*m[2,2])
    double q5 = atan2(m21, sin(q6)*m22);
    if (m21 < 0) {
        // This is tricky
        q5 = q5 + M_PI;
    }

    if (flip_ee) {
        q4 = q4 + M_PI;
        q5 = -q5;
        q6 = q6 + M_PI;
    }

    q4 = wrapAngle(q4);
    q5 = wrapAngle(q5);
    q6 = wrapAngle(q6);

    if (q4 < m_lim_lo[4] || q4 > m_lim_up[4]
            || q5 < m_lim_lo[5] || q5 > m_lim_up[5]
            || q6 < m_lim_lo[6] || q6 > m_lim_up[6]) {

        return false;
    }
    // else
    out_q4 = q4;
    out_q5 = q5;
    out_q6 = q6;
    return true;
}

bool KinematicsSolverLWR4::calculateIk(const KDL::Frame& T_A0_A7d, const double& elbow_circle_angle,
                                                        int flip_case, Solution& out_sol) const {
    if (flip_case < 0 || flip_case >= 8) {
        throw std::invalid_argument("Wrong value of flip_case, should be between 0 and 7");
    }
    const bool cases[8][3] = {
            {false, false, false},
            {false, false, true},
            {false, true, false},
            {false, true, true},
            {true, false, false},
            {true, false, true},
            {true, true, false},
            {true, true, true}};

    return calculateIk(T_A0_A7d, elbow_circle_angle,
        cases[flip_case][0], cases[flip_case][1], cases[flip_case][2], out_sol);
}

bool KinematicsSolverLWR4::calculateIk(const KDL::Frame& T_A0_A7d, const double& elbow_circle_angle,
        bool flip_shoulder, bool flip_elbow, bool flip_ee, Solution& out_sol) const {

    double height;
    double dist_a;
    KDL::Frame shoulder_frame;
    double tmp_q3;
    KDL::Vector dir_vec;

    if (!_calculateIkPart0(T_A0_A7d, height, dist_a, shoulder_frame, tmp_q3, dir_vec)) {
        //std::cout << "_calculateIkPart0 failed" << std::endl;
        return false;
    }
    JntArray q;
    if (!_calculateIkPart1(elbow_circle_angle, flip_shoulder, flip_elbow, height, dist_a,
                                        shoulder_frame, tmp_q3, dir_vec, q[0], q[1], q[2], q[3])) {
        //std::cout << "_calculateIkPart1 failed" << std::endl;
        return false;
    }
    if (!_calculateIkPart2(T_A0_A7d, q[0], q[1], q[2], q[3], flip_ee, q[4], q[5], q[6])) {
        //std::cout << "_calculateIkPart2 failed" << std::endl;
        return false;
    }
    //std::cout << "calculateIk ok" << std::endl;
    out_sol.q = q;
    out_sol.elbow_circle_angle = elbow_circle_angle;
    out_sol.flip_shoulder = flip_shoulder;
    out_sol.flip_elbow = flip_elbow;
    out_sol.flip_ee = flip_ee;
    return true;
}

bool KinematicsSolverLWR4::calculateIkSet(const KDL::Frame& T_A0_A7d,
                                std::vector<Solution>& solutions, int& solutions_count) const {
    const bool cases[8][3] = {
            {false, false, false},
            {false, false, true},
            {false, true, false},
            {false, true, true},
            {true, false, false},
            {true, false, true},
            {true, true, false},
            {true, true, true}};

    if (solutions.size() == 0) {
        throw std::logic_error(
            "KinematicsSolverLWR4::calculateIkSet: vector of result solutions is not allocated");
    }

    double height;
    double dist_a;
    KDL::Frame shoulder_frame;
    double tmp_q3;
    KDL::Vector dir_vec;

    if (!_calculateIkPart0(T_A0_A7d, height, dist_a, shoulder_frame, tmp_q3, dir_vec)) {
        //std::cout << "_calculateIkPart0 failed" << std::endl;
        return false;
    }

    solutions_count = 0;
    for (int i = 0; i < m_elbow_angle_samples.size(); ++i) {
        const double& elbow_ang = m_elbow_angle_samples[i];
        double out_q0, out_q1, out_q2, out_q3, out_q4, out_q5, out_q6;

        for (int case_id = 0; case_id < 8; ++case_id) {

            const bool flip_shoulder = cases[case_id][0];
            const bool flip_elbow = cases[case_id][1];
            const bool flip_ee = cases[case_id][2];

            // TODO: this can be done better (first calculate for flip_shoulder, flip_elbow
            // and then for flip_ee).
            JntArray q;
            if (!_calculateIkPart1(elbow_ang, flip_shoulder, flip_elbow, height, dist_a,
                                        shoulder_frame, tmp_q3, dir_vec, q[0], q[1], q[2], q[3])) {
                //std::cout << "_calculateIkPart1 failed" << std::endl;
                continue;
            }
            if (!_calculateIkPart2(T_A0_A7d, q[0], q[1], q[2], q[3], flip_ee,
                                                                            q[4], q[5], q[6])) {
                //std::cout << "_calculateIkPart2 failed" << std::endl;
                continue;
            }

            Solution& out_sol = solutions[solutions_count];
            out_sol.q = q;
            out_sol.elbow_circle_angle = elbow_ang;
            out_sol.flip_shoulder = flip_shoulder;
            out_sol.flip_elbow = flip_elbow;
            out_sol.flip_ee = flip_ee;

            ++solutions_count;
            if (solutions_count >= solutions.size()) {
                break;
            }
        }
    }
    return true;
}

void KinematicsSolverLWR4::calculateFk(const double& q0, const double& q1, const double& q2,
                        const double& q3, const double& q4, const double& q5, const double& q6,
                                                                        KDL::Frame& out_T) const {
    const double s1 = sin(q0);
    const double c1 = cos(q0);
    const double s2 = sin(q1);
    const double c2 = cos(q1);
    const double s3 = sin(q2);
    const double c3 = cos(q2);
    const double s4 = sin(q3);
    const double c4 = cos(q3);
    const double s5 = sin(q4);
    const double c5 = cos(q4);
    const double s6 = sin(q5);
    const double c6 = cos(q5);
    const double s7 = sin(q6);
    const double c7 = cos(q6);

    const double& A = m_A;
    const double& B = m_B;
    const double& C = m_C;
    const double& D = m_D;

    const double c4s6 = s6*c4;
    const double c4c5c6 = c4*c5*c6;
    const double c4c5s6 = c4s6*c5;
    const double s4s6 = s4*s6;
    const double s5c6s7 = s5*s7*c6;
    const double c4s5s7 = s5*s7*c4;
    const double s4c6 = s4*c6;
    const double s5c6c7 = s5*c6*c7;
    const double c4s5c7 = s5*c4*c7;
    const double s4c5c6 = s4*c5*c6;
    const double s4s5c7 = s4*s5*c7;
    const double c5s7 = s7*c5;
    const double c3s5s6 = s5*s6*c3;
    const double s3s5s6 = s3*s5*s6;
    const double s2c3 = s2*c3;
    const double c5c7 = c5*c7;
    const double c2s3 = s3*c2;
    const double c4c6 = c4*c6;
    const double s4c5s6 = s4s6*c5;
    const double e1 = (s4s6 + c4c5c6)*c7;
    const double e2 = (s4s6 + c4c5c6)*s7;
    const double e3 = (s4c5c6 - c4s6)*s7;
    const double e4 = (s4c5c6 - c4s6)*c7;
    const double e6 = (-A*s4c6 - B*s4 + A*c4c5s6);
    const double e8 = (A*s4c5s6 + A*c4c6 + B*c4 + C);
    const double e9 = e6*c2*c3 + e8*s2 + C*s2 - A*s3s5s6*c2;
    const double m00 = -((e1 - c4s5s7)*s3 + (s5c6c7 + c5s7)*c3)*s1 + ((e1 - c4s5s7)*c2*c3 + (e4 - s4*s5*s7)*s2 - (s5c6c7 + c5s7)*c2s3)*c1;
    const double m01 = -((-e2 - c4s5c7)*s3 + (-s5c6s7 + c5c7)*c3)*s1 + ((-e2 - c4s5c7)*c2*c3 + (-e3 - s4s5c7)*s2 - (-s5c6s7 + c5c7)*c2s3)*c1;
    const double m02 = -((-s4c6 + c4c5s6)*s3 + c3s5s6)*s1 + ((-s4c6 + c4c5s6)*c2*c3 + (s4c5s6 + c4c6)*s2 - s3s5s6*c2)*c1;
    const double m03 = -(e6*s3 + A*c3s5s6)*s1 + e9*c1;
    const double m10 = ((e1 - c4s5s7)*s3 + (s5c6c7 + c5s7)*c3)*c1 + ((e1 - c4s5s7)*c2*c3 + (e4 - s4*s5*s7)*s2 - (s5c6c7 + c5s7)*c2s3)*s1;
    const double m11 = ((-e2 - c4s5c7)*s3 + (-s5c6s7 + c5c7)*c3)*c1 + ((-e2 - c4s5c7)*c2*c3 + (-e3 - s4s5c7)*s2 - (-s5c6s7 + c5c7)*c2s3)*s1;
    const double m12 = ((-s4c6 + c4c5s6)*s3 + c3s5s6)*c1 + ((-s4c6 + c4c5s6)*c2*c3 + (s4c5s6 + c4c6)*s2 - s3s5s6*c2)*s1;
    const double m13 = (e6*s3 + A*c3s5s6)*c1 + e9*s1;
    const double m20 = -(e1 - c4s5s7)*s2c3 + (e4 - s4*s5*s7)*c2 + (s5c6c7 + c5s7)*s2*s3;
    const double m21 = -(-e2 - c4s5c7)*s2c3 + (-e3 - s4s5c7)*c2 + (-s5c6s7 + c5c7)*s2*s3;
    const double m22 = -(-s4c6 + c4c5s6)*s2c3 + (s4c5s6 + c4c6)*c2 + s2*s3s5s6;
    const double m23 = -e6*s2c3 + e8*c2 + A*s2*s3s5s6 + C*c2 + D;

    KDL::Vector nx(m00, m10, m20);
    KDL::Vector ny(m01, m11, m21);
    KDL::Vector nz(m02, m12, m22);
    out_T = KDL::Frame( KDL::Rotation(nx, ny, nz), KDL::Vector(m03, m13, m23) );
}

double KinematicsSolverLWR4::calculateJntDist(const KinematicsSolverLWR4::JntArray& a,
                                                    const KinematicsSolverLWR4::JntArray& b) {
    double sq_dist = 0.0;
    for (int i = 0; i < a.size(); ++i) {
        sq_dist += (a[i]-b[i]) * (a[i]-b[i]);
    }
    return sqrt(sq_dist);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// KinematicsSolverVelma //////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

KinematicsSolverVelma::KinematicsSolverVelma(const std::string& urdf_string, double elbow_ang_incr)
: m_T_Er_Gr(KDL::Rotation::RPY(0, M_PI/2, 0), KDL::Vector(0.235, 0, -0.078))
, m_T_El_Gl(KDL::Rotation::RPY(0, -M_PI/2, 0), KDL::Vector(-0.235, 0, -0.078))
, m_T_Er_Pr(KDL::Rotation::RPY(0, M_PI/2, 0), KDL::Vector(0.115, 0, -0.078))
, m_T_El_Pl(KDL::Rotation::RPY(0, -M_PI/2, 0), KDL::Vector(-0.115, 0, -0.078))
, m_T_Gr_Er(m_T_Er_Gr.Inverse())
, m_T_Gl_El(m_T_El_Gl.Inverse())
, m_T_Pr_Er(m_T_Er_Pr.Inverse())
, m_T_Pl_El(m_T_El_Pl.Inverse())
, m_ik_solver_lwr(elbow_ang_incr)
{
    if (!kdl_parser::treeFromString(urdf_string, m_tree)){
        std::cout << "KinematicsSolverVelma: Failed to construct kdl tree" << std::endl;
        throw std::runtime_error("KinematicsSolverVelma: Failed to construct kdl tree");
    }
    m_tree.getChain("torso_base", "calib_left_arm_base_link", m_chain_left);
    m_tree.getChain("torso_base", "calib_right_arm_base_link", m_chain_right);
    m_tree.getChain("torso_base", "torso_link0", m_chain_torso);

    m_tree.getChain("torso_base", "right_arm_4_link", m_chain_elbow_right);
    m_tree.getChain("torso_base", "left_arm_4_link", m_chain_elbow_left);


    m_pfk_left.reset( new KDL::ChainFkSolverPos_recursive(m_chain_left) );
    m_pfk_right.reset( new KDL::ChainFkSolverPos_recursive(m_chain_right) );

    m_pfk_elbow_left.reset( new KDL::ChainFkSolverPos_recursive(m_chain_elbow_left) );
    m_pfk_elbow_right.reset( new KDL::ChainFkSolverPos_recursive(m_chain_elbow_right) );

    m_pfk_torso.reset( new KDL::ChainFkSolverPos_recursive(m_chain_torso) );
}

KinematicsSolverVelmaPtr KinematicsSolverVelma::fromROSParam(ros::NodeHandle& nh,
                                                                        double elbow_ang_incr) {
    std::string robot_description_str;
    std::string robot_description_semantic_str;
    nh.getParam("/robot_description", robot_description_str);

    return KinematicsSolverVelmaPtr(
                                new KinematicsSolverVelma(robot_description_str, elbow_ang_incr));
}

int KinematicsSolverVelma::getMaximumSolutionsCount() const {
    return m_ik_solver_lwr.getMaximumSolutionsCount();
}

KDL::Frame KinematicsSolverVelma::getTorsoFk(double torso_angle) const {
    KDL::Frame p_out;
    KDL::JntArray q(1);
    q(0) = torso_angle;
    m_pfk_torso->JntToCart(q, p_out);
    return p_out;
}

KDL::Frame KinematicsSolverVelma::getArmElbowFk(Side side, double torso_angle, double q0,
                                                        double q1, double q2, double q3) const {
    KDL::Frame p_out;
    KDL::JntArray q(5);
    q(0) = torso_angle;
    q(1) = q0;
    q(2) = q1;
    q(3) = q2;
    q(4) = q3;
    if (side == LEFT) {
        m_pfk_elbow_left->JntToCart(q, p_out);
    }
    else if (side == RIGHT) {
        m_pfk_elbow_right->JntToCart(q, p_out);
    }
    else {
        throw std::invalid_argument("unknown side");
    }

    return p_out;
}

KDL::Frame KinematicsSolverVelma::getArmBaseFk(Side side, double torso_angle) const {
    KDL::Frame p_out;
    KDL::JntArray q(1);
    q(0) = torso_angle;
    if (side == LEFT) {
        m_pfk_left->JntToCart(q, p_out);
    }
    else if (side == RIGHT) {
        m_pfk_right->JntToCart(q, p_out);
    }
    else {
        throw std::invalid_argument("unknown side");
    }

    return p_out;

    // Hard-coded kinematics:
    // const double s0 = sin(torso_angle);
    // const double c0 = cos(torso_angle);
    // const double m21 = 0.0;
    // const double m22 = 0.5;
    // const double m23 = 1.20335;
    // if (side == LEFT) {
    //     const double m00 = -0.5*s0;
    //     const double m01 = -c0;
    //     const double m02 = -0.86602540378614*s0;
    //     const double m03 = -0.000188676*s0;
    //     const double m10 = 0.5*c0;
    //     const double m11 = -s0;
    //     const double m12 = 0.86602540378614*c0;
    //     const double m13 = 0.000188676*c0;
    //     const double m20 = -0.866025403786140;
    //     KDL::Vector nx(m00, m10, m20);
    //     KDL::Vector ny(m01, m11, m21);
    //     KDL::Vector nz(m02, m12, m22);
    //     return KDL::Frame( KDL::Rotation(nx, ny, nz), KDL::Vector(m03, m13, m23) );
    // }
    // else if (side == RIGHT) {
    //     const double m00 = -0.5*s0;
    //     const double m01 = -c0;
    //     const double m02 = 0.86602540378614*s0;
    //     const double m03 = 0.000188676*s0;
    //     const double m10 = 0.5*c0;
    //     const double m11 = -s0;
    //     const double m12 = -0.86602540378614*c0;
    //     const double m13 = -0.000188676*c0;
    //     const double m20 = 0.866025403786140;
    //     KDL::Vector nx(m00, m10, m20);
    //     KDL::Vector ny(m01, m11, m21);
    //     KDL::Vector nz(m02, m12, m22);
    //     return KDL::Frame( KDL::Rotation(nx, ny, nz), KDL::Vector(m03, m13, m23) );
    // }
    // else {
    //     throw std::invalid_argument("unknown side");
    // }
}

KDL::Frame KinematicsSolverVelma::getArmBaseFkInv(Side side, double torso_angle) const {
    return getArmBaseFk(side, torso_angle).Inverse();
}

void KinematicsSolverVelma::getArmFk(Side side, double torso_angle, double q0, double q1,
            double q2, double q3, double q4, double q5, double q6, KDL::Frame& out_T_B_E) const {

    KDL::Frame T_B_AB = getArmBaseFk(side, torso_angle);
    KDL::Frame T_AB_E;
    m_ik_solver_lwr.calculateFk(q0, q1, q2, q3, q4, q5, q6, T_AB_E);
    out_T_B_E = T_B_AB * T_AB_E;
}

void KinematicsSolverVelma::getArmFk(Side side, double torso_angle, const ArmJntArray& q,
                                                                    KDL::Frame& out_T_B_E) const {
    getArmFk(side, torso_angle, q[0], q[1], q[2], q[3], q[4], q[5], q[6], out_T_B_E);
}

const KDL::Frame& KinematicsSolverVelma::getT_E_G(Side side) const {
    if (side == LEFT) {
        return m_T_El_Gl;
    }
    else if (side == RIGHT) {
        return m_T_Er_Gr;
    }
    else {
        throw std::invalid_argument("unknown side");
    }
}

const KDL::Frame& KinematicsSolverVelma::getT_E_P(Side side) const {
    if (side == LEFT) {
        return m_T_El_Pl;
    }
    else if (side == RIGHT) {
        return m_T_Er_Pr;
    }
    else {
        throw std::invalid_argument("unknown side");
    }
}

const KDL::Frame& KinematicsSolverVelma::getT_G_E(Side side) const {
    if (side == LEFT) {
        return m_T_Gl_El;
    }
    else if (side == RIGHT) {
        return m_T_Gr_Er;
    }
    else {
        throw std::invalid_argument("unknown side");
    }
}

const KDL::Frame& KinematicsSolverVelma::getT_P_E(Side side) const {
    if (side == LEFT) {
        return m_T_Pl_El;
    }
    else if (side == RIGHT) {
        return m_T_Pr_Er;
    }
    else {
        throw std::invalid_argument("unknown side");
    }
}

bool KinematicsSolverVelma::calculateIkSetArm(Side side, double torso_angle,
                            const KDL::Frame& T_B_E, KinematicsSolverLWR4::Solutions& solutions,
                            int& solutions_count) const {
    KDL::Frame T_A0_A7d = getArmBaseFkInv(side, torso_angle) * T_B_E;
    return m_ik_solver_lwr.calculateIkSet(T_A0_A7d, solutions, solutions_count);
}

bool KinematicsSolverVelma::calculateIkArm(Side side, double torso_angle, const KDL::Frame& T_B_E,
                double elbow_circle_angle, bool flip_shoulder, bool flip_elbow, bool flip_ee,
                KinematicsSolverLWR4::Solution& out_sol) const {
    KDL::Frame T_A0_A7d = getArmBaseFkInv(side, torso_angle) * T_B_E;
    return m_ik_solver_lwr.calculateIk(T_A0_A7d, elbow_circle_angle, flip_shoulder, flip_elbow,
                                                                                flip_ee, out_sol);
}

bool KinematicsSolverVelma::calculateIkArm(Side side, double torso_angle, const KDL::Frame& T_B_E,
                                                double elbow_circle_angle, int flip_case,
                                                KinematicsSolverLWR4::Solution& out_sol) const {
    KDL::Frame T_A0_A7d = getArmBaseFkInv(side, torso_angle) * T_B_E;
    return m_ik_solver_lwr.calculateIk(T_A0_A7d, elbow_circle_angle, flip_case, out_sol);
}
