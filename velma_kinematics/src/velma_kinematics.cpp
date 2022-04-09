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

KinematicsSolverLWR4::KinematicsSolverLWR4()
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
    , m_elbow_ang_incr( 0.523583333 ) // about 30 deg.
    , m_solutions_count(0)
{
    for (double elbow_ang = -M_PI; elbow_ang < M_PI; elbow_ang += m_elbow_ang_incr) {
        m_elbow_angle_samples.push_back(elbow_ang);
    }
    m_solutions.resize(m_elbow_angle_samples.size()*8);
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


bool KinematicsSolverLWR4::_calculateIkPart0() {
    if (m_T_A0_A7d_dirty) {
        m_T_A0_A6d = m_T_A0_A7d * m_T_A7_A6;

        m_dir_vec = m_T_A0_A6d.p - m_pt_shoulder;
        double dist = m_dir_vec.Norm();
        if (_calcQ3(dist, m_q3)) {
            m_ik_success = true;
            double area = _heronArea(dist, m_arm_len_a, m_arm_len_b);
            m_height = area / (0.5*dist);
            m_dist_a = sqrt( m_arm_len_a*m_arm_len_a - m_height*m_height );

            // The elbow circle angle is calculated wrt the axis of the first joint of the arm.
            // This is the parameter for IK that deals with redundancy problem.

            const KDL::Vector& shoulder_vec = m_pt_shoulder;
            KDL::Vector nz = m_dir_vec;
            KDL::Vector nx = shoulder_vec;
            KDL::Vector ny = nz*nx;
            nx = ny*nz;
            nx.Normalize();
            ny.Normalize();
            nz.Normalize();
            m_shoulder_frame = KDL::Frame( KDL::Rotation(nx, ny, nz), m_pt_shoulder );
        }
        else {
            m_ik_success = false;
        }
        m_T_A0_A7d_dirty = false;
    }

    return m_ik_success;
}

bool KinematicsSolverLWR4::_calculateIkPart1(const double& elbow_circle_angle, bool flip_shoulder,
                bool flip_elbow, double& out_q0, double& out_q1, double& out_q2, double& out_q3) {
    if (!m_ik_success) {
        return false;
    }

    /*
    KDL::Vector dir_vec = T_A0_A6d.p - m_pt_shoulder;
    double dist = dir_vec.Norm();
    double q3;
    if (!_calcQ3(dist, q3)) {
        return false;
    }

    double area = _heronArea(dist, m_arm_len_a, m_arm_len_b);
    double height = area / (0.5*dist);
    double dist_a = sqrt( m_arm_len_a*m_arm_len_a - height*height );

    // The elbow circle angle is calculated wrt the axis of the first joint of the arm.
    // This is the parameter for IK that deals with redundancy problem.

    KDL::Vector shoulder_vec = m_pt_shoulder;
    KDL::Vector nz = dir_vec;
    KDL::Vector nx = shoulder_vec;
    KDL::Vector ny = nz*nx;
    nx = ny*nz;
    nx.Normalize();
    ny.Normalize();
    nz.Normalize();
    KDL::Frame shoulder_frame( KDL::Rotation(nx, ny, nz), m_pt_shoulder );
    */
    double q3 = m_q3;

    KDL::Frame shoulder_frame_rot = m_shoulder_frame * KDL::Frame( KDL::Rotation::RotZ(elbow_circle_angle), KDL::Vector() );
    KDL::Vector elbow_pt = shoulder_frame_rot * KDL::Vector(m_height, 0, m_dist_a);
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
    const KDL::Vector& dir_vec_A0 = m_dir_vec;
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
        double& out_q4, double& out_q5, double& out_q6) {

    if (!m_ik_success) {
        return false;
    }

    //print T_A0_A7d
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
        bool flip_shoulder, bool flip_elbow, bool flip_ee, Solution& out_sol) {

    setEePose(T_A0_A7d);
    return calculateIk(elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee, out_sol);
}

bool KinematicsSolverLWR4::calculateIk(const double& elbow_circle_angle,
        bool flip_shoulder, bool flip_elbow, bool flip_ee, Solution& out_sol) {
    if (!_calculateIkPart0()) {
        //std::cout << "_calculateIkPart0 failed" << std::endl;
        return false;
    }
    JntArray q;
    if (!_calculateIkPart1(elbow_circle_angle, flip_shoulder, flip_elbow, q[0], q[1], q[2], q[3])) {
        //std::cout << "_calculateIkPart1 failed" << std::endl;
        return false;
    }
    if (!_calculateIkPart2(m_T_A0_A7d, q[0], q[1], q[2], q[3], flip_ee, q[4], q[5], q[6])) {
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

bool KinematicsSolverLWR4::calculateIkSet() {
    const bool cases[8][3] = {
            {false, false, false},
            {false, false, true},
            {false, true, false},
            {false, true, true},
            {true, false, false},
            {true, false, true},
            {true, true, false},
            {true, true, true}};

    m_solutions_count = 0;
    //int solution_idx = 0;
    for (int i = 0; i < m_elbow_angle_samples.size(); ++i) {
        const double& elbow_ang = m_elbow_angle_samples[i];
        double out_q0, out_q1, out_q2, out_q3, out_q4, out_q5, out_q6;

        for (int case_id = 0; case_id < 8; ++case_id) {

            if (calculateIk(elbow_ang, cases[case_id][0], cases[case_id][1], cases[case_id][2],
                                                                m_solutions[m_solutions_count])) {
                ++m_solutions_count;
            }
            if (!m_ik_success) {
                // No solution is possible for sure
                return false;
            }
        }
    }
    return true;
}

const KinematicsSolverLWR4::Solutions& KinematicsSolverLWR4::getSolutions() const {
    return m_solutions;
}

int KinematicsSolverLWR4::getSolutionsCount() const {
    return m_solutions_count;
}

void KinematicsSolverLWR4::setEePose(const KDL::Frame& T_A0_A7d) {
    m_T_A0_A7d_dirty = true;
    m_T_A0_A7d = T_A0_A7d;
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

KinematicsSolverVelma::KinematicsSolverVelma()
: m_left_arm_base_fk_dirty(true)
, m_right_arm_base_fk_dirty(true)
, m_T_Er_Gr(KDL::Rotation::RPY(0, M_PI/2, 0), KDL::Vector(0.235, 0, -0.078))
, m_T_El_Gl(KDL::Rotation::RPY(0, -M_PI/2, 0), KDL::Vector(-0.235, 0, -0.078))
, m_T_Er_Pr(KDL::Rotation::RPY(0, M_PI/2, 0), KDL::Vector(0.115, 0, -0.078))
, m_T_El_Pl(KDL::Rotation::RPY(0, -M_PI/2, 0), KDL::Vector(-0.115, 0, -0.078))
, m_T_Gr_Er(m_T_Er_Gr.Inverse())
, m_T_Gl_El(m_T_El_Gl.Inverse())
, m_T_Pr_Er(m_T_Er_Pr.Inverse())
, m_T_Pl_El(m_T_El_Pl.Inverse())
{}

void KinematicsSolverVelma::setTorsoAngle(const double& torso_angle) {
    m_torso_angle = torso_angle;
    m_left_arm_base_fk_dirty = true;
    m_right_arm_base_fk_dirty = true;
}

const KinematicsSolverLWR4::Solutions& KinematicsSolverVelma::getSolutions() const {
    return m_ik_solver_lwr.getSolutions();
}

int KinematicsSolverVelma::getSolutionsCount() const {
    return m_ik_solver_lwr.getSolutionsCount();
}

const KDL::Frame& KinematicsSolverVelma::getArmBaseFk(Side side) {
    if (side == LEFT) {
        if (m_left_arm_base_fk_dirty) {
            // FK for left ARM base
            const double s0 = sin(m_torso_angle);
            const double c0 = cos(m_torso_angle);
            const double m00 = -0.5*s0;
            const double m01 = -c0;
            const double m02 = -0.86602540378614*s0;
            const double m03 = -0.000188676*s0;
            const double m10 = 0.5*c0;
            const double m11 = -s0;
            const double m12 = 0.86602540378614*c0;
            const double m13 = 0.000188676*c0;
            const double m20 = -0.866025403786140;
            const double m21 = 0.0;
            const double m22 = 0.5;
            const double m23 = 1.20335;
            KDL::Vector nx(m00, m10, m20);
            KDL::Vector ny(m01, m11, m21);
            KDL::Vector nz(m02, m12, m22);
            m_left_arm_base_fk = KDL::Frame( KDL::Rotation(nx, ny, nz), KDL::Vector(m03, m13, m23) );
            m_left_arm_base_fk_inv = m_left_arm_base_fk.Inverse();
            m_left_arm_base_fk_dirty = false;
        }
        return m_left_arm_base_fk;
    }
    else if (side == RIGHT) {
        if (m_right_arm_base_fk_dirty) {
            // FK for right arm base
            const double s0 = sin(m_torso_angle);
            const double c0 = cos(m_torso_angle);
            const double m00 = -0.5*s0;
            const double m01 = -c0;
            const double m02 = 0.86602540378614*s0;
            const double m03 = 0.000188676*s0;
            const double m10 = 0.5*c0;
            const double m11 = -s0;
            const double m12 = -0.86602540378614*c0;
            const double m13 = -0.000188676*c0;
            const double m20 = 0.866025403786140;
            const double m21 = 0.0;
            const double m22 = 0.5;
            const double m23 = 1.20335;
            KDL::Vector nx(m00, m10, m20);
            KDL::Vector ny(m01, m11, m21);
            KDL::Vector nz(m02, m12, m22);
            m_right_arm_base_fk = KDL::Frame( KDL::Rotation(nx, ny, nz), KDL::Vector(m03, m13, m23) );
            m_right_arm_base_fk_inv = m_right_arm_base_fk.Inverse();
            m_right_arm_base_fk_dirty = false;
        }
        return m_right_arm_base_fk;
    }
    else {
        throw std::invalid_argument("unknown side");
    }
}

const KDL::Frame& KinematicsSolverVelma::getArmBaseFkInv(Side side) {
    getArmBaseFk(side);
    if (side == LEFT) {
        // update the fk, if needed
        return m_left_arm_base_fk_inv;
    }
    else if (side == RIGHT) {
        // update the fk, if needed
        return m_right_arm_base_fk_inv;
    }
    else {
        throw std::invalid_argument("unknown side");
    }
}

void KinematicsSolverVelma::getArmFk(Side side, double q0, double q1, double q2, double q3,
                                        double q4, double q5, double q6, KDL::Frame& out_T_B_E) {

    const KDL::Frame& T_B_AB = getArmBaseFk(side);
    KDL::Frame T_AB_E;
    m_ik_solver_lwr.calculateFk(q0, q1, q2, q3, q4, q5, q6, T_AB_E);
    out_T_B_E = T_B_AB * T_AB_E;
}

void KinematicsSolverVelma::getArmFk(Side side, const ArmJntArray& q, KDL::Frame& out_T_B_E) {
    getArmFk(side, q[0], q[1], q[2], q[3], q[4], q[5], q[6], out_T_B_E);
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

bool KinematicsSolverVelma::calculateIkSetArm(Side side, const KDL::Frame& T_B_E) {
    KDL::Frame T_A0_A7d = getArmBaseFkInv(side) * T_B_E;
    m_ik_solver_lwr.setEePose(T_A0_A7d);
    return m_ik_solver_lwr.calculateIkSet();
}

bool KinematicsSolverVelma::calculateIkArm(Side side, const KDL::Frame& T_B_E, double elbow_circle_angle,
        bool flip_shoulder, bool flip_elbow, bool flip_ee, KinematicsSolverLWR4::Solution& out_sol) {
    KDL::Frame T_A0_A7d = getArmBaseFkInv(side) * T_B_E;
    m_ik_solver_lwr.setEePose(T_A0_A7d);
    return m_ik_solver_lwr.calculateIk(elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee,
                                                                                        out_sol);
}
