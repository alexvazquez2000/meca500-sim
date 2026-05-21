package com.alex.meca500.kinematics;

/**
 * One row of a Denavit-Hartenberg table (standard/Craig convention).
 * T_i = Rot_z(theta+thetaOffset) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
 *
 * @param a           link length (mm)
 * @param alpha       link twist (radians)
 * @param d           link offset (mm)
 * @param thetaOffset constant added to the joint variable (radians), usually 0
 */
public record DHParameter(double a, double alpha, double d, double thetaOffset) {}
