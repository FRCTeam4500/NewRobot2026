/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** This is a simple container for math methods which are useful */
public class ExtendedMath {
  public static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  public static Rotation2d angleBetween(Translation2d a, Translation2d b) {
    return a.getAngle().minus(b.getAngle());
  }

  /**
   * Returns the length of a in the direction of b. Calculated as dot(a, b)/|b|, or 0 if b is 0.
   *
   * @param a the vector to be measured
   * @param b the direction in which to measure it
   * @return
   */
  public static double scalarProjectionOf(Translation2d a, Translation2d b) {
    var norm = b.getNorm();
    if (norm == 0) return 0;
    return dot(a, b) / norm;
  }

  /**
   * divides by the norm
   *
   * @param translation thing to be normalized
   * @return translation with a magnitude of 1
   */
  public static Translation2d normalize(Translation2d translation) {
    return translation.div(translation.getNorm());
  }

  /**
   * applies a "deadzone," area within which input is zeroed
   *
   * @param value, value to be deadzoned
   * @param deadzone, deadzone size
   * @return 0 if |value| is less than deadzone, otherwise value.
   */
  public static double withHardDeadzone(double value, double deadzone) {
    if (Math.abs(value) < deadzone) return 0;
    return value;
  }

  /**
   * Changes the slope outside the deadzone.
   *
   * @param input number to be deadzoned
   * @param slope slope of function outside of zone
   * @param deadzone zone size
   * @return 0 if input is in the deadzone, otherwise slope * sign(input) * (|input|-deadzone)
   */
  public static double withContinuousDeadzone(double input, double slope, double deadzone) {
    if (input <= -deadzone) return (input + deadzone) * slope;
    if (-deadzone < input && input < deadzone) return 0;
    return (input - deadzone) * slope;
  }

  /**
   * piecewise linear function s.t. if input is 0, output is 0, input is 1, output is 1, and it has
   * a deadzone
   *
   * @param input number, call it x
   * @param deadzone deadzone size
   * @return if |x| is less then deadzone, 0, otherwise it's a line from the deadzone boundary
   *     (deadzone,0) through (1,1)
   */
  public static double withContinuousDeadzone(double input, double deadzone) {
    return withContinuousDeadzone(input, (1 / (1 - deadzone)), deadzone);
  }

  /**
   * A custom mod function which returns a remainder with the same sign as the dividend. This is
   * different from using {@code %}, which returns the remainder with the same sign as the divisor.
   *
   * @param a the dividend
   * @param n the divisor
   * @return the remainder with the same sign as {@code a}
   */
  public static double customMod(double a, double n) {
    return a - Math.floor(a / n) * n;
  }

  /**
   * I think this is just for performance, but it's not clear that it's necessary?
   *
   * @return an equivalent rotation to the argument
   */
  public static Rotation2d wrapRotation2d(Rotation2d rotationToWrap) {
    return Rotation2d.fromRadians(MathUtil.angleModulus(rotationToWrap.getRadians()));
  }

  /**
   * wraps a rotation count into a {@link Rotation2d}
   *
   * @param rotations rotation count
   * @return the {@link Rotation2d} representing the appropriate angle.
   */
  public static Rotation2d wrapDouble(double rotations) {
    return Rotation2d.fromRotations(rotations % 1);
  }

  /**
   * are they close to each other?
   *
   * @param threshold acceptable distance
   * @return are they close enough?
   */
  public static boolean within(double a, double b, double threshold) {
    return Math.abs(a - b) < Math.abs(threshold);
  }

  /**
   * are they close to each other?
   *
   * @param threshold acceptable distance
   * @return are they close enough?
   */
  public static boolean within(Rotation2d a, Rotation2d b, Rotation2d threshold) {
    return within(a.getDegrees(), b.getDegrees(), threshold.getDegrees());
  }

  /**
   * are they within a rectangle of each other?
   *
   * @param threshold defines the rectangle sizes (note that the x-value is half the width, and same
   *     for height)
   * @return true if the second is within the rectangle centered on the first
   */
  public static boolean within(Translation2d a, Translation2d b, Translation2d threshold) {
    return within(a.getX(), b.getX(), threshold.getX())
        && within(a.getY(), b.getY(), threshold.getY());
  }

  /**
   * Are the two translations within a radius of each other?
   *
   * @param r the acceptable distance
   * @return true if they're close enough to each other.
   */
  public static boolean within(Translation2d a, Translation2d b, double r) {
    return a.minus(b).getNorm() < r;
  }

  /**
   * Are they close to each other?
   *
   * @param threshold the acceptable distances
   * @return if they are close enough on the X, Y, and rotational axes.
   */
  public static boolean within(Pose2d a, Pose2d b, Pose2d threshold) {
    return within(a.getTranslation(), b.getTranslation(), threshold.getTranslation())
        && within(a.getRotation(), b.getRotation(), threshold.getRotation());
  }

  /**
   * @param threshold the acceptable x, y, and angular speed differences.
   * @return Are they close enough to each other?
   */
  public static boolean within(ChassisSpeeds a, ChassisSpeeds b, ChassisSpeeds threshold) {
    return within(a.vxMetersPerSecond, b.vxMetersPerSecond, threshold.vxMetersPerSecond)
        && within(a.vyMetersPerSecond, b.vyMetersPerSecond, threshold.vyMetersPerSecond)
        && within(
            a.omegaRadiansPerSecond, b.omegaRadiansPerSecond, threshold.omegaRadiansPerSecond);
  }

  /**
     * This method is used to calculate the target you should actually aim at when trying to shoot while moving. 
     * @param target The actual target you want to shoot at (field relative)
     * @param robotPose The robot's current translation (field relative)
     * @param robotSpeed The robot's current speeds (field relative)
     * @param robotAccel The robot's current accelerations (field relative)
     * @param latencySeconds The latency between the the time when the projectile leaves the shooter and the time at which the data was collected to decide the setpoints at that time. 
     * Start at around 0.1 seconds, should't need to tune it super accurately.
     * @param distanceToTime A mapping from distance from target to time in air. This should probably be measured irl
     * @return The target you should actually aim at to make your shots
     * see https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494/10
     * see https://github.com/rr1706/2022-Main/blob/9f72c3ec5e64c61344845051c57035b0f7320a54/src/main/java/frc/robot/commands/TurretedShooter/SmartShooter.java#L103
     */
    public static Translation2d calculateTargetOnMove(Translation2d target, Translation2d robotPose, ChassisSpeeds robotSpeed, Translation2d robotAccel, double latencySeconds, InterpolatingDoubleTreeMap distanceToTime) {
        // calculate air time if we shot at our current position
        double shotTime = distanceToTime.get(target.getDistance(robotPose)); 
        // define a virtual target which is where we actually should shoot
        Translation2d virtualTarget = target; 
        // we need to loop this part to try and drive the time delta between shotTime and newShotTime as low as possible. 
        // to be honest 5 interations might be too low, I'm mostly copying 1706 in 2022: 
        // https://github.com/rr1706/2022-Main/blob/9f72c3ec5e64c61344845051c57035b0f7320a54/src/main/java/frc/robot/commands/TurretedShooter/SmartShooter.java#L103
        for (int i = 0; i < 5; i++) { 
            // calculate where how far off target the projectile will be when it should land in the goal
            // we do this by assuming that there are no forces on the projectile in the air, so its 
            // change in horizontal velocity only depends on how fast the robot was going
            // the robot accel part is to accounnt for the fact that we don't know the robot's current velocity,
            // just its velocity some time ago decided by our latency. So instead of assuming a constant velocity
            // over that time period, we assume a constant acceleration, and use that to calculate our
            // current velocity.
            Translation2d offset = new Translation2d( 
                (robotSpeed.vxMetersPerSecond + robotAccel.getX() * latencySeconds) * shotTime, 
                (robotSpeed.vyMetersPerSecond + robotAccel.getY() * latencySeconds) * shotTime);
            // update our virtual target using the original target translation and the previously calculated offset
            // this is our estimate for where we should actually aim to score in our true target
            virtualTarget = target.minus(offset);
            // lookup our shot time for this new shot
            double newShotTime = distanceToTime.get(virtualTarget.getDistance(robotPose));
            // if the times are very different (0.01 might be too low a number here)
            // we should do another loop, because that means that our offset is using the wrong
            // shotTime in its calculation
            if (Math.abs(newShotTime - shotTime) > 0.01) {
                // if they are quite different, iterate and do another loop
                shotTime = newShotTime;
            } else {
                // if they are very similar, we have a good target, so we can return now
                break;
            }
        }
        // return our calculated virtual target that we should shoot at
        return virtualTarget;
    }
}