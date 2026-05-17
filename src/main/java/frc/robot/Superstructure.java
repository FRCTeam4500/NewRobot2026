package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.orchestra.Orc;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.ExtendedMath;
import frc.robot.utilities.StopTilting;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * A class that holds together the top half of our robot. Basically everything except the
 * drivetrain. It exposes command factories which combine the various subsystems
 */
public class Superstructure implements Loggable {
  // Create objects for all non-drivebase subsystems

  private Shooter shooter;
  private Hopper hopper;
  private Intake intake;
  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotSpeeds;
  private InterpolatingDoubleTreeMap distanceToTimeMap;

  public Superstructure(Swerve swerve) {
    shooter = new Shooter();
    hopper = new Hopper();
    intake = new Intake();
    this.robotPose = swerve::getPose;
    this.robotSpeeds = swerve::getSpeeds;
    distanceToTimeMap = swerve.distanceToTimeMap;
    StopTilting.setupSuperstructure(new Transform3d[] {}, new double[] {});
  }

  public void log(String path) {
    // Call log() methods for contained subsystems
    StopTilting.updateCenterOfMass(new Transform3d[] {});
    HoundLog.log(path, "shooter", shooter);
    HoundLog.log(path, "Hopper", hopper);
    HoundLog.log(path, "Intake", intake);
  }

  public Command sing() {
    return Orc.startSinging();
  }

  public Command stopSinging() {
    return Orc.stopSinging();
  }

  public Command stow() {
    return shooter.idle().alongWith(hopper.beltDriveStop());
  }

  // ------------------SHOOTER+HOPPER-------------------------------------------
  public Command StartShooter() {

    // return shooter.RestetHood().andThen(Commands.waitSeconds(.5)).andThen(
    return shooter.readyShoot(
        this.robotPose, () -> ExtendedMath.getCurrentTarget(robotPose.get().getTranslation()));
    /*this.robotPose,
    () ->
        ExtendedMath.calculateTargetOnMove(
            ExtendedMath.getCurrentTarget(robotPose.get().getTranslation()),
            robotPose.get().getTranslation(),
            robotSpeeds.get(),
            new Translation2d(),
            0,
            distanceToTimeMap));*/
  }

  public Command ZeroHood() {
    return shooter.RestetHood();
  }

  public Command StartShooterTest() {

    return shooter.test(
        this.robotPose, () -> ExtendedMath.getCurrentTarget(robotPose.get().getTranslation()));
  }

  public Command StopShooterRev() {
    return shooter.idle();
  }

  public Command StopShooter() {
    return hopper.beltDriveStop();
  }

  public Command increaseDistance() {
    return shooter.adjustDistance(0.1).alongWith(hopper.adjustDistance(0.1));
  }

  public Command decreaseDistance() {
    return shooter.adjustDistance(-0.1).alongWith(hopper.adjustDistance(-0.1));
  }

  public Command shoot() {
    return hopper.beltDriveShoot(
        this.robotPose,
        () ->
            ExtendedMath.calculateTargetOnMove(
                ExtendedMath.getCurrentTarget(robotPose.get().getTranslation()),
                robotPose.get().getTranslation(),
                robotSpeeds.get(),
                new Translation2d(),
                0,
                distanceToTimeMap));
  }

  // --------------------------------------intake----------------------------------------------

  public Command ExtendIntake() {
    return intake.extendIntake();
  }

  public Command RetractIntake() {
    return intake.retractIntake();
  }

  public Command StartIntake() {
    return intake.startIntake();
  }

  public Command StopIntake() {
    return intake.stopIntake();
  }

  public Command PulseIntake() {
    return intake.flexIntake();
  }

  public Command SlightRetract() {
    return intake.moveIntake(6);
  }

  // ---------------------------------------TagAlign-----------------------------------------

  public Command AlignLeft() {
    return Commands.deferredProxy(
        () -> {
          if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            return AutoBuilder.pathfindToPose(new Pose2d(), new PathConstraints(5, 7, 3, 4));
          } else {
            return AutoBuilder.pathfindToPose(new Pose2d(), new PathConstraints(5, 7, 3, 4));
          }
        });
  }

  public Command AlignCenter() {
    return Commands.deferredProxy(
        () -> {
          if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            return AutoBuilder.pathfindToPose(new Pose2d(), new PathConstraints(5, 7, 3, 4));
          } else {
            return AutoBuilder.pathfindToPose(new Pose2d(), new PathConstraints(5, 7, 3, 4));
          }
        });
  }

  public Command AlignClimb() {
    return Commands.deferredProxy(
        () -> {
          if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            return AutoBuilder.pathfindToPose(new Pose2d(), new PathConstraints(5, 7, 3, 4));
          } else {
            return AutoBuilder.pathfindToPose(new Pose2d(), new PathConstraints(5, 7, 3, 4));
          }
        });
  }

  public Command AlignRight() {
    return Commands.deferredProxy(
        () -> {
          if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            return AutoBuilder.pathfindToPose(new Pose2d(), new PathConstraints(5, 7, 3, 4));
          } else {
            return AutoBuilder.pathfindToPose(new Pose2d(), new PathConstraints(5, 7, 3, 4));
          }
        });
  }
}
