package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.orchestra.Orc;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
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
  private Swerve swerve;

  public Superstructure(Supplier<Pose2d> robotPose, Swerve swerve) {
    shooter = new Shooter();
    hopper = new Hopper();
    intake = new Intake();
    this.robotPose = robotPose;
    this.swerve = swerve;
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
    return shooter.idle().andThen(hopper.beltDriveStop());
  }

  // ------------------SHOOTER+HOPPER-------------------------------------------
  public Command StartShooter() {
    // return shooterFly.speedup();
    return shooter.readyShoot(
        this.robotPose,
        () -> {
          if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            return new Translation2d(4.625594, 4.034536); // red hub
          } else {
            return new Translation2d(4.625594, 4.034536); // blue hub
          }
        },
        this.swerve);
  }

  public Command StartShooterTest() {
    return shooter.test(
        this.swerve,
        () -> {
          if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
            return new Translation2d(4.625594, 4.034536); // red hub
          } else {
            return new Translation2d(4.625594, 4.034536); // blue hub
          }
        });
  }

  public Command StopShooter() {
    return shooter.idle().andThen(hopper.beltDriveStop());
  }

  public Command shoot() {
    return hopper.beltDriveShoot();
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
    return Commands.none();
  }

  // ---------------------------------------TagAlign-----------------------------------------

  public Command AlignLeft() {
    return Commands.none();
  }

  public Command AlignCenter() {
    return Commands.none();
  }

  public Command AlignClimb() {
    return Commands.none();
  }

  public Command AlignRight() {
    return Commands.none();
  }
}
