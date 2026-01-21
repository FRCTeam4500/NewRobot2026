package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.orchestra.Orc;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.StopTilting;
import frc.robot.utilities.logging.Loggable;

/**
 * A class that holds together the top half of our robot. Basically everything except the
 * drivetrain. It exposes command factories which combine the various subsystems
 */
public class Superstructure implements Loggable {
  // Create objects for all non-drivebase subsystems

  
  private Shooter shooter;
  private Supplier<Pose2d> robotPose;
  private Swerve swerve;

  public Superstructure(Supplier<Pose2d> robotPose, Swerve swerve) {
    shooter = new Shooter();
    this.robotPose = robotPose;
    this.swerve =swerve;
    StopTilting.setupSuperstructure(new Transform3d[] {}, new double[] {});
  }

  public void log(String path) {
    // Call log() methods for contained subsystems
    StopTilting.updateCenterOfMass(new Transform3d[] {});
  }

  public Command sing() {
    return Orc.startSinging();
  }

  public Command stopSinging() {
    return Orc.stopSinging();
  }

  public Command stow() {
    return Commands.none();
  }

  public Command StartShooter(){
    // return shooterFly.speedup();
    return shooter.readyShoot(this.robotPose, () -> {
      if(DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
        return new Translation2d(4.625594,4.034536); // red hub
      } else {
        return new Translation2d(4.625594,4.034536); // blue hub
      }
    },this.swerve);
  }

  public Command StopShooter(){
    return shooter.idle();
  }
}
