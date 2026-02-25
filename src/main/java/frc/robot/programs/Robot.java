// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.programs;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.logging.HoundLog;

public class Robot extends LoggedRobot {
  private Swerve swerve;
  private Superstructure structure;
  private CommandXboxController xbox;
  private CommandXboxController xbox2;

  /** make a robot */
  public Robot() {
    swerve = new Swerve();
    structure =
        new Superstructure(
            () -> {
              return swerve.getPose();
            });
    DriverStation.silenceJoystickConnectionWarning(true);
    xbox = new CommandXboxController(2);
    xbox2 = new CommandXboxController(1);
    swerve.setDefaultCommand(swerve.angleCentric(xbox.getHID()));

    setupDriveController();
    setupOperatorController();
    setupAuto();
  }

  private void setupOperatorController() {

    // rev shooter
    Trigger revShooter = xbox2.rightTrigger();
    revShooter.whileTrue(structure.StartShooter());

    // intake
    Trigger ActivateIntake = xbox2.leftTrigger();
    ActivateIntake.whileTrue(structure.StartIntake());
    ActivateIntake.onFalse(structure.StopIntake());

    // extend intake
    Trigger ExtendIntake = xbox2.y();
    ExtendIntake.onTrue(structure.ExtendIntake());

    // retract intake
    Trigger RetractIntake = xbox2.a();
    RetractIntake.onTrue(structure.RetractIntake());

  

    // climb lock
    // prep climb
    // climb
    // declimb
    // intake flexing
    Trigger FlexIntake = xbox.x();
    FlexIntake.whileTrue(structure.PulseIntake());
    FlexIntake.onFalse(structure.ExtendIntake());
    // stow shooter
    Trigger StowShooter = xbox2.b();
    StowShooter.onTrue(structure.stow());
  }

  private void setupDriveController() {
    Trigger onBlue =
        new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
    Trigger onRed = onBlue.negate();

    // face forward
    Trigger faceForwards = new Trigger(() -> xbox.getRightY() < -0.5);
    faceForwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceForwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));

    // face backwards
    Trigger faceBackwards = new Trigger(() -> xbox.getRightY() > 0.5);
    faceBackwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
    faceBackwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));

    // reset angle
    Trigger resetHeading = xbox.a();
    resetHeading.and(onBlue).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(0)));
    resetHeading.and(onRed).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(180)));

    // auto align: dpad
    // to climb
    Trigger AlignClimb = xbox.povDown().debounce(0.2);
    AlignClimb.whileTrue(structure.AlignClimb());
    // center
    Trigger AlignCenter = xbox.povDown().debounce(0.2);
    AlignCenter.whileTrue(structure.AlignCenter());
    // left trench
    Trigger AlignLeftTrench = xbox.povLeft().debounce(0.2);
    AlignLeftTrench.whileTrue(structure.AlignLeft());
    // right trench
    Trigger AlignRightTrench = xbox.povRight().debounce(0.2);
    AlignRightTrench.whileTrue(structure.AlignRight());
    // angle centric lb
    Trigger SetAngleCentric = xbox.leftBumper();
    SetAngleCentric.onTrue(swerve.angleCentric(xbox.getHID()));
    // hub centric rb
    Trigger SetHubCentric = xbox.rightBumper();
    SetHubCentric.onTrue(swerve.hubCentricDrive(xbox.getHID()));
    // slowmode lt
    Trigger SetSlowMode = xbox.leftTrigger();

    // shoot rt
    Trigger shoot = xbox.rightTrigger();
    shoot.onTrue(structure.shoot());
    shoot.onFalse(structure.StopShooter());
  }

  private void setupAuto() {

    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.setDefaultOption("None", Commands.none());
    NamedCommands.registerCommand("StartShooter", structure.StartShooter());
    NamedCommands.registerCommand("Shoot", structure.shoot());
    NamedCommands.registerCommand("StopShooter", structure.StopShooter());

    NamedCommands.registerCommand("StartIntake", structure.StartIntake());
    NamedCommands.registerCommand("StopIntake", structure.StopIntake());
    NamedCommands.registerCommand("ExtendIntake", structure.ExtendIntake());
    SmartDashboard.putData("Auto Chooser", chooser);
    chooser.addOption("center shoot/climb", new PathPlannerAuto("Auto 1"));
    chooser.addOption("left shoot/climb", new PathPlannerAuto("Auto 2a"));
    chooser.addOption("midle set", new PathPlannerAuto("Auto 4a"));
    chooser.addOption("2 midle cycle", new PathPlannerAuto("Auto 5a"));
    chooser.addOption("5 M auto", new PathPlannerAuto("New Auto"));

    RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
  }

  @Override
  public void robotPeriodic() {
    double start = Timer.getFPGATimestamp();
    HoundLog.log("Swerve", swerve);
    HoundLog.log("Superstrucutre", structure);
    double loggingLoop = Timer.getFPGATimestamp() - start;

    start = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    double commandsLoop = Timer.getFPGATimestamp() - start;

    HoundLog.log("DogLog", "Logging Loop Time", loggingLoop * 1000);
    HoundLog.log("DogLog", "Commands Loop Time", commandsLoop * 1000);
    HoundLog.log("DogLog", "Total Loop Time", 1000 * (commandsLoop + loggingLoop));
  }
}
