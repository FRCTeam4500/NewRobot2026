package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.WiringConstants;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Hopper extends SubsystemBase implements Loggable {

  private Motor hopperMotorBeltdrive;
  private Motor hopperMotorBeltdrive2;
  SysIDCommands angleSysId;

  public static int beltdrivespeed = 100;

  public Hopper() {

    hopperMotorBeltdrive =
        Motor.fromTalonFX(
            WiringConstants.HopperMotors.hopperMotorBeltdrive,
            (TalonFX MotorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 100;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              MotorFx.getConfigurator().apply(config);
            },
            null,
            0,
            FeedbackController.fromPID(
                1,
                0,
                0,
                (PIDController pid) -> {
                  pid.setTolerance(1);
                }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Velocity);
    

    hopperMotorBeltdrive2 =
        Motor.fromTalonFX(
            WiringConstants.HopperMotors.hopperMotorBeltdrive2ID,
            (TalonFX MotorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 100;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              MotorFx.getConfigurator().apply(config);
            },
            null,
            0,
            FeedbackController.fromPID(
                1,
                0,
                0,
                (PIDController pid) -> {
                  pid.setTolerance(1);
                }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Velocity);
     angleSysId =hopperMotorBeltdrive.getSysIDCommands("hopper belt drive neo", 1, 10, 10, hopperMotorBeltdrive2);
  }

  public Command beltDriveShoot() {
    return Commands.runOnce(
            () -> {
              hopperMotorBeltdrive.setTarget(beltdrivespeed);
              hopperMotorBeltdrive2.setTarget(beltdrivespeed);
            },
            this)
        .andThen(
            () -> {
              Commands.waitUntil(
                  () -> {
                    return hopperMotorBeltdrive.atTarget();
            
                  });
            });
  }

  public Command beltDriveStop() {
    return Commands.runOnce(
            () -> {
              hopperMotorBeltdrive.setVoltage(0);
              hopperMotorBeltdrive2.setVoltage(0);
            },
            this)
        .andThen(
            () -> {
              Commands.waitUntil(
                  () -> {
                    return hopperMotorBeltdrive.atTarget();
                  });
            });
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "BeltDriveSpeed", hopperMotorBeltdrive.getVelocity());
    HoundLog.log(path, "hopperMotorBeltDrive", hopperMotorBeltdrive.atTarget());
    SmartDashboard.putData("Angle Dynamic Forward", angleSysId.dynamicForward());
    SmartDashboard.putData("Angle Dynamic Reverse", angleSysId.dynamicReverse());
    SmartDashboard.putData("Angle Quasistatic Forward", angleSysId.quasistaticForward());
    SmartDashboard.putData("Angle Quasistatic Reverse", angleSysId.quasistaticReverse());
 
  }
}
