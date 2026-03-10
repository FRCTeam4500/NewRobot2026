package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleSubscriber;
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
import java.util.function.Supplier;

public class Hopper extends SubsystemBase implements Loggable {

  private Motor hopperMotorBeltdrive;
  private Motor hopperMotorBeltdrive2;
  SysIDCommands angleSysId;
  private DoubleSubscriber hopperSpeed;
  private InterpolatingDoubleTreeMap flywheelSpeed = new InterpolatingDoubleTreeMap();

  public static int beltdrivespeed = 50;

  public Hopper() {
    hopperSpeed = HoundLog.tunable("Hopper Speed", 50.0);

    flywheelSpeed.put(1.583, 45.0);
    flywheelSpeed.put(2.12, 48.0); // meters , motor speed units
    flywheelSpeed.put(2.329, 50.0);
    flywheelSpeed.put(2.996, 50.0);
    flywheelSpeed.put(3.334, 52.0);
    flywheelSpeed.put(3.565, 52.0);
    flywheelSpeed.put(3.630, 55.0);
    flywheelSpeed.put(4.7, 75.0);

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
    angleSysId =
        hopperMotorBeltdrive.getSysIDCommands(
            "hopper belt drive neo", 1, 10, 10, hopperMotorBeltdrive2);
  }

  public Command beltDriveShoot(Supplier<Pose2d> robotPose, Supplier<Translation2d> target) {
    return Commands.runOnce(
            () -> {
              double distance = robotPose.get().getTranslation().getDistance(target.get());
              hopperMotorBeltdrive.setTarget(
                  flywheelSpeed.get(distance)); // flywheelSpeed.get(distance)
              hopperMotorBeltdrive2.setTarget(flywheelSpeed.get(distance));
            },
            this)
        .andThen(Commands.idle());
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
    HoundLog.log(path, "HopperMotor1", hopperMotorBeltdrive);
    HoundLog.log(path, "HopperMotor2", hopperMotorBeltdrive2);
    HoundLog.log(path, "BeltDriveSpeed", hopperMotorBeltdrive.getVelocity());
    HoundLog.log(path, "hopperMotorBeltDrive", hopperMotorBeltdrive.atTarget());
    SmartDashboard.putData("Angle Dynamic Forward", angleSysId.dynamicForward());
    SmartDashboard.putData("Angle Dynamic Reverse", angleSysId.dynamicReverse());
    SmartDashboard.putData("Angle Quasistatic Forward", angleSysId.quasistaticForward());
    SmartDashboard.putData("Angle Quasistatic Reverse", angleSysId.quasistaticReverse());
  }
}
