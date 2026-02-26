package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.WiringConstants;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Climber extends SubsystemBase implements Loggable {

  private Motor climberMotor1;
  private Motor climberMotor2;

  public static double CLIMBER_TARGET = 30.0; // not final make double

  public Climber() {
    climberMotor1 =
        Motor.fromTalonFX(
            WiringConstants.ClimberMotors.ClimberMotor1,
            (TalonFX motorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 40;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              motorFx.getConfigurator().apply(config);
            },
            (FeedforwardSim balls) -> {},
            0,
            FeedbackController.fromPID(
                1,
                0,
                0,
                (PIDController pid) -> {
                  pid.setTolerance(1);
                }),
            FeedforwardController.forArmGravity(0, 0, 0, 0),
            TargetType.Position);

    climberMotor2 =
        Motor.fromTalonFX(
            WiringConstants.ClimberMotors.ClimberMotor2,
            (TalonFX motorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 40;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              motorFx.getConfigurator().apply(config);
            },
            (FeedforwardSim balls) -> {},
            0,
            FeedbackController.fromPID(
                1,
                0,
                0,
                (PIDController pid) -> {
                  pid.setTolerance(1);
                }),
            FeedforwardController.forArmGravity(0, 0, 0, 0),
            TargetType.Position);
    climberMotor1.getSysIDCommands("climber", 0, 0, 0, climberMotor2); // add motor 2 to end
  }

  public Command runClimber() {
    return Commands.runOnce(
            () -> {
              climberMotor1.setTarget(CLIMBER_TARGET);
              climberMotor2.setTarget(CLIMBER_TARGET);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return (climberMotor1.atTarget());
                }));
  }

  // end of auto climb
  public Command releaseClimber() {
    return Commands.runOnce(
            () -> {
              climberMotor1.setTarget(0);
              climberMotor2.setTarget(0);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return (climberMotor1.atTarget());
                }));
  }

  @Override
  public void log(String path) {

    HoundLog.log(path, "climberMotor1AtTarget", climberMotor1.atTarget());
    HoundLog.log(path, "climberMotor1Position", climberMotor1.getPosition());
    HoundLog.log(path, "climberMotor2AtTarget", climberMotor2.atTarget());
    HoundLog.log(path, "climberMotor2Position", climberMotor2.getPosition());
  }
}
