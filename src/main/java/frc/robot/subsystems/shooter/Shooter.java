package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleSubscriber;
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
import java.util.function.Supplier;

public class Shooter extends SubsystemBase implements Loggable {
  private Motor flywheel1;
  private Motor flywheel2;
  private Motor hood;
  private double flywheelSpeedlog;
  private double hoodAngleLog;
  private double distance;
  private Transform2d shooterTransform = new Transform2d(0.5, 0.0, Rotation2d.kZero);
  private InterpolatingDoubleTreeMap flywheelSpeed = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap hoodAngle = new InterpolatingDoubleTreeMap();
  private DoubleSubscriber flywheelSubscriber;
  private DoubleSubscriber turetSubscriber;
  private DoubleSubscriber andgleSuscriber;
  private DoubleSubscriber PIDP;

  public Shooter() {

    flywheelSubscriber = HoundLog.tunable("Flywheel Speed", 0.0);
    turetSubscriber = HoundLog.tunable("TuretHood", 0.0);
    andgleSuscriber = HoundLog.tunable("turetangle", 0.0);
    PIDP = HoundLog.tunable("PID P value", 0.85);

    // find flywheel speed
    flywheelSpeed.put(2.12, 48.0); // meters , motor speed units
    flywheelSpeed.put(4.417, 48.1);

    // find hood angle
    hoodAngle.put(2.12, 0.0); // meters , angle degrees
    hoodAngle.put(1.0, 0.0);


    // for testing
    PIDController FlywheelPID = new PIDController(0, 0, 0);
    FlywheelPID.setTolerance(1);

    flywheel1 =
        Motor.fromTalonFX(
            WiringConstants.ShooterMotors.flywheelMotor1,
            (TalonFX MotorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 100;
              config.CurrentLimits.StatorCurrentLimit = 80;
              config.CurrentLimits.StatorCurrentLimitEnable = false;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              MotorFx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromTunablePID(FlywheelPID, PIDP),
            FeedforwardController.forConstantGravity(0, 0.1111, 0.11642, 0.018187),
            TargetType.Velocity);
    flywheel2 =
        Motor.fromTalonFX(
            WiringConstants.ShooterMotors.flywheelMotor2,
            (TalonFX MotorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 100;
              config.CurrentLimits.StatorCurrentLimit = 80;
              config.CurrentLimits.StatorCurrentLimitEnable = false;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              MotorFx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromTunablePID(FlywheelPID, PIDP),
            FeedforwardController.forConstantGravity(0, 0.11108, 0.11641, 0.018121),
            TargetType.Velocity);
    flywheel1
        .getSysIDCommands("flywheelMotorkraken", 1, 10, 10, flywheel2)
        .putOnDashboard("flywheel", this);

    hood =
        Motor.fromSparkMax(
            WiringConstants.ShooterMotors.turretheadMotor,
            false,
            (SparkMax sparkMotor) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.encoder.positionConversionFactor(1.0);
              config.encoder.velocityConversionFactor(1.0);
              config.smartCurrentLimit(40);
              config.idleMode(IdleMode.kBrake);
              config.inverted(false);
              sparkMotor.configure(
                  config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) -> {
              sim.withHardstops(0, 4);
            },
            0,
            FeedbackController.fromPID(
                1.5,
                0,
                0,
                (PIDController pid) -> {
                  pid.setTolerance(0.05);
                }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Position);
  }

  public Command readyShoot(Supplier<Pose2d> robotPose, Supplier<Translation2d> target) {
    // spin up the wheels
    // make the hood at the right angle
    // Rotation2d turretAngle = targetAngle.plus(robotPose.get().getRotation()); // might be minus
    // for turret
    return Commands.run(
        () -> {
          // a lot of math
          double distance = robotPose.get().getTranslation().getDistance(target.get());
          flywheelSpeedlog = flywheelSpeed.get(distance);
          hoodAngleLog = hoodAngle.get(distance);
          flywheel1.setTarget(flywheelSpeed.get(distance));
          flywheel2.setTarget(flywheelSpeed.get(distance));

          hood.setTarget(hoodAngle.get(distance));
        },
        this);
  }

  public Command test(Supplier<Pose2d> robotPose, Supplier<Translation2d> target) {
    return Commands.run(
        () -> {
          flywheel1.setTarget(flywheelSubscriber.get()); // flywheelSubscriber.get()
          flywheel2.setTarget(flywheelSubscriber.get());

          hood.setTarget(turetSubscriber.get()); // turetSubscriber.get()

          double distance = robotPose.get().getTranslation().getDistance(target.get());
          this.distance = distance;
        },
        this);
  }

  public Command idle() {
    // flywheel.setTarget(0);
    return Commands.runOnce(
            () -> {
              flywheel1.setVoltage(0);
              flywheel2.setVoltage(0);

              hood.setTarget(0);
              ;
            },
            this)
        .andThen(Commands.idle());
  }

  @Override
  public void log(String path) {

    HoundLog.log(path, "flywheelSpeed", flywheel1.getVelocity());
    HoundLog.log(path, "hoodAnlge", hood.getPosition());
    HoundLog.log(path, "expectedFlywheelSpeed", flywheelSpeedlog);
    HoundLog.log(path, "expectedHoodAngle", hoodAngleLog);
    HoundLog.log(path, "flywheelAtTrarget", flywheel1.atTarget());
    HoundLog.log(path, "HoodAtTarget", hood.atTarget());
    HoundLog.log(path, "robotDistance", this.distance);
  }
}
