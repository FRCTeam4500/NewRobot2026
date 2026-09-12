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
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.function.DoubleSupplier;
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
  private DoubleSupplier PIDHood;
  private SysIDCommands angleSysId;
  private DoubleSubscriber hoodGravityFeedforward;
  private double distanceOffset;

  public Shooter() {

    flywheelSubscriber = HoundLog.tunable("Flywheel Speed", 0.0);
    turetSubscriber = HoundLog.tunable("TuretHood", 0.0);
    andgleSuscriber = HoundLog.tunable("turetangle", 0.0);
    PIDP = HoundLog.tunable("PID P value", 0.3);
    hoodGravityFeedforward = HoundLog.tunable("Hood Gravity Feedforward", 0.0);

    // find flywheel speed
    flywheelSpeed.put(1.73166, 47.0);
    flywheelSpeed.put(2.059, 48.0);
    flywheelSpeed.put(2.375, 48.0);
    flywheelSpeed.put(2.686, 49.0);
    flywheelSpeed.put(2.917, 50.0);
    flywheelSpeed.put(3.174, 52.0);
    flywheelSpeed.put(3.338, 53.0);
    flywheelSpeed.put(3.622, 54.0);
    flywheelSpeed.put(3.967, 55.0);
    flywheelSpeed.put(4.144, 55.0);
    flywheelSpeed.put(4.367, 58.0);
    flywheelSpeed.put(5.075, 62.0);
    flywheelSpeed.put(5.19, 62.0);
    flywheelSpeed.put(5.2, 50.0);
    flywheelSpeed.put(5.3604, 50.0);
    flywheelSpeed.put(6.855, 50.0);
    flywheelSpeed.put(8.314, 58.0);
    flywheelSpeed.put(9.516, 58.0);
    flywheelSpeed.put(10.9, 80.0);

    // find hood angle
    hoodAngle.put(1.73166, 0.0);
    hoodAngle.put(2.059, 2.0);
    hoodAngle.put(2.375, 4.0);
    hoodAngle.put(2.686, 6.0);
    hoodAngle.put(2.917, 6.5);
    hoodAngle.put(3.174, 6.5);
    hoodAngle.put(3.338, 6.5);
    hoodAngle.put(3.622, 8.0);
    hoodAngle.put(3.967, 10.0);
    hoodAngle.put(4.144, 10.0);
    hoodAngle.put(4.367, 10.0);
    hoodAngle.put(5.075, 12.0);
    hoodAngle.put(5.19, 12.0);
    hoodAngle.put(5.2, 20.0);
    hoodAngle.put(5.3604, 20.0);
    hoodAngle.put(6.855, 24.0);
    hoodAngle.put(8.314, 24.0);
    hoodAngle.put(9.516, 24.0);
    hoodAngle.put(10.9, 24.0);

    // find flywheel speed -NEW
    // flywheelSpeed.put(1.789, 47.0);
    // flywheelSpeed.put(1.997, 49.0);
    // flywheelSpeed.put(2.378, 50.0);
    // flywheelSpeed.put(2.771, 55.0);
    // flywheelSpeed.put(3.324, 57.5);
    // flywheelSpeed.put(3.613, 61.0);
    // flywheelSpeed.put(4.096, 61.0);
    // flywheelSpeed.put(4.412, 62.0);
    // flywheelSpeed.put(4.752, 64.0);
    // flywheelSpeed.put(5.1, 64.0);
    // flywheelSpeed.put(5.2, 50.0);
    // flywheelSpeed.put(5.3604, 50.0);
    // flywheelSpeed.put(6.855, 50.0);
    // flywheelSpeed.put(8.314, 58.0);
    // flywheelSpeed.put(9.516, 58.0);
    // flywheelSpeed.put(10.9, 80.0);

    // find hood angle -NEW
    // hoodAngle.put(1.789, 0.0);
    // hoodAngle.put(1.997, 2.0);
    // hoodAngle.put(2.378, 6.0);
    // hoodAngle.put(2.771, 6.0);
    // hoodAngle.put(3.324, 9.0);
    // hoodAngle.put(3.613, 10.0);
    // hoodAngle.put(4.096, 12.0);
    // hoodAngle.put(4.412, 16.0);
    // hoodAngle.put(4.752, 17.0);
    // hoodAngle.put(5.1, 17.0);
    // hoodAngle.put(5.2, 20.0);
    // hoodAngle.put(5.3604, 20.0);
    // hoodAngle.put(6.855, 24.0);
    // hoodAngle.put(8.314, 24.0);
    // hoodAngle.put(9.516, 24.0);
    // hoodAngle.put(10.9, 24.0);

    // for testing
    PIDController FlywheelPID = new PIDController(0, 0, 0);
    FlywheelPID.setTolerance(1);

    PIDHood = () -> 0.5;
    PIDController ExtenionPID = new PIDController(0, 0, 0);
    ExtenionPID.setTolerance(.05);

    flywheel1 =
        Motor.fromTalonFX(
            WiringConstants.ShooterMotors.flywheelMotor1,
            (TalonFX MotorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 50;
              config.CurrentLimits.StatorCurrentLimit = 60;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              MotorFx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromTunablePID(FlywheelPID, PIDP),
            FeedforwardController.forConstantGravity(0, 0.098553, 0.11532, 0.040236),
            TargetType.Velocity);
    flywheel2 =
        Motor.fromTalonFX(
            WiringConstants.ShooterMotors.flywheelMotor2,
            (TalonFX MotorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 50;
              config.CurrentLimits.StatorCurrentLimit = 60;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              MotorFx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromTunablePID(FlywheelPID, PIDP),
            FeedforwardController.forConstantGravity(0, 0.10196, 0.11401, 0.044742),
            TargetType.Velocity);
    angleSysId = flywheel1.getSysIDCommands("flywheelMotorkraken", 1, 10, 10, flywheel2);

    double hoodGearReduction = 53;
    hood =
        Motor.fromSparkMax(
            WiringConstants.ShooterMotors.turretheadMotor,
            false,
            (SparkMax sparkMotor) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.encoder.positionConversionFactor(1.0 / hoodGearReduction * 360);
              config.encoder.velocityConversionFactor(1.0 / hoodGearReduction * 360);
              config.smartCurrentLimit(30);
              config.idleMode(IdleMode.kBrake);
              config.inverted(false);
              sparkMotor.configure(
                  config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) -> {
              sim.withHardstops(0, 4);
            },
            0,
            // FeedbackController.fromTunablePID(ExtenionPID, PIDHood),
            FeedbackController.fromPID(.1, 0, 0, pid -> pid.setTolerance(0.05)),
            FeedforwardController.forConstantGravity(.45, 0, 0, 0),
            TargetType.Position);

    flywheel1.setVoltage(0);
    flywheel2.setVoltage(0);
  }

  public Command readyShoot(Supplier<Pose2d> robotPose, Supplier<Translation2d> target) {
    // spin up the wheels
    // make the hood at the right angle
    // Rotation2d turretAngle = targetAngle.plus(robotPose.get().getRotation()); // might be minus
    // for turret
    return Commands.run(
        () -> {
          // a lot of math
          double distance =
              robotPose.get().getTranslation().getDistance(target.get()) + distanceOffset;
          flywheelSpeedlog = flywheelSpeed.get(distance);

          hoodAngleLog = hoodAngle.get(distance);
          flywheel1.setTarget(flywheelSpeed.get(distance));
          flywheel2.setTarget(flywheelSpeed.get(distance));
          PIDHood = () -> 2;
          hood.setTarget(hoodAngle.get(distance));

          this.distance = distance;
        },
        this);
  }

  public Command adjustDistance(double change) {
    return Commands.runOnce(() -> distanceOffset += change);
  }

  public Command test(Supplier<Pose2d> robotPose, Supplier<Translation2d> target) {
    return Commands.run(
        () -> {
          flywheel1.setTarget(flywheelSubscriber.get()); // flywheelSubscriber.get()
          flywheel2.setTarget(flywheelSubscriber.get());

          hood.setTarget(turetSubscriber.get()); // turetSubscriber.get()
          // hood.setVoltage(hoodGravityFeedforward.get()
          // *Math.cos(Units.degreesToRadians(hood.getPosition())));

          double distance = robotPose.get().getTranslation().getDistance(target.get());
          this.distance = distance;
        },
        this);
  }

  public Command idle() {
    return Commands.runOnce(
            () -> {
              // for testing
              flywheel1.setVoltage(0);
              flywheel2.setVoltage(0);
              // for matches
              // flywheel1.setTarget(47);
              // flywheel2.setTarget(47);
              PIDHood = () -> 0.5;
              // hood.setTarget(0.5);
              hood.setVoltage(0);
              // hood.setTarget(5);
            },
            this)
        .andThen(Commands.idle());
  }

  public Command RestetHood() {
    return Commands.runOnce(() -> hood.resetPosition(0), this);
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "ShooterFlywhel1", flywheel1);
    HoundLog.log(path, "ShooterFlywhel2", flywheel2);
    HoundLog.log(path, "hoodMoter", hood);
    HoundLog.log(path, "flywheelSpeed", flywheel1.getVelocity());
    HoundLog.log(path, "hoodAnlge", hood.getPosition());
    HoundLog.log(path, "expectedFlywheelSpeed", flywheelSpeedlog);
    HoundLog.log(path, "expectedHoodAngle", hoodAngleLog);
    HoundLog.log(path, "flywheelAtTrarget", flywheel1.atTarget());
    HoundLog.log(path, "HoodAtTarget", hood.atTarget());
    HoundLog.log(path, "robotDistance", this.distance);
    HoundLog.log(path, "hoodPValue", PIDHood.getAsDouble());
    HoundLog.log(path, "distance offset", distanceOffset);
  }
}
