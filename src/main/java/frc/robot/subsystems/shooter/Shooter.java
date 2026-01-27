package frc.robot.subsystems.shooter;



import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Shooter extends SubsystemBase implements Loggable {
    private Motor flywheel;
    private Motor hood;
    private Motor turret;
    private Transform2d shooterTransform = new Transform2d(0.5, 0.0, Rotation2d.kZero);
    private InterpolatingDoubleTreeMap flywheelSpeed = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap hoodAngle = new InterpolatingDoubleTreeMap();
    private DoubleSubscriber flywheelSubscriber;
    private DoubleSubscriber turetSubscriber;
    private DoubleSubscriber andgleSuscriber;

    public Shooter() {

        //find flywheel speed
        flywheelSpeed.put(1.0, 500.0);  // meters , motor speed units
        flywheelSpeed.put(2.0, 1000.0); 
        flywheelSpeed.put(2.0, 1000.0); 
        flywheelSpeed.put(2.0, 1000.0); 
        flywheelSpeed.put(2.0, 1000.0); 

        // find hood angle
        hoodAngle.put(1.0, 60.0);  // meters , angle degrees
        hoodAngle.put(1.0, 60.0);
        hoodAngle.put(1.0, 60.0);
        hoodAngle.put(1.0, 60.0);
        hoodAngle.put(1.0, 60.0);

        // for testing
        flywheelSubscriber = HoundLog.tunable("Flywheel Speed", 0.0);
        turetSubscriber = HoundLog.tunable( "TuretHood", 0.0);
        andgleSuscriber = HoundLog.tunable("turetangle", 0.0);

        flywheel = Motor.fromTalonFX(
                WiringConstants.ShooterMotors.FlywheelMotor2, 
                (TalonFX MotorFx) -> {
                    TalonFXConfiguration config = new TalonFXConfiguration();
                    config.CurrentLimits.SupplyCurrentLimit = 60;
                    config.CurrentLimits.SupplyCurrentLimitEnable = true;
                    config.Feedback.SensorToMechanismRatio = 1;
                    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                    MotorFx.getConfigurator().apply(config); 
                }, 
                null, 
                0, 
                FeedbackController.fromPID(0.1, 0, 0, (PIDController pid) -> { 
                    pid.setTolerance(0.5);
                }), 
                FeedforwardController.forConstantGravity(0, 0, 0, 0), 
                TargetType.Velocity);
                flywheel.getSysIDCommands("flywheelMotorkraken", 1, 10, 10);
        
        hood = Motor.fromSparkMax(
            990, 
            false, 
            (SparkMax sparkMotor) ->{
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1/360);
                config.encoder.velocityConversionFactor(1/360);
                config.smartCurrentLimit(30);
                config.idleMode(IdleMode.kBrake); 
                config.inverted(true);
                sparkMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            (FeedforwardSim sim) ->{
                sim.withHardstops(-90, 90);
            }, 
            0, 
            FeedbackController.fromPID(1, 0, 0, (PIDController pid) ->{
                pid.setTolerance(0.5);
            }), 
            FeedforwardController.forConstantGravity(0, 0, 0, 0), 
            TargetType.Position);
        
        turret = Motor.fromSparkMax(
            90, 
            false, 
            (SparkMax sparkMotor) ->{
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1/360);
                config.encoder.velocityConversionFactor(1/360);
                config.smartCurrentLimit(30);
                config.inverted(false);
                config.idleMode(IdleMode.kBrake);
                sparkMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                
            }, 
            (FeedforwardSim sim) ->{
                sim.withHardstops(0, 90);
            }, 
            0, 
            FeedbackController.fromPID(1, 0, 0, (PIDController pid) ->{
                pid.setTolerance(0.5);
            }), 
            FeedforwardController.forConstantGravity(0, 0, 0, 0), 
            TargetType.Position);



    }

    public Command readyShoot(Supplier<Pose2d> robotPose, Supplier<Translation2d> target, Swerve swerve ) {
        // face the turret at the target
        // spin up the wheels
        // make the hood at the right angle
        return Commands.run(() -> {
            // a lot of math
            Rotation2d targetAngle = robotPose.get().plus(shooterTransform).getTranslation().minus(target.get()).getAngle();
            Rotation2d turretAngle = targetAngle.plus(robotPose.get().getRotation()); // might be minus
            double distance = robotPose.get().getTranslation().getDistance(target.get());
            flywheel.setTarget(flywheelSpeed.get(distance));
            swerve.setTargetHeading(targetAngle);
            hood.setTarget(hoodAngle.get(distance));
            turret.setTarget(turretAngle.getDegrees());

        }, this);
    }
    
    public Command test() {
        return Commands.run(() -> {
            flywheel.setTarget(flywheelSubscriber.get());
            hood.setTarget(turetSubscriber.get());
            turret.setTarget(andgleSuscriber.get());
        },this);
    }

    public Command idle() {
        // flywheel.setTarget(0);
        return Commands.runOnce(() -> {
            flywheel.setVoltage(0);
            turret.setVoltage(0);
            hood.setVoltage(0);  

        }, this).andThen(Commands.idle());
    }


    





    
    @Override
    public void log(String path) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
    
}