package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.WiringConstants;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.logging.Loggable;

public class Flywheel extends SubsystemBase implements Loggable{

    private Motor flywheelMotor1;
    private Motor flywheelMotor2;
    private Motor turretheadMotor;
    private Motor turretturnMotor;
    private static int flywheelSpeed = 5600; //RPM

    public Flywheel () {

        flywheelMotor1 = Motor.fromSparkMax(                             
            WiringConstants.ShooterMotors.FlywheelMotor, 
            false, 
            (SparkMax sparkmotor) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1.0);
                config.encoder.velocityConversionFactor(1.0);
                config.smartCurrentLimit(60);
                config.idleMode(IdleMode.kCoast);
                sparkmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            null, 
            0, 
            FeedbackController.fromPID(0.04, 0, 0, (PIDController pid) ->{
                pid.setTolerance(0.5);
            }), 
            FeedforwardController.forConstantGravity(0, 0, 0, 0), 
            TargetType.Velocity);  
            flywheelMotor1.getSysIDCommands("flywheelneo", 1, 10, 10);



            flywheelMotor2 = Motor.fromTalonFX(
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
                flywheelMotor2.getSysIDCommands("flywheelMotorkraken", 1, 10, 10);



    public TurretHead (){
        turretheadMotor = Motor.fromSparkMax(                             
            WiringConstants.ShooterMotors.turretheadMotor 
            false, 
            (SparkMax sparkmotor) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1.0);
                config.encoder.velocityConversionFactor(1.0);
                config.smartCurrentLimit(60);
                config.idleMode(IdleMode.kCoast);
                sparkmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            null, 
            0, 
            FeedbackController.fromPID(0.04, 0, 0, (PIDController pid) ->{
                pid.setTolerance(0.5);
            }), 
            FeedforwardController.forArmGravity(0, 0, 0, 0), 
            TargetType.Position);  
            turretheadMotor.getSysIDCommands("turrethead neo", 0, 0, 0);
    }

public TurretTurn (){
        turretturnMotor = Motor.fromSparkMax(                             
            WiringConstants.ShooterMotors.turretturnMotor 
            false, 
            (SparkMax sparkmotor) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1.0);
                config.encoder.velocityConversionFactor(1.0);
                config.smartCurrentLimit(60);
                config.idleMode(IdleMode.kCoast);
                sparkmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            null, 
            0, 
            FeedbackController.fromPID(0.04, 0, 0, (PIDController pid) ->{
                pid.setTolerance(0.5);
            }), 
            FeedforwardController.forConstantGravity(0, 0, 0, 0), 
            TargetType.Position);  
            turretheadMotor.getSysIDCommands("turret turn neo", 0, 0, 0);
}
    }

    public Command speedup(){
        return Commands.runOnce(()-> {
            flywheelMotor2.setTarget(flywheelSpeed);
        }, this).andThen(
            Commands.waitUntil(() ->{ return flywheelMotor2.atTarget();}));
    }

    public Command stopShooter(){

        return Commands.runOnce(() -> {
            flywheelMotor2.setTarget(0);
            
        }, this).andThen(
            Commands.waitUntil(() -> {
                return flywheelMotor2.atTarget();
            })
        );        
    }







    @Override
    public void log(String path) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
    
}
