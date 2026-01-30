package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.logging.Loggable;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.WiringConstants;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;




public class Intake extends SubsystemBase implements Loggable{
    
    private Motor intakeMotorDrive;
    private Motor intakeMotorExtension;
    private final int intakeSpeed = 100;


    public Intake(){
        intakeMotorDrive = Motor.fromSparkMax(
            WiringConstants.IntakeMotors.IntakeMotor,
            false,
            (SparkMax sparkmotor) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1.0);
                config.encoder.velocityConversionFactor(1.0);
                config.smartCurrentLimit(60);
                config.idleMode(IdleMode.kCoast);
                sparkmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) ->{},
            0,
            FeedbackController.fromPID(0, 0, 0, (PIDController pid) -> {
                pid.setTolerance(0.5);
            }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Velocity);
            intakeMotorDrive.getSysIDCommands("intake drive neo", 0, 0, 0);
            
             
        intakeMotorExtension = Motor.fromSparkMax(
            WiringConstants.IntakeMotors.IntakeMotorExtension,
        false,
        (SparkMax sparkmotor) -> {
            SparkMaxConfig config = new SparkMaxConfig();
            config.encoder.positionConversionFactor(1.0);
            config.encoder.velocityConversionFactor(1.0);
            config.smartCurrentLimit(60);
            config.idleMode(IdleMode.kCoast);
            sparkmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        },
        (FeedforwardSim sim) -> {sim.withHardstops(0, 20);},
        0,
        FeedbackController.fromPID(0, 0, 0, (PIDController pid) -> {
            pid.setTolerance(0.5);
        }),
        FeedforwardController.forConstantGravity(0, 0, 0, 0),
        TargetType.Velocity);
        intakeMotorExtension.getSysIDCommands("intake extend neo", 0, 0, 0);
        
    }

    public Command startIntake() {
        return Commands.runOnce(() -> {
            intakeMotorDrive.setTarget(intakeSpeed);
        }, this).andThen(Commands.waitUntil(()-> {
            return intakeMotorDrive.atTarget();
        }));
            
    }

    public Command stopIntake() {
        return Commands.runOnce(() -> {
            intakeMotorDrive.setTarget(0);
        }, this).andThen(() -> {
            Commands.waitUntil(()->{
                return intakeMotorDrive.atTarget();
            });
        });
    }


    public Command extendIntake(){
        return Commands.runOnce(() -> {
            intakeMotorExtension.setTarget(20);
        }, this). andThen(Commands.waitUntil(() -> {
            return intakeMotorExtension.atTarget();
        }));
    }

    public Command retractIntake(){
        return Commands.runOnce(() -> {
            intakeMotorExtension.setTarget(0);
        }, this).andThen(Commands.waitUntil(() -> {
            return intakeMotorExtension.atTarget();
        }));
    }

    @Override
    public void log(String path) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }


}



