package frc.robot.subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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

public class Hopper extends SubsystemBase implements Loggable {
    //private static boolean pulse = false;
    //private Motor hopperMotorExtension;
    private Motor hopperMotorBeltdrive;
    //private Motor hopperMotorDish;
    public static int beltdrivespeed = 10; //placeholder
    //private static int dishspeed = 10; //placeholder

    public Hopper(){
        /*hopperMotorExtension = Motor.fromSparkMax(
            WiringConstants.HopperMotors.hopperMotorExtension,
            false,
            (SparkMax sparkmotor) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1.0);
                config.encoder.velocityConversionFactor(1.0);
                config.smartCurrentLimit(40);
                config.idleMode(IdleMode.kCoast);
                sparkmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) -> {sim.withHardstops(0, 20);},
            0,
            FeedbackController.fromPID(0, 0, 0, (PIDController pid) -> {
                pid.setTolerance(0.5);
            }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Position);
            hopperMotorExtension.getSysIDCommands("hopper extension neo", 0, 0, 0);*/

            
            hopperMotorBeltdrive = Motor.fromSparkMax(
            WiringConstants.HopperMotors.hopperMotorExtension,
            false,
            (SparkMax sparkmotor) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1.0);
                config.encoder.velocityConversionFactor(1.0);
                config.smartCurrentLimit(40);
                config.idleMode(IdleMode.kCoast);
                sparkmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromPID(0, 0, 0, (PIDController pid) -> {
                pid.setTolerance(0.5);
            }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Velocity);
            hopperMotorBeltdrive.getSysIDCommands("hopper belt drive neo", 0, 0, 0);

              /*  hopperMotorDish = Motor.fromSparkMax(
            WiringConstants.HopperMotors.hopperMotorExtension,
            false,
            (SparkMax sparkmotor) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1.0);
                config.encoder.velocityConversionFactor(1.0);
                config.smartCurrentLimit(40);
                config.idleMode(IdleMode.kCoast);
                sparkmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromPID(0, 0, 0, (PIDController pid) -> {
                pid.setTolerance(0.5);
            }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Velocity);
            hopperMotorDish.getSysIDCommands("hopper dish neo", 0, 0, 0);*/
        

    }

    /*public Command extendHopper(){
        return Commands.runOnce(() -> {
            hopperMotorExtension.setTarget(20);
        }, this).andThen(Commands.waitUntil(() -> {
            return hopperMotorExtension.atTarget();
        }));
    }

    public Command retractHopper(){
        return Commands.runOnce(() -> {
            hopperMotorExtension.setTarget(0);
        }, this).andThen(Commands.waitUntil(() -> {
            return hopperMotorExtension.atTarget();
        }));
    }*/

    public Command beltDriveShoot(){
        return Commands.runOnce(() -> {
            hopperMotorBeltdrive.setTarget(beltdrivespeed);
        }, this).andThen(() -> {
            Commands.waitUntil(() -> {
                return hopperMotorBeltdrive.atTarget();
            });
        });
    }

    public Command beltDriveStop(){
        return Commands.runOnce(() -> {
            hopperMotorBeltdrive.setTarget(0);
        }, this).andThen(() -> {
            Commands.waitUntil(() -> {
                return hopperMotorBeltdrive.atTarget();
            });
        });
    }

    /*public Command runDish(){
        return Commands.runOnce(() -> {
            
            if(pulse){
                hopperMotorDish.setTarget(dishspeed);
                pulse=false;
            }
            else{
                hopperMotorDish.setTarget(0);
                pulse = true;
            }
            
        }, this).andThen(Commands.waitUntil(() -> {
            Commands.waitSeconds(5).execute();
            return hopperMotorDish.atTarget();
        }));
    }

    public Command stopDish(){
        return Commands.runOnce(() -> {
            hopperMotorDish.setTarget(0);
        }, this).andThen(Commands.waitUntil(() -> {
            return hopperMotorDish.atTarget();
        }));
    }*/



	@Override
	public void log(String path) {
	
        HoundLog.log(path, "hopperMotorBeltDrive", hopperMotorBeltdrive.atTarget());

	}

}