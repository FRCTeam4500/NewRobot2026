package frc.robot.subsystems.intake;

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
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase implements Loggable {

  private Motor intakeMotorDrive;
  private Motor intakeMotorDrive2;
  private Motor intakeMotorExtension;
  private Motor intakeMotorExtension2;
  private final int intakeSpeed = 80;
  private final int intakeVoltage = 12;
  private final double maxExtention1 = 8.571;
  private final double maxExtention2 = 8.571;

  private final double relativeMaxExtention = 0.8;
  private DoubleSupplier PIDP;

  private int pulse = 1;
  private double PulseWaitTime = 0.5;

  public Intake() {
    PIDP = () -> 0.5;
    PIDController ExtenionPID = new PIDController(0, 0, 0);
    ExtenionPID.setTolerance(1);

    /*
    * intakeMotorDrive =
       Motor.fromSparkMax(
           WiringConstants.IntakeMotors.IntakeMotor,
           false,
           (SparkMax sparkmotor) -> {
             SparkMaxConfig config = new SparkMaxConfig();
             config.encoder.positionConversionFactor(1.0);
             config.encoder.velocityConversionFactor(1.0);
             config.smartCurrentLimit(100);
             config.idleMode(IdleMode.kCoast);
             config.inverted(true);
             sparkmotor.configure(
                 config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
           },
           (FeedforwardSim sim) -> {},
           0,
           FeedbackController.fromPID(
               1,
               0,
               0,
               (PIDController pid) -> {
                 pid.setTolerance(10);
               }),
           FeedforwardController.forConstantGravity(0, 0, 0, 0),
           TargetType.Velocity);
    */

    intakeMotorDrive =
        Motor.fromTalonFX(
            WiringConstants.IntakeMotors.IntakeMotor1,
            (TalonFX MotorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 45;
              config.CurrentLimits.StatorCurrentLimit = 70;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              MotorFx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromPID(
                1,
                0,
                0,
                (PIDController pid) -> {
                  pid.setTolerance(10);
                }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Velocity);

    intakeMotorDrive2 =
        Motor.fromTalonFX(
            WiringConstants.IntakeMotors.IntakeMotor2,
            (TalonFX MotorFx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.SupplyCurrentLimit = 45;
              config.CurrentLimits.StatorCurrentLimit = 70;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.CurrentLimits.SupplyCurrentLimitEnable = true;
              config.Feedback.SensorToMechanismRatio = 1;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              MotorFx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromPID(
                1,
                0,
                0,
                (PIDController pid) -> {
                  pid.setTolerance(10);
                }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Velocity);
    intakeMotorDrive.getSysIDCommands("intake drive kraken", 0, 0, 0, intakeMotorDrive2);

    intakeMotorExtension =
        Motor.fromSparkMax(
            WiringConstants.IntakeMotors.IntakeMotorExtension,
            false,
            (SparkMax sparkmotor) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.encoder.positionConversionFactor(1.0);
              config.encoder.velocityConversionFactor(1.0);
              config.smartCurrentLimit(100);
              config.idleMode(IdleMode.kCoast);
              config.inverted(true);
              sparkmotor.configure(
                  config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) -> {
              sim.withHardstops(0, 6.6);
            },
            0,
            FeedbackController.fromTunablePID(ExtenionPID, PIDP),
            FeedforwardController.forArmGravity(0, 0, 0, 0),
            TargetType.Position);

    intakeMotorExtension2 =
        Motor.fromSparkMax(
            WiringConstants.IntakeMotors.IntakeMotorExtension2,
            false,
            (SparkMax sparkmotor) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.encoder.positionConversionFactor(1.0);
              config.encoder.velocityConversionFactor(1.0);
              config.smartCurrentLimit(100);
              config.idleMode(IdleMode.kCoast);
              config.inverted(false);
              sparkmotor.configure(
                  config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) -> {
              sim.withHardstops(0, 6.6);
            },
            0,
            FeedbackController.fromTunablePID(ExtenionPID, PIDP),
            FeedforwardController.forArmGravity(0, 0, 0, 0),
            TargetType.Position);
    intakeMotorExtension.getSysIDCommands("intake extend neo", 0, 0, 0, intakeMotorExtension2);
  }

  public Command zeroAtFloor() {
    return Commands.runOnce(
        () -> {
          intakeMotorExtension.resetPosition(maxExtention1);
          intakeMotorExtension2.resetPosition(maxExtention2);
        });
  }

  public Command startIntake() {
    return Commands.runOnce(
            () -> {
              intakeMotorDrive.setVoltage(intakeVoltage);
              intakeMotorDrive2.setVoltage(intakeVoltage);
              ;
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return intakeMotorDrive.atTarget();
                }));
  }

  public Command stopIntake() {
    return Commands.runOnce(
            () -> {
              intakeMotorDrive.setVoltage(0);
              intakeMotorDrive2.setVoltage(0);
            },
            this)
        .andThen(
            () -> {
              Commands.waitUntil(
                  () -> {
                    return intakeMotorDrive.atTarget();
                  });
            });
  }

  public Command reverseIntake() {
    return Commands.runOnce(
            () -> {
              intakeMotorDrive.setVoltage(-intakeVoltage);
              intakeMotorDrive2.setVoltage(-intakeVoltage);
            },
            this)
        .andThen(
            () -> {
              Commands.waitUntil(
                  () -> {
                    return intakeMotorDrive.atTarget();
                  });
            });
  }

  public Command extendIntake() {
    return Commands.runOnce(
            () -> {
              PIDP = () -> 0.5;
              intakeMotorExtension.setTarget(maxExtention1);
              intakeMotorExtension2.setTarget(maxExtention2);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return intakeMotorExtension.atTarget();
                }));
  }

  public Command killIntake() {
    return Commands.run(
        () -> {
          PIDP = () -> 0.5;
          intakeMotorExtension.setVoltage(0);
          intakeMotorExtension2.setVoltage(0);
        },
        this);
  }

  public Command flexIntake() {
    // return moveIntake(0.7)
    //     .andThen(Commands.waitSeconds(0.25))
    //     .andThen(moveIntake(0.6))
    //     .andThen(Commands.waitSeconds(0.25))
    //     .andThen(moveIntake(0.5))
    //     .andThen(Commands.waitSeconds(0.25))
    //     .andThen(moveIntake(0.4))
    //     .andThen(Commands.waitSeconds(0.25))
    //     .andThen(moveIntake(0.3))
    //     .andThen(Commands.waitSeconds(0.25))
    //     .andThen(moveIntake(0.2))
    //     .andThen(Commands.waitSeconds(0.25))
    //     .andThen(moveIntake(0.1))
    //     .andThen(Commands.waitSeconds(0.25))
    //     .andThen(moveIntake(0))
    //     .andThen(Commands.waitSeconds(0.25));
    return moveIntake(3)
        .alongWith(
            Commands.runOnce(
                () -> {
                  intakeMotorDrive.setVoltage(intakeVoltage);
                  intakeMotorDrive2.setVoltage(intakeVoltage);
                  ;
                }))
        .withTimeout(1)
        .andThen(
            moveIntake(maxExtention1)
                .alongWith(
                    Commands.runOnce(
                        () -> {
                          intakeMotorDrive.setVoltage(0);
                          intakeMotorDrive2.setVoltage(0);
                          ;
                        }))
                .withTimeout(1))
        .repeatedly();
  }

  private Command moveIntake(double position) {
    return Commands.runOnce(
            () -> {
              if (intakeMotorExtension.getPosition() > position) {
                PIDP = () -> 10;
              } else {
                PIDP = () -> 0.5;
              }
              intakeMotorExtension.setTarget(position);
              intakeMotorExtension2.setTarget(position);
            },
            this)
        .andThen(Commands.waitUntil(intakeMotorExtension::atTarget));
  }

  // return Commands.run(
  //     () -> {
  //       if (pulse == 1) {
  //         retractIntake();
  //         pulse = 0;
  //         Commands.waitSeconds(PulseWaitTime);

  //       } else {
  //         extendIntake();
  //         this.pulse = 1;
  //         Commands.waitSeconds(PulseWaitTime);
  //       }
  //     },
  //     this);

  public Command extendIntakeWithGravity() {
    return Commands.runOnce(
            () -> {
              intakeMotorExtension.setTarget(relativeMaxExtention);
            },
            this)
        .andThen(
            Commands.waitUntil(
                    () -> {
                      return intakeMotorExtension.atTarget();
                    })
                .andThen(
                    Commands.runOnce(
                        () -> {
                          intakeMotorExtension.setVoltage(0);
                        },
                        this)));
  }

  public Command retractIntake() {
    return Commands.runOnce(
            () -> {
              PIDP = () -> 10;
              intakeMotorExtension.setTarget(0);
              intakeMotorExtension2.setTarget(0);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return intakeMotorExtension.atTarget();
                }));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "IntakeDriveMotor", intakeMotorDrive);
    HoundLog.log(path, "IntakeDriveMotor2", intakeMotorDrive2);
    HoundLog.log(path, "intakeExtention1", intakeMotorExtension);
    HoundLog.log(path, "intakeExtention2", intakeMotorExtension2);
    HoundLog.log(path, "intakeMotorDriveAtTarget", intakeMotorDrive.atTarget());
    HoundLog.log(path, "intakeMotorPValue", PIDP.getAsDouble());
    HoundLog.log(path, "intakeDriveSpeed", intakeMotorDrive.getVelocity());
    HoundLog.log(path, "IntakeExtentionAtTarget", intakeMotorExtension.atTarget());
    HoundLog.log(path, "intakeMotorExtension", intakeMotorExtension.getPosition());
    HoundLog.log(path, "intakeMotorExtension2", intakeMotorExtension2.getPosition());
  }
}
