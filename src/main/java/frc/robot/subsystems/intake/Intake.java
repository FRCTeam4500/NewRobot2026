package frc.robot.subsystems.intake;

public class Intake extends SubsystemBase implements Loggable {
    
    private Motor intakeMotor;


    public Intake(){
        intakeMotor = Motor.fromSparkMax(
            WiringConstants.IntakeMotors.IntakeMotor,
            false,
            (SparkMax sparkmotor) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.encoder.positionConversionFactor(1.0);
                config.encoder.velocityConversionFactor(1.0);
                config.smartCurrentLimit(60);
                config.idleMode(IdleMode.kCoast);
                sparkmotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }
            null,
            0,
            FeedbackController.fromPID(0, 0, 0, (PID Controller pid) -> {
                pid.setTolerance(0.5)
            }),
            FeedforwardController.forConstantGravity(0, 0, 0, 0),
            TargetType.Velocity);
            intakeMotor.getSysIDCommands("intakeneo", 0, 0, 0);
            
    }


    public Command startIntake() {
        Commands.runOnce(() -> {
            intakeMotor.setTarget(2.0);
        }, this).andThen(() -> {
            Commands.waitUntil(intake.atTarget());
        })
    }

    public Command stopIntake() {
        Commands.runOnce(() -> {
            intakeMotor.setTarget(0);
        }, this).andThen(() -> {
            Commands.waitUntil(intake.atTarget());
        })
    }


}



