package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Shooter extends SubsystemBase implements Loggable {
    private Motor flywheel;
    private Motor hood;
    private Motor turret;
    private Transform2d shooterTransform = new Transform2d(0.5, 0.0, Rotation2d.kZero);
    private InterpolatingDoubleTreeMap flywheelSpeed;
    private InterpolatingDoubleTreeMap hoodAngle;
    private DoubleSubscriber flywheelSubscriber;

    public Shooter() {
        flywheelSpeed.put(1.0, 500.0);  // meters , motor speed units
        flywheelSpeed.put(2.0, 1000.0);
        hoodAngle.put(1.0, 60.0);
        flywheelSubscriber = HoundLog.tunable("Flywheel Speed", 0.0);
    }

    public Command readyShoot(Supplier<Pose2d> robotPose, Supplier<Translation2d> target) {
        // face the turret at the target
        // spin up the wheels
        // make the hood at the right angle
        return Commands.run(() -> {
            Rotation2d targetAngle = robotPose.get().plus(shooterTransform).getTranslation().minus(target.get()).getAngle();
            Rotation2d turretAngle = targetAngle.plus(robotPose.get().getRotation()); // might be minus
            double distance = robotPose.get().getTranslation().getDistance(target.get());
            flywheel.setTarget(flywheelSpeed.get(distance));
            hood.setTarget(hoodAngle.get(distance));
            turret.setTarget(turretAngle.getDegrees());
        }, this);
    }
    
    public Command test() {
        return Commands.run(() -> {
            flywheel.setTarget(flywheelSubscriber.get());
        },this);
    }

    public Command idle() {
        // flywheel.setTarget(0);
        return Commands.runOnce(() -> {
            flywheel.setVoltage(0);

        }, this).andThen(Commands.idle());
    }


    





    
    @Override
    public void log(String path) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
    
}