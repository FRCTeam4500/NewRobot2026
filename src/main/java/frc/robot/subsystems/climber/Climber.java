package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.Loggable;

public class Climber extends SubsystemBase implements Loggable {
    
    
    private Motor climberMotor;

    public Climber(){
        climberMotor = Motor.fromTalonFX(
            0, 
            (TalonFX motorFx) -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits.SupplyCurrentLimit = 40;
            }, 
            (FeedforwardSim balls) -> {
                
            }, 
            0, 
            FeedbackController.fromPID(1, 0, 0, (PIDController pid) ->
            {pid.setTolerance(1);}), 
            FeedforwardController.forArmGravity(0, 0, 0, 0), 
            TargetType.Position);
            climberMotor.getSysIDCommands("climber", 0, 0, 0);
            
        
    }
    
    



    public Command runClimber() {
        return Commands.runOnce();
    }
    



    
    @Override
    public void log(String path) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }



}
