package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShiftUtil {
    private static final double firstRumble = 10;
    private static final double secondRumble = 0;
    private static final double[] rumbleTimes = {
        130 + firstRumble, 130 + secondRumble, // 2:20 2:10
        105 + firstRumble, 105 + secondRumble, // 1:55 1:45
        80 + firstRumble, 80 + secondRumble,   // 1:30 1:20
        55 + firstRumble, 55 + secondRumble,   // 1:05 0:55
        30 + firstRumble, 30 + secondRumble,   // 0:40 0:30
        0 + firstRumble, 0 + secondRumble      // 0:10 0:00
    };
    private static final boolean[] rumbled = {
        true, true, 
        true, true, 
        true, true, 
        true, true, 
        true, true,
        true, true
    };

    static {
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            rumbled[0] = false;
            rumbled[1] = false;
            rumbled[2] = false;
            rumbled[3] = false;
            rumbled[4] = false;
            rumbled[5] = false;
            rumbled[6] = false;
            rumbled[7] = false;
            rumbled[8] = false;
            rumbled[9] = false;
            rumbled[10] = false;
            rumbled[11] = false;
        }));
        RobotModeTriggers.teleop().onFalse(Commands.runOnce(() -> {
            rumbled[0] = true;
            rumbled[1] = true;
            rumbled[2] = true;
            rumbled[3] = true;
            rumbled[4] = true;
            rumbled[5] = true;
            rumbled[6] = true;
            rumbled[7] = true;
            rumbled[8] = true;
            rumbled[9] = true;
            rumbled[10] = true;
            rumbled[11] = true;
        }));
    }

    public static final Trigger rumble = new Trigger(() -> {
        if (!DriverStation.isFMSAttached()) {
            return false;
        }
        double matchTime = DriverStation.getMatchTime();
        for (int i = 0; i < rumbleTimes.length; i++) {
            if (matchTime <= rumbleTimes[i] && !rumbled[i]) {
                rumbled[i] = true;
                return true;
            }
        }
        return false;
    });
}
