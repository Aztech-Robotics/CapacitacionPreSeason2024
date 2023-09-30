package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controllers {
    public static final CommandXboxController control0 = new CommandXboxController(0);
    public static final CommandXboxController control1 = new CommandXboxController(1); 

    public static double getLeftY_0 (){
        return MathUtil.applyDeadband(-control0.getLeftY(), 0.2); 
    }

    public static Trigger getTriggerButtonA_0 (){
        return control0.a();
    }
}
