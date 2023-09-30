package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controllers;
import frc.robot.subsystems.NeoMotor;

public class NeoWithJoystick extends CommandBase {
  private final NeoMotor mNeoMotor = NeoMotor.getInstance(); 
  public NeoWithJoystick() {
    addRequirements(mNeoMotor);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    mNeoMotor.setDutyCycleOutput(Controllers.getLeftY_0());
  }

  @Override
  public void end(boolean interrupted) {
    mNeoMotor.setDutyCycleOutput(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
