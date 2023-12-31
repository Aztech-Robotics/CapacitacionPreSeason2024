package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.NeoWithPosition;
import frc.robot.commands.NeoWithJoystick;
import frc.robot.subsystems.NeoMotor;

public class Robot extends TimedRobot {
  //Otro cambio ejemplo 
  private Command m_autonomousCommand; 
  private final NeoMotor neoMotorSubsystem = NeoMotor.getInstance(); 
  private final NeoWithJoystick neoWithJoystick = new NeoWithJoystick(); 
  private final NeoWithPosition neoWithPosition = new NeoWithPosition(-10); 

  //Se ejecuta una vez cuando el robot se inicializa al momento de prender 
  @Override
  public void robotInit() {
    neoMotorSubsystem.setDefaultCommand(neoWithJoystick); 
    Controllers.getTriggerButtonA_0().onTrue(neoWithPosition); 
  }

  //Se ejecuta periodicamente durante el tiempo que el robot esta prendido
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  //Se ejecuta una vez cuando el robot se deshabilita
  @Override
  public void disabledInit() {}

  //Se ejecuta periodicamente cuando el robot esta deshabilitado
  @Override
  public void disabledPeriodic() {}

  //Se ejecuta una vez cuando empieza el autonomo
  @Override
  public void autonomousInit() {
    m_autonomousCommand = null;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  //Se ejecuta periodicamente cuando el autonomo esta activado
  @Override
  public void autonomousPeriodic() {}

  //Se ejecuta una vez cuando el modo teleoperado se activa
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  //Se ejecuta mientras el modo teleoperado esta activado
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
