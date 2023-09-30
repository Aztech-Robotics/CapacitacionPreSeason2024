package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FalconMotor extends SubsystemBase {
  private static FalconMotor mFalconMotor;
  //Declaracion de objeto 
  private final TalonFX motor = new TalonFX(Constants.FalconMotorConstants.id_falcon);
  //Delaracion de objeto de configuracion para el motor
  private TalonFXConfiguration motor_config = new TalonFXConfiguration(); 
  
  private FalconMotor() {
    /*
     * Atributo que permite configurar el Continuos Wrap o el 
     * equivalente a PIDWrappingEnabled de Spark Max
     */
    motor_config.ClosedLoopGeneral.ContinuousWrap = true; 
  }

  public static FalconMotor getInstance (){
    if (mFalconMotor == null){
      mFalconMotor = new FalconMotor(); 
    }
    return mFalconMotor;
  }

  @Override
  public void periodic() {
  }
}
