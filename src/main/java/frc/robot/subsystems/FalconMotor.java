package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FalconMotor extends SubsystemBase {
  private static FalconMotor mFalconMotor;
  //Declaracion de objeto 
  private final TalonFX motor = new TalonFX(Constants.FalconMotorConstants.id_falcon);
  //Delaracion de objeto de configuracion para el motor
  private TalonFXConfiguration motor_config = new TalonFXConfiguration(); 
  
  private FalconMotor() {
    //Subconfiguracion de Current-Limits
    /* Para entender la diferencia entre Supply Current Limit y Stator Current Limit
     * revisar: https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html#current-limit 
     */
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs(); 
      //El limite de corriente se activa hasta que exceda... (A)
      currentLimitsConfigs.SupplyCurrentThreshold = 0;
      //Por al menos... (Seg)
      currentLimitsConfigs.SupplyTimeThreshold = 0;
      //Si el limite de corriente se activa, entonces la corriente se mantiene a... (A)
      currentLimitsConfigs.SupplyCurrentLimit = 0;
      //Habilita el limite de corriente de entrada 
      currentLimitsConfigs.SupplyCurrentLimitEnable = true;
      //Habilita el limite de corriente del motor 
      currentLimitsConfigs.StatorCurrentLimitEnable = true;
      //Si el limite de corriente se activa, entonces la corriente se mantiene a... (A)
      currentLimitsConfigs.StatorCurrentLimit = 0;
      
    //Subconfiguracion de Motor Output
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
      //Si el request de salida se encuentra dentro del rango de banda muerta, se da una salida neutral
      motorOutputConfigs.DutyCycleNeutralDeadband = 0;
      //Determina la direccion de salida del motor 
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
      //El modo neutral determina el comportamiento del motor cuando su salida es 0 (no se mueve)
      motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        
    //Limits Switch
    /*
     * Forward Limit aplica cuando hay request de salida positiva
     * Reverse Limit aplica cuando hay request de salida negativa
     * Para ambos aplican las mismas configuraciones, solo se cambia
     * entre ForwardLimit y ReverseLimit en el caso de los fisicos, y
     * entre ForwardSoftLimit y ReverseSoftLimit en el caso de los 
     * logicos 
     */

    //Subconfiguracion de Limit-Switch (fisicos)
    HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs(); 
      //Configuraciones aplicables para el limit conectado directamente al motor
      limitSwitchConfigs.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin; 
      //Configuraciones aplicables si tenemos un limit en el canbus
      limitSwitchConfigs.ForwardLimitRemoteSensorID = 0;
      //Normally Closed o Normally Open
      limitSwitchConfigs.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed; 
      //Para habilitar el limit
      limitSwitchConfigs.ForwardLimitEnable = true;
      //Para que cuando se presione el limit la posicion del rotor se restablezca al valor dado
      limitSwitchConfigs.ForwardLimitAutosetPositionValue = 0; 
      limitSwitchConfigs.ForwardLimitAutosetPositionEnable = true; 
      
    //Subconfiguracion de Limit-Switch (logicos)
    SoftwareLimitSwitchConfigs softLimitSwitchConfigs = new SoftwareLimitSwitchConfigs(); 
      softLimitSwitchConfigs.ForwardSoftLimitThreshold = 0;
      softLimitSwitchConfigs.ForwardSoftLimitEnable = true;
      
    //Subconfiguracion de Open Loop Ramps 
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
      //Cuanto tiempo toma ir de 0 a 1 de salida en modos de open loop (0 a 1 seg)
      openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0;
      //Cuanto tiempo toma ir de 0A a 300A de salida en modos de open loop (0 a 10 seg)
      openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0;
      //Cuanto tiempo toma ir de 0V a 12V de salida en modos de open loop (0 a 1 seg)
      openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0;
      
    //Subconfiguracion de Closed-Loop-Ramps
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
      //Cuanto tiempo toma ir de 0 a 1 de salida en modos de closed loop (0 a 1 seg)
      closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0;
      //Cuanto tiempo toma ir de 0A a 300A de salida en modos de closed loop (0 a 10 seg)
      closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0;
      //Cuanto tiempo toma ir de 0V a 12V de salida en modos de closed loop (0 a 1 seg)
      closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0;

    //Subconfiguracion de Feedback 
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs(); 
      feedbackConfigs.FeedbackRemoteSensorID = 0;
      feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; 
      feedbackConfigs.FeedbackRotorOffset = 0;
      feedbackConfigs.RotorToSensorRatio = 0;
      feedbackConfigs.SensorToMechanismRatio = 0;

    //Subconfiguracion de Closed-Loop
    ClosedLoopGeneralConfigs closedLoopConfigs = new ClosedLoopGeneralConfigs();
      //Permite configurar el Continuos Wrap o el equivalente a PIDWrappingEnabled en un Neo
      closedLoopConfigs.ContinuousWrap = true; 

    /*
     * Los slots solo guardan constantes para el controlador PID y
     * FeedForward (Voltage), se pueden utilizar 3 (0, 1 y 2)
     */
    //Subconfiguracion de los Slots
    Slot0Configs slot0Configs = new Slot0Configs(); 
      //Determinar Kp
      slot0Configs.kP = 0;
      //Determinar Ki
      slot0Configs.kI = 0;
      //Determinar Kd
      slot0Configs.kD = 0;
      //Determinar Ks
      slot0Configs.kS = 0;
      //Determinar Kv
      slot0Configs.kV = 0;

    //Subconfiguracion de Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
      //Maxima velocidad o maxima tasa de cambio de la posicion (rps: rotation per second)
      motionMagicConfigs.MotionMagicCruiseVelocity = 0;
      //Maxima aceleracion o maxima tasa de cambio de la velocidad (rps2: rotation per second squared)
      motionMagicConfigs.MotionMagicAcceleration = 0;
      //Maximo jerk o tasa de cambio de la aceleracion (rps3: rotation per second cubic) 
      motionMagicConfigs.MotionMagicJerk = 0;

    //Asignacion de las subconfiguraciones a la configuracion general
    motor_config.CurrentLimits = currentLimitsConfigs; 
    motor_config.MotorOutput = motorOutputConfigs;
    motor_config.HardwareLimitSwitch = limitSwitchConfigs;
    motor_config.SoftwareLimitSwitch = softLimitSwitchConfigs;
    motor_config.OpenLoopRamps = openLoopRampsConfigs;
    motor_config.ClosedLoopRamps = closedLoopRampsConfigs;
    motor_config.Feedback = feedbackConfigs;
    motor_config.ClosedLoopGeneral = closedLoopConfigs;
    motor_config.Slot0 = slot0Configs;
    motor_config.MotionMagic = motionMagicConfigs; 

    //Guarda la configuracion completa
    motor.getConfigurator().apply(motor_config); 
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
