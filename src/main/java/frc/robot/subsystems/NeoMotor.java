package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeoMotor extends SubsystemBase {
  private static NeoMotor mNeoMotor;
  //Declaracion de motor
  private final CANSparkMax motor = new CANSparkMax(Constants.NeoMotorConstants.id_neo, MotorType.kBrushless);
  //Declaracion de objeto para configurar un encoder absoluto conectado al Spark
  private final SparkMaxAbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  //Declaracion de objeto para configurar un encoder relativo conectado al Spark (8192 son las CPR del Through Bore Encoder, el que mas se utiliza para conectar directamente a los Neos) 
  private final RelativeEncoder alternateEncoder = motor.getAlternateEncoder(8192); 
  //Declaración de objeto para configurar el encoder relativo que viene integrado en el Neo
  private final RelativeEncoder integratedEncoder = motor.getEncoder(); 
  //Declaracion de objeto para configurar un limit fisico en la direccion forward del motor (salida positiva)
  private final SparkMaxLimitSwitch hardLimitForward = motor.getForwardLimitSwitch(com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyClosed);
  //Declaracion de objeto para configurar un limit fisico en la direccion reverse del motor (salida negativa)
  private final SparkMaxLimitSwitch hardLimitReverse = motor.getForwardLimitSwitch(com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyClosed);
  //Declaracion de objeto para configurar el controlador PID del motor
  private final SparkMaxPIDController pidController = motor.getPIDController();

  /*
   * El metodo constructor del subsistema normalmente lo utilizamos para inicializar valores, resetear sensores.
   * Cosas que solo se realizan una vez. Por ejemplo: 
   */
  private NeoMotor() {
    //Configuracion del ABSOLUTE Encoder
    //(True / False) - Si el encoder mide de forma contraria a la salida del motor, osea si da posicion negativa y da salida positiva y viceversa
    absoluteEncoder.setInverted(true);
    //Valor por el que se multiplican las unidades nativas que manda el encoder para obtener la posicion en las unidades que deseas. (Rotaciones, Radianes, Grados, etc)
    absoluteEncoder.setPositionConversionFactor(1);
    //Valor por el que se multiplican las unidades nativas que manda el encoder para obtener la velocidad en las unidades que deseas. (R/s, Rad/s, Deg/s, etc)
    absoluteEncoder.setVelocityConversionFactor(1);
    //Valor que te permite ajustar tu cero si es 30, resta eso a la posicion real que mandaría y te da una nueva posición con el cero que tu consideras
    absoluteEncoder.setZeroOffset(0);
    
    //Configuracion del RELATIVE Encoder. Tanto para el ALTERNO como el INTEGRADO es la misma clase por lo que aplican los mismos metodos y atributos. (Utilizare el integrado)
    //True / False - Si el encoder mide de forma contraria a la salida del motor, osea si da posicion negativa y da salida positiva y viceversa
    integratedEncoder.setInverted(true);
    //Configura cada cuanto tiempo el sensor guarda valores de posicion para poder calcular la velocidad
    integratedEncoder.setMeasurementPeriod(100);
    //Le da una nueva posicion al sensor (Para resetearlo se deja en 0)
    integratedEncoder.setPosition(0); 
    //Valor por el que se multiplican las unidades nativas que manda el encoder para obtener la posicion en las unidades que deseas. (Rotaciones, Radianes, Grados, etc)
    integratedEncoder.setPositionConversionFactor(1);
    //Valor por el que se multiplican las unidades nativas que manda el encoder para obtener la velocidad en las unidades que deseas. (R/s, Rad/s, Deg/s, etc)
    integratedEncoder.setVelocityConversionFactor(1);

    //Configuracion del controlador PID
    //Configura el sensor que utilizara para que el motor se ajuste a la posicion deseada de acuerdo a los valores de dicho sensor. Le puedo pasar objetos de encoder absoluto o relativo.
    pidController.setFeedbackDevice(absoluteEncoder);
    pidController.setFeedbackDevice(alternateEncoder);
    pidController.setFeedbackDevice(integratedEncoder);
    //Configura el valor mínimo y maximo de salida para poder utilizar si el motor utiliza Closed Loop
    pidController.setOutputRange(-1, 1, 0); 
    //Configura el valor de KP para el controlador
    pidController.setP(Constants.NeoMotorConstants.kp_neo, 0); 
    //Configura el valor de KI para el controlador
    pidController.setI(Constants.NeoMotorConstants.ki_neo, 0); 
    //Configura el valor de KD para el controlador
    pidController.setD(Constants.NeoMotorConstants.kd_neo, 0); 
    //Configura el valor de KF para el controlador
    pidController.setFF(Constants.NeoMotorConstants.kf_neo, 0); 
    //Configura el valor de la maxima velocidad para el controlador en caso de utilizar SmartMotion
    pidController.setSmartMotionMaxVelocity(Constants.NeoMotorConstants.kMaxVel_neo, 0);
    //Configura el valor de la maxima velocidad para el controlador en caso de utilizar SmartMotion
    pidController.setSmartMotionMaxAccel(Constants.NeoMotorConstants.kMaxAccel_neo, 0);
    //Configura un error de tolerancia para el controlador
    pidController.setSmartMotionAllowedClosedLoopError(0, 0);
    /*
     * En el caso de encoders absolutos hay una discontinuidad, no importan sus unidades: de 360 regresa a 0,
     * a veces tambien de maneja con signo de -180 a 0 y de 0 a 180, la discontinuidad es de 180 a -180 o viceversa. 
     * Vamos a poner un ejemplo con el rango de 0 a 360. Si estoy en 350 grados y quiero llegar a 0. El calculo de mi error seria:
     * error = 0(setpoint) - 350(current position) = -350 grados de error. 
     * Es un error negativo, sean cuales sean los valores k del controlador PID nos va a dar una salida negativa, por lo que para 
     * llegar a 0 el motor va a recorrer 350 grados de regreso y llegar, pero realmente no es su ruta mas corta, ya que esta seria
     * avanzar 10 grados y llegar a 0. Eso es lo que hace el PIDWrapping en el caso de los Neo, considera esta discontinuidad a la
     * hora de llegar a una posicion deseada para poder tomar la ruta mas corta.
     */
    pidController.setPositionPIDWrappingEnabled(true);  
    /*
     * Valores minimos y maximos que puede marcar el encoder, es 0 si el el rango (0 a 360), -180 si es el rango (-180 a 180), 
     * tambien normalmente se utilizan radianes y es lo mismo, solo que en radianes claro. Y ya teniendo configurado nuestro factor de
     * conversion
     */
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingMaxInput(360);

    //Configuracion de los HardLimits
    //Para habilitarlos / deshabilitarlos
    hardLimitForward.enableLimitSwitch(true);
    hardLimitReverse.enableLimitSwitch(true);
  }

  public static NeoMotor getInstance (){
    if (mNeoMotor == null){
      mNeoMotor = new NeoMotor();
    }
    return mNeoMotor;
  }

  //Este metodo es heredado de la clase Subsistema, se ejecuta periodicamente desde que se declara en alguna parte del codigo
  @Override
  public void periodic() {}

  //Para obtener la posicion del encoder absoluto
  public double getAbsoluteEncoderPosition (){
    return absoluteEncoder.getPosition();
  }

  //Para obtener la velocidad del encoder absoluto
  public double getAbsoluteEncoderVelocity (){
    return absoluteEncoder.getVelocity(); 
  }

  //Para resetear el encoder alterno / integrado a una nueva posicion
  public void resetRelativeEncoder (double new_position){
    integratedEncoder.setPosition(new_position);
  }

  //Para obtener la posicion del encoder alterno / integrado
  public double getRelativeEncoderPosition (){
    return integratedEncoder.getPosition();
  }

  //Para obtener la velocidad del encoder alterno / integrado
  public double getRelativeEncoderVelocity (){
    return integratedEncoder.getVelocity();
  }

  //Para obtener el valor booleano del HardLimit 
  public boolean isForwardLimitPressed () {
    return hardLimitForward.isPressed();
  }
  public boolean isReverseLimitPressed () {
    return hardLimitReverse.isPressed();
  }

  /*
   * Para darle una salida al motor de acuerdo al modo indicado. Los que mas utilizamos son:
   * DutyCycle: De -1 a 1
   * Position: Posicion en las unidades que hayas configurado, las nativas son revoluciones. 
   * Velocity: Posicion en las unidades que hayas configurado, las nativas son RPM. 
   * SmartMotion: Lo mismo que Position pero con un perfil de movimiento (mas controlado, mas suave)
   * SmartVelocity: Lo mismo que Velocity pero con un perfil de movimiento (mas controlado, mas suave)
   */
  public void setDutyCycleOutput (double output) {
    pidController.setReference(output, CANSparkMax.ControlType.kDutyCycle);  
  }
  public void setPositionOutput (double position) {
    pidController.setReference(position, CANSparkMax.ControlType.kPosition);  
  }
  public void setVelocityOutput (double velocity) {
    pidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);  
  }
  public void setSmartMotionOutput (double position) {
    pidController.setReference(position, CANSparkMax.ControlType.kSmartMotion);  
  }
  public void setSmarVelocityOutput (double velocity) {
    pidController.setReference(velocity, CANSparkMax.ControlType.kSmartVelocity);  
  }
}
