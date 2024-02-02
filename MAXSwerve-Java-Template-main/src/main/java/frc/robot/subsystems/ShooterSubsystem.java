
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // imports motor id
  private final CANSparkMax m_shooterMotor1 = new CANSparkMax(
      ShooterConstants.kShooter1CanId, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotor2 = new CANSparkMax(
      ShooterConstants.kShooter2CanId, MotorType.kBrushless);
  // Encoders
  private AbsoluteEncoder m_shooterEncoder1;
  private AbsoluteEncoder m_shooterEncoder2;
  // PID Controllers
  private SparkPIDController m_shooter1PIDController = m_shooterMotor1.getPIDController();
  private SparkPIDController m_shooter2PIDController = m_shooterMotor2.getPIDController();
  // PID Constants for tuning
  double kShooter1P = ShooterConstants.kShooter1P;
  double kShooter1I = ShooterConstants.kShooter1I;
  double kShooter1D = ShooterConstants.kShooter1D;

  double kShooter2P = ShooterConstants.kShooter2P;
  double kShooter2I = ShooterConstants.kShooter2I;
  double kShooter2D = ShooterConstants.kShooter2D;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.

    m_shooterMotor1.restoreFactoryDefaults();
    m_shooterMotor2.restoreFactoryDefaults();
    // Setup encoders and PID controllers for the arm and shooter arms.
    m_shooterEncoder1 = m_shooterMotor1.getAbsoluteEncoder(Type.kDutyCycle);
    m_shooter1PIDController = m_shooterMotor1.getPIDController();
    m_shooter1PIDController.setFeedbackDevice(m_shooterEncoder1);

    m_shooterEncoder2 = m_shooterMotor2.getAbsoluteEncoder(Type.kDutyCycle);
    m_shooter2PIDController = m_shooterMotor2.getPIDController();
    m_shooter2PIDController.setFeedbackDevice(m_shooterEncoder2);

    // Apply position and velocity conversion factors for the encoders. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_shooterEncoder1.setPositionConversionFactor(ShooterConstants.kshooterEncoder1PositionFactor);
    m_shooterEncoder1.setVelocityConversionFactor(ShooterConstants.kshooterEncoder1VelocityFactor);

    m_shooterEncoder2.setPositionConversionFactor(ShooterConstants.kShooterEncoder2PositionFactor);
    m_shooterEncoder2.setVelocityConversionFactor(ShooterConstants.kShooterEncoder2VelocityFactor);
    // Didn't invert the encoders

    m_shooter1PIDController.setP(ShooterConstants.kShooter1P);
    m_shooter1PIDController.setI(ShooterConstants.kShooter1I);
    m_shooter1PIDController.setD(ShooterConstants.kShooter1D);
    m_shooter1PIDController.setFF(ShooterConstants.kShooter1FF);
    m_shooter1PIDController.setOutputRange(ShooterConstants.kShooter1MinOutput,
        ShooterConstants.kShooter1MaxOutput);
    m_shooter1PIDController.setSmartMotionMaxAccel(0.5, 0);
    m_shooter1PIDController.setSmartMotionMaxVelocity(0.5, 0);

    m_shooter2PIDController.setP(ShooterConstants.kShooter2P);
    m_shooter2PIDController.setI(ShooterConstants.kShooter2I);
    m_shooter2PIDController.setD(ShooterConstants.kShooter2D);
    m_shooter2PIDController.setFF(ShooterConstants.kShooter2FF);
    m_shooter2PIDController.setOutputRange(ShooterConstants.kShooter2MinOutput,
        ShooterConstants.kShooter2MaxOutput);
    m_shooter2PIDController.setSmartMotionMaxAccel(0.5, 0);
    m_shooter2PIDController.setSmartMotionMaxVelocity(0.5, 0);

    m_shooterMotor1.setIdleMode(ShooterConstants.kShooterMotor1IdleMode);
    m_shooterMotor1.setSmartCurrentLimit(ShooterConstants.kShooterMotor1CurrentLimit);

    m_shooterMotor2.setIdleMode(ShooterConstants.kShooterMotor2IdleMode);
    m_shooterMotor2.setSmartCurrentLimit(ShooterConstants.kShooterMotor2CurrentLimit);

  }
  // Returns the arm
  public double getArmPosition() {
    return m_shooterEncoder1.getPosition();
  }

  // Returns the Shooter arm
  public double getShooterArmPosition() {
    return m_shooterEncoder2.getPosition();
  }

  // Maintain arm position in degrees
  public void setArmPositionDegrees(double degreesArm) {
    // set degrees for arm, convert to encoder value)
    // double positionArm = degreesArm *
    // DoubleJointedArmConstants.kArmRevolutionsPerDegree;
    m_shooter1PIDController.setReference(degreesArm, ControlType.kPosition);
  }

  // Maintain shooter arm position in degrees
  public void setShooterArmPositionDegrees(double degreesShooterArm) {
    // set degrees for arm, convert to encoder value)
    // double positionShooterArm = degreesShooterArm *
    // DoubleJointedArmConstants.kShooterArmRevolutionsPerDegree;
    m_shooter2PIDController.setReference(degreesShooterArm, ControlType.kPosition);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    log();
    if (TUNING_MODE) {
      tunePIDs();
    }
  }

  public void log() {
    if (LoggingConstants.kLogging) {
      SmartDashboard.putNumber("Arm Position", getArmPosition());
      SmartDashboard.putNumber("Shooter Arm Position", getShooterArmPosition());
    }
  }

  public void addPIDToDashboard() {
    SmartDashboard.putNumber("kArmP", kArmP);
    SmartDashboard.putNumber("kArmI", kArmI);
    SmartDashboard.putNumber("kArmD", kArmD);
    SmartDashboard.putNumber("kShooterP", kShooterP);
    SmartDashboard.putNumber("kShooterI", kShooterI);
  }

  public void tunePIDs() {
    kArmP = SmartDashboard.getNumber("kArmP", 0);
    kArmI = SmartDashboard.getNumber("kArmI", 0);
    kArmD = SmartDashboard.getNumber("kArmD", 0);
    SmartDashboard.putNumber("kArmP", kArmP);
    SmartDashboard.putNumber("kArmI", kArmI);
    SmartDashboard.putNumber("kArmD", kArmD);

    kShooterP = SmartDashboard.getNumber("kShooterP", 0);
    kShooterI = SmartDashboard.getNumber("kShooterI", 0);
    SmartDashboard.putNumber("kShooterP", kShooterP);
    SmartDashboard.putNumber("kShooterI", kShooterI);
  }

}

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  /*
   * public void forward() {
   * m_shooterMotor1.set(speed1);
   * m_shooterMotor2.set(speed2);
   * 
   * }
   * 
   * public void stop() {
   * m_shooterMotor1.set(0);
   * m_shooterMotor2.set(0);
   * }
   * 
   * @Override
   * public void periodic() {
   * // This method will be called once per scheduler run
   * tuneSpeeds();
   * log();
   * }
   * 
   * public void log() {
   * if (LoggingConstants.kLogging) {
   * }
   * }
   * 
   * public void tuneSpeeds() {
   * speed1 = SmartDashboard.getNumber("shooter1 speed", ShooterConstants.speed1);
   * speed2 = SmartDashboard.getNumber("shooter2 speed", ShooterConstants.speed2);
   * 
   * SmartDashboard.putNumber("shooter1 speed", speed1);
   * SmartDashboard.putNumber("shooter2 speed", speed2);
   * }
   */

}