
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LoggingConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  // imports motor id
  private final CANSparkMax m_shooterMotor1 = new CANSparkMax(
      ShooterConstants.kShooter1CanId, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotor2 = new CANSparkMax(
      ShooterConstants.kShooter2CanId, MotorType.kBrushless);

  private double speed1 = ShooterConstants.speed1;
  private double speed2 = ShooterConstants.speed2;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    SmartDashboard.putNumber("shooter1 speed", speed1);
    SmartDashboard.putNumber("shooter2 speed", speed2);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  public void forward() {
    m_shooterMotor1.set(speed1);
    m_shooterMotor2.set(speed2);

  }

  public void stop() {
    m_shooterMotor1.set(0);
    m_shooterMotor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tuneSpeeds();
    log();
  }

  public void log() {
    if (LoggingConstants.kLogging) {
    }
  }

  public void tuneSpeeds() {
    speed1 = SmartDashboard.getNumber("shooter1 speed", ShooterConstants.speed1);
    speed2 = SmartDashboard.getNumber("shooter2 speed", ShooterConstants.speed2);

    SmartDashboard.putNumber("shooter1 speed", speed1);
    SmartDashboard.putNumber("shooter2 speed", speed2);
  }

}