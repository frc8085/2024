

  
    

  

  
    
  



  

  

    

  package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LoggingConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.DriveConstants;

public class ShooterSubsystem extends SubsystemBase {
  // imports motor id
  private final CANSparkMax m_shooterMotor1 = new CANSparkMax(ShooterConstants.kShooter1CanId, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotor2 = new CANSparkMax(ShooterConstants.kShooter2CanId, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    SmartDashboard.putNumber("shooter1 speed", ShooterConstants.speed1);
    SmartDashboard.putNumber("shooter2 speed", ShooterConstants.speed2);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */


   public void forward1() {
      m_shooterMotor1.set(SmartDashboard.getNumber("shooter1 speed", ShooterConstants.speed1));
   
  }

   public void forward2() {
         m_shooterMotor2.set(SmartDashboard.getNumber("shooter2 speed", ShooterConstants.speed2));
    }




   public void stop1() {
      m_shooterMotor1.set(0);
  
    
  }

   public void stop2() {
    
  
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
    double speed1 = SmartDashboard.getNumber("shooter1 speed", ShooterConstants.speed1);
    double speed2 = SmartDashboard.getNumber("shooter2 speed", ShooterConstants.speed1);

    SmartDashboard.putNumber("shooter1 speed", speed1);
    SmartDashboard.putNumber("shooter2 speed", speed2);
  }

}