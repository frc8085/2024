package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {
  // imports motor id
  private final CANSparkMax m_intakeMotor = new CANSparkMax(DriveConstants.kIntake1CanId, MotorType.kBrushless);
  DigitalInput breakBeam = new DigitalInput(IntakeConstants.kIRPort);

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {

  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  public void forward() {
    m_intakeMotor.set(1);
  }

  public void stop() {
    m_intakeMotor.set(0);
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }

  public void log() {
    SmartDashboard.putBoolean("Break Beam", breakBeam.get());
  }

}