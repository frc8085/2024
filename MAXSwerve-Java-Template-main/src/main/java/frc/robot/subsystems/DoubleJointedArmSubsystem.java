
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DoubleJointedArmConstants;
import frc.robot.Constants.LoggingConstants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DoubleJointedArmSubsystem extends SubsystemBase {
    // imports motor id
    private final CANSparkFlex m_armMotor = new CANSparkFlex(
            DoubleJointedArmConstants.kArmCanId, MotorType.kBrushless);
    private final CANSparkMax m_shooterArmMotor = new CANSparkMax(
            DoubleJointedArmConstants.kShooterArmCanId, MotorType.kBrushless);

    /** Creates a new ExampleSubsystem. */
    public DoubleJointedArmSubsystem() {
        SmartDashboard.putNumber("Arm position", 0);
        SmartDashboard.putNumber("Shooter position", 0);
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */

    public void setArmPosition() {

    }

    public void setShooterPosition() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        log();
    }

    public void log() {
        if (LoggingConstants.kLogging) {
        }
    }

}