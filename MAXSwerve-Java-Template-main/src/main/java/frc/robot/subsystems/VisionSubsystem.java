package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {
    public double getAprilTagID() {
        // The computer fought with us :( thats why it looks so weird
        double[] id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid")
                .getDoubleArray(new double[6]);
        return id[0];
    };

    public double[] getAprilTagLocation() {
        double[] location = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
                .getDoubleArray(new double[6]);
        return location;
    }
}

/**
 * NOTE:
 * What neural network model should we use? Teachable machine is a classifer
 * meaning it classifies stuff.
 * However a detection model will require a bit more work and *some* python (I
 * can do that)
 * but, as in the name, it only detects which is what we need it for. -Frank
 */