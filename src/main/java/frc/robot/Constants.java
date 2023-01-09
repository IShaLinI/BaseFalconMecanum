package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;

public class Constants {
    public static class DriveConstants  {

        //TODO Change these to match your robot
        public static final int FRONT_LEFT_ID = 1;
        public static final int FRONT_RIGHT_ID = 2;
        public static final int BACK_LEFT_ID = 3;
        public static final int BACK_RIGHT_ID = 4;
        public static final int GYRO_ID = 5;

        //TODO Change these to match your robot. Accurate Cad model will give more accurate results than a tape measure. THESE MATTER ALOT.
        public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(0.381, 0.381);
        public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(0.381, -0.381);
        public static final Translation2d BACK_LEFT_POSITION = new Translation2d(-0.381, 0.381);
        public static final Translation2d BACK_RIGHT_POSITION = new Translation2d(-0.381, -0.381);

        //Conversions so you can get usable numbers from encoders
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final double FALCON_CPR = 2048;
        public static final double FALCON_TO_METERS = Math.PI * WHEEL_DIAMETER / 2048 * KitbotGearing.k10p71.value;

        public static final double MAX_SPEED = 10;

    }

    public static class VisionConstants {

        //TODO Change the name of your camera here to whatever it is in the PhotonVision UI.
        public static final String FORWARD_CAMERA_NAME = "limelight";

        //TODO Update this for the actual robot. Accurate Cad model will give more accurate results than a tape measure. THIS MATTERS ALOT.
        public static final Transform3d robotToCam = null;

    }
}
