package frc.robot;

public final class Constants {
    
    public static final class Teleop {
        public static final double JOYSTICK_DEADBAND = 0.12;
        public static final double MAX_SPEED_M_S = 2.0;
        public static final double MAX_ACCEL_M_S2 = 0.1;
        public static final double MAX_ANG_SPEED_RAD_S = Math.PI;
        public static final double MAX_ANG_ACCEL_RAD_S2 = Math.PI;


    }

    public static final class Auto { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double MAX_SPEED_M_S = 3;
        public static final double MAX_ACCEL_M_S2 = 3;
        public static final double MAX_ANG_SPEED_RAD_S = Math.PI;
        public static final double MAX_ANG_ACCEL_RAD_S2 = Math.PI;
    
        //Values for the PID holonomic follower used in auto
        public static final double KP_X_AXIS = 1;
        public static final double KP_Y_AXIS = 1;
        public static final double KP_THETA = 1;

        //trajectory end conditions to finish a trajectory command
        public static final double WAIT_TIME_S = 1.0; //time after trajectory was predicted to finish before finishing command if the target wasn't reached yet 
        public static final double TOLERANCE_XY_METERS = 0.1; 
        public static final double TOLERANCE_ROTATION_RAD = Math.toRadians(5);
    }
}
