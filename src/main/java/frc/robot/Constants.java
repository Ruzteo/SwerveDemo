package frc.robot;

public final class Constants {

    public static class SwerveConstants{

        public static final double kDt = 0.20; 
        public static final boolean isSim = true; 

        public static final double kAngleMaxVelocity = 1.75; 
        public static final double kAngleMaxAcceleration = 0.75; 

        public static final double kAnglePID_P = 1.0;  
        public static final double kAnglePID_I = 0.0;  
        public static final double kAnglePID_D = 0.7;

        public static final double kAngleFF_S = 1.1;  
        public static final double kAngleFF_A = 0.0;  
        public static final double kAngleFF_V = 1.3;


        public static final double kDriveMaxVelocity = 1.75; 
        public static final double kDriveMaxAcceleration = 0.75; 

        public static final double kDrivePID_P = 1.0;  
        public static final double kDrivePID_I = 0.0;  
        public static final double kDrivePID_D = 0.7;

        public static final double kDriveFF_S = 1.1;  
        public static final double kDriveFF_A = 0.0;  
        public static final double kDriveFF_V = 1.3;
        
        

      
    }
}
