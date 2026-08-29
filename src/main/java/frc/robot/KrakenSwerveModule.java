package frc.robot;
import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class KrakenSwerveModule{
    private SwerveModuleState state; 
    private Translation2d position; 

    private PIDController drivePID; 
    private SimpleMotorFeedforward driveFeedforward; 

    private ProfiledPIDController anglePID; 
    private SimpleMotorFeedforward angleFeedforward; 

    private TalonFX driveMotor; 
    private TalonFX angleMotor; 
    private AbsoluteEncoder angleEncoder; 


    DCMotorSim angleSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, 12.8), 
            DCMotor.getKrakenX60(1));

    DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.01, 6.75), 
            DCMotor.getKrakenX60(1));

    public KrakenSwerveModule(Translation2d position, int driveMotorID, int angleMotorID, int encoderID){

        this.position = position; 
        driveMotor = new TalonFX(driveMotorID);
        angleMotor = new TalonFX(angleMotorID);

        drivePID = new PIDController(
            Constants.SwerveConstants.kDrive_kp, 
            Constants.SwerveConstants.kDrive_ki, 
            Constants.SwerveConstants.kDrive_kd);

        driveFeedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.kDrive_ks,
            Constants.SwerveConstants.kDrive_kv, 
            Constants.SwerveConstants.kDrive_ka);

        anglePID = new ProfiledPIDController(
            Constants.SwerveConstants.kAngle_kp,
            Constants.SwerveConstants.kAngle_ki,
            Constants.SwerveConstants.kAngle_kd, 
            new TrapezoidProfile.Constraints(
                Constants.SwerveConstants.kAngleMaxVelocity, Constants.SwerveConstants.kAngleMaxAccel));
        anglePID.enableContinuousInput(-Math.PI, Math.PI);

        angleFeedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.kAngle_ks,
            Constants.SwerveConstants.kAngle_kv,
            Constants.SwerveConstants.kAngle_ka);

    }

    public Translation2d getPosition(){
        return position; 
    }

    public void setDesiredState(SwerveModuleState desiredState){
        state = desiredState; 

        state.optimize(new Rotation2d(angleSim.getAngularPositionRad()));
        double driveVoltage = 0; 

        if(state.speedMetersPerSecond > 0.001){
        double driveOut = drivePID.calculate(
        driveSim.getAngularVelocityRadPerSec() * Units.inchesToMeters(Constants.SwerveConstants.kWheelDiameter), state.speedMetersPerSecond); 
        double driveFFout = driveFeedforward.calculate(state.speedMetersPerSecond); 

        driveVoltage = MathUtil.clamp((driveOut + driveFFout), -12, 12);
        }
      
        double angleOut = anglePID.calculate(
            angleSim.getAngularPositionRad(), state.angle.getRadians());
        double angleFFout = angleFeedforward.calculate(anglePID.getSetpoint().velocity); 

        double angleVoltage = MathUtil.clamp((angleOut + angleFFout), -12, 12);

        //SmartDashboard.putNumber("driveVoltage", driveVoltage);
        //SmartDashboard.putNumber("angleVoltage", angleVoltage);

        driveSim.setInputVoltage(driveVoltage);
        angleSim.setInputVoltage(angleVoltage); 

        driveSim.update(Constants.SwerveConstants.kDt);

        angleSim.update(Constants.SwerveConstants.kDt);

        //SmartDashboard.putNumber("angleWanted", state.angle.getDegrees());
        //SmartDashboard.putNumber("angleReal", angleSim.getAngularPosition().in(Degrees));

        //SmartDashboard.putNumber("driveWanted", state.speedMetersPerSecond);
        //SmartDashboard.putNumber("driveReal", driveSim.getAngularVelocityRadPerSec()  * Units.inchesToMeters(2));

    }

    public SwerveModuleState getCurrentState(){
        return new SwerveModuleState(
            driveSim.getAngularVelocityRadPerSec() * Units.inchesToMeters(Constants.SwerveConstants.kWheelDiameter),
            new Rotation2d(angleSim.getAngularPositionRad()));
    }

    public SwerveModuleState getDesiredState(){
        return state; 
    }

    
}


