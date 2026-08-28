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

    private static double kDt = 0.02; 
    private static double kMaxVelocity = 1.75; 
    private static double kMaxAcceleration = 0.75; 
    private static double kP = 1.0; 
    private static double kI = 0.0;
    private static double kD = 0.7;
    private static double kS = 1.1;
    private static double kA = 0;
    private static double kV = 1.3;

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

        drivePID = new PIDController(4, 0, 0);
        driveFeedforward = new SimpleMotorFeedforward(0, 2.8, 0);

        anglePID = new ProfiledPIDController(
            10, 0, 0.1, new TrapezoidProfile.Constraints(20, 10));
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        angleFeedforward = new SimpleMotorFeedforward(0, 0.28, 0);
    }

    public Translation2d getPosition(){
        return position; 
    }

    public void setDesiredState(SwerveModuleState desiredState){
        state = desiredState; 

        state.optimize(new Rotation2d(angleSim.getAngularPositionRad()));


        double driveOut = drivePID.calculate(
            driveSim.getAngularVelocityRadPerSec() * Units.inchesToMeters(2), state.speedMetersPerSecond); 
        double driveFFout = driveFeedforward.calculate(state.speedMetersPerSecond); 

        double driveVoltage = MathUtil.clamp((driveOut + driveFFout), -12, 12);

        double angleOut = anglePID.calculate(
            angleSim.getAngularPositionRad(), state.angle.getRadians());
        double angleFFout = angleFeedforward.calculate(anglePID.getSetpoint().velocity); 

        double angleVoltage = MathUtil.clamp((angleOut + angleFFout), -12, 12);

        SmartDashboard.putNumber("driveVoltage", driveVoltage);
        SmartDashboard.putNumber("angleVoltage", angleVoltage);

        driveSim.setInputVoltage(driveVoltage);
        angleSim.setInputVoltage(angleVoltage); 

        driveSim.update(kDt);
        angleSim.update(kDt);

        SmartDashboard.putNumber("angleWanted", state.angle.getDegrees());
        SmartDashboard.putNumber("angleReal", angleSim.getAngularPosition().in(Degrees));

        SmartDashboard.putNumber("driveWanted", state.speedMetersPerSecond);
        SmartDashboard.putNumber("driveReal", driveSim.getAngularVelocityRadPerSec()  * Units.inchesToMeters(2));

    }

    public SwerveModuleState getCurrentState(){
        return new SwerveModuleState(
            driveSim.getAngularVelocityRadPerSec() * Units.inchesToMeters(2),
            new Rotation2d(angleSim.getAngularPositionRad()));
    }

    public SwerveModuleState getDesiredState(){
        return state; 
    }

    
}


