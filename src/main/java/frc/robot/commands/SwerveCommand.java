package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;


public class SwerveCommand extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationXSup;
    private DoubleSupplier rotationYSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier dampenSup;
    private PIDController rotationController;
    

    public SwerveCommand(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationXSup, DoubleSupplier rotationYSup, BooleanSupplier robotCentricSup, BooleanSupplier dampen) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        //TODO: Tune heading PID
        rotationController = new PIDController(Constants.Swerve.HeadingKP, Constants.Swerve.HeadingKI, Constants.Swerve.HeadingKD );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Constants.Swerve.HeadingTolerence);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationXSup = rotationXSup;
        this.rotationYSup = rotationYSup;
        this.robotCentricSup = robotCentricSup;
        this.dampenSup = dampen;
        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband, Dampen */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double rotationVal = 0;
        double rotationValX = MathUtil.applyDeadband(rotationXSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double rotationValY = MathUtil.applyDeadband(rotationYSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double dynamicHeading = Math.atan2(-rotationValY, rotationValX);
        
        
     //heading direction state
        switch(States.driveState){
            case forwardHold:

                //heading lock - forward
               rotationVal = rotationController.calculate(s_Swerve.getHeading().getRadians(), Units.degreesToRadians(0));
               System.out.println("HEADING LOCK");
                break;
            case backwardHold:

                //heading lock - backward
                rotationVal = rotationController.calculate(s_Swerve.getHeading().getRadians(), Units.degreesToRadians(180));
                break;
            case leftHold:

                //heading lock - left
                rotationVal = rotationController.calculate(s_Swerve.getHeading().getRadians(), Units.degreesToRadians(90));
                break;
            case rightHold:

                //heading lock - right
                rotationVal = rotationController.calculate(s_Swerve.getHeading().getRadians(), Units.degreesToRadians(270));
                break;
            case DynamicLock:
        
                //heading lock - dynamic
                rotationVal = rotationController.calculate(s_Swerve.getHeading().getRadians(), Units.degreesToRadians(dynamicHeading));
                break;
            case standard:
            
                //normal
                rotationVal = rotationValX * Constants.Swerve.maxAngularVelocity;
                break;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal,
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}