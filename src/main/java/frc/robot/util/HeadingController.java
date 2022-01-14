package frc.robot.util;

import static frc.robot.Constants.loopTime;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Swerve;

public class HeadingController {

    private static final double STABILISATION_DEADBAND = 1.0; // Degrees
    private static final double LOCK_DEADBAND = 2.0; // Degrees

    private static TrapezoidProfile.Constraints PROFILED_PID_CONSTRAINTS;

    private final PIDController stabilisationController;
    private final PIDController lockController;
    private final PIDController inPlaceController;

    enum ControlState {
        OFF,
        LOCK,
        STABILIZE
    }

    private ControlState state = ControlState.OFF;
    private boolean inPlace = false;
    private boolean isTurning = false;

    public HeadingController(
            double stableP,
            double stableD,
            double lockP,
            double lockI,
            double lockD,
            double inPlaceP,
            double inPlaceI,
            double inPlaceD) {

        // We shouldn't need to use kI to stablelise the controller because it doesn't have to be super
        // accurate
        stabilisationController =
                new PIDController(
                        stableP, 0.0, // We don't use I for the imprecise controllers
                        stableD, loopTime);

        PROFILED_PID_CONSTRAINTS = new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI);

        lockController = new PIDController(lockP, lockI, lockD, loopTime);
        inPlaceController = new PIDController(inPlaceP, inPlaceI, inPlaceD, loopTime);

        stabilisationController.enableContinuousInput(-Math.PI, Math.PI);
        lockController.enableContinuousInput(-Math.PI, Math.PI);
        inPlaceController.enableContinuousInput(-Math.PI, Math.PI);
        stabilisationController.setTolerance(STABILISATION_DEADBAND);
        lockController.setTolerance(LOCK_DEADBAND);
        inPlaceController.setTolerance(LOCK_DEADBAND);
    }

    public void lockHeading(Rotation2d heading) {
        state = ControlState.LOCK;
        lockController.setSetpoint(heading.getRadians());
        inPlaceController.setSetpoint(heading.getRadians());
    }

    /** Have the heading controller stabilise the current heading. */
    public void stabiliseHeading() {
        state = ControlState.STABILIZE;
        inPlace = false;
        isTurning = true; // Force the controller to reset on next update.
    }

    public void disable() {
        state = ControlState.OFF;
    }

    /**
     * Update the heading controller. This needs to be called from the periodic function of your
     * swerve drive subsystem.
     *
     * @param desiredSpeeds The speeds that the controller wants the robot to be driving at
     * @param heading The current heading of the robot
     */
    public void update(ChassisSpeeds desiredSpeeds, Rotation2d heading) {
        switch (state) {
            case LOCK:
                desiredSpeeds.omegaRadiansPerSecond += updateLock(desiredSpeeds, heading);
                return;
            case STABILIZE:
                desiredSpeeds.omegaRadiansPerSecond += updateStabilisation(desiredSpeeds, heading);
        }
    }

    /**
     * Return the value from the lock controller or the
     *
     * @param desiredSpeeds The desired speeds of the swerve drive
     * @param heading The current heading of the swerve drive
     * @return A double to add to the rotation velocity to PID to a heading.
     */
    private double updateLock(ChassisSpeeds desiredSpeeds, Rotation2d heading) {
        final double inPlaceTurnZone = Swerve.maxRotationalVelocity * 0.025;
        if (Math.hypot(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond)
                < inPlaceTurnZone) {
            if (!inPlace) {
                inPlace = true;
            }
            return inPlaceController.calculate(heading.getRadians());
        } else {
            inPlace = false;
            return lockController.calculate(heading.getRadians());
        }
    }

    /**
     * Update the heading stabilisation controller
     *
     * @param desiredSpeeds The desired speed of the swerve drive
     * @param heading The current heading of the swerve drive
     * @return A double to add to the rotation velocity to maintain a heading.
     */
    private double updateStabilisation(ChassisSpeeds desiredSpeeds, Rotation2d heading) {
        final double unlockZone = 0.1;
        if (Math.hypot(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond) > unlockZone) {
            if (isTurning) {
                stabilisationController.setSetpoint(heading.getRadians());
                stabilisationController.reset();
                isTurning = false;
            }
            return stabilisationController.calculate(heading.getRadians());
        }
        isTurning = true;
        return 0.0;
    }
}