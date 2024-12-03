package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.util.Deadband;

public class DriveWithJoysticks extends Command {
    private DriveTemplate drive;
    private CommandJoystick leftJoystick;
    private CommandJoystick rightJoystick;
    private double maxLinearVelocity;
    private double maxAngularVelocity;
    private double joystickDeadband;

    public DriveWithJoysticks(
            DriveTemplate drive,
            CommandJoystick leftJoystick,
            CommandJoystick rightJoystick,
            double maxLinearVelocity,
            double maxAngularVelocity,
            double joystickDeadband) {
        this.drive = drive;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        this.maxLinearVelocity = maxLinearVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.joystickDeadband = joystickDeadband;

        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        Translation2d linearSpeeds = getLinearVelocity(-leftJoystick.getX(), -leftJoystick.getY());

        double omega = Deadband.oneAxisDeadband(-rightJoystick.getX(), joystickDeadband);
        omega = Math.copySign(omega * omega, omega);

        drive.setGoalSpeeds(
                new ChassisSpeeds(
                        linearSpeeds.getX() * maxLinearVelocity,
                        linearSpeeds.getY() * maxLinearVelocity,
                        omega * maxAngularVelocity),
                true);
    }

    /* returns a calculated translation with squared velocity */
    public Translation2d getLinearVelocity(double x, double y) {
        double[] deadbands = Deadband.twoAxisDeadband(x, y, joystickDeadband);

        double xDeadband = deadbands[0];
        double yDeadband = deadbands[1];
        double magnitude = Math.hypot(xDeadband, yDeadband);

        /* joystick x/y is opposite of field x/y
         * therefore, x and y must be flipped for proper rotation of pose
         * BEWARE: not flipping will cause forward on joystick to drive right on field
         */
        Rotation2d direction = new Rotation2d(y, x);
        double squaredMagnitude = magnitude * magnitude;

        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), direction)
                        .transformBy(new Transform2d(squaredMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

        return linearVelocity;
    }
}
