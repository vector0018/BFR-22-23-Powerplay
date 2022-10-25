package org.firstinspires.ftc.teamcode.hardware;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Config
public class SlideConstants {
    public static final double pulleyCircumfirence = 4.40945;
    public static final double slideTicksPerRev = 537.7;
    public static final double maxRPM = 312;

    public static final double maxTargetPosition = 42;
    public static final double minTargetPosition = 0;

    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0.1, 0, 0,
            getMotorVelocityF(maxRPM / 60 * slideTicksPerRev));

    public static double kV = 1.0 / rpmToVelocity(maxRPM);
    public static double kA = 0;
    public static double kStatic = 0;

    public static double maxVel = 50;
    public static double maxAccel = 30;

    public static double encoderTicksToInches(double ticks) {
        return pulleyCircumfirence * ticks / slideTicksPerRev;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * 2 * Math.PI * pulleyCircumfirence / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }


}
