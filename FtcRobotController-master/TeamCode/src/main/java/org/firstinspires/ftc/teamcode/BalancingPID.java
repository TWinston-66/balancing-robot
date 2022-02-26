package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Balancing PID", group="Linear Opmode")
public class BalancingPID extends BalancingRobot{

    double targetAngle = -90;
    double currentAngle = 0.0;
    double LOOP_TIME = 10.0;
    double speed = 0.0;
    ElapsedTime pidTimer = new ElapsedTime();
    double timerValue = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Initialization
        hardware.init(hardwareMap);

        // Ready For Game Start
        telemetry.addData("Status", "Ready For Start");
        telemetry.update();

        // Wait for the game start
        waitForStart();

        //##########################//

        pidTimer.reset();
        while(opModeIsActive()) {
            if((pidTimer.milliseconds() - timerValue) > LOOP_TIME) {
                timerValue = pidTimer.milliseconds();

                currentAngle = -getCurrentAngleX();
                speed = pid(targetAngle, currentAngle);

                // Robot has fallen
                if (currentAngle < 45 || currentAngle > 135) {
                    speed = 0.0;
                }

                hardware.LeftDrive.setPower(speed);
                hardware.RightDrive.setPower(speed);
            }
        }
    }

    public double pid(double target, double current) {
        double power = 0.0;

        double P = 0.0;
        double I = 0.0;
        double D = 0.0;

        double iTerm = 0.0;
        double lastTime = 0.0;
        double oldValue = 0.0;

        // Calculate Time
        double thisTime = pidTimer.milliseconds();
        double dT = thisTime - lastTime;
        lastTime = thisTime;

        // Calculate Error
        double error = target - current;

        // Calculate Integral Term
        iTerm += error * dT;

        // Calculate Derivative Term
        double dTerm = (oldValue - current) / dT;
        oldValue = current;

        // Total PID Error
        power = (error * P) + (iTerm * I) + (dTerm * D);

        power = Range.clip(power, -1.0, 1.0);

        return power;
    }
}
