package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Curve Test", group="Linear Opmode")
public class RyanCurve extends BalancingRobot{

    double targetAngle = 90;
    double currentAngle;
    double speed;
    double angleError;

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

        while(opModeIsActive()) {
                currentAngle = getCurrentAngleX();
                angleError = targetAngle - currentAngle;

                if (currentAngle == 0) {
                    speed = 0.1;
                }
                else {
                    speed = 1.0;
                }
                telemetry.update();
            }
        }
}
