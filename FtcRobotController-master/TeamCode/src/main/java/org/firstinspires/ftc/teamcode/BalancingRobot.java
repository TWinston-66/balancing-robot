package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class BalancingRobot extends LinearOpMode {

    Hardware hardware = new Hardware();


    public double getCurrentAngleZ()
    {
        return hardware.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
    }
    public double getCurrentAngleY()
    {
        return hardware.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).secondAngle;
    }
    public double getCurrentAngleX()
    {
        return hardware.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).thirdAngle;
    }
}
