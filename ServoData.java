package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoData {

    public Servo servo;
    public double startPosition;
    public double targetPosition;

    public ServoData(String servo, double startPosition, double targetPosition, HardwareMap hardwareMap, Servo.Direction direction)
    {
        this.servo = hardwareMap.get(Servo.class, servo);
        this.startPosition = startPosition;
        this.targetPosition = targetPosition;

        this.servo.setPosition(startPosition);

        this.servo.setDirection(direction);

    }
}
