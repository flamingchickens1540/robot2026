package org.team1540.robot2026.util.hid;

import edu.wpi.first.wpilibj.GenericHID;

public class EnvisionController extends GenericHID {
    public enum Button {
        A(1),
        B(2),
        X(3),
        Y(4),
        L_BUMPER(5),
        R_BUMPER(6),
        BACK(7),
        START(8),
        L_STICK(9),
        R_STICK(10),
        G1(12),
        G2(13),
        G3(14),
        G4(15),
        G5(16);

        public final int id;

        Button(int id) {
            this.id = id;
        }

        @Override
        public String toString() {
            return this.name() + " Button";
        }
    }

    public enum Axis {
        LEFT_X(0),
        LEFT_Y(1),
        RIGHT_X(2),
        RIGHT_Y(5),
        LEFT_TRIGGER(3),
        RIGHT_TRIGGER(4);

        public final int id;

        Axis(int id) {
            this.id = id;
        }

        @Override
        public String toString() {
            return this.name() + " Axis";
        }
    }

    public EnvisionController(int port) {
        super(port);
    }

    public double getLeftX() {
        return getRawAxis(Axis.LEFT_X.id);
    }

    public double getLeftY() {
        return getRawAxis(Axis.LEFT_Y.id);
    }

    public double getRightX() {
        return getRawAxis(Axis.RIGHT_X.id);
    }

    public double getRightY() {
        return getRawAxis(Axis.RIGHT_Y.id);
    }

    public double getLeftTrigger() {
        return (getRawAxis(Axis.LEFT_TRIGGER.id) + 1) / 2.0; // Convert from [-1, 1] to [0, 1]
    }

    public double getRightTrigger() {
        return (getRawAxis(Axis.RIGHT_TRIGGER.id) + 1) / 2.0; // Convert from [-1, 1] to [0, 1]
    }

    public boolean getAButton() {
        return getRawButton(Button.A.id);
    }

    public boolean getBButton() {
        return getRawButton(Button.B.id);
    }

    public boolean getXButton() {
        return getRawButton(Button.X.id);
    }

    public boolean getYButton() {
        return getRawButton(Button.Y.id);
    }

    public boolean getLeftBumper() {
        return getRawButton(Button.L_BUMPER.id);
    }

    public boolean getRightBumper() {
        return getRawButton(Button.R_BUMPER.id);
    }

    public boolean getBackButton() {
        return getRawButton(Button.BACK.id);
    }

    public boolean getStartButton() {
        return getRawButton(Button.START.id);
    }

    public boolean getLeftStickButton() {
        return getRawButton(Button.L_STICK.id);
    }

    public boolean getRightStickButton() {
        return getRawButton(Button.R_STICK.id);
    }

    public boolean getG1Button() {
        return getRawButton(Button.G1.id);
    }

    public boolean getG2Button() {
        return getRawButton(Button.G2.id);
    }

    public boolean getG3Button() {
        return getRawButton(Button.G3.id);
    }

    public boolean getG4Button() {
        return getRawButton(Button.G4.id);
    }

    public boolean getG5Button() {
        return getRawButton(Button.G5.id);
    }

    public boolean getLeftOuterPaddle() {
        return getLeftStickButton();
    }

    public boolean getRightOuterPaddle() {
        return getRightStickButton();
    }

    public boolean getLeftInnerPaddle() {
        return getPOV() == 270;
    }

    public boolean getRightInnerPaddle() {
        return getPOV() == 90;
    }

    public boolean getLeftSideButton() {
        return getPOV() == 180;
    }

    public boolean getRightSideButton() {
        return getPOV() == 0;
    }
}
