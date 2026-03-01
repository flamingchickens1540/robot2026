package org.team1540.robot2026.util.hid;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandEnvisionController extends CommandGenericHID {
    private final EnvisionController hid;

    public CommandEnvisionController(int port) {
        super(port);
        hid = new EnvisionController(port);
    }

    @Override
    public EnvisionController getHID() {
        return hid;
    }

    public Trigger a(EventLoop loop) {
        return button(EnvisionController.Button.A.id, loop);
    }

    public Trigger a() {
        return a(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger b(EventLoop loop) {
        return button(EnvisionController.Button.B.id, loop);
    }

    public Trigger b() {
        return b(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger x(EventLoop loop) {
        return button(EnvisionController.Button.X.id, loop);
    }

    public Trigger x() {
        return x(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger y(EventLoop loop) {
        return button(EnvisionController.Button.Y.id, loop);
    }

    public Trigger y() {
        return y(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftBumper(EventLoop loop) {
        return button(EnvisionController.Button.L_BUMPER.id, loop);
    }

    public Trigger leftBumper() {
        return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightBumper(EventLoop loop) {
        return button(EnvisionController.Button.R_BUMPER.id, loop);
    }

    public Trigger rightBumper() {
        return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftStick(EventLoop loop) {
        return button(EnvisionController.Button.L_STICK.id, loop);
    }

    public Trigger leftStick() {
        return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightStick(EventLoop loop) {
        return button(EnvisionController.Button.R_STICK.id, loop);
    }

    public Trigger rightStick() {
        return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger back(EventLoop loop) {
        return button(EnvisionController.Button.BACK.id, loop);
    }

    public Trigger back() {
        return back(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger start(EventLoop loop) {
        return button(EnvisionController.Button.START.id, loop);
    }

    public Trigger start() {
        return start(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger g1(EventLoop loop) {
        return button(EnvisionController.Button.G1.id, loop);
    }

    public Trigger g1() {
        return g1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger g2(EventLoop loop) {
        return button(EnvisionController.Button.G2.id, loop);
    }

    public Trigger g2() {
        return g2(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger g3(EventLoop loop) {
        return button(EnvisionController.Button.G3.id, loop);
    }

    public Trigger g3() {
        return g3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger g4(EventLoop loop) {
        return button(EnvisionController.Button.G4.id, loop);
    }

    public Trigger g4() {
        return g4(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger g5(EventLoop loop) {
        return button(EnvisionController.Button.G5.id, loop);
    }

    public Trigger g5() {
        return g5(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftOuterPaddle(EventLoop loop) {
        return leftStick(loop);
    }

    public Trigger leftOuterPaddle() {
        return leftOuterPaddle(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightOuterPaddle(EventLoop loop) {
        return rightStick(loop);
    }

    public Trigger rightOuterPaddle() {
        return rightOuterPaddle(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftInnerPaddle() {
        return povLeft();
    }

    public Trigger rightInnerPaddle() {
        return povRight();
    }

    public Trigger leftSideButton() {
        return povDown();
    }

    public Trigger rightSideButton() {
        return povUp();
    }

    public Trigger leftTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(EnvisionController.Axis.LEFT_TRIGGER.id, 2 * threshold - 1, loop); // Convert threshold from [0, 1] to [-1, 1]
    }

    public Trigger leftTrigger(double threshold) {
        return leftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftTrigger() {
        return leftTrigger(0.5);
    }

    public Trigger rightTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(EnvisionController.Axis.RIGHT_TRIGGER.id, 2 * threshold - 1, loop); // Convert threshold from [0, 1] to [-1, 1]
    }

    public Trigger rightTrigger(double threshold) {
        return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightTrigger() {
        return rightTrigger(0.5);
    }

    public double getLeftX() {
        return hid.getLeftX();
    }

    public double getLeftY() {
        return hid.getLeftY();
    }

    public double getRightX() {
        return hid.getRightX();
    }

    public double getRightY() {
        return hid.getRightY();
    }
}
