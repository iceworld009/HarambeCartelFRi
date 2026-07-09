public void handleTurret() {
    double x = follower.getPose().getX();
    double y = follower.getPose().getY();

    double dx = HardwareClass.blueX - x;
    double dy = HardwareClass.blueY - y;

    double goalAngle = Math.atan2(dy, dx); // field-relative
    double thetaR = follower.getPose().getHeading(); // robot heading

    double targetAngle = goalAngle - thetaR;

    // normalize to [-pi, pi] (important!)
    targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));

    double targetPosition = convertToNewRange(
            targetAngle,
            Math.PI / 2, -Math.PI / 2,
            312, -312
    );

    targetPosition = Math.max(-260, Math.min(260, targetPosition));

    telemetry.addData("Target Angle (rad)", targetAngle);
    telemetry.addData("Turret Encoder", turret.getPosition());
    telemetry.update();

    turret.goToPosition(targetPosition);
}
