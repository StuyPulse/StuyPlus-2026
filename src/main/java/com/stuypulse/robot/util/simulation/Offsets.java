package com.stuypulse.robot.util.simulation;

import static edu.wpi.first.units.Units.*;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.struct.StructSerializable;

/**
 *
 *
 * <h2>Record that holds CAD Offsets</h2>
 *
 * <p>
 * Sourced from CAD exports to hold a component's positional offsets from their
 * position in CAD
 * to their position in sim
 *
 * <pre>{@code
 * Offsets SHOOTER = new Offsets(-0.1016, 0.2032, 0.3255,
 *         Degrees.of(90), Degrees.of(0), Degrees.of(-90));
 *
 * Pose3d shooterPose = SHOOTER.withRotation(
 *         new Rotation3d(0, 0, turretSim.getAngle().getRadians()));
 * }</pre>
 *
 * @param x     X translation from the robot origin (meters)
 * @param y     Y translation from the robot origin (meters)
 * @param z     Z translation from the robot origin (meters)
 * @param roll  Rotation about the X axis
 * @param pitch Rotation about the Y axis
 * @param yaw   Rotation about the Z axis
 */
public record Offsets (
        Distance x, Distance y, Distance z, Angle roll, Angle pitch, Angle yaw)
        implements StructSerializable {
    public static final OffsetsStruct struct = new OffsetsStruct();

    /**
     *
     *
     * <h4>Constructs an Offsets instance with no rotation</h4>
     *
     * @param x X translation from the robot origin (meters)
     * @param y Y translation from the robot origin (meters)
     * @param z Z translation from the robot origin (meters)
     */
    public Offsets(double x, double y, double z) {
        this(Meters.of(x), Meters.of(y), Meters.of(z), Radians.of(0), Radians.of(0), Radians.of(0));
    }

    /**
     *
     *
     * <h4>Constructs an Offsets instance with no rotation, using your unit of
     * choice</h4>
     *
     * @param x X translation from the robot origin
     * @param y Y translation from the robot origin
     * @param z Z translation from the robot origin
     */
    public Offsets(Distance x, Distance y, Distance z) {
        this(x.in(Meters), y.in(Meters), z.in(Meters));
    }

    /**
     *
     *
     * <h4>Constructs an Offsets instance with no translation</h4>
     *
     * @param roll  Rotation about the X axis
     * @param pitch Rotation about the Y axis
     * @param yaw   Rotation about the Z axis
     */
    public Offsets(Angle roll, Angle pitch, Angle yaw) {
        this(Meters.of(0), Meters.of(0), Meters.of(0), roll, pitch, yaw);
    }

    /**
     *
     *
     * <h4>Constructs an Offsets instance with no rotation</h4>
     *
     * @param x     X translation from the robot origin (meters)
     * @param y     Y translation from the robot origin (meters)
     * @param z     Z translation from the robot origin (meters)
     * @param roll  Rotation about the X axis
     * @param pitch Rotation about the Y axis
     * @param yaw   Rotation about the Z axis
     */
    public Offsets(double x, double y, double z, Angle roll, Angle pitch, Angle yaw) {
        this(Meters.of(x), Meters.of(y), Meters.of(z), roll, pitch, yaw);
    }

    /**
     *
     *
     * <h4>Translational component as a {@link Translation3d}</h4>
     *
     * @return translation from the robot origin
     */
    public Translation3d toTranslation3d() {
        return new Translation3d(x.in(Meters), y.in(Meters), z.in(Meters));
    }

    /**
     *
     *
     * <h4>Rotational component as a {@link Rotation3d}</h4>
     *
     * @return Rotation3d of the roll, pitch, and yaw components of the offset
     */
    public Rotation3d toRotation3d() {
        return new Rotation3d(roll, pitch, yaw);
    }

    /**
     *
     *
     * <h4>This offset as a {@link Pose3d}.</h4>
     *
     * <p>
     * Useful for components whose pose is fully static and requires no adjustments
     *
     * @return pose at this offset's position and orientation
     */
    public Pose3d toPose3d() {
        return new Pose3d(toTranslation3d(), toRotation3d());
    }

    /**
     *
     *
     * <h4>Applies this offset's translation and rotation onto an existing
     * {@link Pose3d}</h4>
     *
     * @param pose the base pose to offset
     * @return a new pose with this offset applied to both translation and rotation
     */
    public Pose3d applyToPose3d(Pose3d pose) {
        return new Pose3d(
                pose.getMeasureX().plus(x),
                pose.getMeasureY().plus(y),
                pose.getMeasureZ().plus(z),
                applyToRotation3d(pose.getRotation()));
    }

    /**
     *
     *
     * <h4>Applies this offset's translation and rotation onto an existing
     * {@link Pose3d}</h4>
     *
     * <p>
     * Translation is applied, but it is <b>robot robot relative</b>
     *
     * @param pose the base pose to offset
     * @return a new pose with this offset applied to both translation and rotation
     */
    public Pose3d applyToPose3dRobotRelative(Pose3d pose) {
        Translation3d rotatedOffset = toTranslation3d().rotateBy(pose.getRotation());
        return new Pose3d(
                pose.getTranslation().plus(rotatedOffset), applyToRotation3d(pose.getRotation()));
    }

    /**
     *
     *
     * <h4>Composes this offset's rotation onto an existing {@link Rotation3d}</h4>
     *
     * @param rotation the base rotation to offset
     * @return a new rotation with this offset's rotation applied on top
     */
    public Rotation3d applyToRotation3d(Rotation3d rotation) {
        return new Rotation3d(
                rotation.getX() + roll.in(Radians),
                rotation.getY() + pitch.in(Radians),
                rotation.getZ() + yaw.in(Radians));
    }

    /**
     *
     *
     * <h4>Pose at this offset's translation with an additional rotation</h4>
     *
     * <p>
     * Applies a rotation to this offsets rotation
     *
     * <pre>{@code
     * SHOOTER_OFFSETS.withRotation(
     *         new Rotation3d(0, 0, turretSim.getAngle().getRadians()));
     * }</pre>
     *
     * @param rotation the additional rotation to add to this offset's rotation
     * @return a new pose at this offset's translation with the combined rotation
     */
    public Pose3d withRotation(Rotation3d rotation) {
        return new Pose3d(toTranslation3d(), toRotation3d().plus(rotation));
    }

    /**
     *
     *
     * <h4>Applies this offset's X/Y translation and yaw onto an existing
     * {@link Pose2d}</h4>
     *
     * <p>
     * Roll and pitch are ignored because they have no 2D equivalent
     *
     * @param pose the base 2D pose to offset
     * @return a new pose of this offset's X, Y, and yaw
     */
    public Pose2d applyToPose2d(Pose2d pose) {
        return new Pose2d(
                pose.getMeasureX().plus(x),
                pose.getMeasureY().plus(y),
                applyToRotation2d(pose.getRotation()));
    }

    /**
     *
     *
     * <h4>Applies this offset's X/Y translation and yaw onto an existing
     * {@link Pose2d}</h4>
     *
     * <p>
     * Translation is applied, but it is <b>robot robot relative</b>
     *
     * @param pose the base 2D pose to offset
     * @return a new pose with this offset applied
     */
    public Pose2d applyToPose2dRobotRelative(Pose2d pose) {
        Translation2d rotatedOffset = new Translation2d(x, y).rotateBy(pose.getRotation());
        return new Pose2d(
                pose.getTranslation().plus(rotatedOffset), applyToRotation2d(pose.getRotation()));
    }

    /**
     *
     *
     * <h4>Applies this offset's yaw onto an existing {@link Rotation2d}</h4>
     *
     * <p>
     * Roll and pitch are ignored because they have no 2D equivalent
     *
     * @param rotation the base 2D rotation to offset
     * @return a new rotation with this offset's yaw applied on top
     */
    public Rotation2d applyToRotation2d(Rotation2d rotation) {
        return new Rotation2d(rotation.getRadians() + yaw.in(Radians));
    }
}

final class OffsetsStruct implements Struct<Offsets> {
    @Override
    public Class<Offsets> getTypeClass() {
        return Offsets.class;
    }

    @Override
    public String getTypeName() {
        return "Offsets";
    }

    @Override
    public int getSize() {
        return kSizeDouble * 6;
    }

    @Override
    public String getSchema() {
        return "double x;double y;double z;double roll;double pitch;double yaw";
    }

    @Override
    public Offsets unpack(ByteBuffer bb) {
        return new Offsets(Meters.of(bb.getDouble()), Meters.of(bb.getDouble()), Meters.of(bb.getDouble()), 
            Radians.of(bb.getDouble()), Radians.of(bb.getDouble()), Radians.of(bb.getDouble()));
    }

    @Override
    public void pack(ByteBuffer bb, Offsets value) {
        bb.putDouble(value.x().in(Meters));
        bb.putDouble(value.y().in(Meters));
        bb.putDouble(value.z().in(Meters));
        bb.putDouble(value.roll().in(Radians));
        bb.putDouble(value.pitch().in(Radians));
        bb.putDouble(value.yaw().in(Radians));
    }
}