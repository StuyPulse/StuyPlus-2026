package com.stuypulse.robot.util.simulation;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

import static edu.wpi.first.units.Units.*;

public class OffsetsStruct implements Struct<Offsets> {
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
