/*
 * Copyright (c) 2009-2018 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.bullet.collision.shapes;

import com.jme3.math.Vector3f;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * An axis-aligned, rectangular-solid CollisionShape based on Bullet's
 * btBoxShape. For a rectangle, use Box2dShape.
 *
 * @author normenhansen
 */
public class BoxCollisionShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(BoxCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the unscaled half extents (not null, no negative component)
     */
    final private Vector3f halfExtents = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a cube-shaped box with the specified half extent.
     *
     * @param halfExtent the desired unscaled half extent on each local axis
     * (not negative)
     */
    public BoxCollisionShape(float halfExtent) {
        Validate.nonNegative(halfExtent, "half extent");

        halfExtents.set(halfExtent, halfExtent, halfExtent);
        createShape();
    }

    /**
     * Instantiate a box shape with the specified half extents.
     *
     * @param xHalfExtent the desired unscaled half extent on the local X axis
     * (not negative)
     * @param yHalfExtent the desired unscaled half extent on the local Y axis
     * (not negative)
     * @param zHalfExtent the desired unscaled half extent on the local Z axis
     * (not negative)
     */
    public BoxCollisionShape(float xHalfExtent, float yHalfExtent,
            float zHalfExtent) {
        Validate.nonNegative(xHalfExtent, "half extent on X");
        Validate.nonNegative(yHalfExtent, "half extent on Y");
        Validate.nonNegative(zHalfExtent, "half extent on Z");

        halfExtents.set(xHalfExtent, yHalfExtent, zHalfExtent);
        createShape();
    }

    /**
     * Instantiate a box shape that encloses the sample locations in the
     * specified FloatBuffer range.
     *
     * @param buffer the buffer that contains the sample locations (not null,
     * unaffected)
     * @param startPosition the position at which the sample locations start
     * (&ge;0, &le;endPosition)
     * @param endPosition the position at which the sample locations end
     * (&ge;startPosition, &le;capacity)
     */
    public BoxCollisionShape(FloatBuffer buffer, int startPosition,
            int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());

        MyBuffer.maxAbs(buffer, startPosition, endPosition, halfExtents);
        createShape();
    }

    /**
     * Instantiate a box shape with the specified half extents.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected)
     */
    public BoxCollisionShape(Vector3f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");

        this.halfExtents.set(halfExtents);
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the half extents of the box.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, no negative component)
     */
    public Vector3f getHalfExtents(Vector3f storeResult) {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;

        if (storeResult == null) {
            return halfExtents.clone();
        } else {
            return storeResult.set(halfExtents);
        }
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Calculate how far the box extends from its center.
     *
     * @return a distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        double xx = scale.x * halfExtents.x;
        double yy = scale.y * halfExtents.y;
        double zz = scale.z * halfExtents.z;
        float result = (float) MyMath.hypotenuseDouble(xx, yy, zz);

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured btBoxShape.
     */
    private void createShape() {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;

        long shapeId = createShape(halfExtents);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(Vector3f halfExtents);
}
