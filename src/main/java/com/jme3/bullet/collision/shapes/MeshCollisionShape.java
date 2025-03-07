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

import com.jme3.bullet.collision.shapes.infos.BoundingValueHierarchy;
import com.jme3.bullet.collision.shapes.infos.CompoundMesh;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A mesh CollisionShape that uses a Bounding Value Hierarchy (BVH), based on
 * Bullet's btBvhTriangleMeshShape. Not for use in dynamic bodies. Collisions
 * between HeightfieldCollisionShape, MeshCollisionShape, and
 * PlaneCollisionShape objects are never detected.
 *
 * TODO add a shape based on btScaledBvhTriangleMeshShape
 *
 * @author normenhansen
 */
public class MeshCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MeshCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * if true, use quantized AABB compression (default=true)
     */
    final private boolean useCompression;
    /**
     * bounding-value hierarchy
     */
    private BoundingValueHierarchy bvh;
    /**
     * native mesh used to construct this shape
     */
    private CompoundMesh nativeMesh;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a shape from the specified collection of native meshes.
     *
     * @param useCompression true to use quantized AABB compression
     * @param meshes the collection on which to base the shape (must contain at
     * least one triangle)
     */
    public MeshCollisionShape(boolean useCompression,
            Collection<IndexedMesh> meshes) {
        Validate.nonEmpty(meshes, "meshes");
        nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : meshes) {
            nativeMesh.add(submesh);
        }
        Validate.require(nativeMesh.countTriangles() > 0,
                "at least one triangle");

        this.useCompression = useCompression;
        createShape();
    }

    /**
     * Instantiate a shape from the specified native mesh(es).
     *
     * @param useCompression true to use quantized AABB compression
     * @param submeshes the mesh(es) on which to base the shape (must contain at
     * least one triangle)
     */
    public MeshCollisionShape(boolean useCompression,
            IndexedMesh... submeshes) {
        Validate.nonEmpty(submeshes, "submeshes");
        nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : submeshes) {
            nativeMesh.add(submesh);
        }
        Validate.require(nativeMesh.countTriangles() > 0,
                "at least one triangle");

        this.useCompression = useCompression;
        createShape();
    }

    /**
     * Instantiate a shape from the specified native mesh(es) and serialized
     * BVH. The submeshes must be equivalent to those used to generate the BVH.
     *
     * @param bvhBytes the serialized BVH (not null, unaffected)
     * @param submeshes the mesh(es) on which to base the shape (must contain at
     * least one triangle)
     */
    public MeshCollisionShape(byte[] bvhBytes, IndexedMesh... submeshes) {
        Validate.nonNull(bvhBytes, "BVH data");
        Validate.nonEmpty(submeshes, "submeshes");
        nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : submeshes) {
            nativeMesh.add(submesh);
        }
        Validate.require(nativeMesh.countTriangles() > 0,
                "at least one triangle");

        useCompression = true;
        bvh = new BoundingValueHierarchy(bvhBytes);
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many triangles are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshTriangles() {
        int result = nativeMesh.countTriangles();
        return result;
    }

    /**
     * Count how many vertices are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshVertices() {
        int numVertices = nativeMesh.countVertices();
        return numVertices;
    }

    /**
     * Serialize the BVH to a byte array.
     *
     * @return a new array containing a serialized version of the BVH
     */
    public byte[] serializeBvh() {
        byte[] result = bvh.serialize();
        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Recalculate this shape's bounding box if necessary.
     */
    @Override
    protected void recalculateAabb() {
        long shapeId = nativeId();
        recalcAabb(shapeId);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured btBvhTriangleMeshShape.
     */
    private void createShape() {
        int numTriangles = nativeMesh.countTriangles();
        assert numTriangles > 0 : numTriangles;

        boolean buildBvh = (bvh == null);
        long meshId = nativeMesh.nativeId();
        long shapeId = createShape(useCompression, buildBvh, meshId);
        setNativeId(shapeId);

        if (buildBvh) {
            bvh = new BoundingValueHierarchy(this);
        } else {
            long bvhId = bvh.nativeId();
            setOptimizedBvh(shapeId, bvhId);
        }

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(boolean useCompression,
            boolean buildBvh, long meshId);

    native private static void recalcAabb(long shapeId);

    native private static void setOptimizedBvh(long shapeId, long bvhId);
}
