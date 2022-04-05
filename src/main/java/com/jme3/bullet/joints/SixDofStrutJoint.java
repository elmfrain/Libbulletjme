package com.jme3.bullet.joints;

import java.util.logging.Logger;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;

import jme3utilities.Validate;

public class SixDofStrutJoint extends SixDofJoint
{

	/**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(SixDofStrutJoint.class.getName());
    // *************************************************************************
    // constructors
    
	public SixDofStrutJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB, Vector3f pivotInA,
			Vector3f pivotInB, Matrix3f rotInA, Matrix3f rotInB, boolean useLinearReferenceFrameA)
	{
		super(rigidBodyA, rigidBodyB, pivotInA, pivotInB, rotInA, rotInB, useLinearReferenceFrameA);
		// TODO Auto-generated constructor stub
	}
	
	public SixDofStrutJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB, Vector3f pivotInA,
			Vector3f pivotInB, boolean useLinearReferenceFrameA)
	{
		super(rigidBodyA, rigidBodyB, pivotInA, pivotInB, useLinearReferenceFrameA);
		// TODO Auto-generated constructor stub
	}
	// *************************************************************************
    // new methods exposed
	
	public void enableSpring(int dofIndex, boolean onOff)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		enableSpring(constraintId, dofIndex, onOff);
	}
	
	public float getCompressionDamping(int dofIndex)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		float result = getCompressionDamping(constraintId, dofIndex);
		
		return result;
	}
	
	public float getEquilibriumPoint(int dofIndex)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		float result = getEquilibriumPoint(constraintId, dofIndex);
		
		return result;
	}
	
	public float getReboundDamping(int dofIndex)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		float result = getReboundDamping(constraintId, dofIndex);
		
		return result;
	}
	
	public float getSpringRate(int dofIndex)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		float result = getSpringRate(constraintId, dofIndex);
		
		return result;
	}
	
	public boolean isSpringEnabled(int dofIndex)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		boolean result = isSpringEnabled(constraintId, dofIndex);
		
		return result;
	}
	
	public void setCompressionDamping(int dofIndex, float compressionDamping)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		setCompressionDamping(constraintId, dofIndex, compressionDamping);
	}
	
	public void setEquilibriumPoint()
	{
		long constraintId = nativeId();
		setEquilibriumPoint(constraintId);
	}
	
	public void setEquilibriumPoint(int dofIndex)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		setEquilibriumPoint(constraintId, dofIndex);
	}
	
	public void setReboundDamping(int dofIndex, float reboundDamping)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		setReboundDamping(constraintId, dofIndex, reboundDamping);
	}
	
	public void setSpringRate(int dofIndex, float springRate)
	{
		Validate.inRange(dofIndex, "DOF index", 0, 5);
		
		long constraintId = nativeId();
		setSpringRate(constraintId, dofIndex, springRate);
	}
	
	// *************************************************************************
    // SixDofJoint methods
	
	 /**
     * Create a new, double-ended bt6DofStrutConstraint.
     *
     * @param bodyIdA the ID of the body for the A end (not 0)
     * @param bodyIdB the ID of the body for the B end (not 0)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param rotInA the orientation of the joint in A's local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     * @return the ID of the new joint
     */
    @Override
    native protected long createJoint(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameA);

    /**
     * Create a new, single-ended bt6DofStrutConstraint.
     *
     * @param bodyIdB the ID of the body for the B end (not 0)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameB true&rarr;use body A, false&rarr;use body
     * B
     * @return the ID of the new joint
     */
    @Override
    native protected long createJoint1(long bodyIdB, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameB);
	// *************************************************************************
    // native private methods
	
	native private static void enableSpring(long jointId, int dofIndex, 
			boolean onOff);
	
	native private static float getCompressionDamping(long jointId, int dofIndex);
	
	native private static float getEquilibriumPoint(long jointId, int dofIndex);
	
	native private static float getReboundDamping(long jointId, int dofIndex);
	
	native private static float getSpringRate(long jointId, int dofIndex);
	
	native private static boolean isSpringEnabled(long jointId, int dofIndex);
	
	native private static void setCompressionDamping(long jointId, int dofIndex, 
			float compressionDamping);
	
	native private static void setEquilibriumPoint(long jointId);
	
	native private static void setEquilibriumPoint(long jointId, int dofIndex);
	
	native private static void setReboundDamping(long jointId, int dofIndex, 
			float reboundDamping);
	
	native private static void setSpringRate(long jointId, int dofIndex, 
			float springRate);
}
