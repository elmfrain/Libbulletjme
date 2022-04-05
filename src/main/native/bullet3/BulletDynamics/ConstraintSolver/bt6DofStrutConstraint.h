#ifndef BT_6DOF_STRUT_CONSTRAINT_H
#define BT_6DOF_STRUT_CONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btTypedConstraint.h"
#include "btGeneric6DofConstraint.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define bt6DofStrutConstraintData2 bt6DofStrutConstraintDoubleData2
#define bt6DofStrutConstraintDataName "bt6DofStrutConstraintDoubleData2"
#else
#define bt6DofStrutConstraintData2 bt6DofStrutConstraintData
#define bt6DofStrutConstraintDataName "bt6DofStrutConstraintData"
#endif  //BT_USE_DOUBLE_PRECISION

/// Generic 6 DOF constraint that allows to set spring motors to any translational and rotational DOF

/// DOF index used in enableSpring() and setSpringRate() means:
/// 0 : translation X
/// 1 : translation Y
/// 2 : translation Z
/// 3 : rotation X (3rd Euler rotational around new position of X axis, range [-PI+epsilon, PI-epsilon] )
/// 4 : rotation Y (2nd Euler rotational around new position of Y axis, range [-PI/2+epsilon, PI/2-epsilon] )
/// 5 : rotation Z (1st Euler rotational around Z axis, range [-PI+epsilon, PI-epsilon] )

ATTRIBUTE_ALIGNED16(class)
bt6DofStrutConstraint : public btGeneric6DofConstraint
{
protected:
	bool m_springEnabled[6];
	btScalar m_equilibriumPoint[6];
    btScalar m_springRate[6];
	btScalar m_compressionDamping[6];  // in N / (m/s)
    btScalar m_reboundDamping[6]; // in N / (m/s)
	void init();
	void internalUpdateSprings(btConstraintInfo2 * info);

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	bt6DofStrutConstraint(btRigidBody & rbA, btRigidBody & rbB, const btTransform& frameInA, const btTransform& frameInB, bool useLinearReferenceFrameA);
	bt6DofStrutConstraint(btRigidBody & rbB, const btTransform& frameInB, bool useLinearReferenceFrameB);
	void enableSpring(int index, bool onOff);
	void setCompressionDamping(int index, btScalar compressionDamping);
	void setEquilibriumPoint();           // set the current constraint position/orientation as an equilibrium point for all DOF
	void setEquilibriumPoint(int index);  // set the current constraint position/orientation as an equilibrium point for given DOF
	void setEquilibriumPoint(int index, btScalar val);
    void setReboundDamping(int index, btScalar reboundDamping);
    void setSpringRate(int index, btScalar springRate);

	bool isSpringEnabled(int index) const
	{
		return m_springEnabled[index];
	}

    btScalar getSpringRate(int index) const
    {
        return m_springRate[index];
    }

	btScalar getCompressionDamping(int index) const
	{
		return m_compressionDamping[index];
	}

    btScalar getReboundDamping(int index) const
    {
        return m_reboundDamping[index];
    }

	btScalar getEquilibriumPoint(int index) const
	{
		return m_equilibriumPoint[index];
	}

	virtual void setAxis(const btVector3& axis1, const btVector3& axis2);

	virtual void getInfo2(btConstraintInfo2 * info);

	virtual int calculateSerializeBufferSize() const;
	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

struct bt6DofStrutConstraintData
{
    btGeneric6DofConstraintData m_6dofData;

    int m_springEnabled[6];
    float m_equilibriumPoint[6];
    float m_springRate[6];
    float m_compressionDamping[6];
    float m_reboundDamping[6];
};

struct bt6DofStrutConstraintDoubleData2
{
    btGeneric6DofConstraintDoubleData2 m_6dofData;

    int m_springEnabled[6];
    double m_equilibriumPoint[6];
    double m_springRate[6];
    double m_compressionDamping[6];
    double m_reboundDamping[6];
};

SIMD_FORCE_INLINE int bt6DofStrutConstraint::calculateSerializeBufferSize() const
{
	return sizeof(bt6DofStrutConstraintData2);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* bt6DofStrutConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
	bt6DofStrutConstraintData2* dof = (bt6DofStrutConstraintData2*) dataBuffer;
	btGeneric6DofConstraint::serialize(&dof->m_6dofData, serializer);

	int i;
	for (i = 0; i < 6; i++)
	{
        dof->m_springEnabled[i] = m_springEnabled[i] ? 1 : 0;
		dof->m_equilibriumPoint[i] = m_equilibriumPoint[i];
        dof->m_springRate[i] = m_springRate[i];
		dof->m_compressionDamping[i] = m_compressionDamping[i];
		dof->m_reboundDamping[i] = m_reboundDamping[i];
	}
	return bt6DofStrutConstraintDataName;
}

#endif  // BT_6DOF_STRUT_CONSTRAINT_H