#include "bt6DofStrutConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"

bt6DofStrutConstraint::bt6DofStrutConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB, bool useLinearReferenceFrameA)
	: btGeneric6DofConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA)
{
    init();
}

bt6DofStrutConstraint::bt6DofStrutConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameB)
	: btGeneric6DofConstraint(rbB, frameInB, useLinearReferenceFrameB)
{
    init();
}

void bt6DofStrutConstraint::init()
{
    m_objectType = D6_STRUT_CONSTRAINT_TYPE;

    for(int i = 0; i < 6; i++)
    {
        m_springEnabled[i] = false;
        m_springRate[i] = btScalar(0.f);
        m_equilibriumPoint[i] = btScalar(0.f);
        m_compressionDamping[i] = btScalar(0.f);
        m_reboundDamping[i] = btScalar(0.f);
    }
}

void bt6DofStrutConstraint::enableSpring(int index, bool onOff)
{
    btAssert((index >= 0) && (index < 6));
	m_springEnabled[index] = onOff;
	if (index < 3)
	{
		m_linearLimits.m_enableMotor[index] = onOff;
	}
	else
	{
		m_angularLimits[index - 3].m_enableMotor = onOff;
	}
}

void bt6DofStrutConstraint::setSpringRate(int index, btScalar springRate)
{
	btAssert((index >= 0) && (index < 6));
	m_springRate[index] = springRate;
}

void bt6DofStrutConstraint::setCompressionDamping(int index, btScalar compressionDamping)
{
	btAssert((index >= 0) && (index < 6));
	m_compressionDamping[index] = compressionDamping;
}

void bt6DofStrutConstraint::setReboundDamping(int index, btScalar reboundDamping)
{
    btAssert((index >= 0) && (index < 6));
    m_reboundDamping[index] = reboundDamping;
}

void bt6DofStrutConstraint::setEquilibriumPoint()
{
	calculateTransforms();
	int i;

	for (i = 0; i < 3; i++)
	{
		m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
	}
	for (i = 0; i < 3; i++)
	{
		m_equilibriumPoint[i + 3] = m_calculatedAxisAngleDiff[i];
	}
}

void bt6DofStrutConstraint::setEquilibriumPoint(int index)
{
	btAssert((index >= 0) && (index < 6));
	calculateTransforms();
	if (index < 3)
	{
		m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
	}
	else
	{
		m_equilibriumPoint[index] = m_calculatedAxisAngleDiff[index - 3];
	}
}

void bt6DofStrutConstraint::setEquilibriumPoint(int index, btScalar val)
{
	btAssert((index >= 0) && (index < 6));
	m_equilibriumPoint[index] = val;
}

void bt6DofStrutConstraint::internalUpdateSprings(btConstraintInfo2* info)
{
	// it is assumed that calculateTransforms() have been called before this call
	int i;
	//btVector3 relVel = m_rbB.getLinearVelocity() - m_rbA.getLinearVelocity();
	for (i = 0; i < 3; i++)
	{
		if (m_springEnabled[i])
		{
            btScalar velFactor = info->fps / btScalar(info->m_numIterations);
			// get current and previous position of constraint
			btScalar currPos = m_calculatedLinearDiff[i];
            btScalar prevPos = m_prevCalculatedLinearDiff[i];
			// calculate difference
			btScalar delta = currPos - m_equilibriumPoint[i];
			// spring force is (delta * m_springRate) according to Hooke's Law
			btScalar springForce = delta * m_springRate[i];
            // calculate realtime velocity
            btScalar velocity = (currPos - prevPos) * velFactor;
            // damping force is (velocity * m_dampingForce)
            btScalar damping = 0 < velocity ? m_compressionDamping[i] : m_reboundDamping[i];
            btScalar dampingForce = velocity * damping;
            btScalar netForce = springForce + dampingForce;
			m_linearLimits.m_targetVelocity[i] = velFactor * netForce;
			m_linearLimits.m_maxMotorForce[i] = btFabs(netForce);
		}
	}
	for (i = 0; i < 3; i++)
	{
		if (m_springEnabled[i + 3])
		{
            btScalar velFactor = info->fps / btScalar(info->m_numIterations);
			// get current position of constraint
			btScalar currPos = m_calculatedAxisAngleDiff[i];
			// calculate difference
			btScalar delta = currPos - m_equilibriumPoint[i + 3];
			// spring force is (-delta * m_springRate) according to Hooke's Law
			btScalar force = -delta * m_springRate[i + 3];
			m_angularLimits[i].m_targetVelocity = velFactor * force;
			m_angularLimits[i].m_maxMotorForce = btFabs(force);
		}
	}
}

void bt6DofStrutConstraint::getInfo2(btConstraintInfo2* info)
{
	// this will be called by constraint solver at the constraint setup stage
	// set current motor parameters
	internalUpdateSprings(info);
	// do the rest of job for constraint setup
	btGeneric6DofConstraint::getInfo2(info);
}

void bt6DofStrutConstraint::setAxis(const btVector3& axis1, const btVector3& axis2)
{
	btVector3 zAxis = axis1.normalized();
	btVector3 yAxis = axis2.normalized();
	btVector3 xAxis = yAxis.cross(zAxis);  // we want right coordinate system

	btTransform frameInW;
	frameInW.setIdentity();
	frameInW.getBasis().setValue(xAxis[0], yAxis[0], zAxis[0],
								 xAxis[1], yAxis[1], zAxis[1],
								 xAxis[2], yAxis[2], zAxis[2]);

	// now get constraint frame in local coordinate systems
	m_frameInA = m_rbA.getCenterOfMassTransform().inverse() * frameInW;
	m_frameInB = m_rbB.getCenterOfMassTransform().inverse() * frameInW;

	calculateTransforms();
}