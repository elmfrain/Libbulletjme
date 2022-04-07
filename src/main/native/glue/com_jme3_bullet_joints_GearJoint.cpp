#include "com_jme3_bullet_joints_GearJoint.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    createJoint
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_GearJoint_createJoint
(JNIEnv *pEnv, jclass, jlong bodyIdA, jlong bodyIdB, jobject axisInA, 
    jobject axisInB, jfloat ratio) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHK(pEnv, pBodyA, "Rigid body A does not exist.", 0)
    btAssert(pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHK(pEnv, axisInA, "The axisInA vector does not exist.", 0)
    btVector3 axisA;
    jmeBulletUtil::convert(pEnv, axisInA, &axisA);

    NULL_CHK(pEnv, axisInB, "The axisInB vector does not exist.", 0)
    btVector3 axisB;
    jmeBulletUtil::convert(pEnv, axisInB, &axisB);

    btGearConstraint *
        pJoint = new btGearConstraint(*pBodyA, *pBodyB, axisA, axisB, 
        ratio); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    getAxisA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_getAxisA
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeResult) {
    btGearConstraint *pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    NULL_CHK(pEnv, storeResult, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &pJoint->getAxisA(), storeResult);
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    getAxisB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_getAxisB
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeResult) {
    btGearConstraint *pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    NULL_CHK(pEnv, storeResult, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &pJoint->getAxisB(), storeResult);
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    getRatio
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_GearJoint_getRatio
(JNIEnv *pEnv, jclass, jlong jointId) {
    btGearConstraint *pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.", 0.f)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    return pJoint->getRatio();
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    setAxisA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_setAxisA
(JNIEnv *pEnv, jclass, jlong jointId, jobject axisA) {
    btGearConstraint *pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    NULL_CHK(pEnv, axisA, "The axisA vector does not exist.",)
    btVector3 axisInA;
    jmeBulletUtil::convert(pEnv, axisA, &axisInA);

    pJoint->setAxisA(axisInA);
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    setAxisB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_setAxisB
(JNIEnv *pEnv, jclass, jlong jointId, jobject axisB) {
    btGearConstraint *pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    NULL_CHK(pEnv, axisB, "The axisB vector does not exist.",)
    btVector3 axisInB;
    jmeBulletUtil::convert(pEnv, axisB, &axisInB);

    pJoint->setAxisB(axisInB);
}
/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    setRatio
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_setRatio
(JNIEnv *pEnv, jclass, jlong jointId, jfloat ratio) {
    btGearConstraint *pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    pJoint->setRatio(ratio);
}