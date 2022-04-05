#include "com_jme3_bullet_joints_SixDofStrutJoint.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    createJoint
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_createJoint
(JNIEnv *pEnv, jobject, jlong bodyIdA, jlong bodyIdB, jobject pivotInA, 
        jobject rotInA, jobject pivotInB, jobject rotInB, 
        jboolean useLinearReferenceFrameA) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHK(pEnv, pBodyA, "Rigid body A does not exist.", 0)
    btAssert(pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHK(pEnv, pivotInA, "The pivotInA vector does not exist.", 0)
    NULL_CHK(pEnv, rotInA, "The rotInA matrix does not exist.", 0)
    btTransform frameInA;
    jmeBulletUtil::convert(pEnv, pivotInA, &frameInA.getOrigin());
    jmeBulletUtil::convert(pEnv, rotInA, &frameInA.getBasis());

    NULL_CHK(pEnv, pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHK(pEnv, rotInB, "The rotInB matrix does not exist.", 0)
    btTransform frameInB;
    jmeBulletUtil::convert(pEnv, pivotInB, &frameInB.getOrigin());
    jmeBulletUtil::convert(pEnv, rotInB, &frameInB.getBasis());

    bt6DofStrutConstraint *
            pJoint = new bt6DofStrutConstraint(*pBodyA, *pBodyB,
            frameInA, frameInB, useLinearReferenceFrameA); //dance021???

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    createJoint1
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_createJoint1
(JNIEnv *pEnv, jobject, jlong bodyIdB, jobject pivotInB, jobject rotInB,
        jboolean useLinearReferenceFrameB) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHK(pEnv, pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHK(pEnv, rotInB, "The rotInB matrix does not exist.", 0)
    btTransform frameInB;
    jmeBulletUtil::convert(pEnv, pivotInB, &frameInB.getOrigin());
    jmeBulletUtil::convert(pEnv, rotInB, &frameInB.getBasis());

    bt6DofStrutConstraint *
        pJoint = new bt6DofStrutConstraint(*pBodyB, frameInB,
        useLinearReferenceFrameB); //dance021????

    return reinterpret_cast<jlong> (pJoint);
}
/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    enableSpring
 * Signature: (JIZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_enableSpring
(JNIEnv *pEnv, jclass, jlong jointId, jint index, jboolean onOff) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);
    btAssert(index >= 0);
    btAssert(index < 6);

    pJoint->enableSpring(index, onOff);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    getCompressionDamping
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_getCompressionDamping
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.", 0)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);
    btAssert(index >= 0);
    btAssert(index < 6);

    btScalar result = pJoint->getCompressionDamping(index);
    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    getEquilibriumPoint
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_getEquilibriumPoint
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.", 0)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);
    btAssert(index >= 0);
    btAssert(index < 6);

    btScalar result = pJoint->getEquilibriumPoint(index);
    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    getReboundDamping
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_getReboundDamping
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.", 0)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);
    btAssert(index >= 0);
    btAssert(index < 6);

    btScalar result = pJoint->getReboundDamping(index);
    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    getSpringRate
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_getSpringRate
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.", 0)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);
    btAssert(index >= 0);
    btAssert(index < 6);

    btScalar result = pJoint->getSpringRate(index);
    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    isSpringEnabled
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_isSpringEnabled
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.", 0)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);
    btAssert(index >= 0);
    btAssert(index < 6);

    bool result = pJoint->isSpringEnabled(index);
    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setCompressionDamping
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setCompressionDamping
(JNIEnv *pEnv, jclass, jlong jointId, jint index, jfloat compressionDamping) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);
    btAssert(index >= 0);
    btAssert(index < 6);

   pJoint->setCompressionDamping(index, compressionDamping);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setEquilibriumPoint
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setEquilibriumPoint__J
(JNIEnv *pEnv, jclass, jlong jointId) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);

    pJoint->setEquilibriumPoint();
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setEquilibriumPoint
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setEquilibriumPoint__JI
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);

    pJoint->setEquilibriumPoint(index);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setReboundDamping
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setReboundDamping
(JNIEnv *pEnv, jclass, jlong jointId, jint index, jfloat reboundDamping) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);
    btAssert(index >= 0);
    btAssert(index < 6);

   pJoint->setReboundDamping(index, reboundDamping);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setSpringRate
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setSpringRate
(JNIEnv *pEnv, jclass, jlong jointId, jint index, jfloat springRate) {
    bt6DofStrutConstraint *pJoint
            = reinterpret_cast<bt6DofStrutConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The bt6DofStrutConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == D6_STRUT_CONSTRAINT_TYPE);
    btAssert(index >= 0);
    btAssert(index < 6);

   pJoint->setSpringRate(index, springRate);
}