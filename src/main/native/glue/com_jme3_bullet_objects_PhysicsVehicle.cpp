/*
 * Copyright (c) 2009-2012 jMonkeyEngine
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

/*
 * Author: Normen Hansen
 */

#include "com_jme3_bullet_objects_PhysicsVehicle.h"
#include "jmeBulletUtil.h"
#include "jmePhysicsSpace.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    addWheel
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;FFJZ)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_addWheel
(JNIEnv *pEnv, jclass, jlong vehicleId, jobject locationVector,
        jobject directionVector, jobject axleVector, jfloat restLength,
        jfloat radius, jlong tuningId, jboolean frontWheel) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.", 0)

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.", 0)
    btVector3 location;
    jmeBulletUtil::convert(pEnv, locationVector, &location);

    NULL_CHK(pEnv, directionVector, "The direction vector does not exist.", 0)
    btVector3 direction;
    jmeBulletUtil::convert(pEnv, directionVector, &direction);

    NULL_CHK(pEnv, axleVector, "The axle vector does not exist.", 0)
    btVector3 axle;
    jmeBulletUtil::convert(pEnv, axleVector, &axle);

    btRaycastVehicle::btVehicleTuning *pTuning
            = reinterpret_cast<btRaycastVehicle::btVehicleTuning *> (
            vehicleId);
    NULL_CHK(pEnv, pTuning, "The btVehicleTuning does not exist.", 0);

    btWheelInfo *pWheel = &pVehicle->addWheel(location, direction, axle,
            restLength, radius, *pTuning, frontWheel);
    int idx = pVehicle->getNumWheels();
    return idx - 1;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    applyEngineForce
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_applyEngineForce
(JNIEnv *pEnv, jclass, jlong vehicleId, jint wheelIndex, jfloat force) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",)
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pVehicle->getNumWheels());

    pVehicle->applyEngineForce(force, wheelIndex);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    brake
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_brake
(JNIEnv *pEnv, jclass, jlong vehicleId, jint wheelIndex, jfloat value) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",)
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pVehicle->getNumWheels());

    pVehicle->setBrake(value, wheelIndex);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    createRaycastVehicle
 * Signature: (JJ)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_createRaycastVehicle
(JNIEnv *pEnv, jclass, jlong bodyId, jlong casterId) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.", 0);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    pBody->setActivationState(DISABLE_DEACTIVATION);

    btVehicleRaycaster *pCaster
            = reinterpret_cast<btDefaultVehicleRaycaster *> (casterId);
    NULL_CHK(pEnv, pCaster, "The btVehicleRaycaster does not exist.", 0)

    btRaycastVehicle::btVehicleTuning tuning;
    btRaycastVehicle *
            pVehicle = new btRaycastVehicle(tuning, pBody, pCaster); //dance032

    return reinterpret_cast<jlong> (pVehicle);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    createVehicleRaycaster
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_createVehicleRaycaster
(JNIEnv *pEnv, jclass, jlong spaceId) {
    jmeClasses::initJavaClasses(pEnv);

    jmePhysicsSpace *pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0)

    btDefaultVehicleRaycaster *
            pCaster = new btDefaultVehicleRaycaster(pSpace->getDynamicsWorld()); //dance033

    return reinterpret_cast<jlong> (pCaster);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    finalizeNative
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_finalizeNative
(JNIEnv *pEnv, jclass, jlong casterId, jlong vehicleId) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",);

    delete pVehicle; //dance032

    btVehicleRaycaster *pCaster
            = reinterpret_cast<btVehicleRaycaster *> (casterId);
    NULL_CHK(pEnv, pCaster, "The btVehicleRaycaster does not exist.",);

    delete pCaster; //dance033
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    getCurrentVehicleSpeedKmHour
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_getCurrentVehicleSpeedKmHour
(JNIEnv *pEnv, jclass, jlong vehicleId) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.", 0);

    return pVehicle->getCurrentSpeedKmHour();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    getForwardAxisIndex
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_getForwardAxisIndex
(JNIEnv *pEnv, jclass, jlong vehicleId) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.", 0);

    int axisIndex = pVehicle->getForwardAxis();
    return (jint) axisIndex;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    getForwardVector
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_getForwardVector
(JNIEnv *pEnv, jclass, jlong vehicleId, jobject storeVector) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",);

    const btVector3& forwardVector = pVehicle->getForwardVector();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    jmeBulletUtil::convert(pEnv, &forwardVector, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    getRightAxisIndex
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_getRightAxisIndex
(JNIEnv *pEnv, jclass, jlong vehicleId) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.", 0);

    int axisIndex = pVehicle->getRightAxis();
    return (jint) axisIndex;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    getNumWheels
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_getNumWheels
(JNIEnv *pEnv, jclass, jlong vehicleId) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.", 0);

    int count = pVehicle->getNumWheels();
    return (jint) count;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    getUpAxisIndex
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_getUpAxisIndex
(JNIEnv *pEnv, jclass, jlong vehicleId) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.", 0);

    int axisIndex = pVehicle->getUpAxis();
    return (jint) axisIndex;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    rayCast
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_rayCast
(JNIEnv *pEnv, jclass, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.", 0);
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pVehicle->getNumWheels());

    btWheelInfo& wheel = pVehicle->m_wheelInfo[wheelIndex];
    btScalar result = pVehicle->rayCast(wheel);

    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    resetSuspension
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_resetSuspension
(JNIEnv *pEnv, jclass, jlong vehicleId) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",);

    pVehicle->resetSuspension();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    setCoordinateSystem
 * Signature: (JIII)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_setCoordinateSystem
(JNIEnv *pEnv, jclass, jlong vehicleId, jint right, jint up, jint forward) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",);
    btAssert(right >= 0);
    btAssert(right <= 2);
    btAssert(up >= 0);
    btAssert(up <= 2);
    btAssert(forward >= 0);
    btAssert(forward <= 2);

    pVehicle->setCoordinateSystem(right, up, forward);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    steer
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_steer
(JNIEnv *pEnv, jclass, jlong vehicleId, jint wheelIndex, jfloat angle) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",);
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pVehicle->getNumWheels());

    pVehicle->setSteeringValue(angle, wheelIndex);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsVehicle
 * Method:    updateWheelTransform
 * Signature: (JIZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsVehicle_updateWheelTransform
(JNIEnv *pEnv, jclass, jlong vehicleId, jint wheelIndex,
        jboolean interpolated) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",);
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pVehicle->getNumWheels());

    pVehicle->updateWheelTransform(wheelIndex, interpolated);
}
