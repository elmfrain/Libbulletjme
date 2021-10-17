/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_jme3_bullet_CollisionSpace */

#ifndef _Included_com_jme3_bullet_CollisionSpace
#define _Included_com_jme3_bullet_CollisionSpace
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    addCollisionObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_addCollisionObject
  (JNIEnv *, jclass, jlong, jlong);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    contactTest
 * Signature: (JJLcom/jme3/bullet/collision/PhysicsCollisionListener;)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_CollisionSpace_contactTest
  (JNIEnv *, jclass, jlong, jlong, jobject);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    createCollisionSpace
 * Signature: (FFFFFFI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_CollisionSpace_createCollisionSpace
  (JNIEnv *, jobject, jfloat, jfloat, jfloat, jfloat, jfloat, jfloat, jint);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_finalizeNative
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    getDeterministicOverlappingPairs
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_CollisionSpace_getDeterministicOverlappingPairs
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    getNumCollisionObjects
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_CollisionSpace_getNumCollisionObjects
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    hasClosest
 * Signature: (JII)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_CollisionSpace_hasClosest
  (JNIEnv *, jclass, jlong, jint, jint);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    hasContact
 * Signature: (JII)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_CollisionSpace_hasContact
  (JNIEnv *, jclass, jlong, jint, jint);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    rayTest_native
 * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;JLjava/util/List;I)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_rayTest_1native
  (JNIEnv *, jclass, jobject, jobject, jlong, jobject, jint);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    removeCollisionObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_removeCollisionObject
  (JNIEnv *, jclass, jlong, jlong);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    setDeterministicOverlappingPairs
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_setDeterministicOverlappingPairs
  (JNIEnv *, jclass, jlong, jboolean);

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    sweepTest_native
 * Signature: (JLcom/jme3/math/Transform;Lcom/jme3/math/Transform;JLjava/util/List;F)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_sweepTest_1native
  (JNIEnv *, jclass, jlong, jobject, jobject, jlong, jobject, jfloat);

#ifdef __cplusplus
}
#endif
#endif
