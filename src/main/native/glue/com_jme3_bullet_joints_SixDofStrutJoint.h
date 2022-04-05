/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_jme3_bullet_joints_SixDofStrutJoint */

#ifndef _Included_com_jme3_bullet_joints_SixDofStrutJoint
#define _Included_com_jme3_bullet_joints_SixDofStrutJoint
#ifdef __cplusplus
extern "C" {
#endif
#undef com_jme3_bullet_joints_SixDofStrutJoint_numAxes
#define com_jme3_bullet_joints_SixDofStrutJoint_numAxes 3L
/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    createJoint
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_createJoint
  (JNIEnv *, jobject, jlong, jlong, jobject, jobject, jobject, jobject, jboolean);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    createJoint1
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_createJoint1
  (JNIEnv *, jobject, jlong, jobject, jobject, jboolean);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    enableSpring
 * Signature: (JIZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_enableSpring
  (JNIEnv *, jclass, jlong, jint, jboolean);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    getCompressionDamping
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_getCompressionDamping
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    getEquilibriumPoint
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_getEquilibriumPoint
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    getReboundDamping
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_getReboundDamping
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    getSpringRate
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_getSpringRate
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    isSpringEnabled
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_isSpringEnabled
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setCompressionDamping
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setCompressionDamping
  (JNIEnv *, jclass, jlong, jint, jfloat);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setEquilibriumPoint
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setEquilibriumPoint__J
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setEquilibriumPoint
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setEquilibriumPoint__JI
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setReboundDamping
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setReboundDamping
  (JNIEnv *, jclass, jlong, jint, jfloat);

/*
 * Class:     com_jme3_bullet_joints_SixDofStrutJoint
 * Method:    setSpringRate
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofStrutJoint_setSpringRate
  (JNIEnv *, jclass, jlong, jint, jfloat);

#ifdef __cplusplus
}
#endif
#endif
