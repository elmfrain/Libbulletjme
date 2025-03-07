/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_jme3_bullet_collision_shapes_CollisionShape */

#ifndef _Included_com_jme3_bullet_collision_shapes_CollisionShape
#define _Included_com_jme3_bullet_collision_shapes_CollisionShape
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    getShapeType
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_getShapeType
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_finalizeNative
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    getAabb
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_getAabb
  (JNIEnv *, jclass, jlong, jobject, jobject, jobject, jobject);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    getLocalScaling
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_getLocalScaling
  (JNIEnv *, jclass, jlong, jobject);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    getMargin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_getMargin
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isConcave
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isConcave
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isContactFilterEnabled
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isContactFilterEnabled
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isConvex
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isConvex
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isInfinite
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isInfinite
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isNonMoving
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isNonMoving
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isPolyhedral
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isPolyhedral
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    setContactFilterEnabled
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_setContactFilterEnabled
  (JNIEnv *, jclass, jlong, jboolean);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    setLocalScaling
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_setLocalScaling
  (JNIEnv *, jclass, jlong, jobject);

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    setMargin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_setMargin
  (JNIEnv *, jclass, jlong, jfloat);

#ifdef __cplusplus
}
#endif
#endif
