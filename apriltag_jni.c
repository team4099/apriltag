//
// Created by Saraansh Wadkar on 10/15/22.
//

#include <jni.h>

#include <android/bitmap.h>
#include <android/log.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag16h5.h"
#include "apriltag_pose.h"

#define  LOG_TAG    "apriltag_jni_output"

#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

static struct {
    apriltag_detector_t *td;
    apriltag_family_t *tf;
    void(*tf_destroy)(apriltag_family_t*);

    jclass al_cls;
    jmethodID al_constructor, al_add;
    jclass ad_cls;
    jmethodID ad_constructor;
    jfieldID ad_id_field, ad_hamming_field, ad_c_field, ad_p_field;
    jclass apriltag_pose_class;
    jmethodID apriltag_pose_constructor;
    jfieldID apriltag_pose_id_field, apriltag_pose_hamming_field, apriltag_pose_center_field, apriltag_pose_corners_field, apriltag_pose_translationMeters_1_field, apriltag_pose_rotation_1_field, apriltag_pose_translationMeters_2_field, apriltag_pose_rotation_2_field, apriltag_pose_poseConfidence_field;

} state;

JNIEXPORT void JNICALL Java_edu_umich_eecs_april_apriltag_ApriltagNative_native_1init
        (JNIEnv *env, jclass cls)
{
    // Just do method lookups once and cache the results

    // Get ArrayList methods
    jclass al_cls = (*env)->FindClass(env, "java/util/ArrayList");
    if (!al_cls) {
        LOGE(
                            "couldn't find ArrayList class");
        return;
    }
    state.al_cls = (*env)->NewGlobalRef(env, al_cls);

    state.al_constructor = (*env)->GetMethodID(env, al_cls, "<init>", "()V");
    state.al_add = (*env)->GetMethodID(env, al_cls, "add", "(Ljava/lang/Object;)Z");
    if (!state.al_constructor || !state.al_add) {
        LOGE(
                            "couldn't find ArrayList methods");
        return;
    }

    // Get ApriltagDetection methods
    jclass ad_cls = (*env)->FindClass(env, "edu/umich/eecs/april/apriltag/ApriltagDetection");
    if (!ad_cls) {
        LOGE(
                            "couldn't find ApriltagDetection class");
        return;
    }
    state.ad_cls = (*env)->NewGlobalRef(env, ad_cls);

    state.ad_constructor = (*env)->GetMethodID(env, ad_cls, "<init>", "()V");
    if (!state.ad_constructor) {
        LOGE(
                            "couldn't find ApriltagDetection constructor");
        return;
    }

    state.ad_id_field = (*env)->GetFieldID(env, ad_cls, "id", "I");
    state.ad_hamming_field = (*env)->GetFieldID(env, ad_cls, "hamming", "I");
    state.ad_c_field = (*env)->GetFieldID(env, ad_cls, "c", "[D");
    state.ad_p_field = (*env)->GetFieldID(env, ad_cls, "p", "[D");
    if (!state.ad_id_field ||
        !state.ad_hamming_field ||
        !state.ad_c_field ||
        !state.ad_p_field) {
        LOGE(
                            "couldn't find ApriltagDetection fields");
        return;
    }

    // Get ApriltagPose methods
    jclass apriltag_pose_class = (*env)->FindClass(env, "edu/umich/eecs/april/apriltag/ApriltagPose");
    if (!apriltag_pose_class) {
        LOGE(
                            "couldn't find ApriltagPose class");
        return;
    }
    state.apriltag_pose_class = (*env)->NewGlobalRef(env, apriltag_pose_class);

    state.apriltag_pose_constructor = (*env)->GetMethodID(env, apriltag_pose_class, "<init>", "()V");
    if (!state.apriltag_pose_constructor) {
        LOGE(
                            "couldn't find ApriltagDetection constructor");
        return;
    }

    state.apriltag_pose_id_field = (*env)->GetFieldID(env, apriltag_pose_class, "id", "I");
    state.apriltag_pose_hamming_field = (*env)->GetFieldID(env, apriltag_pose_class, "hamming", "I");
    state.apriltag_pose_center_field = (*env)->GetFieldID(env, apriltag_pose_class, "center", "[D");
    state.apriltag_pose_corners_field = (*env)->GetFieldID(env, apriltag_pose_class, "corners", "[D");
    state.apriltag_pose_translationMeters_1_field = (*env)->GetFieldID(env, apriltag_pose_class, "translationMeters_1", "[D");
    state.apriltag_pose_rotation_1_field = (*env)->GetFieldID(env, apriltag_pose_class, "rotation_1", "[D");
    state.apriltag_pose_translationMeters_2_field = (*env)->GetFieldID(env, apriltag_pose_class, "translationMeters_2", "[D");
    state.apriltag_pose_rotation_2_field = (*env)->GetFieldID(env, apriltag_pose_class, "rotation_2", "[D");
    state.apriltag_pose_poseConfidence_field = (*env)->GetFieldID(env, apriltag_pose_class, "poseConfidence", "D");

    if (!state.apriltag_pose_id_field ||
        !state.apriltag_pose_hamming_field ||
        !state.apriltag_pose_center_field ||
        !state.apriltag_pose_corners_field ||
        !state.apriltag_pose_translationMeters_1_field ||
        !state.apriltag_pose_rotation_1_field ||
        !state.apriltag_pose_translationMeters_2_field ||
        !state.apriltag_pose_rotation_2_field ||
        !state.apriltag_pose_poseConfidence_field
        ) {
            LOGE("couldn't find AprilTagPose fields");
        return;
    }


}

/*
 * Class:     edu_umich_eecs_april_apriltag_ApriltagNative
 * Method:    yuv_to_rgb
 * Signature: ([BIILandroid/graphics/Bitmap;)V
 */
JNIEXPORT void JNICALL Java_edu_umich_eecs_april_apriltag_ApriltagNative_yuv_1to_1rgb
        (JNIEnv *env, jclass cls, jbyteArray _src, jint width, jint height, jobject _dst)
{
    // NV21 Format
    // width*height    luma (Y) bytes followed by
    // width*height/2  chroma (UV) bytes interleaved as V,U

    jbyte *src = (*env)->GetByteArrayElements(env, _src, NULL);
    jint *dst = NULL;
    AndroidBitmap_lockPixels(env, _dst, &dst);

    if (!dst) {
        LOGE(
                            "couldn't lock bitmap");
        return;
    }

    AndroidBitmapInfo bmpinfo;
    if (AndroidBitmap_getInfo(env, _dst, &bmpinfo) ||
        bmpinfo.width*bmpinfo.height != width*height ||
        bmpinfo.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
        __android_log_print(ANDROID_LOG_ERROR, "apriltag_jni",
                            "incorrect bitmap format: %d x %d  %d",
                            bmpinfo.width, bmpinfo.height, bmpinfo.format);
        return;
    }

    int uvstart = width * height;
    for (int j = 0; j < height; j += 1) {
        for (int i = 0; i < width; i += 1) {
            int y = (unsigned char)src[j*width + i];
            int offset = uvstart + (j >> 1)*width + (i & ~1);
            int u = (unsigned char)src[offset + 1];
            int v = (unsigned char)src[offset + 0];

            y = y < 16 ? 16 : y;

            int a0 = 1192 * (y - 16);
            int a1 = 1634 * (v - 128);
            int a2 = 832 * (v - 128);
            int a3 = 400 * (u - 128);
            int a4 = 2066 * (u - 128);

            int r = (a0 + a1) >> 10;
            int g = (a0 - a2 - a3) >> 10;
            int b = (a0 + a4) >> 10;

            r = r < 0 ? 0 : (r > 255 ? 255 : r);
            g = g < 0 ? 0 : (g > 255 ? 255 : g);
            b = b < 0 ? 0 : (b > 255 ? 255 : b);

            // Output image in portrait orientation
            dst[(i+1)*height - j-1] = 0xff000000 | (r << 16) | (g << 8) | b;
        }
    }

    (*env)->ReleaseByteArrayElements(env, _src, src, 0);
    AndroidBitmap_unlockPixels(env, _dst);
}

/*
 * Class:     edu_umich_eecs_april_apriltag_ApriltagNative
 * Method:    apriltag_init
 * Signature: (Ljava/lang/String;IDDI)V
 */
JNIEXPORT void JNICALL Java_edu_umich_eecs_april_apriltag_ApriltagNative_apriltag_1init
        (JNIEnv *env, jclass cls, jstring _tfname, jint errorbits, jdouble decimate,
         jdouble sigma, jint nthreads) {
    // Do cleanup in case we're already initialized
    if (state.td) {
        apriltag_detector_destroy(state.td);
        state.td = NULL;
    }
    if (state.tf) {
        state.tf_destroy(state.tf);
        state.tf = NULL;
    }

    // Initialize state
    const char *tfname = (*env)->GetStringUTFChars(env, _tfname, NULL);

    if (!strcmp(tfname, "tag36h11")) {
        state.tf = tag36h11_create();
        state.tf_destroy = tag36h11_destroy;
    } else if (!strcmp(tfname, "tag16h5")) {
        state.tf = tag16h5_create();
        state.tf_destroy = tag16h5_destroy;
    } else {
        __android_log_print(ANDROID_LOG_ERROR, "apriltag_jni",
                            "invalid tag family: %s", tfname);
        (*env)->ReleaseStringUTFChars(env, _tfname, tfname);
        return;
    }
    (*env)->ReleaseStringUTFChars(env, _tfname, tfname);

    state.td = apriltag_detector_create();
    apriltag_detector_add_family_bits(state.td, state.tf, errorbits);
    state.td->quad_decimate = decimate;
    state.td->quad_sigma = sigma;
    state.td->nthreads = nthreads;
}

/*
 * Class:     edu_umich_eecs_april_apriltag_ApriltagNative
 * Method:    apriltag_detect_yuv
 * Signature: ([BII)Ljava/util/ArrayList;
 */
JNIEXPORT jobject JNICALL Java_edu_umich_eecs_april_apriltag_ApriltagNative_apriltag_1detect_1yuv
        (JNIEnv *env, jclass cls, jbyteArray _buf, jint width, jint height) {
    // If not initialized, init with default settings
    if (!state.td) {
        state.tf = tag36h11_create();
        state.td = apriltag_detector_create();
        apriltag_detector_add_family_bits(state.td, state.tf, 2);
        state.td->quad_decimate = 2.0;
        state.td->quad_sigma = 0.0;
        state.td->nthreads = 4;
        __android_log_write(ANDROID_LOG_INFO, "apriltag_jni",
                            "using default parameters");
    }

    // Use the luma channel (the first width*height elements)
    // as grayscale input image
    jbyte *buf = (*env)->GetByteArrayElements(env, _buf, NULL);
    image_u8_t im = {
            .buf = (uint8_t*)buf,
            .height = height,
            .width = width,
            .stride = width
    };
    zarray_t *detections = apriltag_detector_detect(state.td, &im);
    (*env)->ReleaseByteArrayElements(env, _buf, buf, 0);

    // al = new ArrayList();
    jobject al = (*env)->NewObject(env, state.al_cls, state.al_constructor);
    for (int i = 0; i < zarray_size(detections); i += 1) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // ad = new ApriltagDetection();
        jobject ad = (*env)->NewObject(env, state.ad_cls, state.ad_constructor);
        (*env)->SetIntField(env, ad, state.ad_id_field, det->id);
        (*env)->SetIntField(env, ad, state.ad_hamming_field, det->hamming);
        jdoubleArray ad_c = (*env)->GetObjectField(env, ad, state.ad_c_field);
        (*env)->SetDoubleArrayRegion(env, ad_c, 0, 2, det->c);
        jdoubleArray ad_p = (*env)->GetObjectField(env, ad, state.ad_p_field);
        (*env)->SetDoubleArrayRegion(env, ad_p, 0, 8, (double*)det->p);

        // al.add(ad);
        (*env)->CallBooleanMethod(env, al, state.al_add, ad);

        // Need to respect the local reference limit
        (*env)->DeleteLocalRef(env, ad);
        (*env)->DeleteLocalRef(env, ad_c);
        (*env)->DeleteLocalRef(env, ad_p);
    }

    // Cleanup
    apriltag_detections_destroy(detections);

    return al;
}

JNIEXPORT jobject JNICALL
Java_edu_umich_eecs_april_apriltag_ApriltagNative_getApriltagPoses(JNIEnv *env, jclass clazz,
                                                                   jdouble tag_size_meters,
                                                                   jbyteArray _buf, jint width,
                                                                   jint height, jdouble fx,
                                                                   jdouble fy, jdouble cx,
                                                                   jdouble cy) {
    // Use the luma channel (the first width*height elements)
    // as grayscale input image
    jbyte *buf = (*env)->GetByteArrayElements(env, _buf, NULL);
    image_u8_t im = {
            .buf = (uint8_t*)buf,
            .height = height,
            .width = width,
            .stride = width
    };
    zarray_t *detections = apriltag_detector_detect(state.td, &im);
    (*env)->ReleaseByteArrayElements(env, _buf, buf, 0);

    // al = new ArrayList();
    jobject al = (*env)->NewObject(env, state.al_cls, state.al_constructor);
    for (int i = 0; i < zarray_size(detections); i += 1) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        apriltag_detection_info_t tag_info = {
                .tagsize = tag_size_meters,
                .det = det,
                .fx = fx,
                .fy = fy,
                .cx = cx,
                .cy = cy
        };

        apriltag_pose_t pose1 = {};
        apriltag_pose_t pose2 = {};
        double err1 = 0.0;
        double err2 = 0.0;
        int iterations = 25; // TODO figure out best iteration value (higher = slower but lower = less accurate)
        estimate_tag_pose_orthogonal_iteration(
                &tag_info,
                &err1,
                &pose1,
                &err2,
                &pose2,
                iterations
                );
        if (err1 > err2){
            apriltag_pose_t temp_pose = pose1;
            pose1 = pose2;
            pose2 = temp_pose;
            double temp_err = err1;
            err1 = err2;
            err2 = temp_err;
        }
        double pose_confidence = err1/err2;

        // apriltag_pose = new ApriltagPose();
        jobject apriltag_pose = (*env)->NewObject(env, state.apriltag_pose_class, state.apriltag_pose_constructor);
        (*env)->SetIntField(env, apriltag_pose, state.apriltag_pose_id_field, det->id);
        (*env)->SetIntField(env, apriltag_pose, state.apriltag_pose_hamming_field, det->hamming);
        (*env)->SetDoubleField(env, apriltag_pose, state.apriltag_pose_poseConfidence_field, pose_confidence);

        jdoubleArray apriltag_pose_center = (*env)->GetObjectField(env, apriltag_pose, state.apriltag_pose_center_field);
        (*env)->SetDoubleArrayRegion(env, apriltag_pose_center, 0, 2, det->c);
        jdoubleArray apriltag_pose_corners = (*env)->GetObjectField(env, apriltag_pose, state.apriltag_pose_corners_field);
        (*env)->SetDoubleArrayRegion(env, apriltag_pose_corners, 0, 8, (double*)det->p);
//
        jdoubleArray apriltag_pose_translation_1 = (*env)->GetObjectField(env, apriltag_pose, state.apriltag_pose_translationMeters_1_field);
        (*env)->SetDoubleArrayRegion(env, apriltag_pose_translation_1, 0, 3, (double*)(pose1.t->data));
//
        jdoubleArray apriltag_pose_rotation_1 = (*env)->GetObjectField(env, apriltag_pose, state.apriltag_pose_rotation_1_field);
        (*env)->SetDoubleArrayRegion(env, apriltag_pose_rotation_1, 0, 9, pose1.R->data);
//
        if(pose2.t){
            jdoubleArray apriltag_pose_translation_2 = (*env)->GetObjectField(env, apriltag_pose, state.apriltag_pose_translationMeters_2_field);
            (*env)->SetDoubleArrayRegion(env, apriltag_pose_translation_2, 0, 3, pose2.t->data);
            jdoubleArray apriltag_pose_rotation_2 = (*env)->GetObjectField(env, apriltag_pose, state.apriltag_pose_rotation_2_field);
            (*env)->SetDoubleArrayRegion(env, apriltag_pose_rotation_2, 0, 9, pose2.R->data);
            (*env)->CallBooleanMethod(env, al, state.al_add, apriltag_pose);
            (*env)->DeleteLocalRef(env, apriltag_pose_translation_2);
            (*env)->DeleteLocalRef(env, apriltag_pose_rotation_2);
        } else {
            // al.add(apriltag_pose);
            (*env)->CallBooleanMethod(env, al, state.al_add, apriltag_pose);
        }

        // Need to respect the local reference limit
        (*env)->DeleteLocalRef(env, apriltag_pose);
        (*env)->DeleteLocalRef(env, apriltag_pose_center);
        (*env)->DeleteLocalRef(env, apriltag_pose_corners);
        (*env)->DeleteLocalRef(env, apriltag_pose_translation_1);
        (*env)->DeleteLocalRef(env, apriltag_pose_rotation_1);

    }

    // Cleanup
    apriltag_detections_destroy(detections);

    return al;
}