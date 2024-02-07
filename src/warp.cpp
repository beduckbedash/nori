/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    //throw NoriException("Warp::squareToTent() is not yet implemented!");
    float x = sample.x() < 0.5f ? (std::sqrt(2.f * sample.x()) - 1.f) : (1.f - std::sqrt(2.f - 2.f * sample.x()));
    float y = sample.y() < 0.5f ? (std::sqrt(2.f * sample.y()) - 1.f) : (1.f - std::sqrt(2.f - 2.f * sample.y()));
    return Point2f(x, y);
}

float Warp::squareToTentPdf(const Point2f &p) {
    //throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
    float Px = std::abs(p.x()) >= 1.f ? 0.f : (1.f - std::abs(p.x()));
    float Py = std::abs(p.y()) >= 1.f ? 0.f : (1.f - std::abs(p.y()));
    return Px * Py;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
    Point2f uOffset = Point2f(2.f * sample.x() - 1.f, 2.f * sample.y() - 1.f);//2.f * sample - Point2f(1.f, 1.f);
    if (uOffset.x() == 0 && uOffset.y() == 0)
        return Point2f(0.f, 0.f);

    float theta, r;
    if (std::abs(uOffset.x()) > std::abs(uOffset.y()))
    {
        r = uOffset.x();
        theta = M_PI * 0.25f * (uOffset.y() / uOffset.x());
    }
    else
    {
        r = uOffset.y();
        theta = M_PI * 0.5f - M_PI * 0.25f * (uOffset.x() / uOffset.y());
    }
    //r = std::sqrt(sample.x());
    //theta = 2.f * M_PI * sample.y();
    return Point2f(r * std::cos(theta), r * std::sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    //throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
    if (std::sqrt(p.x() * p.x() + p.y() * p.y()) > 1)
        return 0.f;
    return INV_PI;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformSphere() is not yet implemented!");
    float z = 1 - 2 * sample.x();
    float r = std::sqrt(std::max((float)0, (float)1 - z * z));
    float phi = 2 * M_PI * sample.y();
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToUniformSpherePdf() is not yet implemented!");
    return INV_PI*0.25;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
    /*float z = sample.x();
    float r = std::sqrt(1.f - z * z);
    float phi = 2.f * M_PI * sample.y();
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);
    */
    float xi1 = sample.x(), xi2 = sample.y();
    float x, y, z, phi, cos_theta, sin_theta;
    phi = 2 * M_PI * xi1;
    cos_theta = 1 - xi2;
    sin_theta = sqrt(1 - pow(cos_theta, 2));
    x = sin_theta * cos(phi);
    y = sin_theta * sin(phi);
    z = cos_theta;
    return Vector3f(x, y, z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
    if (std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z()) > 1 || v.z() < 0.f)
        return 0.f;
    return INV_PI * 0.5f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
    float xi1 = sample.x(), xi2 = sample.y();
    float x, y, z, phi, cos_theta, sin_theta;
    phi = 2 * M_PI * xi1;
    sin_theta = sqrt(xi2);
    cos_theta = sqrt(1 - xi2);
    x = sin_theta * cos(phi);
    y = sin_theta * sin(phi);
    z = cos_theta;
    return Point3f(x, y, z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
    if (std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z()) > 1 || v.z() < 0.f)
        return 0.f;
    return INV_PI * v.z();
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    //throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
    float phi = 2.f * M_PI * sample.x();
    float tan2Theta = -alpha * alpha * log(sample.y());
    float cos2Theta = 1.f / (tan2Theta + 1.f);
    float cosTheta = sqrt(cos2Theta);
    float sinTheta = sqrt(1.f - cos2Theta);

    return Vector3f(sinTheta*cos(phi), sinTheta*sin(phi), cosTheta);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    //throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
    float cosTheta = m.z();
    if (cosTheta <= 0.f)return 0.f;
    float cos2Theta = cosTheta * cosTheta;
    float tan2Theta = (1.f - cos2Theta) / cos2Theta;
    float alpha2 = alpha * alpha;
    float numerator = exp(-tan2Theta / alpha2);
    float denominator = alpha2 * cos2Theta* cosTheta;
    return numerator / denominator * INV_PI;
}

NORI_NAMESPACE_END
