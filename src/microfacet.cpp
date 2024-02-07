/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
    	//throw NoriException("MicrofacetBRDF::eval(): not implemented!");
        /*Color3f diffuse = m_kd * INV_PI;
        Vector3f wh = (bRec.wi + bRec.wo) / bRec.wi.dot(bRec.wo);
        float cosThetaI = bRec.wi.z(), cosThetaO = bRec.wo.z(), cosThetaH = wh.z();
        
        if (cosThetaI <= 0.f || cosThetaO <= 0.f || cosThetaH <= 0.f)
            return Color3f(0.f);

        //D
        float cos2Theta = cosThetaH * cosThetaH;
        float tan2Theta = (1.f - cos2Theta) / cos2Theta;
        float alpha2 = m_alpha * m_alpha;
        float numerator = exp(-tan2Theta / alpha2);
        float denominator = alpha2 * cos2Theta * cosThetaH;
        float D = numerator / denominator * INV_PI;

        //F
        float F = fresnel(bRec.wi.dot(wh), m_extIOR, m_intIOR);

        //G
        auto b = [](float p)->float
        {
            if (p < 1.6f)
            {
                float p2 = p * p;
                return (3.535 * p + 2.181 * p2) / (1.f + 2.276 * p + 2.577 * p2);
            }
            else
            {
                return 1.f;
            }
        };
        auto c = [](const Vector3f& wv, Vector3f& wh)->float
        {
            return (wv.dot(wh) / wv.dot(Vector3f(0.f, 0.f, 1.f))) > 0.f ? 1.f : 0.f;
        };

        float tanThetaI = sqrt(1.f - cosThetaI*cosThetaI)/cosThetaI;
        float tanThetaO = sqrt(1.f - cosThetaO*cosThetaO)/cosThetaO;
        float G1 = b(1.f / (m_alpha * tanThetaI)) * c(bRec.wi, wh);
        float G2 = b(1.f / (m_alpha * tanThetaO)) * c(bRec.wo, wh);
        float G = G1 * G2;
        
        //final
        return diffuse + m_ks * F * D * G / 4.f / cosThetaI / cosThetaO / cosThetaH;*/
        Vector3f wh = (bRec.wi + bRec.wo).normalized(); // half vector
        float cosThetaI = Frame::cosTheta(bRec.wi);
        float cosThetaO = Frame::cosTheta(bRec.wo);
        float cosThetaH = Frame::cosTheta(wh);
        float D = DistributeBeckmann(wh, m_alpha);
        float F = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);  // cos(wi, wh) not cos(wi, N)
        float G = G1(bRec.wi, wh, m_alpha) * G1(bRec.wo, wh, m_alpha);
        return (m_kd / M_PI) + m_ks * ((D * F * G) / (4.0f * cosThetaI * cosThetaO * cosThetaH));
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
    	//throw NoriException("MicrofacetBRDF::pdf(): not implemented!");
        /*
        Vector3f wh = (bRec.wi + bRec.wo) / bRec.wi.dot(bRec.wo);
        float cosThetaO = bRec.wo.z(), cosThetaH = wh.z();

        if (cosThetaO <= 0.f)
            return 0.f;

        //D
        float cos2Theta = cosThetaH * cosThetaH;
        float tan2Theta = (1.f - cos2Theta) / cos2Theta;
        float alpha2 = m_alpha * m_alpha;
        float numerator = exp(-tan2Theta / alpha2);
        float denominator = alpha2 * cos2Theta * cosThetaH;
        float D = numerator / denominator * INV_PI;
        
        float Jh = 0.25f / (wh.dot(bRec.wo));

        return m_ks * D * Jh + (1 - m_ks) * cosThetaO * INV_PI;
        */
        if (bRec.wo.z() <= 0)
        {
            return 0;
        }
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float D = DistributeBeckmann(wh, m_alpha);
        float jacobian = 1 / (4.0f * abs(wh.dot(bRec.wo)));
        return m_ks * D * Frame::cosTheta(wh) * jacobian + (1 - m_ks) * Frame::cosTheta(bRec.wo) * INV_PI;
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
    	//throw NoriException("MicrofacetBRDF::sample(): not implemented!");

        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        if (Frame::cosTheta(bRec.wi) <= 0)
        {
            return Color3f(0.f);
        }

        if (m_ks >= _sample.x())
        {/*
            bRec.measure = ESolidAngle;
            bRec.wo = Warp::squareToBeckmann(_sample, m_alpha);
            float _pdf = Warp::squareToBeckmannPdf(bRec.wo, m_alpha);

            return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
            */

            Point2f sample(_sample.x() / m_ks, _sample.y());
            Vector3f wh = Warp::squareToBeckmann(sample, m_alpha);
            bRec.wo = ((2.0f * wh.dot(bRec.wi) * wh) - bRec.wi).normalized();
        }
        else
        {
            /*
            if (Frame::cosTheta(bRec.wi) <= 0)
                return Color3f(0.0f);

            bRec.measure = ESolidAngle;
            bRec.wo = Warp::squareToCosineHemisphere(_sample);
            bRec.eta = 1.0f;
            float _pdf = Warp::squareToCosineHemispherePdf(bRec.wo);

            return m_kd / Frame::cosTheta(bRec.wo) / pdf(bRec);
            */
            Point2f sample((_sample.x() - m_ks) / (1.f - m_ks), _sample.y());
            bRec.wo = Warp::squareToCosineHemisphere(sample);
        }

        if (bRec.wo.z() < 0.f)
        {
            return Color3f(0.0f);
        }

        return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }

    static float DistributeBeckmann(const Vector3f& wh, float alpha)
    { // Beckmann normal distribution term
        float tanTheta = Frame::tanTheta(wh);
        float cosTheta = Frame::cosTheta(wh);
        float a = std::exp(-(tanTheta * tanTheta) / (alpha * alpha));
        float b = M_PI * alpha * alpha * std::pow(cosTheta, 4.0f);
        return a / b;
    }

    static float G1(const Vector3f& wv, const Vector3f& wh, float alpha)
    { // Beckmann geometric masking term
        float c = wv.dot(wh) / Frame::cosTheta(wv);
        if (c <= 0)
        {
            return 0;
        }
        float b = 1.0f / (alpha * Frame::tanTheta(wv));
        return b < 1.6f ? (3.535f * b + 2.181f * b * b) / (1.f + 2.276f * b + 2.577f * b * b) : 1;
    }
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
