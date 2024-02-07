/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        //throw NoriException("Unimplemented!");
        
        float cosThetaI = bRec.wi.z();
        bool bEntering = cosThetaI > 0;

        float etaI = bEntering ? m_extIOR : m_intIOR;
        float etaT = bEntering ? m_intIOR : m_extIOR;
        float r = fresnel(cosThetaI, etaI, etaT);


        if (cosThetaI < 0.0f)
        {
            cosThetaI = -cosThetaI;
        }

        if (sample.x() < r)
        {
            //reflect
            bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
        }
        else if (r > 0.f || r < 1.f)
        {
            //refract
            float eta = etaI / etaT;

            float sin2ThetaI = std::max(0.f, 1.f - cosThetaI * cosThetaI);

            float sin2ThetaT = eta * eta * sin2ThetaI;
            float sinThetaI = sqrt(1.f - cosThetaI * cosThetaI);

            float cosThetaT = std::sqrt(1 - sin2ThetaT);
            //bRec.wo = eta * -bRec.wi + (eta * cosThetaI - cosThetaT) * Vector3f(0.f,0.f,1.f);

            bRec.wo = Vector3f(sqrt(sin2ThetaT) / sinThetaI * -bRec.wi.x(), sqrt(sin2ThetaT) / sinThetaI * -bRec.wi.y(), cosThetaT / cosThetaI * -bRec.wi.z());
        }
        bRec.measure = EDiscrete;
        return Color3f(1.f);
    }

    std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }

private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END