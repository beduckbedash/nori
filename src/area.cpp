#pragma once

#include <nori/emitter.h>
#include <nori/mesh.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter {
public:
    AreaLight(const PropertyList& props) {
        m_Radiance = props.getColor("radiance");
    }


    std::string toString() const {
        return tfm::format(
            "AreaLight[\n"
            "m_Radiance = \"%f, %f, %f\"n"
            "]",
            m_Radiance.x(), m_Radiance.y(), m_Radiance.z()
        );
    }

    virtual Color3f getRadiance() const
    {
        return m_Radiance;
    };

    virtual void setMesh(Mesh* mesh)
    {
        m_Mesh = mesh;
    }

    virtual void generatePhoton(Sampler* sampler, Vector3f& origin, Vector3f& dir, Color3f& power, int N)
    {
        Vector3f normal;
        float pdf;
        m_Mesh->sample(sampler, origin, normal, pdf);
        float A = 1.f/pdf;

        Vector3f cosDir = Warp::squareToUniformHemisphere(sampler->next2D());
        Frame frame(normal);
        dir = frame.toWorld(cosDir);
        
        power = (m_Radiance) / (N * pdf);
    }

protected:
    Color3f m_Radiance;
    Mesh* m_Mesh;
};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END