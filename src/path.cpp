#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class PathIntegrator : public Integrator
{
public:
	PathIntegrator(const PropertyList& props) {
		/*
		m_myProperty = props.getString("myProperty");
		std::cout << "Parameter value was : " << m_myProperty << std::endl;
		*/
	}
	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
	{
		
		Intersection its;
		Color3f Lo(0.f), Beta(1.f);
		Ray3f pathRay = ray;

		for (int bounces = 0 ;; ++bounces)
		{
			if (!scene->rayIntersect(pathRay, its))
				break;

			if (bounces > 3)
			{
				float maxCoeff = Beta.maxCoeff();
				float q = std::min(0.99f, maxCoeff);
				if (sampler->next1D() > q)
					break;
				Beta /= q;
			}

			Vector3f Nx = its.shFrame.n;
			Vector3f wo = (-pathRay.d).normalized();
			if (its.mesh->isEmitter())
			{
				if (Nx.dot(wo) > 0) {
					Lo += Beta * its.mesh->getEmitter()->getRadiance();
				}
			}

			BSDFQueryRecord bRec(its.toLocal(wo));
			Beta *= its.mesh->getBSDF()->sample(bRec, sampler->next2D());
			//Beta /= its.mesh->getBSDF()->pdf(bRec);

			Vector3f wi = its.toWorld(bRec.wo);
			pathRay = Ray3f(its.p, wi);
		}
		return Lo;
		
	}

	std::string toString() const {
		return "PathIntegrator[]";
	}

protected:
};


NORI_REGISTER_CLASS(PathIntegrator, "path_ems");
NORI_NAMESPACE_END