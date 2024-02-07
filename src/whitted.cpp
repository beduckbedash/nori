#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator
{
public:
	WhittedIntegrator(const PropertyList& props) {
		/*
		m_myProperty = props.getString("myProperty");
		std::cout << "Parameter value was : " << m_myProperty << std::endl;
		*/
	}
	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
	{		
		float RR = 0.95f;  //Russian Roulette
		if (sampler->next1D() > RR)
			return Color3f(0.f);

		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);
		
		Color3f Le(0.f), Lr(0.f), Lo(0.f);
		Vector3f Nx = its.shFrame.n, Ny;
		if (its.mesh->isEmitter())
		{
			if (Nx.dot(-ray.d)) {
				Le = its.mesh->getEmitter()->getRadiance();
			}
		}

		const BSDF* bsdf = its.mesh->getBSDF();
		Vector3f x = its.p, y;
		if (bsdf->isDiffuse())
		{
			std::vector<Mesh*> Meshes = scene->getMeshes();
			int LightsNums = 0;
			for (int i = 0; i < Meshes.size(); ++i)
			{
				float G = 0.f, V = 1.f;
				if (Meshes[i]->isEmitter())
				{
					float Pdf;
					Meshes[i]->sample(sampler, y, Ny, Pdf);
					Ny = Ny.normalized();
					Vector3f wi = (y - x).normalized(), wo = -ray.d.normalized();

					Color3f curLe = Ny.dot(-wi) < 0.f ? Color3f(0.f) : Meshes[i]->getEmitter()->getRadiance();

					BSDFQueryRecord bRec(its.toLocal(wi), its.toLocal(wo), ESolidAngle);
					Color3f fr = bsdf->eval(bRec);

					Intersection shadowIts;
					if (scene->rayIntersect(Ray3f(x, wi, Epsilon, (y - x).norm() - Epsilon), shadowIts))
						if (!shadowIts.mesh->isEmitter())
							V = 0.f;

					G = V * fabs(Nx.dot(wi)) * fabs(Ny.dot(-wi)) / (y - x).squaredNorm();
					Lr += fr * G * curLe / Pdf;
					LightsNums++;
				}
			}
			Lo += (Lr / LightsNums + Le) / RR;

			return Lo;
		}
		else
		{
			Vector3f wo = -ray.d.normalized(), wi;
			BSDFQueryRecord bRec(its.toLocal(wo));
			Color3f c = bsdf->sample(bRec, sampler->next2D());
			wi = its.toWorld(bRec.wo);
			Ray3f scatter_ray(x, wi);
			return Li(scene, sampler, scatter_ray) * c / RR;
		}
	}

	std::string toString() const {
		return "WhittedIntegrator[]";
	}

protected:
};


NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END