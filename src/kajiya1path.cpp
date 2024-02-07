#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class Kajiya1PathIntegrator : public Integrator
{
public:
	Kajiya1PathIntegrator(const PropertyList& props) {
		/*
		m_myProperty = props.getString("myProperty");
		std::cout << "Parameter value was : " << m_myProperty << std::endl;
		*/
	}
	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
	{
		/*
		Intersection its;
		Color3f Lo(0.f), Le(0.f);
		if (!scene->rayIntersect(ray, its))
			return Lo;
		
		if (its.mesh->isEmitter())
			Le = its.mesh->getEmitter()->getRadiance();
		return reflectedRadianceEst(scene, sampler, ray, its) + Le;
		*/
		
		Intersection its;
		Color3f Lo(0.f), Beta(1.f);
		Ray3f pathRay = ray;

		//if (!scene->rayIntersect(pathRay, its))
			//return Lo;

		float weight_bsdf = 0.f, weight_light = 0.f;
		bool last_sample_is_ems = false;
		for (int bounces = 0;; ++bounces)
		{
			if (!scene->rayIntersect(pathRay, its))
				break;

			Vector3f Nx = its.shFrame.n, x = its.p;
			Vector3f wo = (-pathRay.d).normalized();
			const BSDF* bsdf = its.mesh->getBSDF();

			//emitted radiance
			if (its.mesh->isEmitter())
			{
				//if (its.shFrame.n.dot((-pathRay.d).normalized()) > 0)
				{
					
				}
				if (bounces == 0)
					Lo += Beta * its.mesh->getEmitter()->getRadiance();
			}

			if (bounces > 3)
			{
				float maxCoeff = Beta.maxCoeff();
				float q = std::min(0.99f, maxCoeff);
				//float q = 0.95f;
				if (sampler->next1D() > q)
					break;
				Beta /= q;
			}

			last_sample_is_ems = false;
			//direct radiance
			
			if (its.mesh->getBSDF()->isDiffuse())
			{
				std::vector<Mesh*> Meshes = scene->getMeshes();
				int Lights = 0;
				Color3f Ld(0.f);
				
				for (int i = 0; i < Meshes.size(); ++i)
				{
					float G = 0.f, V = 1.f;
					if (Meshes[i]->isEmitter())
					{
						float Pdf;
						Vector3f y, Ny;
						Meshes[i]->sample(sampler, y, Ny, Pdf);
						//Ny = Ny.normalized();
						Vector3f wi = (y - x).normalized();
						Pdf = Meshes[i]->Pdf(x, y, Ny, wi);//?

						Intersection shadowIts;
						if (scene->rayIntersect(Ray3f(x, wi, Epsilon, (y - x).norm() - Epsilon), shadowIts))
							if (!shadowIts.mesh->isEmitter())
							{
								V = 0.f;
							}
							{
								last_sample_is_ems = true;

								Color3f curLe = Ny.dot(-wi) < 0.f ? Color3f(0.f) : Meshes[i]->getEmitter()->getRadiance();
								curLe = (Pdf > 0.f && !std::isnan(Pdf) && !std::isinf(Pdf)) ? curLe / Pdf : Color3f(0.f);
								
								BSDFQueryRecord bRec(its.toLocal(wo), its.toLocal(wi), ESolidAngle);
								Color3f fr = bsdf->eval(bRec);

								G = V * fabs(Nx.dot(wi)) * fabs(Ny.dot(-wi)) / (y - x).squaredNorm();
								Ld += fr * G * curLe * Beta;
							}

						Lights++;
					}
				}
				Lo += Ld / Lights;
			}

			//indirect radiance
			BSDFQueryRecord bRec(its.toLocal(wo));
			Beta *= bsdf->sample(bRec, sampler->next2D());
			//Beta /= bsdf->pdf(bRec);//?

			Vector3f wi = its.toWorld(bRec.wo);
			pathRay = Ray3f(its.p, wi);
		}
		return Lo;

	}

	Color3f reflectedRadianceEst(const Scene* scene, Sampler* sampler, const Ray3f& ray, Intersection its) const
	{
		Color3f Lo(0.f);
		float q = 0.95f;
		if (sampler->next1D() < q)
		{
			Lo = directRadianceEst(scene, sampler, ray, its) + indirectRadianceEst(scene, sampler, ray, its);
			return Lo / q;
		}
		else
			return Lo;
	}

	Color3f directRadianceEst(const Scene* scene, Sampler* sampler, const Ray3f& ray, Intersection its) const
	{
		//Intersection its;
		//Color3f Lo(0.f), Le(0.f);
		Vector3f x = its.p, wo = -ray.d.normalized(), Nx = its.shFrame.n;
		const BSDF* bsdf = its.mesh->getBSDF();
		std::vector<Mesh*> Meshes = scene->getMeshes();
		int Lights = 0;
		Color3f Ld(0.f), Le(0.f);
		for (int i = 0; i < Meshes.size(); ++i)
		{
			float G = 0.f, V = 1.f;
			if (Meshes[i]->isEmitter())
			{
				float Pdf;
				Vector3f y, Ny;
				Meshes[i]->sample(sampler, y, Ny, Pdf);
				Ny = Ny.normalized();
				Vector3f wi = (y - x).normalized();
				Pdf = Meshes[i]->Pdf(x, y, Ny, wi);

				Intersection shadowIts;
				if (scene->rayIntersect(Ray3f(x, wi, Epsilon, (y - x).norm() - Epsilon), shadowIts))
					if (shadowIts.mesh->isEmitter())
					{
						Color3f curLe = Ny.dot(-wi) < 0.f ? Color3f(0.f) : shadowIts.mesh->getEmitter()->getRadiance();

						BSDFQueryRecord bRec(its.toLocal(wo), its.toLocal(wi), ESolidAngle);
						Color3f fr = bsdf->eval(bRec);

						G = V * fabs(Nx.dot(wi)) * fabs(Ny.dot(-wi)) / (y - x).squaredNorm();
						Ld += fr * G * curLe / Pdf;
					}

				Lights++;
			}
		}

		if (its.mesh->isEmitter())
			Le = its.mesh->getEmitter()->getRadiance();

		return Ld / Lights;
	}

	Color3f indirectRadianceEst(const Scene* scene, Sampler* sampler, const Ray3f& ray, Intersection its) const
	{
		//Intersection its;
		Color3f Lo(0.f), Le(0.f);
		//if (!scene->rayIntersect(ray, its))
			//return Lo;

		//float maxCoeff = Beta.maxCoeff();
		//float q = std::min(0.99f, maxCoeff);
		float q = 0.95f;
		//if (sampler->next1D() < q)
		{
			Vector3f wo = -ray.d.normalized();
			BSDFQueryRecord bRec(its.toLocal(wo));
			Color3f beta = its.mesh->getBSDF()->sample(bRec, sampler->next2D());

			Ray3f pathRay(its.p, its.toWorld(bRec.wo.normalized()));
			Intersection newIts;
			if (scene->rayIntersect(pathRay, newIts))
			{
				return beta * reflectedRadianceEst(scene, sampler, pathRay, newIts);
			}
		}
		return Lo;


		//Beta /= q;

	}

	std::string toString() const {
		return "Kajiya1PathIntegrator[]";
	}

protected:
};


NORI_REGISTER_CLASS(Kajiya1PathIntegrator, "path_mats");
NORI_NAMESPACE_END