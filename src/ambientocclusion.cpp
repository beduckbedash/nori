#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AOIntegrator : public Integrator
{
public:
	AOIntegrator(const PropertyList& props) {
		/*
		m_myProperty = props.getString("myProperty");
		std::cout << "Parameter value was : " << m_myProperty << std::endl;
		*/
	}
	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
	{
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		
		Color3f Radiance(0.f);
		Point3f Position(its.p);
		Vector3f Normal(its.shFrame.n);
		Point2f PositionSample = sampler->next2D();
		Vector3f HemispherePosition = Warp::squareToCosineHemisphere(PositionSample);
		float CosineTheta = HemispherePosition.z();
		float Pdf = Warp::squareToCosineHemispherePdf(HemispherePosition);
		HemispherePosition = its.shFrame.toWorld(HemispherePosition);

		//Vector3f Wh = (HemispherePosition - Position).normalized();
		float Vis = 1.f;
		if (scene->getAccel()->rayIntersect(Ray3f(Position, HemispherePosition), its, true))
			Vis = 0.f;

		Radiance += Vis * CosineTheta * INV_PI / Pdf;
		return Radiance;
	}

	/*
	std::string toString() const {
		return tfm::format(
			"NormalIntegrator[\n"
			" myProperty = \"%s\"\n"
			"]",
			m_myProperty
		);
	}
	*/

	std::string toString() const {
		return "AOIntegrator[]";
	}

protected:
	std::string m_myProperty;
};


NORI_REGISTER_CLASS(AOIntegrator, "ao");
NORI_NAMESPACE_END