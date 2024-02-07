#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <queue>
#include <set>
#include <stack>
#include <algorithm>

NORI_NAMESPACE_BEGIN

//delcare
struct Photon
{
	Point3f pos;
	Vector3f dir; //incident direction
	Color3f power;

	Photon() = default;
	Photon(Point3f& inPos, Vector3f& inDir, Color3f& inPower)
		:pos(inPos)
		, dir(inDir)
		, power(inPower)
	{
	};

	Photon& operator=(const Photon& rhs)
	{
		this->pos = rhs.pos;
		this->dir = rhs.dir;
		this->power = rhs.power;
		return *this;
	};
};

struct RangedNearestPhotons
{
	Point3f target;
	int max_photons;
	float* distSq;
	int* indexes;
	int current_photons;
	float max_distSq;
};

//KD-Tree part
/*
struct kdNode
{
	Photon pn;
	kdNode* left;
	kdNode* right;
};
*/

struct kd
{
	kd() = default;
	~kd()
	{
		if (left)
		{
			delete left;
			left = nullptr;
		}
		if (right)
		{
			delete right;
			right = nullptr;
		}
	}
	void init(float p, uint32_t a)
	{
		splitPos = p;
		splitAxis = a;
		rightChild = (1 << 29) - 1;
		hasLeftChild = 0;
	}

	void initLeaf() {
		splitAxis = 3;
		rightChild = (1 << 29) - 1;
		hasLeftChild = 0;
	}

	float splitPos;
	uint32_t splitAxis : 2;
	uint32_t hasLeftChild : 1, rightChild : 29;

	//
	kd* left = nullptr;
	kd* right = nullptr;
	bool bLeaf = false;
	int value;
	int axis = 0;
};

template <typename NodeData> class KdTree {
public:
	KdTree(const std::vector<NodeData>& data);
	~KdTree() {
		delete[] nodes;
		delete[] nodeData;
	}
	template <typename LookupProc> void Lookup(const Point3f& p,
		LookupProc& process, float& maxDistSquared) const;
private:
	void recursiveBuild(uint32_t nodeNum, int start, int end, const NodeData** buildNodes);
	
	template <typename LookupProc> 
	void privateLookup(uint32_t nodeNum, const Point3f& p, LookupProc& process, float& maxDistSquared) const;

	kd* nodes;
	NodeData* nodeData;
	uint32_t nNodes, nextFreeNode;
};

template <typename NodeData> struct CompareNode {
	CompareNode(int a) { axis = a; }
	int axis;
	bool operator()(const NodeData* d1, const NodeData* d2) const {
		return d1->pos[axis] == d2->pos[axis] ? (d1 < d2) :
			d1->pos[axis] < d2->pos[axis];
	}
};

template <typename NodeData>
KdTree<NodeData>::KdTree(const std::vector<NodeData>& d) {
	nNodes = d.size();
	nextFreeNode = 1;
	nodes = new kd[nNodes];
	nodeData = new NodeData[nNodes];
	std::vector<const NodeData*> buildNodes(nNodes, NULL);
	for (uint32_t i = 0; i < nNodes; ++i)
		buildNodes[i] = &d[i];

	recursiveBuild(0, 0, nNodes, &buildNodes[0]);
}

template <typename NodeData> 
void KdTree<NodeData>::recursiveBuild(uint32_t nodeNum, int start, int end, const NodeData** buildNodes) {
	if (start + 1 == end) {
		nodes[nodeNum].initLeaf();
		nodeData[nodeNum] = *buildNodes[start];
		return;
	}

	BoundingBox3f bound;
	for (int i = start; i < end; ++i)
		bound.expandBy(buildNodes[i]->pos);

	Vector3f extent = bound.getExtents();
	int splitAxis = std::max(extent.x(), std::max(extent.y(), extent.z()));
	int splitPos = (start + end) / 2;
	std::nth_element(&buildNodes[start], &buildNodes[splitPos],
		&buildNodes[end], CompareNode<NodeData>(splitAxis));

	nodes[nodeNum].init(buildNodes[splitPos]->pos[splitAxis], splitAxis);
	nodeData[nodeNum] = *buildNodes[splitPos];
	if (start < splitPos) {
		nodes[nodeNum].hasLeftChild = 1;
		uint32_t childNum = nextFreeNode++;
		recursiveBuild(childNum, start, splitPos, buildNodes);
	}
	if (splitPos + 1 < end) {
		nodes[nodeNum].rightChild = nextFreeNode++;
		recursiveBuild(nodes[nodeNum].rightChild, splitPos + 1,
			end, buildNodes);
	}
}

template <typename NodeData> template <typename LookupProc>
void KdTree<NodeData>::Lookup(const Point3f& p, LookupProc& proc,
	float& maxDistSquared) const {
	privateLookup(0, p, proc, maxDistSquared);
}


template <typename NodeData> template <typename LookupProc>
void KdTree<NodeData>::privateLookup(uint32_t nodeNum, const Point3f& p,
	LookupProc& process, float& maxDistSquared) const {
	kd* node = &nodes[nodeNum];

	int axis = node->splitAxis;
	if (axis != 3) {
		float dist2 = (p[axis] - node->splitPos) * (p[axis] - node->splitPos);
		if (p[axis] <= node->splitPos) {
			if (node->hasLeftChild)
				privateLookup(nodeNum + 1, p, process, maxDistSquared);
			if (dist2 < maxDistSquared && node->rightChild < nNodes)
				privateLookup(node->rightChild, p, process, maxDistSquared);
		}
		else {
			if (node->rightChild < nNodes)
				privateLookup(node->rightChild, p, process, maxDistSquared);
			if (dist2 < maxDistSquared && node->hasLeftChild)
				privateLookup(nodeNum + 1, p, process, maxDistSquared);
		}
	}

	float dist2 = calcDistSq(nodeData[nodeNum].pos, p);
	if (dist2 < maxDistSquared)
		process(p, nodeData[nodeNum], dist2, maxDistSquared);
}
/*
static kd* buildkdTree(std::vector<int>& p, int depth, int lastSplit, Photon* arr, int eleNums, int start, int end)
{
	if (start == end)
		return nullptr;

	if (end - start == 1)
	{
		return new kd(p[0], true);
	}

	kd* root = new kd();

	std::sort(p.begin() + start, p.begin() + end, [&arr, &lastSplit](int e1, int e2)->bool {
		switch (lastSplit)
		{
		case 0:
			return arr[e1].pos.x() < arr[e2].pos.x();
		case 1:
			return arr[e1].pos.y() < arr[e2].pos.y();
		default:
			return arr[e1].pos.z() < arr[e2].pos.z();
		}
		});
	size_t mid = (start + end) / 2;
	root->value = *(p.begin() + mid);
	root->axis = lastSplit;

	lastSplit = ++lastSplit % 3;
	root->left = buildkdTree(p, depth + 1, lastSplit, arr, eleNums, start, mid);
	root->right = buildkdTree(p, depth + 1, lastSplit, arr, eleNums, mid, end);

	return root;
}
*/
using Ty = std::pair<float, int>;

//template <typename Ty>
struct distCompare {
	bool operator() (Ty a, Ty b) {
		return a.first < b.first;
	}
};

static float calcDistSq(const Point3f& a, const Point3f& b)
{
	float x = a.x() - b.x(), y = a.y() - b.y(), z = a.z() - b.z();
	return x * x + y * y + z * z;
}

static void rangedkdSearchKNN(kd* root, int value, int k, Photon* arr, int eleNums, RangedNearestPhotons& rnp)
{
	Point3f target = rnp.target;

	//std::priority_queue<int, std::vector<int>, decltype([](int e1, int e2)->bool {return false;})> kIdx;//cpp20
	std::priority_queue<Ty, std::vector<Ty>, distCompare> kIdx;
	std::set<int> kIdxSet;

	float nearestSq = 0.f;

	kd* curNode = root;
	//std::vector<kd*> backPath;
	std::stack<kd*> backPath;
	std::set<int> pathSet;
	Point3f ref = arr[curNode->value].pos;
	while (!!curNode && !curNode->bLeaf)
	{
		backPath.push(curNode);
		pathSet.insert(curNode->value);

		ref = arr[curNode->value].pos;

		if (kIdxSet.find(curNode->value) == kIdxSet.end())
		{
			float distToTarget = calcDistSq(ref, target);
			//if (distToTarget < rnp.max_distSq)
			{
				kIdxSet.insert(curNode->value);
				kIdx.push(std::make_pair(distToTarget, curNode->value));
				rnp.indexes[rnp.current_photons] = curNode->value;
				rnp.distSq[rnp.current_photons] = distToTarget;
				rnp.current_photons++;
			}
		}

		switch (curNode->axis)
		{
		case 0:
			curNode = (target.x() > ref.x()) ? curNode->right : curNode->left;
			break;
		case 1:
			curNode = (target.y() > ref.y()) ? curNode->right : curNode->left;
			break;
		default:
			curNode = (target.z() > ref.z()) ? curNode->right : curNode->left;
			;
		}
	}

	nearestSq = calcDistSq(target, ref);
	while (!backPath.empty())
	{
		const kd* backNode = backPath.top();
		int idx = backNode->value, axis = backNode->axis;
		kd* left = backNode->left, * right = backNode->right;
		backPath.pop();

		const Point3f& ref = arr[idx].pos;
		kd* insertNode = nullptr;
		switch (axis)
		{
		case 0:
		{
			float x = ref.x() - target.x();
			if (nearestSq > x * x)
			{
				if (kIdxSet.find(idx) == kIdxSet.end())
				{
					float distToTarget = calcDistSq(ref, target);
					if (distToTarget < rnp.max_distSq)
					{
						kIdxSet.insert(idx);
						kIdx.push(std::make_pair(distToTarget, idx));
						rnp.indexes[rnp.current_photons] = idx;
						rnp.distSq[rnp.current_photons] = distToTarget;
						rnp.current_photons++;
					}
				}
				if (!!left)backPath.push(left);
				if (!!right)backPath.push(right);
			}
			else
			{
				insertNode = (target.x() > ref.x()) ? right : left;
				if (!!insertNode && pathSet.find(idx) == pathSet.end())
				{
					backPath.push(insertNode);
					pathSet.insert(idx);
				}
			}
			break;
		}
		case 1:
		{
			float y = ref.y() - target.y();
			if (nearestSq > y * y)
			{
				if (kIdxSet.find(idx) == kIdxSet.end())
				{
					float distToTarget = calcDistSq(ref, target);
					if (distToTarget < rnp.max_distSq)
					{
						kIdxSet.insert(idx);
						kIdx.push(std::make_pair(distToTarget, idx));
						rnp.indexes[rnp.current_photons] = idx;
						rnp.distSq[rnp.current_photons] = distToTarget;
						rnp.current_photons++;
					}
				}
				if (!!left)backPath.push(left);
				if (!!right)backPath.push(right);
			}
			else
			{
				insertNode = (target.y() > ref.y()) ? right : left;
				if (!!insertNode && pathSet.find(idx) == pathSet.end())
				{
					backPath.push(insertNode);
					pathSet.insert(idx);
				}
			}
			break;
		}
		default:
		{
			float z = ref.z() - target.z();
			if (nearestSq > z * z)
			{
				if (kIdxSet.find(idx) == kIdxSet.end())
				{
					float distToTarget = calcDistSq(ref, target);
					if (distToTarget < rnp.max_distSq)
					{
						kIdxSet.insert(idx);
						kIdx.push(std::make_pair(distToTarget, idx));
						rnp.indexes[rnp.current_photons] = idx;
						rnp.distSq[rnp.current_photons] = distToTarget;
						rnp.current_photons++;
					}
				}
				if (!!left)backPath.push(left);
				if (!!right)backPath.push(right);
			}
			else
			{
				insertNode = (target.z() > ref.z()) ? right : left;
				if (!!insertNode && pathSet.find(idx) == pathSet.end())
				{
					backPath.push(insertNode);
					pathSet.insert(idx);
				}
			}
		}
		}

		if (kIdx.size() > k)
		{
			int extraN = kIdx.size() - k;
			while (extraN--)
			{
				kIdx.pop();
			}
		}
		if (kIdx.size() > 0)
		{
			nearestSq = calcDistSq(target, arr[kIdx.top().second].pos);
		}
	}
}

//Photon Mapping part
const int MAX_PHOTONS_NUMS = 200000;
class PhotonMap
{
public:
	int m_photon_nums;
	int m_max_photon_nums;
	Photon** m_photon = nullptr;
	BoundingBox3d m_boundingbox;
	//void store(Photon pn);
	kd* m_kd_root;

	//new kd
	KdTree<Photon>* m_kd_tree;
	std::vector<Photon> m_photons;

	PhotonMap()
	{
		m_photon_nums = 0;
		m_max_photon_nums = MAX_PHOTONS_NUMS;
		//m_photon = new Photon*[MAX_PHOTONS_NUMS];
		m_photons.reserve(MAX_PHOTONS_NUMS);
	}

	PhotonMap(int max)
	{
		m_photon_nums = 0;
		m_max_photon_nums = MAX_PHOTONS_NUMS;
		//m_photon = new Photon*[max];
		m_photons.reserve(max);
	}

	~PhotonMap()
	{
		if (!!m_photon)
			delete m_photon;
		m_photon = nullptr;
		
		if (!!m_kd_root)
			delete m_kd_root;
		m_kd_root = nullptr;
	}

	void store(Photon pn)
	{
		if (m_photon_nums >= m_max_photon_nums)
			return;
		/*
		m_photon[m_photon_nums]->pos = pn.pos;
		m_photon[m_photon_nums]->dir = pn.dir;
		m_photon[m_photon_nums]->power = pn.power;
		*/
		m_photon_nums++;
		m_photons.emplace_back(pn);
	}

	bool buildAccel()
	{
		/*
		std::vector<int> indexArr;
		for (int i = 0; i < m_photon_nums; ++i)
		{
			indexArr.emplace_back(i);
		}

		m_kd_root = buildkdTree(indexArr, 0, 0, m_photon, m_photon_nums, 0, m_photon_nums);
		
		return (m_kd_root != nullptr);
		*/
		
		m_kd_tree = new KdTree<Photon>(m_photons);
		return (m_kd_tree != nullptr);
	}

};

struct ClosePhoton {
	ClosePhoton(const Photon* p = nullptr, float md2 = INFINITY)
		: photon(p), distanceSquared(md2) { }
	bool operator<(const ClosePhoton& p2) const {
		return distanceSquared == p2.distanceSquared ?
			(photon < p2.photon) : (distanceSquared < p2.distanceSquared);
	}
	const Photon* photon;
	float distanceSquared;
};

struct rangedSearchProc
{
	rangedSearchProc(uint32_t mp, ClosePhoton* buf)
	{
		photons = buf;
		nLookup = mp;
		nFound = 0;
	}

	void operator()(const Point3f& p, const Photon& photon, float dist2, float& maxDistSquared) 
	{
		if (nFound < nLookup) {
			photons[nFound++] = ClosePhoton(&photon, dist2);
			if (nFound == nLookup) {
				std::make_heap(&photons[0], &photons[nLookup]);
				maxDistSquared = photons[0].distanceSquared;
			}
		}
		else {
			std::pop_heap(&photons[0], &photons[nLookup]);
			photons[nLookup - 1] = ClosePhoton(&photon, dist2);
			std::push_heap(&photons[0], &photons[nLookup]);
			maxDistSquared = photons[0].distanceSquared;
		}
	}

	ClosePhoton* photons;
	uint32_t nLookup, nFound;
};



class Photon_MappingIntegrator : public Integrator
{
public:
	Photon_MappingIntegrator(const PropertyList& props) {
		/*
		m_myProperty = props.getString("myProperty");
		std::cout << "Parameter value was : " << m_myProperty << std::endl;
		*/
		m_photonmap = new PhotonMap();
		m_photonmap->m_max_photon_nums = MAX_PHOTONS_NUMS;
	}
	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
	{
		Intersection its;
		Color3f Lo(0.f), Beta(1.f);
		Ray3f pathRay = ray;

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
				if (bounces == 0)
					Lo += Beta * its.mesh->getEmitter()->getRadiance();
			}

			//RR
			if (bounces > 3)
			{
				float maxCoeff = Beta.maxCoeff();
				float q = std::min(0.99f, maxCoeff);
				//float q = 0.95f;
				if (sampler->next1D() > q)
					break;
				Beta /= q;
			}

			//direct
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
			
			//pm
			if (its.mesh->getBSDF()->isDiffuse())
			{
				Lo += Beta * getIrradiance(x, Nx, 0.6, 100, bsdf, its, wo);
				break;
			}

			//indirect radiance
			BSDFQueryRecord bRec(its.toLocal(wo));
			Beta *= bsdf->sample(bRec, sampler->next2D());

			Vector3f wi = its.toWorld(bRec.wo);
			pathRay = Ray3f(its.p, wi);
		}
		return Lo;
	}

	void tracePhoton(const Scene* scene, Sampler* sampler, Ray3f& ray, Point3f& world, int depth, Color3f power)
	{
		Intersection its;
		if (scene->rayIntersect(ray, its))
		{
			const BSDF* bsdf = its.mesh->getBSDF();
			if (bsdf->isDiffuse())
			{
				Vector3f dir = -ray.d;
				m_photonmap->store(Photon(its.p, dir, power));
			}
			if (sampler->next1D() < RR)
				return;

			BSDFQueryRecord bRec(its.toLocal((-ray.d).normalized()));
			bsdf->sample(bRec, sampler->next2D());
			power *= bsdf->eval(bRec);
			power /= (1-RR);
			tracePhoton(scene, sampler, Ray3f(its.p, its.toWorld(bRec.wo)), its.p, depth++, power);
			/*
			if (bsdf)
			{
				if (bsdf->isDiffuse())
				{
					//BSDFQueryRecord bRec(its.toLocal((-ray.d).normalized()));
					//bRec.measure = EMeasure::ESolidAngle;
					Vector3f dir = -ray.d;
					//power *= bsdf->eval(bRec);
					m_photonmap->store(Photon(its.p, dir, power));
				}
				else
				{
					//calc ray mirror/specular
					BSDFQueryRecord bRec(its.toLocal((-ray.d).normalized()));
					bsdf->sample(bRec, sampler->next2D());
					power *= bsdf->eval(bRec);
					tracePhoton(scene, sampler, Ray3f(its.p, its.toWorld(bRec.wo)), its.p, depth++, power);
				}
			}
			*/
		}
	}

	void generatePhotonMap(const Scene* scene, Sampler* sampler)
	{
		Vector3f origin, dir;
		Color3f power = Color3f(1.f);
		Point3f world;

		std::vector<Mesh*> meshes = scene->getMeshes();
		for (auto& mesh : meshes)
		{
			if (Emitter* emitter = mesh->getEmitter())
			{
				for (int i = 0; i < 100000; ++i)
				{
					Intersection res;
					emitter->generatePhoton(sampler, origin, dir, power, 1);
					Ray3f ray(origin, dir);
					tracePhoton(scene, sampler, ray, world, 0, power);
				}
			}
		}
	}

	virtual void preprocess(const Scene* scene) 
	{
		std::unique_ptr<Sampler> sampler(scene->getSampler()->clone());
		generatePhotonMap(scene, sampler.get());
		m_photonmap->buildAccel();
	}

	Color3f getIrradiance(Vector3f pos, Vector3f normal, float max_dist, float N, const BSDF* bsdf, Intersection its, Vector3f wo) const
	{
		Color3f res(0,0,0);
		/*
		RangedNearestPhotons rnp;
		rnp.current_photons = 0;
		rnp.max_photons = 10000;
		rnp.distSq = new float[10000];
		rnp.indexes = new int[10000];
		rnp.max_distSq = max_dist * max_dist;
		rnp.target = pos;
		*/
		ClosePhoton* closePhotons = new ClosePhoton[N];
		rangedSearchProc proc(N, closePhotons);
		const Point3f target = pos;
		float max_distSq = max_dist * max_dist;
		//rangedkdSearchKNN(m_photonmap->m_kd_root, 0, 100, m_photonmap->m_photon, m_photonmap->m_photon_nums, rnp);
		m_photonmap->m_kd_tree->Lookup(target, proc, max_distSq);
		/*
		if (rnp.current_photons <= 8)
		{
			return res;
		}*/
		
		for (int i = 0; i < proc.nFound; i++)
		{
			const Photon* po = closePhotons[i].photon;
			BSDFQueryRecord bRec(its.toLocal(wo), its.toLocal(po->dir), ESolidAngle);
			Color3f fr = bsdf->eval(bRec);

			res += fr * po->power;

			//if (normal.dot(po->dir) > 0.f)
			{

			}
		}

		res *= (1.f/ (100000.f * M_PI * max_distSq));

		delete[] closePhotons;
		//delete[] rnp.distSq;
		//delete[] rnp.indexes;
		return res;
	}

	std::string toString() const {
		return "Photon_MappingIntegrator[]";
	}
protected:
	PhotonMap* m_photonmap;
	const float RR = 0.99f;
};


NORI_REGISTER_CLASS(Photon_MappingIntegrator, "pm");
NORI_NAMESPACE_END