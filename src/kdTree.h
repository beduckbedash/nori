#pragma once
#include <nori/object.h>
#include <nori/frame.h>
#include <nori/bbox.h>
#include <nori/dpdf.h>

NORI_NAMESPACE_BEGIN

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
	kd(int inValue, bool inLeaf)
		: value(inValue)
		, bLeaf(inLeaf)
	{ 
	}

	kd* left = nullptr;
	kd* right = nullptr;
	bool bLeaf = false;
	int value;
	int axis = 0;
};

template<typename TElement>
kd* buildkdTree(std::vector<int>& p, int depth, int lastSplit, TElement** arr, int eleNums)
{
	if (p.size() == 0)
		return nullptr;

	if (p.size() == 1)
	{
		return new kd(p[0], true);
	}

	kd* root = new kd();

	std::sort(p.begin(), p.end(), [&lastSplit](int e1, int e2)->bool {
		switch (lastSplit)
		{
		case 0:
			return test2[e1].x < test2[e2].x;
		case 1:
			return test2[e1].y < test2[e2].y;
		default:
			return test2[e1].z < test2[e2].z;
		}
		});
	size_t mid = (p.size()) / 2;
	root->value = *(p.begin() + mid);
	//printf("%d\n", root->value)
	std::vector<int> pl, pr;
	pl.insert(pl.end(), p.begin(), p.begin() + mid);
	pr.insert(pr.end(), p.begin() + mid + 1, p.end());
	printVector(pl);
	printVector(pr);

	root->axis = lastSplit++;
	lastSplit = lastSplit % 3;
	root->left = buildkdTree(pl, depth + 1, lastSplit);
	root->right = buildkdTree(pr, depth + 1, lastSplit);

	return root;
}



NORI_NAMESPACE_END
