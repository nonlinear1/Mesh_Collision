/**
*
* k-d tree implementation file
*
* author: Arvind Rao
* license: GNU
*
*
*/
#include "myKDtree.h"
#include <iostream>

// implementation of Node Class

Node::Node(int _n) { left = 0; right = 0; }
Node::Node(int _n, vecType _data) : data(_data) { left = 0; right = 0; }

bool Node::isLastInnerNode() {
	if (left == nullptr && right == nullptr)
		return false;
	if (left == nullptr && right->right == nullptr && right->left == nullptr)
		return true;
	if (right == nullptr && left->right == nullptr && left->left == nullptr)
		return true;
	if (right->right == nullptr && right->left == nullptr && left->right == nullptr && left->left == nullptr)
		return true;
	return false;

}

Node::~Node() {}


// implementation of myKDtree

myKDtree::myKDtree() {}
myKDtree::~myKDtree() {}

/*
*
*
*/
myKDtree::vecType myKDtree::findMedian(int axis, std::list<myKDtree::vecType> &plist, std::list<myKDtree::vecType> &left,
	std::list<myKDtree::vecType> &right)
{
	myKDtree::vecType median;
	int size = plist.size();
	int med = ceil(float(size) / float(2));
	int count = 0;

	if (size == 1)
		return plist.front();

	// Using lambda function here, to define comparison function--parametrized by 'axis'
	plist.sort([&](myKDtree::vecType& a, myKDtree::vecType& b) {return a[axis] < b[axis]; });

	for (auto& x : plist)
	{
		if (count < med)
			left.push_back(x);
		else
			right.push_back(x);
		++count;
	}
	median = left.back();
	left.pop_back();
	return median;
}

void myKDtree::print_data(vecType pt)
{
	for (int i = 0; i < N; i++)
	{
		std::cout << pt[i] << ", ";
	}
	std::cout << "\n";
}

/*
*
*
*/
void myKDtree::printTree(Node* head)
{
	//find the tree depth 
	int maxdepth = 3;
	int spaces = pow(2, maxdepth + 1) - 1;
	int depth = 0;

	std::cout << "\n**** Print of Tree **********\n";
	std::queue< Node* > current, next;
	Node * temp = head;
	current.push(temp);

	while (!current.empty())
	{
		temp = current.front();
		current.pop();

		if (temp == nullptr)
			std::cout << "NULL\n";
		else
		{
			myKDtree::print_data(temp->data);
			std::cout << "center: ";
			myKDtree::print_data(temp->center);
			std::cout << "halfDepth: " << temp->halfDepth << std::endl;
			std::cout << "halfHeight: " << temp->halfHeight << std::endl;
			std::cout << "halfWidth: " << temp->halfWidth << std::endl;

			next.push(temp->left);
			next.push(temp->right);
		}
		if (current.empty())
		{
			depth++;
			std::cout << "level: " << depth << "\n";
			std::swap(current, next);
		}
	}
}
//this function updates the center and thw width hieght and depth of the obb of this head node
void myKDtree::setHeadVariables(Node* head, std::list<myKDtree::vecType>& plist) {

	float maxX = -INFINITY;
	float maxY = -INFINITY;
	float maxZ = -INFINITY;

	float minX = INFINITY;
	float minY = INFINITY;
	float minZ = INFINITY;

	for (auto& n : plist)
	{
		if (n.x < minX)
			minX = n.x;
		if (n.y < minY)
			minY = n.y;
		if (n.z < minZ)
			minZ = n.z;

		if (n.x > maxX)
			maxX = n.x;
		if (n.y > maxY)
			maxY = n.y;
		if (n.z > maxZ)
			maxZ = n.z;
	}
	head->center = glm::vec4((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2, 1);
	head->axisX = glm::vec3(1, 0, 0);
	head->axisY = glm::vec3(0, 1, 0);
	head->axisZ = glm::vec3(0, 0, 1);
	head->halfWidth = head->center.x - minX;
	head->halfHeight = head->center.y - minY;
	head->halfDepth = head->center.z - minZ;

	//std::cout << " head->center: " << head->center.x << " , " << head->center.y << " , " << head->center.z << std::endl;
	//std::cout << " halfWidth: " << head->halfWidth << " halfHeight: " << head->halfHeight << " halfDepth: " << head->halfDepth << std::endl;





}

/*
*  algorithm is based on http://en.wikipedia.org/wiki/Kd_tree
*/
void myKDtree::makeTree(std::list<myKDtree::vecType>& plist)
{
	Node* head = new Node(3);
	myKDtree::_makeTree(head, plist, 0);
	myKDtree::root = head;
}

void myKDtree::_makeTree(Node* head, std::list<myKDtree::vecType>& plist, int depth)
{
	if (!plist.empty())
	{
		int k = N;
		int axis = depth % k;

		std::list<myKDtree::vecType> left_list;
		std::list<myKDtree::vecType> right_list;
		myKDtree::vecType median = myKDtree::findMedian(axis, plist, left_list, right_list);
		head->data = median;

		setHeadVariables(head, plist);

		Node* left_node = new Node(k);
		Node* right_node = new Node(k);

		myKDtree::_makeTree(left_node, left_list, depth + 1);
		if (!left_list.empty()) head->left = left_node;

		myKDtree::_makeTree(right_node, right_list, depth + 1);
		if (!right_list.empty()) head->right = right_node;
	}
}

