/*
*
* k-d tree declaration file
*
* author: Arvind Rao
* license: GNU
*
*
*/

#ifndef KDTREE_
#define KDTREE_
#define _USE_MATH_DEFINES

#include <memory>
#include <vector>
#include <list>
#include <cmath>
#include <queue>
#include <utility>
#include <glm\glm.hpp>

const int N = 3;
class Node
{
public:
	typedef glm::vec4 vecType;
	Node* left;// = std::unique_ptr<Node>( new Node(3) );
	Node* right; //( new Node(3));
	vecType data;

	//added
	glm::vec4 center;
	glm::vec3 axisX;
	glm::vec3 axisY;
	glm::vec3 axisZ;

	float halfWidth;		//local x-axis
	float halfHeight;		//local y-axis
	float halfDepth;		//local z-axis

							//default constructor
	Node(int _n);

	//copy constructor
	Node(int _n, vecType _data);

	//return true if one of the sons or both are leafs
	bool isLastInnerNode();

	//default deconstructor
	~Node();



};

class myKDtree
{

public:
	//typedef float                numType;
	typedef Node::vecType vecType;

	//default constructor
	myKDtree();

	//default deconstructor
	~myKDtree();

	/*
	*   Return the tree root node
	*/
	Node* getRoot() const { return root; };
	/*
	* support function for printTree
	*/
	void print_data(vecType pt);

	/*  prints the tree
	*   and really works best for small trees
	*   as a test of tree construction.
	*/
	void printTree(Node* head);

	//function for finding the median of a list of points
	vecType findMedian(int axis, std::list<vecType> &plist, std::list<vecType> &left,
		std::list<vecType> &right);
	//function for making the tree
	void makeTree(std::list<vecType> &plist);

	void setHeadVariables(Node* head, std::list<vecType> &plist);


private:
	//helper for makeTree 
	void _makeTree(Node* head, std::list<vecType> &plist, int depth);

	Node* root;
};


#endif