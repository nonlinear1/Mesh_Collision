#include <iostream>
#include "display.h"
#include "inputManager.h"
#include "myKDtree.h"

Display display(DISPLAY_WIDTH, DISPLAY_HEIGHT, "OpenGL");	
Scene scn(glm::vec3(0.0f, 0.0f, -15.0f), CAM_ANGLE, relation, NEAR, FAR);
glm::vec3 YAxis = glm::vec3(0, 1, 0);
glm::vec3 TranslatePos = glm::vec3(5, 0, 0);

std::queue<std::pair<Node*, Node*>> queue;


bool SolveEquision(Node* A, Node* B, glm::vec3 L, glm::mat4 TRTB, glm::mat4 TRTA) {
	glm::vec4 centerB = TRTB*B->center;
	glm::vec4 centerA = TRTA*A->center;

	glm::vec3 T = glm::vec3(centerB.x - centerA.x, centerB.y - centerA.y, centerB.z - centerA.z);

	float p1 = glm::abs(glm::dot(A->halfWidth*A->axisX, L));
	float p2 = glm::abs(glm::dot(A->halfHeight*A->axisY, L));
	float p3 = glm::abs(glm::dot(A->halfDepth*A->axisZ, L));
	float p4 = glm::abs(glm::dot(B->halfWidth*B->axisX, L));
	float p5 = glm::abs(glm::dot(B->halfHeight*B->axisY, L));
	float p6 = glm::abs(glm::dot(B->halfDepth*B->axisZ, L));

	float leftSide = glm::abs(glm::dot(T, L));
	float rightSide = p1 + p2 + p3 + p4 + p5 + p6;

	if (leftSide > rightSide)
		return true;
	return false;

}

bool checkCollision(Node* A, Node* B, glm::mat4 TRTB, glm::mat4 TRTA) {

	//std::cout << "S-----------------------S" << std::endl;

	bool collison = true;
	glm::vec3 L;

	L = glm::vec3(TRTA * glm::vec4( A->axisX,1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisY, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisZ, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTB * glm::vec4(B->axisX, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTB * glm::vec4(B->axisY, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTB * glm::vec4(B->axisZ, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisX, 1)) * glm::vec3(TRTB * glm::vec4(B->axisX, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisX, 1))* glm::vec3(TRTB * glm::vec4(B->axisY, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisX, 1)) *glm::vec3(TRTB * glm::vec4(B->axisZ, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisY, 1)) * glm::vec3(TRTB * glm::vec4(B->axisX, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisY, 1)) *glm::vec3(TRTB * glm::vec4(B->axisY, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisY, 1)) * glm::vec3(TRTB * glm::vec4(B->axisZ, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisZ, 1)) *glm::vec3(TRTB * glm::vec4(B->axisX, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisZ, 1)) * glm::vec3(TRTB * glm::vec4(B->axisY, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;

	L = glm::vec3(TRTA * glm::vec4(A->axisZ, 1)) * glm::vec3(TRTB * glm::vec4(B->axisZ, 1));
	if (SolveEquision(A, B, L, TRTB, TRTA))
		collison = false;


	//std::cout << "E-----------------------E" << std::endl;

	return collison;
}

void print_queue(std::queue<std::pair<Node*, Node*>> q)
{
	while (!q.empty())
	{
		std::cout << q.front().first->center.x <<" , "<< q.front().first->center.y << " , " << q.front().first->center.z <<std::endl;
		std::cout << q.front().second->center.x << " , " << q.front().second->center.y << " , " << q.front().second->center.z << std::endl;

		q.pop();
	}
	std::cout << std::endl;
}


bool findCollision( Scene& scn, int first, int second) {
	//scn.shapes[first]->checkCollision(scn.shapes[first]->mesh->myKdtree.getRoot(), scn.shapes[second]->mesh->myKdtree.getRoot());
	queue.push(std::pair<Node*, Node*>(scn.shapes[first]->mesh->myKdtree.getRoot(), scn.shapes[second]->mesh->myKdtree.getRoot()));
	
	while (!queue.empty()) {
		std::pair<Node*, Node*> curPair = queue.front();
		queue.pop();

		if (checkCollision(curPair.first, curPair.second, scn.shapes[0]->makeTransScale(glm::mat4(1)), scn.shapes[1]->makeTransScale(glm::mat4(1)))) {
			//std::cout << "in collision width:" << curPair.first->halfWidth <<" height: "<< curPair.first->halfHeight << " depth: " << curPair.first->halfDepth << std::endl;
			//std::cout << "in collision width:" << curPair.second->halfWidth << " height: " << curPair.second->halfHeight << " depth: " << curPair.second->halfDepth << std::endl;
			//std::cout << "------------" << std::endl;
			//scn.draw(0, 0, true, 1, curPair.first);
			//std::cout << " halfWidth: " << curPair.first->halfWidth << " halfHeight: " << curPair.first->halfHeight << " halfDepth: " << curPair.first->halfDepth << std::endl;
			//std::cout << " center: " << curPair.first->center.x << " , " << curPair.first->center.y << " , " << curPair.first->center.z << std::endl;	
			//std::cout << " 1halfWidth: " << curPair.second->halfWidth << " halfHeight: " << curPair.second->halfHeight << " halfDepth: " << curPair.second->halfDepth << std::endl;
			//std::cout << " 1center: " << curPair.second->center.x << " , " << curPair.second->center.y << " , " << curPair.second->center.z << std::endl;
			//scn.posititonX = 0;
			//scn.posititonY = 0;

			if (curPair.first->isLastInnerNode() && curPair.second->isLastInnerNode()) {
				//std::cout << "got to last inner node1: " << curPair.first->halfWidth << curPair.first->halfHeight << curPair.first->halfDepth << std::endl;
				//std::cout << "got to last inner node2: " << curPair.second->halfWidth << curPair.second->halfHeight << curPair.second->halfDepth << std::endl;
				//std::cout << " halfWidth: " << curPair.first->halfWidth << " halfHeight: " << curPair.first->halfHeight << " halfDepth: " << curPair.first->halfDepth << std::endl;
				//std::cout << " center: " << curPair.first->center.x << " , " << curPair.first->center.y << " , " << curPair.first->center.z << std::endl;
				scn.posititonX = 0;
				scn.posititonY = 0;
				scn.draw(0, 0, true, 1, curPair.first);
				scn.draw(0, 0, true, 0, curPair.second);

				//std::cout <<" END" << std::endl;
				return true;
			}
			else if (curPair.second->isLastInnerNode() && curPair.first != nullptr) {
				//std::cout << "CASE 1" << std::endl;
				queue.push(std::pair<Node*, Node*>(curPair.first->left, curPair.second));
				queue.push(std::pair<Node*, Node*>(curPair.first->right, curPair.second));
			}
			else if (curPair.first->isLastInnerNode() && curPair.second != nullptr) {
				//std::cout << "CASE 2" << std::endl;

				queue.push(std::pair<Node*, Node*>(curPair.first, curPair.second->left));
				queue.push(std::pair<Node*, Node*>(curPair.first, curPair.second->right));
			}
			else {
				//print_queue(queue);

				//std::cout << "CASE 3" << std::endl;
				queue.push(std::pair<Node*, Node*>(curPair.first->left, curPair.second->left));
				queue.push(std::pair<Node*, Node*>(curPair.first->right, curPair.second->left));
				queue.push(std::pair<Node*, Node*>(curPair.first->left, curPair.second->right));
				queue.push(std::pair<Node*, Node*>(curPair.first->right, curPair.second->right));
				//print_queue(queue);

			}

		}


	}

	return true;
}




int main(int argc, char** argv)
{

	initCallbacks(display);
	//scn.addShape("./res/objs/bigbox.obj","./res/textures/box0.bmp");
	//scn.addShape("./res/objs/testboxNoUV.obj");
	//scn.addShape("./res/objs/testboxNoUV.obj", 6);

	scn.addShape("./res/objs/monkey3.obj", "./res/textures/grass.bmp", 512U);
	scn.addShape("./res/objs/monkey3.obj", "./res/textures/grass.bmp", 512U);

	//scn.addShape("./res/objs/monkeyNoUV.obj", "./res/textures/grass.bmp", 1600U);
	//scn.addShape("./res/objs/monkeyNoUV.obj", "./res/textures/grass.bmp", 10000U);
	//scn.addShape("./res/objs/monkeyNoUV.obj", "./res/textures/grass.bmp");

	scn.addShader("./res/shaders/basicShader");
	scn.addShader("./res/shaders/pickingShader");

	// Each will rotate around its Y axis, and will be moved a predefined distance
	for (int i = 0; i < scn.shapes.size(); i++) {
		scn.shapes[i]->myTranslate((float)i * TranslatePos, 0);
		scn.shapes[i]->myRotate(180, glm::vec3(0,1,0));

	}

	while(!display.toClose())
	{
		display.Clear(0.7f, 0.7f, 0.7f, 1.0f);
		
		// Rotate shapes
		/*
		for (auto& shape : scn.shapes) {
			shape->myRotate(0.05f, YAxis);
		}
		*/
		//scn.shapes[0]->mesh->myKdtree.getRoot()->center = scn.makeTrans()*scn.shapes[0]->mesh->myKdtree.getRoot()->center;
		//std::cout << "BEFORE center1: " << scn.shapes[1]->mesh->myKdtree.getRoot()->center.x << " , " << scn.shapes[1]->mesh->myKdtree.getRoot()->center.y << " , " << scn.shapes[1]->mesh->myKdtree.getRoot()->center.z << std::endl;
		
		//scn.shapes[1]->mesh->myKdtree.getRoot()->center = scn.makeTrans()*scn.shapes[1]->makeTransScale(glm::mat4(1))*firstcenter;
		
		//scn.shapes[1]->mesh->myKdtree.getRoot()->center.z = scn.shapes[0]->mesh->myKdtree.getRoot()->center.z;
		//std::cout << "center: " << scn.shapes[1]->mesh->myKdtree.getRoot()->center.x<<" , "<< scn.shapes[1]->mesh->myKdtree.getRoot()->center.y << " , " << scn.shapes[1]->mesh->myKdtree.getRoot()->center.z << std::endl;

		findCollision(scn,0,1);

		//bool weHaveCollision = scn.shapes[1]->checkCollision(scn.shapes[1]->mesh->myKdtree.getRoot(), scn.shapes[0]->mesh->myKdtree.getRoot(), scn.shapes[0]->makeTransScale(glm::mat4(1)), scn.shapes[1]->makeTransScale(glm::mat4(1)));
		
		//std::cout << "AFTER center1: " << scn.shapes[1]->mesh->myKdtree.getRoot()->center.x<<" , "<< scn.shapes[1]->mesh->myKdtree.getRoot()->center.y << " , " << scn.shapes[1]->mesh->myKdtree.getRoot()->center.z << std::endl;
		//std::cout << "center0: " << scn.shapes[0]->mesh->myKdtree.getRoot()->center.x<<" , "<< scn.shapes[0]->mesh->myKdtree.getRoot()->center.y << " , " << scn.shapes[0]->mesh->myKdtree.getRoot()->center.z << std::endl;

		/*
		if (weHaveCollision) {
			scn.posititonX = 0;
			scn.posititonY = 0;
		}
		*/
		//move shape 1 with velocity into shape 0
		float p1 = scn.posititonX;
		float p2 = scn.posititonY;
		scn.shapes[1]->myTranslate(glm::vec3(p1,p2,0),0);

		scn.draw(0,0,true,0,nullptr);
		display.SwapBuffers();
		display.pullEvent();
	}

	return 0;
}

