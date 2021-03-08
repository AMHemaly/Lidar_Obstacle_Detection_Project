/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "./render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTreeImpl
{
	Node* root;

	KdTreeImpl()
	: root(NULL)
	{}

	void insertNode(Node** node, int depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point,id);
		}
		else
		{
			uint cd = depth%3;
			if((*node)->point[cd] > point[cd])
				insertNode(&((*node)->left),depth+1, point, id);
			else
				insertNode(&((*node)->right),depth+1, point, id);

		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertNode(&root,0,point,id);

	}

	void searchNode(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if(    ((target[0]-distanceTol) <= node->point[0]) && ((target[0]+distanceTol) >= node->point[0]) 
				&& ((target[1]-distanceTol) <= node->point[1]) && ((target[1]+distanceTol) >= node->point[1])
				&& ((target[2]-distanceTol) <= node->point[2]) && ((target[2]+distanceTol) >= node->point[2]))
			{
				float dis = sqrt( (target[0]-node->point[0])*(target[0]-node->point[0])
				                 +(target[1]-node->point[1])*(target[1]-node->point[1])
				                 +(target[2]-node->point[2])*(target[2]-node->point[2]));
				if(dis<=distanceTol)
					ids.push_back(node->id);
			}

			if((target[depth%3]-distanceTol) < node->point[depth%3])
				searchNode(target, node->left, depth+1,distanceTol,ids);
			if((target[depth%3]+distanceTol) > node->point[depth%3])
				searchNode(target, node->right, depth+1,distanceTol,ids);
		}

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{

		std::vector<int> ids;
		searchNode(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




