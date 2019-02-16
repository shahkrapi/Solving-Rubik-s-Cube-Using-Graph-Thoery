#include <fstream>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <queue>  
#include <stack>
#include <string>
#include <new>
#include <omp.h>
#include <map>
#include <array>
#include <iterator>
#include <vector>
#include <stdlib.h>

using Eigen::Matrix3i;
using namespace std;

int countF = 0;
int countF2 = 0;
int countFDash = 0; 
int mode = 0;// 0 -> BFS, 1 -> DFS 
bool solution = false;


struct cube{
	Matrix3i front;
	Matrix3i up;
	Matrix3i down;
	Matrix3i back;
	Matrix3i left;
	Matrix3i right;
	string face;
	short level;
	short cost;
	int faceCost[6];
};

queue<cube> myqueue;
queue<cube> myqueue_back;
queue<cube> myqueue_front; 

stack<cube> mystack[4];
queue<cube> queueForHash[4];

vector<cube> vectorForHash[49];
cube input;

short computeCost(cube *c){

	Matrix3i temp;

	short center;
	short cost = 0;
	// int face_cost[6];

	temp = c->up;
	center = temp(1,1);
	// c->faceCost[0] = (temp.array() != center).count();
	c->faceCost[0] = (temp.array() != center).count();
	cost = cost + c->faceCost[0];

	temp = c->front;
	center = temp(1,1);
	// get<1>(c->faceCost) = (temp.array() != center).count();
	c->faceCost[1] = (temp.array() != center).count();
	cost = cost + c->faceCost[1];
	
	temp = c->left;
	center = temp(1,1);
	// get<2>(c->faceCost) = (temp.array() != center).count();
	c->faceCost[2] = (temp.array() != center).count();
	cost = cost + c->faceCost[2];
	
	temp = c->right;
	center = temp(1,1);
	// get<3>(c->faceCost) = (temp.array() != center).count();
	c->faceCost[3] = (temp.array() != center).count();
	cost = cost + c->faceCost[3];
	
	temp = c->back;
	center = temp(1,1);
	// get<4>(c->faceCost) = (temp.array() != center).count();
	c->faceCost[4] = (temp.array() != center).count();
	cost = cost + c->faceCost[4];
	
	temp = c->down;
	center = temp(1,1);
	// get<5>(c->faceCost) = (temp.array() != center).count();
	c->faceCost[5] = (temp.array() != center).count();
	cost = cost + c->faceCost[5];
    
	sort(c->faceCost,c->faceCost+6);
	return cost;
	
}


void print_cube(cube c)
{
	cout<<"----------------------Print Cube------------------------------------------------\nUp \n" ;

		for(int i=0; i<3; i++){
			for(int j=0;j<3;j++){
				cout<<int (c.up(i,j))<<" ";
		

			}	
			cout<<"\n";
		}
	cout<<"\nDown \n";
	for(int i=0; i<3; i++){
			for(int j=0;j<3;j++){
				cout<<int (c.down(i,j))<<" ";

			}	
			cout<<"\n";

		}
	cout<<"\nFront \n";
	for(int i=0; i<3; i++){
			for(int j=0;j<3;j++){
				cout<<int (c.front(i,j))<<" ";
		

			}	
			cout<<"\n";

		}
	cout<<"\nBack \n";
	for(int i=0; i<3; i++){
			for(int j=0;j<3;j++){
				cout<<int (c.back(i,j))<<" ";
		

			}	
			cout<<"\n";

		}
	cout<<"\nRight \n";
	for(int i=0; i<3; i++){
			for(int j=0;j<3;j++){
				cout<<int (c.right(i,j))<<" ";
		

			}	
			cout<<"\n";

		}
	cout<<"\nLeft \n";
	for(int i=0; i<3; i++){
			for(int j=0;j<3;j++){
				cout<<int (c.left(i,j))<<" ";
		

			}	
			cout<<"\n";

		}
	cout<<"Path: "<<c.face<<"\n";
	
}

void push_in(cube *c, int mode)
{
	c->cost = computeCost(c);
	if(mode == 1)
		myqueue_front.push(*c);	
	else if (mode == 2)
		mystack[omp_get_thread_num()].push(*c);
	else if(mode == 0)
		myqueue_back.push(*c);
	else if(mode == 3)
		queueForHash[omp_get_thread_num()].push(*c);
}



void F2(cube *c,int mode){
	c->front.transposeInPlace();
	c->front.col(0).swap(c->front.col(2));
	
	for(int i=0; i<3; i++){
		swap(c->up(2,i), c->right(i,0));
		swap(c->left(2-i,2), c->up(2, i));
		swap(c->down(0, 2-i), c->left(2-i, 2));
	}
		c->front.transposeInPlace();
	c->front.col(0).swap(c->front.col(2));
	
	for(int i=0; i<3; i++){
		swap(c->up(2,i), c->right(i,0));
		swap(c->left(2-i,2), c->up(2, i));
		swap(c->down(0, 2-i), c->left(2-i, 2));
	}
	countF2 ++;

	(c->face).append("F2");

	push_in(c, mode);

}

void F(cube *c,int mode){
	c->front.transposeInPlace();
	c->front.col(0).swap(c->front.col(2));
	
	for(int i=0; i<3; i++){
		swap(c->up(2,i), c->right(i,0));
		swap(c->left(2-i,2), c->up(2, i));
		swap(c->down(0, 2-i), c->left(2-i, 2));
	}
	countF++;
	// myqueue.push(*c);
	(c->face).append("F1");

	push_in(c, mode);

}


void Fdash(cube *c,int mode){
	c->front.transposeInPlace();
	c->front.row(0).swap(c->front.row(2));
    int temp;


    for(int i=0; i<3; i++){
	    temp = c->up(2,i);
	    c->up(2,i) = c->right(i,0);
	    c->right(i,0) = c->down(0,2-i);
	    c->down(0,2-i) = c->left(2-i,2);
	    c->left(2-i,2) = temp; 
	}   

	countFDash++;
	// myqueue.push(*c);
	(c->face).append("F3");
	push_in(c, mode);

	
}


void allMoves(cube *c,int mode){

	cube f = *c;
	cube fDash = *c;
	cube f2 = *c;

	F(&f,mode);
	Fdash(&fDash,mode);
	F2(&f2,mode);

}

void orientL(cube *c,int mode){
	c->up.transposeInPlace();
	c->up.row(0).swap(c->up.row(2));

	c->down.transposeInPlace();
	c->down.col(0).swap(c->down.col(2));

	Matrix3i temp;
	temp = c->front;
	c->front = c->left;
	c->left = c->back;
	c->back = c->right;
	c->right = temp;

	(c->face).append("-L-");
	
	allMoves(c,mode);


}

void orientR(cube *c,int mode){
	c->up.transposeInPlace();
	c->up.col(0).swap(c->up.col(2));

	c->down.transposeInPlace();
	c->down.row(0).swap(c->down.row(2));

	Matrix3i temp;
	temp = c->front;
	c->front = c->right;
	c->right = c->back;
	c->back = c->left;
	c->left = temp;
	
	(c->face).append("-R-");
	
	allMoves(c,mode);
}

void orientU(cube *c,int mode){
	c->left.transposeInPlace();
	c->left.col(0).swap(c->left.col(2));
	c->right.transposeInPlace();
	c->right.row(0).swap(c->right.row(2));
	Matrix3i temp;
	temp = c->front;
	c->front = c->up;
	c->up = c->back;
	c->back = c->down;
	c->down = temp;
	/*c->up.row(0).swap(c->up.row(2));
	c->back.col(0).swap(c->back.col(2));
	c->up.col(0).swap(c->up.col(2));
	c->back.row(0).swap(c->back.row(2));*/
	c->up.reverseInPlace();
	c->back.reverseInPlace();

	(c->face).append("-U-");
	
	allMoves(c,mode);
}

void orientD(cube *c,int mode){
	c->right.transposeInPlace();
	c->right.col(0).swap(c->right.col(2));
	

	c->left.transposeInPlace();
	c->left.row(0).swap(c->left.row(2));
	
	

	Matrix3i temp;
	temp = c->front;
	c->front = c->down;
	c->down = c->back;
	c->back = c->up;
	c->up = temp;
	//c->up.col(0).swap(c->up.col(2));
	c->down.row(0).swap(c->down.row(2));
	c->down.col(0).swap(c->down.col(2));
	c->back.reverseInPlace();

	(c->face).append("-D-");
	
	allMoves(c,mode);
	

}

void orientB(cube *c,int mode){
    c->up.transposeInPlace();
	c->up.col(0).swap(c->up.col(2));

	c->down.transposeInPlace();
	c->down.row(0).swap(c->down.row(2));

	Matrix3i temp;
	temp = c->front;
	c->front = c->right;
	c->right = c->back;
	c->back = c->left;
	c->left = temp;

	c->up.transposeInPlace();
	c->up.col(0).swap(c->up.col(2));

	c->down.transposeInPlace();
	c->down.row(0).swap(c->down.row(2));

	temp = c->front;
	c->front = c->right;
	c->right = c->back;
	c->back = c->left;
	c->left = temp;
	
	(c->face).append("-B-");
	
	allMoves(c,mode);
}


void generate_queue(int limit, cube input,queue<cube> &myqueue, int mode)
{
    int level_count = 0;
    if(mode == 3)
    	input.face = "";
	// cube solved;
     
	input.cost = computeCost(&input);
	myqueue.push(input);
	
    while (!myqueue.empty())
  	{
    		cube temp = myqueue.front();
    		if (temp.cost == 0 and temp.level > 0 and mode != 0) // Will give problem if root itself is solved!
            {
            	if(mode == 3)
            		return;

               	printf("-----------------Solution at level POS3: %d\n",temp.level);
               	solution = true;
               	cout<<"Path: "<<temp.face<<"\n";
               	print_cube(temp);
            }

    		if (temp.level == limit)
    		{
    			break;
    		} 
    		myqueue.pop();

            if(mode == 0)
    		    vectorForHash[temp.cost].push_back(temp);



	   		if (temp.level <= limit)
	   		{

         		temp.level = temp.level + 1;
	   		
		   		cube tempL = temp;
		   		cube tempB = temp;
		   		cube tempR = temp;
		   		cube tempU = temp;
		   		cube tempD = temp;


		   		if (temp.level == 1)
		   		{
		   			allMoves(&temp,mode);
		   		}
                
		   		orientL(&tempL,mode);
		   		orientB(&tempB,mode);   		
			   	orientR(&tempR,mode);
			    orientU(&tempU,mode);
		   		orientD(&tempD,mode); 	
	   		  		
			}

  	}	
}

int arrayMatch(const int *a, const int *b)
{
	for (int i = 0 ; i < 6; i++)
		if(a[i] != b[i])
			return 1;
	return 0;	
}

void print_array(const int *a)
{
	for (int i = 0 ; i < 6 ; i++)
		cout<<a[i]<<" ";
	cout<<"\n";
}


void checkForSolution(string path, cube c)
{
	for (int i = 0; i < path.length(); i++)
	{
		if(path[i] == '-')
			continue;
		else if(path[i] == 'L')
			orientL(&c, -1);
		else if(path[i] == 'B')
			orientB(&c, -1);
		else if(path[i] == 'R')
			orientR(&c, -1);
		else if(path[i] == 'U')
			orientU(&c, -1);
		else if(path[i] == 'D')
			orientD(&c, -1);
		else if(path[i] == 'F' and path[i+1] == '1')
			F(&c, -1);
		else if(path[i] == 'F' and path[i+1] == '2')
			F2(&c, -1);
		else if(path[i] == 'F' and path[i+1] == '3')
			Fdash(&c, -1);

	}
	if(computeCost(&c) == 0)
		cout<<"Solution found through match at: "<<c.face<<endl;	

}



static long long check = 0;

void match_cube(cube *c)
{
	long long total = 0;
	
	for (int j = 0; j < vectorForHash[c->cost].size(); j++)
	{
		cube temp=vectorForHash[c->cost][j];
		if(arrayMatch(temp.faceCost,c->faceCost)==0)
		{
			cube t = *c;
			checkForSolution(temp.face, t);
			total+=1;
		}
		    
	}
	#pragma omp atomic
		check++;	

}


void printHashMap()
{
	
	long long t = 0,full_count = 0;
	for (int i = 0; i < 49 ; i++ )
	{
		cout<<"Vector: "<<i<<" - "<<vectorForHash[i].size()<<endl;
		full_count = full_count + vectorForHash[i].size();	
	}
	cout<<"Full count: "<<full_count<<endl;
	full_count = 0;
	for (int i = 0; i < 49 ; i++ )
	{
		for (int j = 0 ; j < vectorForHash[i].size(); j++) 
    	{
        	t++;
        	full_count++;	
    	}
		cout<<"Key - "<<i<<" for : "<<t<<" "<<endl;
		t = 0;
   	}
   	cout<<"Full count: "<<full_count<<endl;
   	cout<<endl;	
    		
}

void pushInHashMap()
{	
	while(!myqueue_back.empty())
	{
		cube temp = myqueue_back.front();
		myqueue_back.pop();
		vectorForHash[temp.cost].push_back(temp);
	}

    printHashMap();
}


void optimiseHash()
{
	omp_set_dynamic(0);
	omp_set_num_threads(4);
	#pragma omp parallel for
	for (int i = 0; i < 49 ; i++ )
	{
		printf("Thread tid: %d\n", omp_get_thread_num());
		for (int j = 0 ; j < vectorForHash[i].size(); j++)
		{
			cube temp = vectorForHash[i][j];
			if(temp.cost == 0)
				continue;
			generate_queue(temp.level, temp, queueForHash[omp_get_thread_num()],3);

		}	
	}
}


void depthFirstSearch(int limit, int mode)
{
	printf("Depth First Search\n");
	std::vector<cube> V;
	V.reserve(myqueue_front.size());
	for(size_t numPos = 0; numPos < myqueue_front.size(); ++numPos) 
	{
  		V.push_back(myqueue_front.front());
  		myqueue_front.pop();
  		myqueue_front.push(V.back());
	}
	cout<<V.size()<<endl;

	omp_set_dynamic(0);
	omp_set_num_threads(3);
	long long size_queue = V.size();

	bool go = true;
	long long i;
	#pragma omp parallel for
	for (i = 0 ; i < size_queue; i++)
	{
		cube temp = V[i];
	
		if (temp.cost == 0)
        {
            printf("------%d---------Solution at level POS2: %d\n",omp_get_thread_num(),temp.level);
            cout<<"Path: "<<temp.face<<"\n";
            print_cube(temp);
            solution = true;
            go = false;

        }

		mystack[omp_get_thread_num()].push(temp);
		

		while(!mystack[omp_get_thread_num()].empty() and go==true)
    	{
    		cube t = mystack[omp_get_thread_num()].top();
    		mystack[omp_get_thread_num()].pop();
			
			if (t.cost == 0)
        	{
            	printf("--------%d---------Solution at level pOS1: %d\n",omp_get_thread_num(), t.level);
            	solution = true;
            	cout<<"Path: "<<t.face<<"\n";
            	print_cube(t);
            	go = false;
        	}
            if (t.level == limit)
            {
                match_cube(&temp);
            }
        	if (t.level < limit and go == true)
			{
				// cout<<"hahahaa"<<endl;
				t.level = t.level + 1;
			
   				cube tempL = t;
   				cube tempB = t;
   				cube tempR = t;
   				cube tempU = t;
   				cube tempD = t;
	
	   			orientL(&tempL,mode);
	   			orientB(&tempB,mode);
		   		orientR(&tempR,mode);
		   	 	orientU(&tempU,mode);
   				orientD(&tempD,mode); 	
		  		
			}
			
		}

    	if(solution == true){
    		go = false;
			cout<<"Setting go = false!"<<endl;		
    	}
    
    	if(go == false)
    	{
    		cout<<"Setting i = size_queue + 1"<<endl;
    		i = size_queue + 1;
    	}
	}
}
int main()
{

	int limitBFS = 5;
	int hashlen = 3;
	int limitDFS = 6;
	cube input, solved;
	input.level = solved.level = 0;
	solved.up <<    6, 6, 6,
                    6, 6, 6,
                    6, 6, 6;
                    
    solved.front << 5, 5, 5,
                    5, 5, 5,
                    5, 5, 5;

    solved.left <<  4, 4, 4,
                    4, 4, 4,
                    4, 4, 4;

    solved.back <<  3, 3, 3,
                    3, 3, 3,
                    3, 3, 3;

    solved.right << 2, 2, 2,
                    2, 2, 2,
                    2, 2, 2;

    solved.down <<  1, 1, 1,
                    1, 1, 1,
                    1, 1, 1;
    //  Hashmap BFS
    
    input.up <<     1, 6, 6,
                    4, 6, 4,
                    1, 6, 5;
                    
    input.front << 5, 3, 6,
                    1, 5, 2,
                    3, 1, 2;

    input.left <<  2, 5, 2,
                    2, 4, 5,
                    2, 1, 1;

    input.back <<  4, 4, 3,
                    6, 3, 3,
                    5, 6, 3;

    input.right << 4, 3, 3,
                    5, 2, 2,
                    5, 3, 4;

    input.down <<  4, 2, 6,
                    4, 1, 1,
                    6, 5, 1;

	omp_set_dynamic(0);
	omp_set_num_threads(2);

	#pragma omp parallel for
	for (int i = 0 ; i < 2 ; i++ )
	{
		if(i == 1)
        {   
        	cout<<"Thread 1 running"<<endl;
        	generate_queue(limitBFS, input, myqueue_front, 1);
        }	
        else if (i == 0)
        {
        	cout<<"Thread 2 running"<<endl;
        	generate_queue(hashlen, solved, myqueue_back, 0);
        	cout<<"\nSize of queue:"<<myqueue_back.size()<<endl;
        	pushInHashMap();
        }  	
	} 
	cout<<"---------------------------------------------------------"<<endl;

	#pragma omp barrier

	mode = 3;
	optimiseHash();


	mode = 2;
    // DFS
	if(solution == false)
		depthFirstSearch(limitDFS, mode);


	cout<<"-----\n";
	cout<<"\nSize of queue:"<<myqueue_back.size()<<endl;
	cout<<"\nSize of queue:"<<myqueue_front.size()<<endl;

	cout<<"Check: "<<check<<endl;

	return 0;
	
}
